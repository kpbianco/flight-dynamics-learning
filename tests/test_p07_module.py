from __future__ import annotations

import json
import math
import re
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULE_ID = "P07"
MODULE_FOLDER = ROOT / "modules/07-excite-roll-spiral-and-dutch-roll-modes"
GUIDING_QUESTION = (
    "What inputs, observable effects, and failure modes matter when you excite "
    "Roll, Spiral, and Dutch-Roll Modes?"
)


def _finite_real_scalar(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite real scalar")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite real scalar")
    return result


def _oracle(
    aileron_pulse_deg: object = 2.0,
    bank_release_deg: object = 5.0,
    rudder_pulse_deg: object = 3.0,
    roll_decay_rate_per_s: object = 2.5,
    spiral_decay_rate_per_s: object = 0.025,
    dutch_roll_damping_ratio: object = 0.18,
) -> dict[str, object]:
    """Independent Python oracle; it does not execute or translate MATLAB."""
    aileron_deg = _finite_real_scalar("aileron pulse", aileron_pulse_deg)
    bank_deg = _finite_real_scalar("bank release", bank_release_deg)
    rudder_deg = _finite_real_scalar("rudder pulse", rudder_pulse_deg)
    roll_decay = _finite_real_scalar("roll decay rate", roll_decay_rate_per_s)
    spiral_decay = _finite_real_scalar(
        "spiral decay rate", spiral_decay_rate_per_s
    )
    dutch_zeta = _finite_real_scalar(
        "Dutch-roll damping ratio", dutch_roll_damping_ratio
    )
    if abs(aileron_deg) > 5.0:
        raise ValueError("aileron pulse outside the learning range")
    if abs(bank_deg) > 10.0:
        raise ValueError("bank release outside the learning range")
    if abs(rudder_deg) > 5.0:
        raise ValueError("rudder pulse outside the learning range")
    if not 0.8 <= roll_decay <= 5.0:
        raise ValueError("roll decay rate outside the learning range")
    if spiral_decay < 0.0:
        raise ValueError("spiral decay rate must be nonnegative")
    if spiral_decay > 0.05:
        raise ValueError("spiral decay rate outside the learning range")
    if dutch_zeta < 0.0:
        raise ValueError("Dutch-roll damping ratio must be nonnegative")
    if dutch_zeta > 0.6:
        raise ValueError("Dutch-roll damping ratio outside the learning range")

    reference_true_airspeed_mps = 60.0
    reference_dynamic_pressure_pa = 1325.00798531847
    wing_area_m2 = 16.2
    wing_span_m = 10.9
    roll_inertia_kgm2 = 2500.0
    yaw_inertia_kgm2 = 4000.0
    aileron_control_derivative_per_rad = 0.08
    rudder_control_derivative_per_rad = 0.10
    control_pulse_duration_s = 0.18
    dutch_natural_frequency_rad_s = 1.15
    gravity_mps2 = 9.80665
    bank_linear_limit_deg = 15.0
    sideslip_linear_limit_deg = 5.0

    roll_time_s = tuple(index * 0.02 for index in range(251))
    dutch_time_s = tuple(index * 0.05 for index in range(501))
    spiral_time_s = tuple(index * 0.25 for index in range(481))
    moment_scale_nm = (
        reference_dynamic_pressure_pa * wing_area_m2 * wing_span_m
    )

    aileron_rad = math.radians(aileron_deg)
    aileron_moment_nm = (
        moment_scale_nm * aileron_control_derivative_per_rad * aileron_rad
    )
    initial_roll_rate_rad_s = (
        aileron_moment_nm * control_pulse_duration_s / roll_inertia_kgm2
    )
    roll_rate_rad_s = tuple(
        initial_roll_rate_rad_s * math.exp(-roll_decay * time_s)
        for time_s in roll_time_s
    )
    roll_rate_derivative_rad_s2 = tuple(
        -roll_decay * value for value in roll_rate_rad_s
    )
    roll_bank_change_rad = tuple(
        initial_roll_rate_rad_s
        / roll_decay
        * (1.0 - math.exp(-roll_decay * time_s))
        for time_s in roll_time_s
    )
    roll_rate_deg_s = tuple(math.degrees(value) for value in roll_rate_rad_s)
    roll_bank_change_deg = tuple(
        math.degrees(value) for value in roll_bank_change_rad
    )

    bank_rad = math.radians(bank_deg)
    spiral_exponential = tuple(
        math.exp(-spiral_decay * time_s) for time_s in spiral_time_s
    )
    spiral_bank_rad = tuple(
        bank_rad * exponential for exponential in spiral_exponential
    )
    spiral_roll_rate_rad_s = tuple(
        -spiral_decay * bank for bank in spiral_bank_rad
    )
    spiral_heading_rate_rad_s = tuple(
        gravity_mps2 / reference_true_airspeed_mps * bank
        for bank in spiral_bank_rad
    )
    if spiral_decay == 0.0:
        spiral_heading_change_rad = tuple(
            gravity_mps2
            / reference_true_airspeed_mps
            * bank_rad
            * time_s
            for time_s in spiral_time_s
        )
        spiral_time_constant_s = math.inf
        spiral_half_life_s = math.inf
        spiral_time_scale_is_representable = False
    else:
        if spiral_decay * spiral_time_s[-1] < math.sqrt(sys.float_info.epsilon):
            integral_time_s = tuple(
                time_s - 0.5 * spiral_decay * time_s**2
                for time_s in spiral_time_s
            )
        else:
            integral_time_s = tuple(
                -math.expm1(-spiral_decay * time_s) / spiral_decay
                for time_s in spiral_time_s
            )
        spiral_heading_change_rad = tuple(
            gravity_mps2 / reference_true_airspeed_mps * bank_rad * integral
            for integral in integral_time_s
        )
        spiral_time_constant_s = (
            math.inf
            if spiral_decay < 1.0 / sys.float_info.max
            else 1.0 / spiral_decay
        )
        spiral_half_life_s = (
            math.inf
            if spiral_decay < math.log(2.0) / sys.float_info.max
            else math.log(2.0) / spiral_decay
        )
        spiral_time_scale_is_representable = math.isfinite(
            spiral_time_constant_s
        ) and math.isfinite(spiral_half_life_s)
    spiral_bank_deg = tuple(math.degrees(value) for value in spiral_bank_rad)
    spiral_roll_rate_deg_s = tuple(
        math.degrees(value) for value in spiral_roll_rate_rad_s
    )
    spiral_heading_rate_deg_s = tuple(
        math.degrees(value) for value in spiral_heading_rate_rad_s
    )
    spiral_heading_change_deg = tuple(
        math.degrees(value) for value in spiral_heading_change_rad
    )

    rudder_rad = math.radians(rudder_deg)
    rudder_moment_nm = (
        moment_scale_nm * rudder_control_derivative_per_rad * rudder_rad
    )
    initial_yaw_rate_rad_s = (
        rudder_moment_nm * control_pulse_duration_s / yaw_inertia_kgm2
    )
    initial_sideslip_rate_rad_s = -initial_yaw_rate_rad_s
    dutch_damped_frequency_rad_s = dutch_natural_frequency_rad_s * math.sqrt(
        1.0 - dutch_zeta**2
    )
    dutch_decay_rate_per_s = dutch_zeta * dutch_natural_frequency_rad_s
    dutch_damped_period_s = 2.0 * math.pi / dutch_damped_frequency_rad_s
    dutch_decay_per_period_ratio = math.exp(
        -dutch_decay_rate_per_s * dutch_damped_period_s
    )
    dutch_sideslip_rad = tuple(
        initial_sideslip_rate_rad_s
        / dutch_damped_frequency_rad_s
        * math.exp(-dutch_decay_rate_per_s * time_s)
        * math.sin(dutch_damped_frequency_rad_s * time_s)
        for time_s in dutch_time_s
    )
    dutch_sideslip_rate_rad_s = tuple(
        initial_sideslip_rate_rad_s
        * math.exp(-dutch_decay_rate_per_s * time_s)
        * (
            math.cos(dutch_damped_frequency_rad_s * time_s)
            - dutch_decay_rate_per_s
            / dutch_damped_frequency_rad_s
            * math.sin(dutch_damped_frequency_rad_s * time_s)
        )
        for time_s in dutch_time_s
    )
    dutch_sideslip_acceleration_rad_s2 = tuple(
        initial_sideslip_rate_rad_s
        * math.exp(-dutch_decay_rate_per_s * time_s)
        * (
            -2.0
            * dutch_decay_rate_per_s
            * math.cos(dutch_damped_frequency_rad_s * time_s)
            + (
                dutch_decay_rate_per_s**2 / dutch_damped_frequency_rad_s
                - dutch_damped_frequency_rad_s
            )
            * math.sin(dutch_damped_frequency_rad_s * time_s)
        )
        for time_s in dutch_time_s
    )
    dutch_yaw_rate_rad_s = tuple(
        -value for value in dutch_sideslip_rate_rad_s
    )
    dutch_yaw_acceleration_rad_s2 = tuple(
        -value for value in dutch_sideslip_acceleration_rad_s2
    )
    dutch_sideslip_deg = tuple(
        math.degrees(value) for value in dutch_sideslip_rad
    )
    dutch_yaw_rate_deg_s = tuple(
        math.degrees(value) for value in dutch_yaw_rate_rad_s
    )
    dutch_sideslip_envelope_deg = tuple(
        math.degrees(
            abs(initial_sideslip_rate_rad_s) / dutch_damped_frequency_rad_s
        )
        * math.exp(-dutch_decay_rate_per_s * time_s)
        for time_s in dutch_time_s
    )
    beta_kinematic_residual_rad_s = tuple(
        beta_rate + yaw_rate
        for beta_rate, yaw_rate in zip(
            dutch_sideslip_rate_rad_s, dutch_yaw_rate_rad_s
        )
    )
    yaw_equation_residual_rad_s2 = tuple(
        yaw_acceleration
        - dutch_natural_frequency_rad_s**2 * sideslip
        + 2.0 * dutch_decay_rate_per_s * yaw_rate
        for yaw_acceleration, sideslip, yaw_rate in zip(
            dutch_yaw_acceleration_rad_s2,
            dutch_sideslip_rad,
            dutch_yaw_rate_rad_s,
        )
    )
    modal_energy_rad2_s2 = tuple(
        0.5
        * (
            yaw_rate**2
            + dutch_natural_frequency_rad_s**2 * sideslip**2
        )
        for yaw_rate, sideslip in zip(
            dutch_yaw_rate_rad_s, dutch_sideslip_rad
        )
    )
    modal_energy_rate_rad2_s3 = tuple(
        -2.0 * dutch_decay_rate_per_s * yaw_rate**2
        for yaw_rate in dutch_yaw_rate_rad_s
    )

    return {
        "aileron_pulse_deg": aileron_deg,
        "bank_release_deg": bank_deg,
        "rudder_pulse_deg": rudder_deg,
        "roll_decay_rate_per_s": roll_decay,
        "spiral_decay_rate_per_s": spiral_decay,
        "dutch_roll_damping_ratio": dutch_zeta,
        "roll_time_s": roll_time_s,
        "dutch_roll_time_s": dutch_time_s,
        "spiral_time_s": spiral_time_s,
        "roll_sample_count": len(roll_time_s),
        "dutch_roll_sample_count": len(dutch_time_s),
        "spiral_sample_count": len(spiral_time_s),
        "roll_rate_deg_s": roll_rate_deg_s,
        "roll_rate_derivative_rad_s2": roll_rate_derivative_rad_s2,
        "roll_bank_change_deg": roll_bank_change_deg,
        "spiral_bank_deg": spiral_bank_deg,
        "spiral_roll_rate_deg_s": spiral_roll_rate_deg_s,
        "spiral_heading_rate_deg_s": spiral_heading_rate_deg_s,
        "spiral_heading_change_deg": spiral_heading_change_deg,
        "dutch_roll_sideslip_deg": dutch_sideslip_deg,
        "dutch_roll_sideslip_rate_rad_s": dutch_sideslip_rate_rad_s,
        "dutch_roll_sideslip_acceleration_rad_s2": (
            dutch_sideslip_acceleration_rad_s2
        ),
        "dutch_roll_yaw_rate_deg_s": dutch_yaw_rate_deg_s,
        "dutch_roll_yaw_acceleration_rad_s2": (
            dutch_yaw_acceleration_rad_s2
        ),
        "dutch_roll_sideslip_envelope_deg": dutch_sideslip_envelope_deg,
        "dutch_roll_beta_kinematic_residual_rad_s": (
            beta_kinematic_residual_rad_s
        ),
        "dutch_roll_yaw_equation_residual_rad_s2": (
            yaw_equation_residual_rad_s2
        ),
        "dutch_roll_modal_energy_rad2_s2": modal_energy_rad2_s2,
        "dutch_roll_modal_energy_rate_rad2_s3": modal_energy_rate_rad2_s3,
        "initial_roll_rate_rad_s": initial_roll_rate_rad_s,
        "initial_roll_rate_deg_s": math.degrees(initial_roll_rate_rad_s),
        "initial_yaw_rate_rad_s": initial_yaw_rate_rad_s,
        "initial_yaw_rate_deg_s": math.degrees(initial_yaw_rate_rad_s),
        "initial_sideslip_rate_rad_s": initial_sideslip_rate_rad_s,
        "aileron_moment_nm": aileron_moment_nm,
        "rudder_moment_nm": rudder_moment_nm,
        "roll_moment_scale_nm": moment_scale_nm,
        "roll_asymptotic_bank_change_deg": math.degrees(
            initial_roll_rate_rad_s / roll_decay
        ),
        "roll_time_constant_s": 1.0 / roll_decay,
        "roll_half_life_s": math.log(2.0) / roll_decay,
        "roll_two_percent_settling_time_s": -math.log(0.02) / roll_decay,
        "spiral_time_constant_s": spiral_time_constant_s,
        "spiral_half_life_s": spiral_half_life_s,
        "spiral_time_scale_is_representable": (
            spiral_time_scale_is_representable
        ),
        "spiral_bank_remaining_at_end_ratio": spiral_exponential[-1],
        "dutch_roll_natural_frequency_rad_s": (
            dutch_natural_frequency_rad_s
        ),
        "dutch_roll_damped_frequency_rad_s": dutch_damped_frequency_rad_s,
        "dutch_roll_decay_rate_per_s": dutch_decay_rate_per_s,
        "dutch_roll_damped_period_s": dutch_damped_period_s,
        "dutch_roll_decay_per_period_ratio": dutch_decay_per_period_ratio,
        "roll_peak_rate_deg_s": max(map(abs, roll_rate_deg_s)),
        "dutch_roll_peak_sideslip_deg": max(map(abs, dutch_sideslip_deg)),
        "dutch_roll_peak_yaw_rate_deg_s": max(
            map(abs, dutch_yaw_rate_deg_s)
        ),
        "spiral_heading_range_deg": max(spiral_heading_change_deg)
        - min(spiral_heading_change_deg),
        "dutch_to_roll_time_scale_ratio": dutch_damped_period_s
        / (1.0 / roll_decay),
        "spiral_to_dutch_time_scale_ratio": spiral_time_constant_s
        / dutch_damped_period_s,
        "bank_linear_limit_deg": bank_linear_limit_deg,
        "sideslip_linear_limit_deg": sideslip_linear_limit_deg,
        "is_within_roll_bank_linear_range": max(
            map(abs, roll_bank_change_deg)
        )
        <= bank_linear_limit_deg,
        "is_within_spiral_bank_linear_range": max(map(abs, spiral_bank_deg))
        <= bank_linear_limit_deg,
        "is_within_dutch_roll_sideslip_linear_range": max(
            map(abs, dutch_sideslip_deg)
        )
        <= sideslip_linear_limit_deg,
        "reference_true_airspeed_mps": reference_true_airspeed_mps,
        "reference_dynamic_pressure_pa": reference_dynamic_pressure_pa,
        "wing_area_m2": wing_area_m2,
        "wing_span_m": wing_span_m,
        "roll_inertia_kgm2": roll_inertia_kgm2,
        "yaw_inertia_kgm2": yaw_inertia_kgm2,
        "aileron_control_derivative_per_rad": (
            aileron_control_derivative_per_rad
        ),
        "rudder_control_derivative_per_rad": (
            rudder_control_derivative_per_rad
        ),
        "control_pulse_duration_s": control_pulse_duration_s,
        "gravity_mps2": gravity_mps2,
    }


class P07ArtifactTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        manifest = json.loads(
            (ROOT / "curriculum/modules.json").read_text(encoding="utf-8")
        )
        cls.module = next(
            module for module in manifest["modules"] if module["id"] == MODULE_ID
        )
        cls.text = {
            path.name: path.read_text(encoding="utf-8")
            for path in MODULE_FOLDER.iterdir()
            if path.is_file()
        }

    def test_permanent_manifest_identity_and_artifact_set(self) -> None:
        self.assertEqual(
            {
                "number": self.module["number"],
                "id": self.module["id"],
                "title": self.module["title"],
                "guiding_question": self.module["guiding_question"],
                "phase": self.module["phase"],
                "phase_title": self.module["phase_title"],
                "slug": self.module["slug"],
                "folder": self.module["folder"],
                "implementation_batch": self.module["implementation_batch"],
                "prerequisites": self.module["prerequisites"],
            },
            {
                "number": 7,
                "id": "P07",
                "title": "Excite Roll, Spiral, and Dutch-Roll Modes",
                "guiding_question": GUIDING_QUESTION,
                "phase": 2,
                "phase_title": "Stability and modes",
                "slug": "excite-roll-spiral-and-dutch-roll-modes",
                "folder": "modules/07-excite-roll-spiral-and-dutch-roll-modes",
                "implementation_batch": "P07",
                "prerequisites": ["P06"],
            },
        )
        self.assertEqual(self.module["status"], "implemented")
        self.assertEqual(self.module["evidence_level"], "simulated")

        required = {
            "README.md",
            "lesson.m",
            "model.m",
            "experiment.m",
            "interactive.m",
            "lesson.md",
            "walkthrough.md",
            "checks.md",
            "run_checks.m",
        }
        self.assertTrue(required <= self.text.keys(), required - self.text.keys())
        for name in required:
            with self.subTest(file=name):
                self.assertTrue(self.text[name].strip(), name)
                self.assertTrue(self.text[name].endswith("\n"), name)
                self.assertFalse(self.text[name].endswith("\n\n"), name)

    def test_learning_slice_is_concept_first_and_complete(self) -> None:
        readme = self.text["README.md"]
        lesson_script = self.text["lesson.m"]
        experiment = self.text["experiment.m"]
        lesson = self.text["lesson.md"]
        walkthrough = self.text["walkthrough.md"]
        checks = self.text["checks.md"]
        combined = "\n".join(
            (readme, lesson_script, experiment, lesson, walkthrough, checks)
        ).lower()

        for name, text in {
            "README.md": readme,
            "lesson.m": lesson_script,
            "experiment.m": experiment,
            "lesson.md": lesson,
            "walkthrough.md": walkthrough,
            "checks.md": checks,
        }.items():
            with self.subTest(file=name):
                self.assertIn(GUIDING_QUESTION, text)

        for concept in (
            "p06",
            "roll subsidence",
            "spiral",
            "dutch roll",
            "aileron",
            "bank release",
            "rudder",
            "right-wing-down",
            "sideslip",
            "yaw rate",
            "mechanism",
            "reset",
            "teach-back",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, combined.replace("dutch-roll", "dutch roll"))
        self.assertIn("read", walkthrough.lower())
        self.assertIn("baseline", walkthrough.lower())
        self.assertRegex(walkthrough.lower(), r"one visual transition|one .* at a time")
        self.assertRegex(walkthrough.lower(), r"observe|inspect")
        experiment_lower = experiment.lower()
        self.assertEqual(experiment_lower.count("predict once:"), 1)
        self.assertLess(
            experiment_lower.index("predict once:"),
            experiment_lower.index("baseline=model("),
        )
        for placeholder in (
            "curriculum-scaffolded",
            "not implemented yet",
            "intentionally refuses",
            "activate its governed implementation batch",
            "planned concept loop",
            "todo",
            "tbd",
        ):
            self.assertNotIn(placeholder, combined)

    def test_model_is_transparent_guarded_and_presentation_free(self) -> None:
        model = self.text["model.m"]
        lower = model.lower()
        compact = re.sub(r"\s+", "", model.replace("...", "")).lower()

        self.assertIn(
            "functionout=model(aileronpulse_deg,bankrelease_deg,rudderpulse_deg,"
            "rolldecayrate_per_s,spiraldecayrate_per_s,dutchrolldampingratio)",
            compact,
        )
        self.assertIn("arguments", lower)
        self.assertEqual(model.count("(1,1) double {mustBeReal,mustBeFinite}"), 6)
        for expression in (
            "abs(aileronpulse_deg)>5",
            "abs(bankrelease_deg)>10",
            "abs(rudderpulse_deg)>5",
            "rolldecayrate_per_s<0.8||rolldecayrate_per_s>5",
            "spiraldecayrate_per_s<0",
            "spiraldecayrate_per_s>0.05",
            "dutchrolldampingratio<0",
            "dutchrolldampingratio>0.6",
            "rolltime_s=0:0.02:5;",
            "dutchrolltime_s=0:0.05:25;",
            "spiraltime_s=0:0.25:120;",
            "spiralintegraltime_s=-expm1(-spiraldecayrate_per_s*spiraltime_s)/"
            "spiraldecayrate_per_s;",
        ):
            self.assertIn(expression, compact)
        for identifier in (
            "P07:model:AileronPulseRange",
            "P07:model:BankReleaseRange",
            "P07:model:RudderPulseRange",
            "P07:model:RollDecayRateRange",
            "P07:model:NegativeSpiralDecayRate",
            "P07:model:SpiralDecayRateRange",
            "P07:model:NegativeDutchRollDamping",
            "P07:model:DutchRollDampingRange",
        ):
            self.assertIn(identifier, model)

        for formula in (
            "rollmomentscale_nm=referencedynamicpressure_pa*wingarea_m2*"
            "wingspan_m;",
            "initialrollrate_rad_s=aileronmoment_nm*controlpulseduration_s/"
            "rollinertia_kgm2;",
            "rollrate_rad_s=initialrollrate_rad_s*rollexponential;",
            "rollbankchange_rad=initialrollrate_rad_s/rolldecayrate_per_s*"
            "(1-rollexponential);",
            "spiralbank_rad=bankrelease_rad*spiralexponential;",
            "spiralheadingrate_rad_s=gravity_mps2/referencetrueairspeed_mps*"
            "spiralbank_rad;",
            "initialyawrate_rad_s=ruddermoment_nm*controlpulseduration_s/"
            "yawinertia_kgm2;",
            "initialsidesliprate_rad_s=-initialyawrate_rad_s;",
            "dutchrollyawrate_rad_s=-dutchrollsidesliprate_rad_s;",
            "dutchrollmodalenergy_rad2_s2=0.5*(dutchrollyawrate_rad_s.^2+"
            "dutchrollnaturalfrequency_rad_s^2*dutchrollsideslip_rad.^2);",
        ):
            self.assertIn(formula, compact)
        self.assertIn("dutchRollNaturalFrequency_rad_s=1.15", model)
        self.assertIn("positive beta is air-relative velocity toward body right", model)
        self.assertIn("beta_dot approximately -r", model)
        self.assertIn("not identified aircraft data or a full lateral state-space model", model)

        for presentation_call in (
            "figure(",
            "plot(",
            "bar(",
            "uiaxes(",
            "uifigure(",
            "disp(",
            "fprintf(",
        ):
            self.assertNotIn(presentation_call, lower)
        self.assertNotRegex(
            lower, re.compile(r"^\s*(?:for|while|parfor)\b", re.MULTILINE)
        )

    def test_experiment_has_two_sweeps_metrics_and_broken_case(self) -> None:
        experiment = self.text["experiment.m"]
        lower = experiment.lower()
        compact = re.sub(r"\s+", "", experiment.replace("...", "")).lower()
        self.assertGreaterEqual(experiment.count("%%"), 16)
        self.assertIn("baseline", lower)
        self.assertGreaterEqual(lower.count("sweep"), 2)
        self.assertRegex(lower, r"sweep[^\n]*roll decay|roll[^\n]*sweep")
        self.assertRegex(lower, r"sweep[^\n]*dutch-roll|dutch-roll[^\n]*sweep")
        self.assertIn("broken", lower)
        self.assertIn("reverse the stable spiral sign", lower)
        self.assertIn(
            "exp(+baseline.spiraldecayrate_per_s*brokentime_s)", compact
        )
        self.assertGreaterEqual(lower.count("figure("), 6)
        self.assertIn("xlabel", lower)
        self.assertIn("ylabel", lower)
        for unit in ("1/s", "rad/s", "deg/s", "m/s", "deg", "pa", "s"):
            with self.subTest(unit=unit):
                self.assertIn(unit, lower)
        self.assertIn("fprintf", lower)
        self.assertGreaterEqual(lower.count("assert("), 3)
        self.assertIn(
            "model(2,5,3,rolldecayratesweep_per_s(k),0.025,0.18)", compact
        )
        self.assertIn(
            "model(2,5,3,2.5,0.025,dutchrolldampingsweep(k))", compact
        )
        self.assertNotRegex(lower, r"\bclose\s+all\b")
        self.assertIn("findall(groot", lower)
        self.assertIn("'^p07 '", lower)
        self.assertRegex(lower, r"clear run_checks;\s*run_checks;")

        for variable in (
            "rollDecayRateSweep_per_s",
            "dutchRollDampingSweep",
        ):
            match = re.search(rf"{variable}\s*=\s*\[([^\]]+)\]", experiment)
            self.assertIsNotNone(match, variable)
            values = [float(value) for value in match.group(1).split()]
            self.assertGreaterEqual(len(values), 3, variable)
            self.assertLessEqual(len(values), 12, variable)
            self.assertTrue(all(math.isfinite(value) for value in values), variable)

    def test_interaction_checks_recovery_and_resource_bounds(self) -> None:
        experiment = self.text["experiment.m"]
        interactive = self.text["interactive.m"]
        checks_script = self.text["run_checks.m"]
        interactive_lower = interactive.lower()
        checks_lower = checks_script.lower()
        interactive_compact = re.sub(
            r"\s+", "", interactive.replace("...", "")
        ).lower()
        checks_compact = re.sub(
            r"\s+", "", checks_script.replace("...", "")
        ).lower()

        self.assertIn("clear model;", "\n".join(experiment.splitlines()[:12]).lower())
        self.assertIn("clear model;", "\n".join(interactive_lower.splitlines()[:5]))
        self.assertIn("clear model;", "\n".join(checks_lower.splitlines()[:5]))
        self.assertIn("uifigure(", interactive_lower)
        self.assertIn(
            "existingui=findall(groot,'type','figure','name',"
            "'p07roll,spiral,anddutch-rollmodes')",
            interactive_compact,
        )
        self.assertIn("close(existingui)", interactive_compact)
        self.assertEqual(interactive_lower.count("uislider("), 6)
        for control in (
            "aileron",
            "bank",
            "rudder",
            "roll decay",
            "spiral decay",
            "dutch-roll damping",
        ):
            self.assertIn(control, interactive_lower)
        self.assertIn("valuechangingfcn", interactive_lower)
        self.assertIn("valuechangedfcn", interactive_lower)
        self.assertIn("event.value", interactive_lower)
        self.assertIn("modelfcn=@model;", interactive_compact)
        self.assertIn("out=modelfcn(", interactive_compact)
        self.assertEqual(interactive_compact.count("out=modelfcn("), 1)
        for axis_name in (
            "axrollrate",
            "axspiralbank",
            "axspiralheading",
            "axdutchsideslip",
            "axdutchyawrate",
            "axdutchenergy",
        ):
            self.assertIn(f"cla({axis_name})", interactive_compact)
        self.assertNotIn("yyaxis", interactive_lower)
        for unit in ("1/s", "deg/s", "m/s", "deg", "s"):
            self.assertIn(unit, interactive_lower)

        self.assertGreaterEqual(checks_lower.count("assert("), 45)
        for concept in (
            "determinism",
            "fixed vector shape",
            "independent dimensional buildup",
            "closed-form responses",
            "distinct time scales",
            "zero-input isolation",
            "sign symmetry",
            "neutral and undamped limits",
            "sweep regressions",
            "broken spiral-stability sign",
            "aileronpulserange",
            "bankreleaserange",
            "rudderpulserange",
            "rolldecayraterange",
            "spiraldecayraterange",
            "dutchrolldampingrange",
            "rejected inputs",
        ):
            with self.subTest(concept=concept):
                self.assertIn(
                    concept.replace("-", ""), checks_lower.replace("-", "")
                )
        self.assertIn("edgecasecount==64", checks_compact)
        self.assertIn("rollsamplecount==251", checks_compact)
        self.assertIn("dutchrollsamplecount==501", checks_compact)
        self.assertIn("spiralsamplecount==481", checks_compact)
        self.assertIn("centeredrollbankrate_rad_s=", checks_compact)
        self.assertIn("centeredheadingrate_rad_s=", checks_compact)
        self.assertIn("centeredmodalenergyrate_rad2_s3=", checks_compact)
        self.assertIn("catch exception", checks_lower)
        self.assertIn(
            "strcmp(exception.identifier,expectedidentifier)", checks_compact
        )
        self.assertIn("P07 checks passed", checks_script)

    def test_no_opaque_toolbox_random_external_or_async_behavior(self) -> None:
        matlab = "\n".join(
            self.text[name]
            for name in (
                "model.m",
                "experiment.m",
                "interactive.m",
                "lesson.m",
                "run_checks.m",
            )
        ).lower()
        forbidden_calls = (
            "fsolve",
            "lsqnonlin",
            "fmincon",
            "fminunc",
            "lsqlin",
            "quadprog",
            "optimproblem",
            "solve",
            "vpasolve",
            "trim",
            "findop",
            "operspec",
            "linearize",
            "sim",
            "load_system",
            "open_system",
            "ss",
            "tf",
            "damp",
            "pole",
            "eig",
            "expm",
            "lsim",
            "initial",
            "impulse",
            "step",
            "ode45",
            "ode23",
            "roots",
            "polyfit",
            "readtable",
            "writetable",
            "webread",
            "urlread",
            "fopen",
            "fread",
            "fwrite",
            "serialport",
            "tcpclient",
            "udpport",
            "system",
            "pause",
            "timer",
            "parfeval",
            "input",
            "eval",
            "feval",
            "addpath",
            "rmpath",
        )
        for call in forbidden_calls:
            with self.subTest(call=call):
                self.assertNotRegex(matlab, rf"\b{call}\s*\(")
        self.assertNotRegex(matlab, r"\brand(?:n|i)?\s*\(")
        self.assertNotRegex(matlab, r"\brng\s*\(")
        self.assertNotRegex(matlab, r"\b(?:load|save)\s*\(")
        self.assertNotRegex(
            matlab, re.compile(r"^\s*(?:while|parfor)\b", re.MULTILINE)
        )
        self.assertNotRegex(
            matlab, re.compile(r"^\s*(?:global|persistent)\b", re.MULTILINE)
        )
        self.assertNotRegex(matlab, r"\bclose\s+all\b")


class P07IndependentOracleTests(unittest.TestCase):
    def test_baseline_is_deterministic_and_physically_interpretable(self) -> None:
        first = _oracle()
        second = _oracle()
        self.assertEqual(first, second)
        self.assertAlmostEqual(first["roll_moment_scale_nm"], 233969.9100475354)
        self.assertAlmostEqual(
            first["initial_roll_rate_deg_s"], 2.6953333637476073, places=13
        )
        self.assertAlmostEqual(
            first["roll_asymptotic_bank_change_deg"],
            1.078133345499043,
            places=13,
        )
        self.assertAlmostEqual(first["roll_time_constant_s"], 0.4, places=14)
        self.assertAlmostEqual(
            first["initial_yaw_rate_deg_s"], 3.1585937856417288, places=13
        )
        self.assertAlmostEqual(
            first["dutch_roll_damped_period_s"], 5.554360939932403, places=13
        )
        self.assertAlmostEqual(
            first["dutch_roll_decay_per_period_ratio"],
            0.316715078721968,
            places=14,
        )
        self.assertAlmostEqual(first["spiral_half_life_s"], 27.72588722239781)
        self.assertTrue(first["is_within_roll_bank_linear_range"])
        self.assertTrue(first["is_within_spiral_bank_linear_range"])
        self.assertTrue(first["is_within_dutch_roll_sideslip_linear_range"])

    def test_closed_form_initial_conditions_and_state_identities(self) -> None:
        result = _oracle()
        self.assertEqual(result["roll_sample_count"], 251)
        self.assertEqual(result["dutch_roll_sample_count"], 501)
        self.assertEqual(result["spiral_sample_count"], 481)
        self.assertEqual(result["roll_time_s"][0], 0.0)
        self.assertEqual(result["roll_time_s"][-1], 5.0)
        self.assertEqual(result["dutch_roll_time_s"][-1], 25.0)
        self.assertEqual(result["spiral_time_s"][-1], 120.0)
        self.assertAlmostEqual(
            result["roll_rate_deg_s"][0],
            result["initial_roll_rate_deg_s"],
            places=14,
        )
        self.assertEqual(result["roll_bank_change_deg"][0], 0.0)
        self.assertAlmostEqual(result["spiral_bank_deg"][0], 5.0, places=14)
        self.assertEqual(result["spiral_heading_change_deg"][0], 0.0)
        self.assertEqual(result["dutch_roll_sideslip_deg"][0], 0.0)
        self.assertAlmostEqual(
            result["dutch_roll_yaw_rate_deg_s"][0],
            result["initial_yaw_rate_deg_s"],
            places=14,
        )

        for p_dot, p in zip(
            result["roll_rate_derivative_rad_s2"], result["roll_rate_deg_s"]
        ):
            self.assertAlmostEqual(
                p_dot + result["roll_decay_rate_per_s"] * math.radians(p),
                0.0,
                places=14,
            )
        for phi_dot, phi in zip(
            result["spiral_roll_rate_deg_s"], result["spiral_bank_deg"]
        ):
            self.assertAlmostEqual(
                phi_dot + result["spiral_decay_rate_per_s"] * phi,
                0.0,
                places=14,
            )
        self.assertLess(
            max(map(abs, result["dutch_roll_beta_kinematic_residual_rad_s"])),
            1e-15,
        )
        self.assertLess(
            max(
                map(
                    abs,
                    result["dutch_roll_yaw_equation_residual_rad_s2"],
                )
            ),
            2e-17,
        )
        self.assertAlmostEqual(
            result["dutch_roll_damped_frequency_rad_s"] ** 2
            + result["dutch_roll_decay_rate_per_s"] ** 2,
            result["dutch_roll_natural_frequency_rad_s"] ** 2,
            places=14,
        )
        self.assertTrue(
            all(
                value <= 0.0
                for value in result["dutch_roll_modal_energy_rate_rad2_s3"]
            )
        )

    def test_roll_bank_and_spiral_heading_obey_kinematics(self) -> None:
        result = _oracle()
        roll_dt = result["roll_time_s"][1] - result["roll_time_s"][0]
        centered_roll_rate = tuple(
            math.radians(
                result["roll_bank_change_deg"][index + 1]
                - result["roll_bank_change_deg"][index - 1]
            )
            / (2.0 * roll_dt)
            for index in range(1, result["roll_sample_count"] - 1)
        )
        self.assertLess(
            max(
                abs(centered - math.radians(exact))
                for centered, exact in zip(
                    centered_roll_rate, result["roll_rate_deg_s"][1:-1]
                )
            ),
            2e-5,
        )

        spiral_dt = result["spiral_time_s"][1] - result["spiral_time_s"][0]
        centered_heading_rate = tuple(
            math.radians(
                result["spiral_heading_change_deg"][index + 1]
                - result["spiral_heading_change_deg"][index - 1]
            )
            / (2.0 * spiral_dt)
            for index in range(1, result["spiral_sample_count"] - 1)
        )
        expected_heading_rate = tuple(
            result["gravity_mps2"]
            / result["reference_true_airspeed_mps"]
            * math.radians(phi)
            for phi in result["spiral_bank_deg"][1:-1]
        )
        self.assertLess(
            max(
                abs(centered - exact)
                for centered, exact in zip(
                    centered_heading_rate, expected_heading_rate
                )
            ),
            1e-7,
        )
        self.assertAlmostEqual(
            result["spiral_bank_deg"][-1], 0.2489353418393197, places=13
        )
        self.assertAlmostEqual(
            result["spiral_heading_change_deg"][-1],
            31.061352153300952,
            places=12,
        )

    def test_dutch_roll_energy_obeys_dissipation_identity(self) -> None:
        result = _oracle()
        energy = result["dutch_roll_modal_energy_rad2_s2"]
        energy_rate = result["dutch_roll_modal_energy_rate_rad2_s3"]
        time_s = result["dutch_roll_time_s"]
        time_step_s = time_s[1] - time_s[0]
        centered_energy_rate = tuple(
            (energy[index + 1] - energy[index - 1]) / (2.0 * time_step_s)
            for index in range(1, len(energy) - 1)
        )

        self.assertGreater(max(map(abs, energy_rate)), 1e-3)
        self.assertTrue(
            all(left >= right for left, right in zip(energy, energy[1:]))
        )
        self.assertLess(
            max(
                abs(centered - exact)
                for centered, exact in zip(
                    centered_energy_rate, energy_rate[1:-1]
                )
            ),
            2e-6,
        )
        self.assertAlmostEqual(
            energy[0],
            0.5 * result["initial_yaw_rate_rad_s"] ** 2,
            places=15,
        )

    def test_three_modes_have_distinct_time_scales(self) -> None:
        result = _oracle()
        self.assertAlmostEqual(
            result["dutch_to_roll_time_scale_ratio"],
            13.885902349831007,
            places=13,
        )
        self.assertAlmostEqual(
            result["spiral_to_dutch_time_scale_ratio"],
            7.201548554834609,
            places=13,
        )
        self.assertLess(result["roll_time_constant_s"], 0.5)
        self.assertGreater(result["dutch_roll_damped_period_s"], 5.0)
        self.assertGreater(result["spiral_time_constant_s"], 35.0)

    def test_zero_inputs_isolate_modes_and_signs_reverse(self) -> None:
        baseline = _oracle()
        no_aileron = _oracle(0.0, 5.0, 3.0, 2.5, 0.025, 0.18)
        no_bank = _oracle(2.0, 0.0, 3.0, 2.5, 0.025, 0.18)
        no_rudder = _oracle(2.0, 5.0, 0.0, 2.5, 0.025, 0.18)
        no_excitation = _oracle(0.0, 0.0, 0.0, 2.5, 0.025, 0.18)
        self.assertEqual(no_aileron["roll_rate_deg_s"], (0.0,) * 251)
        self.assertEqual(no_aileron["roll_bank_change_deg"], (0.0,) * 251)
        self.assertEqual(no_aileron["spiral_bank_deg"], baseline["spiral_bank_deg"])
        self.assertEqual(
            no_aileron["dutch_roll_sideslip_deg"],
            baseline["dutch_roll_sideslip_deg"],
        )
        for key in (
            "spiral_bank_deg",
            "spiral_roll_rate_deg_s",
            "spiral_heading_rate_deg_s",
            "spiral_heading_change_deg",
        ):
            self.assertEqual(no_bank[key], (0.0,) * 481)
        for key in (
            "dutch_roll_sideslip_deg",
            "dutch_roll_yaw_rate_deg_s",
            "dutch_roll_modal_energy_rad2_s2",
        ):
            self.assertEqual(no_rudder[key], (0.0,) * 501)
        self.assertEqual(no_excitation["roll_rate_deg_s"], (0.0,) * 251)
        self.assertEqual(no_excitation["spiral_bank_deg"], (0.0,) * 481)
        self.assertEqual(
            no_excitation["dutch_roll_sideslip_deg"], (0.0,) * 501
        )

        opposite_aileron = _oracle(-2.0, 5.0, 3.0, 2.5, 0.025, 0.18)
        opposite_bank = _oracle(2.0, -5.0, 3.0, 2.5, 0.025, 0.18)
        opposite_rudder = _oracle(2.0, 5.0, -3.0, 2.5, 0.025, 0.18)
        for key in ("roll_rate_deg_s", "roll_bank_change_deg"):
            for positive, negative in zip(baseline[key], opposite_aileron[key]):
                self.assertAlmostEqual(positive, -negative, places=14)
        for key in ("spiral_bank_deg", "spiral_heading_change_deg"):
            for positive, negative in zip(baseline[key], opposite_bank[key]):
                self.assertAlmostEqual(positive, -negative, places=14)
        for key in ("dutch_roll_sideslip_deg", "dutch_roll_yaw_rate_deg_s"):
            for positive, negative in zip(baseline[key], opposite_rudder[key]):
                self.assertAlmostEqual(positive, -negative, places=14)
        self.assertEqual(
            opposite_rudder["dutch_roll_modal_energy_rad2_s2"],
            baseline["dutch_roll_modal_energy_rad2_s2"],
        )

    def test_neutral_spiral_and_undamped_dutch_roll_limits(self) -> None:
        neutral = _oracle(2.0, 5.0, 3.0, 2.5, 0.0, 0.18)
        self.assertTrue(
            all(abs(value - 5.0) < 1e-14 for value in neutral["spiral_bank_deg"])
        )
        self.assertEqual(neutral["spiral_roll_rate_deg_s"], (0.0,) * 481)
        heading_rate = neutral["spiral_heading_rate_deg_s"][0]
        for time_s, heading in zip(
            neutral["spiral_time_s"], neutral["spiral_heading_change_deg"]
        ):
            self.assertAlmostEqual(heading, heading_rate * time_s, places=13)
        self.assertTrue(math.isinf(neutral["spiral_time_constant_s"]))
        self.assertTrue(math.isinf(neutral["spiral_half_life_s"]))
        self.assertFalse(neutral["spiral_time_scale_is_representable"])

        near_neutral = _oracle(2.0, 5.0, 3.0, 2.5, 1e-320, 0.18)
        for key in (
            "spiral_bank_deg",
            "spiral_roll_rate_deg_s",
            "spiral_heading_rate_deg_s",
            "spiral_heading_change_deg",
        ):
            self.assertTrue(all(math.isfinite(value) for value in near_neutral[key]))
        self.assertTrue(math.isinf(near_neutral["spiral_time_constant_s"]))
        self.assertTrue(math.isinf(near_neutral["spiral_half_life_s"]))
        self.assertFalse(near_neutral["spiral_time_scale_is_representable"])
        self.assertAlmostEqual(
            near_neutral["spiral_heading_change_deg"][-1],
            near_neutral["gravity_mps2"]
            / near_neutral["reference_true_airspeed_mps"]
            * near_neutral["bank_release_deg"]
            * near_neutral["spiral_time_s"][-1],
            places=13,
        )

        undamped = _oracle(2.0, 5.0, 3.0, 2.5, 0.025, 0.0)
        self.assertEqual(undamped["dutch_roll_decay_rate_per_s"], 0.0)
        self.assertEqual(undamped["dutch_roll_decay_per_period_ratio"], 1.0)
        energy = undamped["dutch_roll_modal_energy_rad2_s2"]
        self.assertLess(max(energy) - min(energy), 2e-18)
        self.assertEqual(
            undamped["dutch_roll_modal_energy_rate_rad2_s3"], (0.0,) * 501
        )

    def test_two_parameter_sweeps_are_independent(self) -> None:
        baseline = _oracle()
        roll_values = (1.0, 1.5, 2.5, 3.5, 5.0)
        roll_results = [
            _oracle(2.0, 5.0, 3.0, value, 0.025, 0.18)
            for value in roll_values
        ]
        roll_sweep_fixed_keys = (
            "spiral_time_s",
            "spiral_bank_deg",
            "spiral_roll_rate_deg_s",
            "spiral_heading_rate_deg_s",
            "spiral_heading_change_deg",
            "spiral_time_constant_s",
            "spiral_half_life_s",
            "spiral_time_scale_is_representable",
            "spiral_bank_remaining_at_end_ratio",
            "spiral_heading_range_deg",
            "is_within_spiral_bank_linear_range",
            "dutch_roll_time_s",
            "dutch_roll_sideslip_deg",
            "dutch_roll_sideslip_rate_rad_s",
            "dutch_roll_sideslip_acceleration_rad_s2",
            "dutch_roll_yaw_rate_deg_s",
            "dutch_roll_yaw_acceleration_rad_s2",
            "dutch_roll_sideslip_envelope_deg",
            "dutch_roll_beta_kinematic_residual_rad_s",
            "dutch_roll_yaw_equation_residual_rad_s2",
            "dutch_roll_modal_energy_rad2_s2",
            "dutch_roll_modal_energy_rate_rad2_s3",
            "initial_yaw_rate_rad_s",
            "initial_yaw_rate_deg_s",
            "initial_sideslip_rate_rad_s",
            "rudder_moment_nm",
            "dutch_roll_natural_frequency_rad_s",
            "dutch_roll_damped_frequency_rad_s",
            "dutch_roll_decay_rate_per_s",
            "dutch_roll_damped_period_s",
            "dutch_roll_decay_per_period_ratio",
            "dutch_roll_peak_sideslip_deg",
            "dutch_roll_peak_yaw_rate_deg_s",
            "is_within_dutch_roll_sideslip_linear_range",
            "spiral_to_dutch_time_scale_ratio",
        )
        for key in roll_sweep_fixed_keys:
            with self.subTest(sweep="roll", unaffected_output=key):
                self.assertEqual(
                    [result[key] for result in roll_results],
                    [baseline[key]] * len(roll_results),
                )
        for key in (
            "roll_time_constant_s",
            "roll_two_percent_settling_time_s",
            "roll_asymptotic_bank_change_deg",
        ):
            values = [result[key] for result in roll_results]
            self.assertTrue(
                all(left > right for left, right in zip(values, values[1:])),
                key,
            )

        dutch_values = (0.00, 0.08, 0.18, 0.30, 0.45)
        dutch_results = [
            _oracle(2.0, 5.0, 3.0, 2.5, 0.025, value)
            for value in dutch_values
        ]
        dutch_sweep_fixed_keys = (
            "roll_time_s",
            "roll_rate_deg_s",
            "roll_rate_derivative_rad_s2",
            "roll_bank_change_deg",
            "initial_roll_rate_rad_s",
            "initial_roll_rate_deg_s",
            "aileron_moment_nm",
            "roll_asymptotic_bank_change_deg",
            "roll_time_constant_s",
            "roll_half_life_s",
            "roll_two_percent_settling_time_s",
            "roll_peak_rate_deg_s",
            "is_within_roll_bank_linear_range",
            "spiral_time_s",
            "spiral_bank_deg",
            "spiral_roll_rate_deg_s",
            "spiral_heading_rate_deg_s",
            "spiral_heading_change_deg",
            "spiral_time_constant_s",
            "spiral_half_life_s",
            "spiral_time_scale_is_representable",
            "spiral_bank_remaining_at_end_ratio",
            "spiral_heading_range_deg",
            "is_within_spiral_bank_linear_range",
        )
        for key in dutch_sweep_fixed_keys:
            with self.subTest(sweep="Dutch", unaffected_output=key):
                self.assertEqual(
                    [result[key] for result in dutch_results],
                    [baseline[key]] * len(dutch_results),
                )
        decay = [
            result["dutch_roll_decay_per_period_ratio"]
            for result in dutch_results
        ]
        periods = [
            result["dutch_roll_damped_period_s"] for result in dutch_results
        ]
        self.assertEqual(decay[0], 1.0)
        self.assertTrue(all(left > right for left, right in zip(decay, decay[1:])))
        self.assertTrue(
            all(left < right for left, right in zip(periods, periods[1:]))
        )

    def test_broken_spiral_sign_creates_slow_out_of_domain_growth(self) -> None:
        baseline = _oracle()
        broken = tuple(
            baseline["bank_release_deg"]
            * math.exp(baseline["spiral_decay_rate_per_s"] * time_s)
            for time_s in baseline["spiral_time_s"]
        )
        first_invalid = next(
            index
            for index, value in enumerate(broken)
            if abs(value) > baseline["bank_linear_limit_deg"]
        )
        self.assertAlmostEqual(broken[-1], 100.42768461593835, places=12)
        self.assertGreater(broken[-1], 100.0 * abs(baseline["spiral_bank_deg"][-1]))
        self.assertGreater(broken[-1], 5.0 * baseline["bank_linear_limit_deg"])
        self.assertEqual(baseline["spiral_time_s"][first_invalid], 44.0)

    def test_malformed_inputs_fail_without_poisoning_recovery(self) -> None:
        baseline = _oracle()
        malformed_cases = (
            ("aileron below range", (-5.1, 5.0, 3.0, 2.5, 0.025, 0.18)),
            ("aileron above range", (5.1, 5.0, 3.0, 2.5, 0.025, 0.18)),
            ("bank below range", (2.0, -10.1, 3.0, 2.5, 0.025, 0.18)),
            ("bank above range", (2.0, 10.1, 3.0, 2.5, 0.025, 0.18)),
            ("rudder below range", (2.0, 5.0, -5.1, 2.5, 0.025, 0.18)),
            ("rudder above range", (2.0, 5.0, 5.1, 2.5, 0.025, 0.18)),
            ("roll below range", (2.0, 5.0, 3.0, 0.79, 0.025, 0.18)),
            ("roll above range", (2.0, 5.0, 3.0, 5.01, 0.025, 0.18)),
            ("negative spiral", (2.0, 5.0, 3.0, 2.5, -0.001, 0.18)),
            ("spiral above range", (2.0, 5.0, 3.0, 2.5, 0.051, 0.18)),
            ("negative Dutch damping", (2.0, 5.0, 3.0, 2.5, 0.025, -0.01)),
            ("Dutch damping above range", (2.0, 5.0, 3.0, 2.5, 0.025, 0.61)),
            ("nan aileron", (math.nan, 5.0, 3.0, 2.5, 0.025, 0.18)),
            ("infinite bank", (2.0, math.inf, 3.0, 2.5, 0.025, 0.18)),
            ("vector rudder", (2.0, 5.0, [3.0], 2.5, 0.025, 0.18)),
            ("complex roll", (2.0, 5.0, 3.0, 2.5 + 1j, 0.025, 0.18)),
            ("text spiral", (2.0, 5.0, 3.0, 2.5, "stable", 0.18)),
            ("boolean damping", (2.0, 5.0, 3.0, 2.5, 0.025, True)),
        )
        for name, values in malformed_cases:
            with self.subTest(case=name), self.assertRaises(ValueError):
                _oracle(*values)
        self.assertEqual(_oracle(), baseline)

    def test_all_accepted_input_boundaries_remain_bounded(self) -> None:
        case_count = 0
        for aileron in (-5.0, 5.0):
            for bank in (-10.0, 10.0):
                for rudder in (-5.0, 5.0):
                    for roll_decay in (0.8, 5.0):
                        for spiral_decay in (0.0, 0.05):
                            for dutch_zeta in (0.0, 0.6):
                                result = _oracle(
                                    aileron,
                                    bank,
                                    rudder,
                                    roll_decay,
                                    spiral_decay,
                                    dutch_zeta,
                                )
                                case_count += 1
                                self.assertEqual(result["roll_sample_count"], 251)
                                self.assertEqual(
                                    result["dutch_roll_sample_count"], 501
                                )
                                self.assertEqual(result["spiral_sample_count"], 481)
                                self.assertTrue(
                                    result["is_within_roll_bank_linear_range"]
                                )
                                self.assertTrue(
                                    result["is_within_spiral_bank_linear_range"]
                                )
                                self.assertTrue(
                                    result[
                                        "is_within_dutch_roll_sideslip_linear_range"
                                    ]
                                )
                                for key in (
                                    "roll_rate_deg_s",
                                    "roll_bank_change_deg",
                                    "spiral_bank_deg",
                                    "spiral_heading_change_deg",
                                    "dutch_roll_sideslip_deg",
                                    "dutch_roll_yaw_rate_deg_s",
                                    "dutch_roll_modal_energy_rad2_s2",
                                ):
                                    self.assertTrue(
                                        all(math.isfinite(value) for value in result[key]),
                                        (
                                            aileron,
                                            bank,
                                            rudder,
                                            roll_decay,
                                            spiral_decay,
                                            dutch_zeta,
                                            key,
                                        ),
                                    )
                                if spiral_decay == 0.0:
                                    self.assertTrue(
                                        math.isinf(result["spiral_time_constant_s"])
                                    )
                                else:
                                    self.assertTrue(
                                        math.isfinite(result["spiral_time_constant_s"])
                                    )
        self.assertEqual(case_count, 64)

    def test_representative_grid_is_finite_and_resource_bounded(self) -> None:
        ailerons = (-5.0, 0.0, 5.0)
        banks = (-10.0, 0.0, 10.0)
        rudders = (-5.0, 0.0, 5.0)
        roll_decay_rates = (0.8, 2.5, 5.0)
        spiral_decay_rates = (0.0, 0.025, 0.05)
        dutch_damping = (0.0, 0.18, 0.6)
        case_count = 0
        for aileron in ailerons:
            for bank in banks:
                for rudder in rudders:
                    for roll_decay in roll_decay_rates:
                        for spiral_decay in spiral_decay_rates:
                            for dutch_zeta in dutch_damping:
                                result = _oracle(
                                    aileron,
                                    bank,
                                    rudder,
                                    roll_decay,
                                    spiral_decay,
                                    dutch_zeta,
                                )
                                case_count += 1
                                for key in (
                                    "roll_peak_rate_deg_s",
                                    "roll_asymptotic_bank_change_deg",
                                    "dutch_roll_peak_sideslip_deg",
                                    "dutch_roll_peak_yaw_rate_deg_s",
                                    "spiral_heading_range_deg",
                                    "dutch_to_roll_time_scale_ratio",
                                ):
                                    self.assertTrue(
                                        math.isfinite(result[key]),
                                        (
                                            aileron,
                                            bank,
                                            rudder,
                                            roll_decay,
                                            spiral_decay,
                                            dutch_zeta,
                                            key,
                                        ),
                                    )
        self.assertEqual(case_count, 729)
        self.assertLessEqual(case_count, 800)


if __name__ == "__main__":
    unittest.main()
