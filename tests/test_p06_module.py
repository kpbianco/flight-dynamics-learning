from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULE_ID = "P06"
MODULE_FOLDER = ROOT / "modules/06-excite-the-short-period-and-phugoid-modes"
GUIDING_QUESTION = (
    "What inputs, observable effects, and failure modes matter when you excite "
    "the Short-Period and Phugoid Modes?"
)


def _finite_real_scalar(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite real scalar")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite real scalar")
    return result


def _oracle(
    elevator_pulse_deg: object = -2.0,
    airspeed_kick_mps: object = 5.0,
    short_period_damping_ratio: object = 0.35,
    phugoid_damping_ratio: object = 0.08,
) -> dict[str, object]:
    """Independent Python oracle; it does not execute or translate MATLAB."""
    elevator_deg = _finite_real_scalar("elevator pulse", elevator_pulse_deg)
    airspeed_mps = _finite_real_scalar("airspeed kick", airspeed_kick_mps)
    short_zeta = _finite_real_scalar(
        "short-period damping ratio", short_period_damping_ratio
    )
    phugoid_zeta = _finite_real_scalar(
        "phugoid damping ratio", phugoid_damping_ratio
    )
    if abs(elevator_deg) > 5.0:
        raise ValueError("elevator pulse outside the learning range")
    if abs(airspeed_mps) > 10.0:
        raise ValueError("airspeed kick outside the learning range")
    if short_zeta < 0.0:
        raise ValueError("short-period damping ratio must be nonnegative")
    if short_zeta > 0.8:
        raise ValueError("short-period damping ratio outside the learning range")
    if phugoid_zeta < 0.0:
        raise ValueError("phugoid damping ratio must be nonnegative")
    if phugoid_zeta > 0.3:
        raise ValueError("phugoid damping ratio outside the learning range")

    reference_true_airspeed_mps = 60.0
    reference_dynamic_pressure_pa = 1325.00798531847
    wing_area_m2 = 16.2
    mean_aerodynamic_chord_m = 1.5
    reference_static_margin_percent_mac = 20.84683357879235
    pitching_moment_slope_per_rad = -1.1324
    elevator_control_derivative_per_rad = -1.3824
    pitch_inertia_kgm2 = 1800.0
    elevator_pulse_duration_s = 0.18
    gravity_mps2 = 9.80665
    fast_time_s = tuple(index * 0.02 for index in range(401))
    slow_time_s = tuple(index * 0.25 for index in range(481))

    moment_scale_nm = (
        reference_dynamic_pressure_pa * wing_area_m2 * mean_aerodynamic_chord_m
    )
    dimensional_pitch_stiffness_nm_per_rad = (
        moment_scale_nm * pitching_moment_slope_per_rad
    )
    dimensional_elevator_derivative_nm_per_rad = (
        moment_scale_nm * elevator_control_derivative_per_rad
    )
    short_natural = math.sqrt(
        -dimensional_pitch_stiffness_nm_per_rad / pitch_inertia_kgm2
    )
    short_beta = math.sqrt(1.0 - short_zeta**2)
    short_damped = short_natural * short_beta
    short_decay = short_zeta * short_natural
    short_period = 2.0 * math.pi / short_damped
    short_decay_per_period = math.exp(-short_decay * short_period)

    elevator_rad = math.radians(elevator_deg)
    elevator_moment_nm = dimensional_elevator_derivative_nm_per_rad * elevator_rad
    initial_pitch_rate_rad_s = (
        elevator_moment_nm * elevator_pulse_duration_s / pitch_inertia_kgm2
    )
    short_alpha_rad = tuple(
        initial_pitch_rate_rad_s
        / short_damped
        * math.exp(-short_decay * time_s)
        * math.sin(short_damped * time_s)
        for time_s in fast_time_s
    )
    short_pitch_rate_rad_s = tuple(
        initial_pitch_rate_rad_s
        * math.exp(-short_decay * time_s)
        * (
            math.cos(short_damped * time_s)
            - short_decay / short_damped * math.sin(short_damped * time_s)
        )
        for time_s in fast_time_s
    )
    short_alpha_deg = tuple(math.degrees(value) for value in short_alpha_rad)
    short_pitch_rate_deg_s = tuple(
        math.degrees(value) for value in short_pitch_rate_rad_s
    )
    short_alpha_envelope_deg = tuple(
        math.degrees(abs(initial_pitch_rate_rad_s) / short_damped)
        * math.exp(-short_decay * time_s)
        for time_s in fast_time_s
    )

    phugoid_lift_speed_gain = 2.0
    phugoid_natural = (
        math.sqrt(phugoid_lift_speed_gain)
        * gravity_mps2
        / reference_true_airspeed_mps
    )
    phugoid_beta = math.sqrt(1.0 - phugoid_zeta**2)
    phugoid_damped = phugoid_natural * phugoid_beta
    phugoid_decay = phugoid_zeta * phugoid_natural
    phugoid_period = 2.0 * math.pi / phugoid_damped
    phugoid_decay_per_period = math.exp(-phugoid_decay * phugoid_period)
    phugoid_speed_mps = tuple(
        airspeed_mps
        * math.exp(-phugoid_decay * time_s)
        * (
            math.cos(phugoid_damped * time_s)
            - phugoid_decay / phugoid_damped * math.sin(phugoid_damped * time_s)
        )
        for time_s in slow_time_s
    )
    phugoid_speed_rate_mps2 = tuple(
        airspeed_mps
        * math.exp(-phugoid_decay * time_s)
        * (
            -2.0 * phugoid_decay * math.cos(phugoid_damped * time_s)
            + (
                phugoid_decay**2 / phugoid_damped - phugoid_damped
            )
            * math.sin(phugoid_damped * time_s)
        )
        for time_s in slow_time_s
    )
    phugoid_gamma_rad = tuple(
        airspeed_mps
        * phugoid_natural**2
        / (gravity_mps2 * phugoid_damped)
        * math.exp(-phugoid_decay * time_s)
        * math.sin(phugoid_damped * time_s)
        for time_s in slow_time_s
    )
    phugoid_gamma_deg = tuple(math.degrees(value) for value in phugoid_gamma_rad)
    phugoid_altitude_m = tuple(
        reference_true_airspeed_mps
        * airspeed_mps
        / gravity_mps2
        * (
            1.0
            - math.exp(-phugoid_decay * time_s)
            * (
                math.cos(phugoid_damped * time_s)
                + phugoid_decay
                / phugoid_damped
                * math.sin(phugoid_damped * time_s)
            )
        )
        for time_s in slow_time_s
    )
    speed_equation_residual_mps2 = tuple(
        speed_rate
        + 2.0 * phugoid_decay * speed
        + gravity_mps2 * gamma
        for speed_rate, speed, gamma in zip(
            phugoid_speed_rate_mps2, phugoid_speed_mps, phugoid_gamma_rad
        )
    )

    return {
        "elevator_pulse_deg": elevator_deg,
        "airspeed_kick_mps": airspeed_mps,
        "short_period_damping_ratio": short_zeta,
        "phugoid_damping_ratio": phugoid_zeta,
        "fast_time_s": fast_time_s,
        "slow_time_s": slow_time_s,
        "fast_sample_count": len(fast_time_s),
        "slow_sample_count": len(slow_time_s),
        "short_period_alpha_deg": short_alpha_deg,
        "short_period_pitch_rate_deg_s": short_pitch_rate_deg_s,
        "short_period_alpha_envelope_deg": short_alpha_envelope_deg,
        "phugoid_speed_perturbation_mps": phugoid_speed_mps,
        "phugoid_speed_rate_mps2": phugoid_speed_rate_mps2,
        "phugoid_flight_path_angle_deg": phugoid_gamma_deg,
        "phugoid_altitude_from_initial_m": phugoid_altitude_m,
        "phugoid_speed_equation_residual_mps2": (
            speed_equation_residual_mps2
        ),
        "short_period_natural_frequency_rad_s": short_natural,
        "short_period_damped_frequency_rad_s": short_damped,
        "short_period_decay_rate_per_s": short_decay,
        "short_period_damped_period_s": short_period,
        "short_period_decay_per_period_ratio": short_decay_per_period,
        "phugoid_natural_frequency_rad_s": phugoid_natural,
        "phugoid_damped_frequency_rad_s": phugoid_damped,
        "phugoid_decay_rate_per_s": phugoid_decay,
        "phugoid_damped_period_s": phugoid_period,
        "phugoid_decay_per_period_ratio": phugoid_decay_per_period,
        "initial_pitch_rate_rad_s": initial_pitch_rate_rad_s,
        "initial_pitch_rate_deg_s": math.degrees(initial_pitch_rate_rad_s),
        "elevator_moment_nm": elevator_moment_nm,
        "moment_scale_nm": moment_scale_nm,
        "dimensional_pitch_stiffness_nm_per_rad": (
            dimensional_pitch_stiffness_nm_per_rad
        ),
        "dimensional_elevator_derivative_nm_per_rad": (
            dimensional_elevator_derivative_nm_per_rad
        ),
        "short_period_peak_alpha_deg": max(map(abs, short_alpha_deg)),
        "short_period_peak_pitch_rate_deg_s": max(
            map(abs, short_pitch_rate_deg_s)
        ),
        "phugoid_peak_speed_mps": max(map(abs, phugoid_speed_mps)),
        "phugoid_peak_flight_path_angle_deg": max(
            map(abs, phugoid_gamma_deg)
        ),
        "phugoid_altitude_range_m": (
            max(phugoid_altitude_m) - min(phugoid_altitude_m)
        ),
        "mode_period_ratio": phugoid_period / short_period,
        "short_period_linear_alpha_limit_deg": 5.0,
        "is_within_short_period_linear_range": (
            max(map(abs, short_alpha_deg)) <= 5.0
        ),
        "reference_true_airspeed_mps": reference_true_airspeed_mps,
        "reference_dynamic_pressure_pa": reference_dynamic_pressure_pa,
        "wing_area_m2": wing_area_m2,
        "mean_aerodynamic_chord_m": mean_aerodynamic_chord_m,
        "reference_static_margin_percent_mac": (
            reference_static_margin_percent_mac
        ),
        "pitching_moment_slope_per_rad": pitching_moment_slope_per_rad,
        "elevator_control_derivative_per_rad": (
            elevator_control_derivative_per_rad
        ),
        "pitch_inertia_kgm2": pitch_inertia_kgm2,
        "elevator_pulse_duration_s": elevator_pulse_duration_s,
        "gravity_mps2": gravity_mps2,
        "phugoid_lift_speed_gain": phugoid_lift_speed_gain,
    }


class P06ArtifactTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        manifest = json.loads((ROOT / "curriculum/modules.json").read_text(encoding="utf-8"))
        cls.module = next(module for module in manifest["modules"] if module["id"] == MODULE_ID)
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
                "number": 6,
                "id": "P06",
                "title": "Excite the Short-Period and Phugoid Modes",
                "guiding_question": GUIDING_QUESTION,
                "phase": 2,
                "phase_title": "Stability and modes",
                "slug": "excite-the-short-period-and-phugoid-modes",
                "folder": "modules/06-excite-the-short-period-and-phugoid-modes",
                "implementation_batch": "P06",
                "prerequisites": ["P05"],
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
            "p05",
            "restoring stiffness",
            "damping",
            "short-period",
            "phugoid",
            "angle of attack",
            "pitch rate",
            "flight-path angle",
            "airspeed",
            "altitude",
            "mechanism",
            "reset",
            "teach-back",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, combined)
        self.assertIn("read", walkthrough.lower())
        self.assertIn("baseline", walkthrough.lower())
        self.assertRegex(walkthrough.lower(), r"one visual transition|one lever")
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
            "functionout=model(elevatorpulse_deg,airspeedkick_mps,"
            "shortperioddampingratio,phugoiddampingratio)",
            compact,
        )
        self.assertIn("arguments", lower)
        self.assertEqual(model.count("(1,1) double {mustBeReal,mustBeFinite}"), 4)
        for expression in (
            "abs(elevatorpulse_deg)>5",
            "abs(airspeedkick_mps)>10",
            "shortperioddampingratio<0",
            "shortperioddampingratio>0.8",
            "phugoiddampingratio<0",
            "phugoiddampingratio>0.3",
            "fasttime_s=0:0.02:8;",
            "slowtime_s=0:0.25:120;",
        ):
            self.assertIn(expression, compact)
        for identifier in (
            "P06:model:ElevatorPulseRange",
            "P06:model:AirspeedKickRange",
            "P06:model:NegativeShortPeriodDamping",
            "P06:model:ShortPeriodDampingRange",
            "P06:model:NegativePhugoidDamping",
            "P06:model:PhugoidDampingRange",
        ):
            self.assertIn(identifier, model)

        for formula in (
            "momentscale_nm=referencedynamicpressure_pa*wingarea_m2*"
            "meanaerodynamicchord_m;",
            "dimensionalpitchstiffness_nmperrad=momentscale_nm*"
            "pitchingmomentslope_perrad;",
            "shortperiodnaturalfrequency_rad_s=sqrt("
            "-dimensionalpitchstiffness_nmperrad/pitchinertia_kgm2);",
            "initialpitchrate_rad_s=elevatormoment_nm*elevatorpulseduration_s/"
            "pitchinertia_kgm2;",
            "phugoidnaturalfrequency_rad_s=sqrt(phugoidliftspeedgain)*gravity_mps2/"
            "referencetrueairspeed_mps;",
            "phugoidspeedequationresidual_mps2=phugoidspeedrate_mps2+"
            "2*phugoiddecayrate_per_s*phugoidspeedperturbation_mps+"
            "gravity_mps2*phugoidflightpathangle_rad;",
            "phugoidaltitudefrominitial_m=referencetrueairspeed_mps*"
            "airspeedkick_mps/gravity_mps2*(1-phugoidexponential.*("
            "cos(phugoiddampedfrequency_rad_s*slowtime_s)+"
            "(phugoiddecayrate_per_s/phugoiddampedfrequency_rad_s).*"
            "sin(phugoiddampedfrequency_rad_s*slowtime_s)));",
            "phugoidaltituderange_m=max(phugoidaltitudefrominitial_m)-"
            "min(phugoidaltitudefrominitial_m);",
        ):
            self.assertIn(formula, compact)
        self.assertIn("pitchingMomentSlope_perRad=-1.1324", model)
        self.assertIn("elevatorControlDerivative_perRad=-1.3824", model)
        self.assertIn("pitchInertia_kgm2=1800", model)
        self.assertIn("elevatorPulseDuration_s=0.18", model)
        self.assertIn("phugoidLiftSpeedGain=2", model)
        self.assertIn("alpha_dot approximately q", model)
        self.assertIn("positive elevator is trailing-edge down", model)
        self.assertIn("not identified aircraft data or a full longitudinal model", model)

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
        self.assertGreaterEqual(experiment.count("%%"), 14)
        self.assertIn("baseline", lower)
        self.assertGreaterEqual(lower.count("sweep"), 2)
        self.assertRegex(lower, r"sweep[^\n]*short-period|short-period[^\n]*sweep")
        self.assertRegex(lower, r"sweep[^\n]*phugoid|phugoid[^\n]*sweep")
        self.assertIn("broken", lower)
        self.assertIn("reverse the damping sign", lower)
        self.assertIn("exp(+baseline.shortperioddecayrate_per_s*brokentime_s)", compact)
        self.assertGreaterEqual(lower.count("figure("), 5)
        self.assertIn("xlabel", lower)
        self.assertIn("ylabel", lower)
        for unit in ("rad/s", "deg/s", "m/s", "deg", "pa", "% mac", "s"):
            with self.subTest(unit=unit):
                self.assertIn(unit, lower)
        self.assertIn("fprintf", lower)
        self.assertGreaterEqual(lower.count("assert("), 3)
        self.assertIn("model(-2,5,shortperioddampingsweep(k),0.08)", compact)
        self.assertIn("model(-2,5,0.35,phugoiddampingsweep(k))", compact)
        self.assertNotRegex(lower, r"\bclose\s+all\b")
        self.assertIn("findall(groot", lower)
        self.assertIn("'^p06 '", lower)
        self.assertRegex(lower, r"clear run_checks;\s*run_checks;")

        for variable in ("shortPeriodDampingSweep", "phugoidDampingSweep"):
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
            "'p06short-periodandphugoidmodes')",
            interactive_compact,
        )
        self.assertIn("close(existingui)", interactive_compact)
        self.assertEqual(interactive_lower.count("uislider("), 4)
        for control in ("elevator", "airspeed", "short", "phugoid"):
            self.assertIn(control, interactive_lower)
        self.assertIn("valuechangingfcn", interactive_lower)
        self.assertIn("valuechangedfcn", interactive_lower)
        self.assertIn("event.value", interactive_lower)
        self.assertIn("modelfcn=@model;", interactive_compact)
        self.assertIn("out=modelfcn(", interactive_compact)
        self.assertEqual(interactive_compact.count("out=modelfcn("), 1)
        for axis_name in (
            "axshortalpha",
            "axshortrate",
            "axphugoidspeed",
            "axphugoidpath",
        ):
            self.assertIn(f"cla({axis_name})", interactive_compact)
        self.assertNotIn("yyaxis", interactive_lower)
        for unit in ("deg/s", "m/s", "deg", "s"):
            self.assertIn(unit, interactive_lower)

        self.assertGreaterEqual(checks_lower.count("assert("), 35)
        for concept in (
            "determinism",
            "fixed vector shape",
            "p05-to-p06 baseline",
            "pole identities",
            "zero-input isolation",
            "sign symmetry",
            "damping limiting cases",
            "sweep regressions",
            "broken damping sign",
            "elevatorpulserange",
            "airspeedkickrange",
            "shortperioddampingrange",
            "phugoiddampingrange",
            "rejected inputs",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept.replace("-", ""), checks_lower.replace("-", ""))
        self.assertIn("edgecasecount==16", checks_compact)
        self.assertIn("fastsamplecount==401", checks_compact)
        self.assertIn("slowsamplecount==481", checks_compact)
        self.assertIn("centeredaltituderate_mps=", checks_compact)
        self.assertIn("kinematicaltituderate_mps=", checks_compact)
        self.assertIn("h_dot = v0 gamma", checks_lower)
        self.assertIn("catch exception", checks_lower)
        self.assertIn(
            "strcmp(exception.identifier,expectedidentifier)", checks_compact
        )
        self.assertIn("P06 checks passed", checks_script)

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


class P06IndependentOracleTests(unittest.TestCase):
    def test_baseline_is_deterministic_and_physically_interpretable(self) -> None:
        first = _oracle()
        second = _oracle()
        self.assertEqual(first, second)
        self.assertAlmostEqual(first["moment_scale_nm"], 32197.69404323882, places=9)
        self.assertAlmostEqual(
            first["dimensional_pitch_stiffness_nm_per_rad"],
            -36460.66873456364,
            places=8,
        )
        self.assertAlmostEqual(
            first["short_period_natural_frequency_rad_s"],
            4.500658515679408,
            places=14,
        )
        self.assertAlmostEqual(
            first["initial_pitch_rate_deg_s"], 8.902018449074667, places=13
        )
        self.assertAlmostEqual(
            first["short_period_damped_period_s"], 1.4903224491094809, places=14
        )
        self.assertAlmostEqual(
            first["phugoid_damped_period_s"], 27.27028356627409, places=13
        )
        self.assertAlmostEqual(first["mode_period_ratio"], 18.298243834794967, places=13)
        self.assertEqual(first["phugoid_lift_speed_gain"], 2.0)
        self.assertTrue(first["is_within_short_period_linear_range"])
        self.assertGreater(first["initial_pitch_rate_rad_s"], 0.0)

    def test_closed_form_initial_conditions_poles_and_dynamic_identities(self) -> None:
        result = _oracle()
        self.assertEqual(result["fast_sample_count"], 401)
        self.assertEqual(result["slow_sample_count"], 481)
        self.assertEqual(result["fast_time_s"][0], 0.0)
        self.assertEqual(result["fast_time_s"][-1], 8.0)
        self.assertEqual(result["slow_time_s"][0], 0.0)
        self.assertEqual(result["slow_time_s"][-1], 120.0)
        self.assertEqual(result["short_period_alpha_deg"][0], 0.0)
        self.assertAlmostEqual(
            result["short_period_pitch_rate_deg_s"][0],
            result["initial_pitch_rate_deg_s"],
            places=14,
        )
        self.assertEqual(result["phugoid_speed_perturbation_mps"][0], 5.0)
        self.assertAlmostEqual(
            result["phugoid_speed_rate_mps2"][0],
            -2.0
            * result["phugoid_decay_rate_per_s"]
            * result["airspeed_kick_mps"],
            places=14,
        )
        self.assertEqual(result["phugoid_flight_path_angle_deg"][0], 0.0)
        self.assertEqual(result["phugoid_altitude_from_initial_m"][0], 0.0)

        for prefix in ("short_period", "phugoid"):
            natural = result[f"{prefix}_natural_frequency_rad_s"]
            damped = result[f"{prefix}_damped_frequency_rad_s"]
            decay = result[f"{prefix}_decay_rate_per_s"]
            self.assertAlmostEqual(damped**2 + decay**2, natural**2, places=14)
        self.assertLess(
            max(map(abs, result["phugoid_speed_equation_residual_mps2"])),
            1e-12,
        )
        self.assertAlmostEqual(
            result["short_period_peak_alpha_deg"], 1.2562585621505105, places=13
        )
        self.assertAlmostEqual(
            result["phugoid_peak_flight_path_angle_deg"],
            5.990827333876157,
            places=13,
        )

    def test_phugoid_altitude_obeys_kinematics_and_signed_range(self) -> None:
        positive = _oracle()
        negative = _oracle(-2.0, -5.0, 0.35, 0.08)

        for result in (positive, negative):
            time_s = result["slow_time_s"]
            altitude_m = result["phugoid_altitude_from_initial_m"]
            gamma_rad = tuple(
                math.radians(value)
                for value in result["phugoid_flight_path_angle_deg"]
            )
            time_step_s = time_s[1] - time_s[0]
            centered_altitude_rate_mps = tuple(
                (altitude_m[index + 1] - altitude_m[index - 1])
                / (2.0 * time_step_s)
                for index in range(1, len(altitude_m) - 1)
            )
            kinematic_altitude_rate_mps = tuple(
                result["reference_true_airspeed_mps"] * value
                for value in gamma_rad[1:-1]
            )

            self.assertGreater(max(map(abs, kinematic_altitude_rate_mps)), 1.0)
            self.assertLess(
                max(
                    abs(centered - kinematic)
                    for centered, kinematic in zip(
                        centered_altitude_rate_mps,
                        kinematic_altitude_rate_mps,
                    )
                ),
                4e-3,
            )
            self.assertAlmostEqual(
                result["phugoid_altitude_range_m"],
                max(altitude_m) - min(altitude_m),
                places=13,
            )
            self.assertAlmostEqual(
                result["phugoid_altitude_range_m"],
                54.35696957803992,
                places=12,
            )

        self.assertEqual(min(positive["phugoid_altitude_from_initial_m"]), 0.0)
        self.assertGreater(max(positive["phugoid_altitude_from_initial_m"]), 0.0)
        self.assertEqual(max(negative["phugoid_altitude_from_initial_m"]), 0.0)
        self.assertLess(min(negative["phugoid_altitude_from_initial_m"]), 0.0)

    def test_two_modes_have_distinct_time_scales(self) -> None:
        result = _oracle()
        self.assertGreater(
            result["short_period_natural_frequency_rad_s"],
            15.0 * result["phugoid_natural_frequency_rad_s"],
        )
        self.assertGreater(result["mode_period_ratio"], 15.0)
        self.assertLess(result["short_period_damped_period_s"], 2.0)
        self.assertGreater(result["phugoid_damped_period_s"], 25.0)

    def test_zero_inputs_isolate_modes_and_signs_reverse(self) -> None:
        baseline = _oracle()
        no_elevator = _oracle(0.0, 5.0, 0.35, 0.08)
        no_airspeed = _oracle(-2.0, 0.0, 0.35, 0.08)
        no_excitation = _oracle(0.0, 0.0, 0.35, 0.08)
        self.assertEqual(no_elevator["short_period_alpha_deg"], (0.0,) * 401)
        self.assertEqual(no_elevator["short_period_pitch_rate_deg_s"], (0.0,) * 401)
        self.assertEqual(
            no_elevator["phugoid_speed_perturbation_mps"],
            baseline["phugoid_speed_perturbation_mps"],
        )
        for key in (
            "phugoid_speed_perturbation_mps",
            "phugoid_speed_rate_mps2",
            "phugoid_flight_path_angle_deg",
            "phugoid_altitude_from_initial_m",
        ):
            self.assertEqual(no_airspeed[key], (0.0,) * 481)
        self.assertEqual(
            no_airspeed["short_period_alpha_deg"],
            baseline["short_period_alpha_deg"],
        )
        self.assertEqual(no_excitation["short_period_alpha_deg"], (0.0,) * 401)
        self.assertEqual(
            no_excitation["phugoid_speed_perturbation_mps"], (0.0,) * 481
        )

        opposite_elevator = _oracle(2.0, 5.0, 0.35, 0.08)
        opposite_airspeed = _oracle(-2.0, -5.0, 0.35, 0.08)
        for positive, negative in zip(
            baseline["short_period_alpha_deg"],
            opposite_elevator["short_period_alpha_deg"],
        ):
            self.assertAlmostEqual(positive, -negative, places=14)
        for key in (
            "phugoid_speed_perturbation_mps",
            "phugoid_speed_rate_mps2",
            "phugoid_flight_path_angle_deg",
            "phugoid_altitude_from_initial_m",
        ):
            for positive, negative in zip(baseline[key], opposite_airspeed[key]):
                self.assertAlmostEqual(positive, -negative, places=14)

    def test_zero_and_positive_damping_limits(self) -> None:
        undamped = _oracle(-2.0, 5.0, 0.0, 0.0)
        damped = _oracle()
        self.assertEqual(undamped["short_period_decay_rate_per_s"], 0.0)
        self.assertEqual(undamped["phugoid_decay_rate_per_s"], 0.0)
        self.assertEqual(undamped["short_period_decay_per_period_ratio"], 1.0)
        self.assertEqual(undamped["phugoid_decay_per_period_ratio"], 1.0)
        self.assertAlmostEqual(
            max(undamped["short_period_alpha_envelope_deg"]),
            min(undamped["short_period_alpha_envelope_deg"]),
            places=14,
        )
        envelope = damped["short_period_alpha_envelope_deg"]
        self.assertTrue(
            all(left >= right for left, right in zip(envelope, envelope[1:]))
        )
        self.assertLess(envelope[-1], envelope[0])
        self.assertLess(damped["short_period_decay_per_period_ratio"], 1.0)
        self.assertLess(damped["phugoid_decay_per_period_ratio"], 1.0)

    def test_two_damping_sweeps_are_independent(self) -> None:
        baseline = _oracle()
        short_values = (0.10, 0.20, 0.35, 0.50, 0.65)
        short_results = [_oracle(-2.0, 5.0, value, 0.08) for value in short_values]
        self.assertEqual(
            [
                (
                    result["phugoid_speed_perturbation_mps"],
                    result["phugoid_speed_rate_mps2"],
                    result["phugoid_flight_path_angle_deg"],
                    result["phugoid_altitude_from_initial_m"],
                    result["phugoid_damped_period_s"],
                    result["phugoid_decay_per_period_ratio"],
                )
                for result in short_results
            ],
            [
                (
                    baseline["phugoid_speed_perturbation_mps"],
                    baseline["phugoid_speed_rate_mps2"],
                    baseline["phugoid_flight_path_angle_deg"],
                    baseline["phugoid_altitude_from_initial_m"],
                    baseline["phugoid_damped_period_s"],
                    baseline["phugoid_decay_per_period_ratio"],
                )
            ]
            * len(short_results),
        )
        short_decay = [
            result["short_period_decay_per_period_ratio"] for result in short_results
        ]
        short_periods = [result["short_period_damped_period_s"] for result in short_results]
        self.assertTrue(all(left > right for left, right in zip(short_decay, short_decay[1:])))
        self.assertTrue(all(left < right for left, right in zip(short_periods, short_periods[1:])))

        phugoid_values = (0.00, 0.04, 0.08, 0.12, 0.20)
        phugoid_results = [
            _oracle(-2.0, 5.0, 0.35, value) for value in phugoid_values
        ]
        self.assertEqual(
            [
                (
                    result["short_period_alpha_deg"],
                    result["short_period_pitch_rate_deg_s"],
                    result["short_period_alpha_envelope_deg"],
                    result["short_period_damped_period_s"],
                    result["short_period_decay_per_period_ratio"],
                )
                for result in phugoid_results
            ],
            [
                (
                    baseline["short_period_alpha_deg"],
                    baseline["short_period_pitch_rate_deg_s"],
                    baseline["short_period_alpha_envelope_deg"],
                    baseline["short_period_damped_period_s"],
                    baseline["short_period_decay_per_period_ratio"],
                )
            ]
            * len(phugoid_results),
        )
        phugoid_decay = [
            result["phugoid_decay_per_period_ratio"] for result in phugoid_results
        ]
        phugoid_periods = [result["phugoid_damped_period_s"] for result in phugoid_results]
        self.assertEqual(phugoid_decay[0], 1.0)
        self.assertTrue(
            all(left > right for left, right in zip(phugoid_decay, phugoid_decay[1:]))
        )
        self.assertTrue(
            all(left < right for left, right in zip(phugoid_periods, phugoid_periods[1:]))
        )

    def test_broken_damping_sign_creates_growth(self) -> None:
        baseline = _oracle()
        broken_values = []
        correct_values = []
        for time_s, correct in zip(
            baseline["fast_time_s"], baseline["short_period_alpha_deg"]
        ):
            if time_s > 2.5:
                break
            broken_rad = (
                baseline["initial_pitch_rate_rad_s"]
                / baseline["short_period_damped_frequency_rad_s"]
                * math.exp(baseline["short_period_decay_rate_per_s"] * time_s)
                * math.sin(
                    baseline["short_period_damped_frequency_rad_s"] * time_s
                )
            )
            broken_values.append(math.degrees(broken_rad))
            correct_values.append(correct)
        broken_peak = max(map(abs, broken_values))
        correct_peak = max(map(abs, correct_values))
        self.assertAlmostEqual(broken_peak, 97.30810844411067, places=11)
        self.assertGreater(broken_peak, 20.0 * correct_peak)
        self.assertGreater(broken_peak, 50.0)

    def test_malformed_inputs_fail_without_poisoning_recovery(self) -> None:
        baseline = _oracle()
        malformed_cases = (
            ("elevator below range", (-5.1, 5.0, 0.35, 0.08)),
            ("elevator above range", (5.1, 5.0, 0.35, 0.08)),
            ("airspeed below range", (-2.0, -10.1, 0.35, 0.08)),
            ("airspeed above range", (-2.0, 10.1, 0.35, 0.08)),
            ("negative short damping", (-2.0, 5.0, -0.01, 0.08)),
            ("short damping above range", (-2.0, 5.0, 0.81, 0.08)),
            ("negative phugoid damping", (-2.0, 5.0, 0.35, -0.01)),
            ("phugoid damping above range", (-2.0, 5.0, 0.35, 0.31)),
            ("nan elevator", (math.nan, 5.0, 0.35, 0.08)),
            ("infinite airspeed", (-2.0, math.inf, 0.35, 0.08)),
            ("vector short damping", (-2.0, 5.0, [0.35], 0.08)),
            ("complex phugoid damping", (-2.0, 5.0, 0.35, 0.08j)),
            ("text elevator", ("up", 5.0, 0.35, 0.08)),
            ("boolean airspeed", (-2.0, True, 0.35, 0.08)),
        )
        for name, values in malformed_cases:
            with self.subTest(case=name), self.assertRaises(ValueError):
                _oracle(*values)
        self.assertEqual(_oracle(), baseline)

    def test_all_accepted_input_boundaries_remain_finite(self) -> None:
        case_count = 0
        for elevator in (-5.0, 5.0):
            for airspeed in (-10.0, 10.0):
                for short_zeta in (0.0, 0.8):
                    for phugoid_zeta in (0.0, 0.3):
                        result = _oracle(
                            elevator, airspeed, short_zeta, phugoid_zeta
                        )
                        case_count += 1
                        self.assertEqual(result["fast_sample_count"], 401)
                        self.assertEqual(result["slow_sample_count"], 481)
                        self.assertTrue(result["is_within_short_period_linear_range"])
                        self.assertLessEqual(
                            result["short_period_peak_alpha_deg"],
                            result["short_period_linear_alpha_limit_deg"],
                        )
                        for key in (
                            "short_period_alpha_deg",
                            "short_period_pitch_rate_deg_s",
                            "phugoid_speed_perturbation_mps",
                            "phugoid_speed_rate_mps2",
                            "phugoid_flight_path_angle_deg",
                            "phugoid_altitude_from_initial_m",
                        ):
                            self.assertTrue(
                                all(math.isfinite(value) for value in result[key]),
                                (elevator, airspeed, short_zeta, phugoid_zeta, key),
                            )
        self.assertEqual(case_count, 16)

    def test_representative_grid_is_finite_and_resource_bounded(self) -> None:
        elevators = (-5.0, -2.5, 0.0, 2.5, 5.0)
        airspeeds = (-10.0, -5.0, 0.0, 5.0, 10.0)
        short_damping = (0.0, 0.2, 0.4, 0.6, 0.8)
        phugoid_damping = (0.0, 0.075, 0.15, 0.225, 0.3)
        case_count = 0
        for elevator in elevators:
            for airspeed in airspeeds:
                for short_zeta in short_damping:
                    for phugoid_zeta in phugoid_damping:
                        result = _oracle(
                            elevator, airspeed, short_zeta, phugoid_zeta
                        )
                        case_count += 1
                        for key in (
                            "short_period_peak_alpha_deg",
                            "short_period_peak_pitch_rate_deg_s",
                            "phugoid_peak_speed_mps",
                            "phugoid_peak_flight_path_angle_deg",
                            "phugoid_altitude_range_m",
                            "mode_period_ratio",
                        ):
                            self.assertTrue(
                                math.isfinite(result[key]),
                                (elevator, airspeed, short_zeta, phugoid_zeta, key),
                            )
        self.assertEqual(case_count, 625)
        self.assertLessEqual(case_count, 700)


if __name__ == "__main__":
    unittest.main()
