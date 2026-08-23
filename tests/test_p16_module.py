from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_ID = "P16"
MODULE_FOLDER = ROOT / "modules/16-schedule-gains-across-flight-conditions"
EVIDENCE_PATH = ROOT / "docs/evidence/P16-2026-08-23.md"
GUIDING_QUESTION = (
    "What inputs, observable effects, and failure modes matter when you schedule "
    "Gains Across Flight Conditions?"
)
REFERENCE_DENSITY_KGPM3 = 0.736115547399152


def _bounded_scalar(name: str, value: object, lower: float, upper: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite real scalar")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite real scalar")
    if not lower <= result <= upper:
        raise ValueError(f"{name} must be between {lower} and {upper} inclusive")
    return result


def _schedule_mode(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("schedule mode must be -1, 0, or +1")
    result = float(value)
    if not math.isfinite(result) or result not in (-1.0, 0.0, 1.0):
        raise ValueError("schedule mode must be -1, 0, or +1")
    return int(result)


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def _oracle(
    true_airspeed_mps: object = 60.0,
    air_density_kgpm3: object = REFERENCE_DENSITY_KGPM3,
    schedule_mode: object = 1,
) -> dict[str, object]:
    """Independent standard-library implementation of the declared equations."""
    reference_true_airspeed_mps = 60.0
    reference_air_density_kgpm3 = REFERENCE_DENSITY_KGPM3
    minimum_true_airspeed_mps = 45.0
    maximum_true_airspeed_mps = 75.0
    minimum_air_density_kgpm3 = 0.5 * reference_air_density_kgpm3
    maximum_air_density_kgpm3 = 1.5 * reference_air_density_kgpm3
    true_airspeed = _bounded_scalar(
        "true airspeed",
        true_airspeed_mps,
        minimum_true_airspeed_mps,
        maximum_true_airspeed_mps,
    )
    air_density = _bounded_scalar(
        "air density",
        air_density_kgpm3,
        minimum_air_density_kgpm3,
        maximum_air_density_kgpm3,
    )
    mode = _schedule_mode(schedule_mode)

    sample_time_s = 0.01
    time_horizon_s = 8.0
    time_s = tuple(
        index * sample_time_s
        for index in range(round(time_horizon_s / sample_time_s) + 1)
    )
    sample_count = len(time_s)
    command_step_time_s = 0.5
    roll_command_step_deg = 10.0
    roll_command_step_rad = math.radians(roll_command_step_deg)
    aileron_command_limit_deg = 15.0
    aileron_command_limit_rad = math.radians(aileron_command_limit_deg)

    reference_dynamic_pressure_pa = (
        0.5
        * reference_air_density_kgpm3
        * reference_true_airspeed_mps**2
    )
    actual_dynamic_pressure_pa = 0.5 * air_density * true_airspeed**2
    actual_dynamic_pressure_ratio = (
        actual_dynamic_pressure_pa / reference_dynamic_pressure_pa
    )
    reference_control_effectiveness_per_s2 = 12.0
    actual_control_effectiveness_per_s2 = (
        reference_control_effectiveness_per_s2
        * actual_dynamic_pressure_ratio
    )
    target_natural_frequency_radps = 2.4
    target_damping_ratio = 0.8

    dynamic_pressure_ratio_knots = (0.5, 0.75, 1.0, 1.25, 1.5)
    roll_angle_gain_table = tuple(
        target_natural_frequency_radps**2
        / (reference_control_effectiveness_per_s2 * ratio)
        for ratio in dynamic_pressure_ratio_knots
    )
    roll_rate_gain_table_s = tuple(
        2.0 * target_damping_ratio * target_natural_frequency_radps
        / (reference_control_effectiveness_per_s2 * ratio)
        for ratio in dynamic_pressure_ratio_knots
    )

    if mode == 1:
        lookup_dynamic_pressure_ratio_raw = actual_dynamic_pressure_ratio
        schedule_source = "computed dynamic pressure from declared rho and V"
    elif mode == 0:
        lookup_dynamic_pressure_ratio_raw = 1.0
        schedule_source = "fixed reference gains at qbar/qbar_ref=1"
    else:
        lookup_dynamic_pressure_ratio_raw = (
            true_airspeed / reference_true_airspeed_mps
        ) ** 2
        schedule_source = "BROKEN true-airspeed-only ratio; density omitted"

    lookup_dynamic_pressure_ratio = _clamp(
        lookup_dynamic_pressure_ratio_raw,
        dynamic_pressure_ratio_knots[0],
        dynamic_pressure_ratio_knots[-1],
    )
    lookup_clamped = (
        lookup_dynamic_pressure_ratio != lookup_dynamic_pressure_ratio_raw
    )
    upper_knot_index_zero = next(
        index
        for index, knot in enumerate(dynamic_pressure_ratio_knots)
        if knot >= lookup_dynamic_pressure_ratio
    )
    if upper_knot_index_zero == 0:
        lower_knot_index_zero = 0
        interpolation_weight = 0.0
    else:
        lower_knot_index_zero = upper_knot_index_zero - 1
        interpolation_weight = (
            lookup_dynamic_pressure_ratio
            - dynamic_pressure_ratio_knots[lower_knot_index_zero]
        ) / (
            dynamic_pressure_ratio_knots[upper_knot_index_zero]
            - dynamic_pressure_ratio_knots[lower_knot_index_zero]
        )

    roll_angle_gain = (
        (1.0 - interpolation_weight)
        * roll_angle_gain_table[lower_knot_index_zero]
        + interpolation_weight * roll_angle_gain_table[upper_knot_index_zero]
    )
    roll_rate_gain_s = (
        (1.0 - interpolation_weight)
        * roll_rate_gain_table_s[lower_knot_index_zero]
        + interpolation_weight
        * roll_rate_gain_table_s[upper_knot_index_zero]
    )
    ideal_roll_angle_gain = target_natural_frequency_radps**2 / (
        reference_control_effectiveness_per_s2
        * lookup_dynamic_pressure_ratio
    )
    ideal_roll_rate_gain_s = (
        2.0 * target_damping_ratio * target_natural_frequency_radps
        / (
            reference_control_effectiveness_per_s2
            * lookup_dynamic_pressure_ratio
        )
    )
    roll_angle_gain_interpolation_error_fraction = (
        roll_angle_gain - ideal_roll_angle_gain
    ) / ideal_roll_angle_gain
    roll_rate_gain_interpolation_error_fraction = (
        roll_rate_gain_s - ideal_roll_rate_gain_s
    ) / ideal_roll_rate_gain_s
    actual_condition_ideal_roll_angle_gain = (
        target_natural_frequency_radps**2
        / (
            reference_control_effectiveness_per_s2
            * actual_dynamic_pressure_ratio
        )
    )
    actual_condition_ideal_roll_rate_gain_s = (
        2.0 * target_damping_ratio * target_natural_frequency_radps
        / (
            reference_control_effectiveness_per_s2
            * actual_dynamic_pressure_ratio
        )
    )
    roll_angle_gain_actual_condition_mismatch_fraction = (
        roll_angle_gain - actual_condition_ideal_roll_angle_gain
    ) / actual_condition_ideal_roll_angle_gain
    roll_rate_gain_actual_condition_mismatch_fraction = (
        roll_rate_gain_s - actual_condition_ideal_roll_rate_gain_s
    ) / actual_condition_ideal_roll_rate_gain_s

    roll_command_rad = tuple(
        0.0 if time < command_step_time_s else roll_command_step_rad
        for time in time_s
    )
    roll_angle_rad = [0.0] * sample_count
    roll_rate_radps = [0.0] * sample_count
    roll_error_rad = [0.0] * sample_count
    aileron_command_unclamped_rad = [0.0] * sample_count
    aileron_command_rad = [0.0] * sample_count
    roll_acceleration_radps2 = [0.0] * sample_count

    for index in range(sample_count):
        roll_error_rad[index] = (
            roll_command_rad[index] - roll_angle_rad[index]
        )
        aileron_command_unclamped_rad[index] = (
            roll_angle_gain * roll_error_rad[index]
            - roll_rate_gain_s * roll_rate_radps[index]
        )
        aileron_command_rad[index] = _clamp(
            aileron_command_unclamped_rad[index],
            -aileron_command_limit_rad,
            aileron_command_limit_rad,
        )
        roll_acceleration_radps2[index] = (
            actual_control_effectiveness_per_s2
            * aileron_command_rad[index]
        )
        if index < sample_count - 1:
            roll_angle_rad[index + 1] = (
                roll_angle_rad[index]
                + sample_time_s * roll_rate_radps[index]
            )
            roll_rate_radps[index + 1] = (
                roll_rate_radps[index]
                + sample_time_s * roll_acceleration_radps2[index]
            )

    aileron_command_saturated = tuple(
        abs(unclamped) > aileron_command_limit_rad
        for unclamped in aileron_command_unclamped_rad
    )
    active_indices = tuple(
        index
        for index, time in enumerate(time_s)
        if time >= command_step_time_s
    )
    roll_command_deg = tuple(math.degrees(value) for value in roll_command_rad)
    roll_angle_deg = tuple(math.degrees(value) for value in roll_angle_rad)
    roll_rate_degps = tuple(math.degrees(value) for value in roll_rate_radps)
    roll_error_deg = tuple(math.degrees(value) for value in roll_error_rad)
    aileron_command_unclamped_deg = tuple(
        math.degrees(value) for value in aileron_command_unclamped_rad
    )
    aileron_command_deg = tuple(
        math.degrees(value) for value in aileron_command_rad
    )
    roll_acceleration_degps2 = tuple(
        math.degrees(value) for value in roll_acceleration_radps2
    )
    roll_tracking_rms_deg = math.sqrt(
        sum(roll_error_deg[index] ** 2 for index in active_indices)
        / len(active_indices)
    )
    final_roll_error_deg = roll_error_deg[-1]
    peak_roll_overshoot_deg = max(
        max(roll_angle_deg[index] for index in active_indices)
        - roll_command_step_deg,
        0.0,
    )
    capture_tolerance_deg = 0.1 * roll_command_step_deg
    capture_index = next(
        (
            index
            for index in active_indices
            if abs(roll_error_deg[index]) <= capture_tolerance_deg
        ),
        None,
    )
    reached_ninety_percent = capture_index is not None
    time_to_ninety_percent_s = (
        time_s[capture_index] - command_step_time_s
        if capture_index is not None
        else time_horizon_s - command_step_time_s
    )
    settling_tolerance_deg = 0.02 * roll_command_step_deg
    settled_by_end = abs(final_roll_error_deg) <= settling_tolerance_deg
    outside_tolerance = tuple(
        index
        for index in active_indices
        if abs(roll_error_deg[index]) > settling_tolerance_deg
    )
    if (
        settled_by_end
        and outside_tolerance
        and outside_tolerance[-1] < sample_count - 1
    ):
        settling_time_s = (
            time_s[outside_tolerance[-1] + 1] - command_step_time_s
        )
    elif settled_by_end and not outside_tolerance:
        settling_time_s = 0.0
    else:
        settling_time_s = time_horizon_s - command_step_time_s

    effective_natural_frequency_radps = math.sqrt(
        actual_control_effectiveness_per_s2 * roll_angle_gain
    )
    effective_damping_ratio = (
        actual_control_effectiveness_per_s2 * roll_rate_gain_s
        / (2.0 * effective_natural_frequency_radps)
    )

    return {
        "true_airspeed_mps": true_airspeed,
        "air_density_kgpm3": air_density,
        "schedule_mode": mode,
        "sample_time_s": sample_time_s,
        "time_horizon_s": time_horizon_s,
        "time_s": time_s,
        "sample_count": sample_count,
        "interval_count": sample_count - 1,
        "active_sample_count": len(active_indices),
        "command_step_time_s": command_step_time_s,
        "roll_command_step_deg": roll_command_step_deg,
        "aileron_command_limit_deg": aileron_command_limit_deg,
        "reference_true_airspeed_mps": reference_true_airspeed_mps,
        "reference_air_density_kgpm3": reference_air_density_kgpm3,
        "minimum_true_airspeed_mps": minimum_true_airspeed_mps,
        "maximum_true_airspeed_mps": maximum_true_airspeed_mps,
        "minimum_air_density_kgpm3": minimum_air_density_kgpm3,
        "maximum_air_density_kgpm3": maximum_air_density_kgpm3,
        "reference_dynamic_pressure_pa": reference_dynamic_pressure_pa,
        "actual_dynamic_pressure_pa": actual_dynamic_pressure_pa,
        "actual_dynamic_pressure_ratio": actual_dynamic_pressure_ratio,
        "reference_control_effectiveness_per_s2": (
            reference_control_effectiveness_per_s2
        ),
        "actual_control_effectiveness_per_s2": (
            actual_control_effectiveness_per_s2
        ),
        "target_natural_frequency_radps": target_natural_frequency_radps,
        "target_damping_ratio": target_damping_ratio,
        "dynamic_pressure_ratio_knots": dynamic_pressure_ratio_knots,
        "roll_angle_gain_table": roll_angle_gain_table,
        "roll_rate_gain_table_s": roll_rate_gain_table_s,
        "lookup_dynamic_pressure_ratio_raw": (
            lookup_dynamic_pressure_ratio_raw
        ),
        "lookup_dynamic_pressure_ratio": lookup_dynamic_pressure_ratio,
        "lookup_clamped": lookup_clamped,
        "lower_knot_index": lower_knot_index_zero + 1,
        "upper_knot_index": upper_knot_index_zero + 1,
        "interpolation_weight": interpolation_weight,
        "schedule_source": schedule_source,
        "roll_angle_gain": roll_angle_gain,
        "roll_rate_gain_s": roll_rate_gain_s,
        "ideal_roll_angle_gain": ideal_roll_angle_gain,
        "ideal_roll_rate_gain_s": ideal_roll_rate_gain_s,
        "roll_angle_gain_interpolation_error_fraction": (
            roll_angle_gain_interpolation_error_fraction
        ),
        "roll_rate_gain_interpolation_error_fraction": (
            roll_rate_gain_interpolation_error_fraction
        ),
        "actual_condition_ideal_roll_angle_gain": (
            actual_condition_ideal_roll_angle_gain
        ),
        "actual_condition_ideal_roll_rate_gain_s": (
            actual_condition_ideal_roll_rate_gain_s
        ),
        "roll_angle_gain_actual_condition_mismatch_fraction": (
            roll_angle_gain_actual_condition_mismatch_fraction
        ),
        "roll_rate_gain_actual_condition_mismatch_fraction": (
            roll_rate_gain_actual_condition_mismatch_fraction
        ),
        "roll_command_deg": roll_command_deg,
        "roll_angle_deg": roll_angle_deg,
        "roll_rate_degps": roll_rate_degps,
        "roll_error_deg": roll_error_deg,
        "aileron_command_unclamped_deg": aileron_command_unclamped_deg,
        "aileron_command_deg": aileron_command_deg,
        "roll_acceleration_degps2": roll_acceleration_degps2,
        "aileron_command_saturated": aileron_command_saturated,
        "aileron_command_saturation_fraction": (
            sum(aileron_command_saturated) / sample_count
        ),
        "roll_tracking_rms_deg": roll_tracking_rms_deg,
        "final_roll_error_deg": final_roll_error_deg,
        "peak_roll_overshoot_deg": peak_roll_overshoot_deg,
        "capture_tolerance_deg": capture_tolerance_deg,
        "reached_ninety_percent": reached_ninety_percent,
        "time_to_ninety_percent_s": time_to_ninety_percent_s,
        "settling_tolerance_deg": settling_tolerance_deg,
        "settled_by_end": settled_by_end,
        "settling_time_s": settling_time_s,
        "peak_absolute_roll_rate_degps": max(map(abs, roll_rate_degps)),
        "peak_absolute_aileron_command_deg": max(
            map(abs, aileron_command_deg)
        ),
        "effective_natural_frequency_radps": (
            effective_natural_frequency_radps
        ),
        "effective_damping_ratio": effective_damping_ratio,
    }


class P16ArtifactTests(unittest.TestCase):
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

    def test_permanent_manifest_identity_and_exact_artifact_set(self) -> None:
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
                "number": 16,
                "id": "P16",
                "title": "Schedule Gains Across Flight Conditions",
                "guiding_question": GUIDING_QUESTION,
                "phase": 4,
                "phase_title": "Autopilots",
                "slug": "schedule-gains-across-flight-conditions",
                "folder": "modules/16-schedule-gains-across-flight-conditions",
                "implementation_batch": "P16",
                "prerequisites": ["P15"],
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
        self.assertEqual(set(self.text), required)
        for name in required:
            with self.subTest(file=name):
                self.assertTrue(self.text[name].strip(), name)
                self.assertTrue(self.text[name].endswith("\n"), name)
                self.assertFalse(self.text[name].endswith("\n\n"), name)

    def test_learning_slice_is_complete_concept_first_and_bounded(self) -> None:
        readme = self.text["README.md"]
        lesson_script = self.text["lesson.m"]
        experiment = self.text["experiment.m"]
        lesson = self.text["lesson.md"]
        walkthrough = self.text["walkthrough.md"]
        checks = self.text["checks.md"]
        combined = "\n".join(
            (readme, lesson_script, experiment, lesson, walkthrough, checks)
        ).lower()
        for name, source in {
            "README.md": readme,
            "lesson.m": lesson_script,
            "experiment.m": experiment,
            "lesson.md": lesson,
            "walkthrough.md": walkthrough,
            "checks.md": checks,
        }.items():
            with self.subTest(file=name):
                self.assertIn(GUIDING_QUESTION, source)
        for concept in (
            "p15",
            "dynamic pressure",
            "true airspeed",
            "density",
            "gain table",
            "interpolation",
            "scheduled",
            "fixed gains",
            "equal-dynamic-pressure",
            "omits density",
            "mechanism",
            "reset",
            "broken",
            "clamp",
            "frozen-condition",
            "teach-back",
            "conceptual",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, combined)
        self.assertRegex(walkthrough.lower(), r"one plot|one .* at a time")
        self.assertIn("does not consume p15", combined)
        self.assertIn("does not prove stability", combined)
        self.assertEqual(experiment.lower().count("predict once:"), 1)
        self.assertLess(
            experiment.lower().index("predict once:"),
            experiment.lower().index("baseline=model("),
        )
        self.assertLess(
            lesson_script.index("experiment;"), lesson_script.index("interactive;")
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
            "functionout=model(trueairspeed_mps,airdensity_kgpm3,schedulemode)",
            compact,
        )
        for expression in (
            "trueairspeed_mps(1,1)double{mustbereal,mustbefinite}=60",
            "airdensity_kgpm3(1,1)double{mustbereal,mustbefinite}="
            "0.736115547399152",
            "schedulemode(1,1)double{mustbereal,mustbefinite}=1",
            "referencetrueairspeed_mps=60;",
            "referenceairdensity_kgpm3=0.736115547399152;",
            "minimumtrueairspeed_mps=45;",
            "maximumtrueairspeed_mps=75;",
            "minimumairdensity_kgpm3=0.5*referenceairdensity_kgpm3;",
            "maximumairdensity_kgpm3=1.5*referenceairdensity_kgpm3;",
            "trueairspeed_mps<minimumtrueairspeed_mps||"
            "trueairspeed_mps>maximumtrueairspeed_mps",
            "airdensity_kgpm3<minimumairdensity_kgpm3||"
            "airdensity_kgpm3>maximumairdensity_kgpm3",
            "schedulemode~=1&&schedulemode~=0&&schedulemode~=-1",
            "sampletime_s=0.01;",
            "timehorizon_s=8;",
            "time_s=0:sampletime_s:timehorizon_s;",
            "commandsteptime_s=0.5;",
            "rollcommandstep_deg=10;",
            "aileroncommandlimit_deg=15;",
            "referencedynamicpressure_pa=0.5*referenceairdensity_kgpm3*"
            "referencetrueairspeed_mps^2;",
            "actualdynamicpressure_pa=0.5*airdensity_kgpm3*"
            "trueairspeed_mps^2;",
            "actualdynamicpressureratio=actualdynamicpressure_pa/"
            "referencedynamicpressure_pa;",
            "referencecontroleffectiveness_per_s2=12;",
            "actualcontroleffectiveness_per_s2="
            "referencecontroleffectiveness_per_s2*actualdynamicpressureratio;",
            "targetnaturalfrequency_radps=2.4;",
            "targetdampingratio=0.8;",
            "dynamicpressureratioknots=[0.50.7511.251.5];",
            "rollanglegaintable=targetnaturalfrequency_radps^2./("
            "referencecontroleffectiveness_per_s2*dynamicpressureratioknots);",
            "rollrategaintable_s=2*targetdampingratio*"
            "targetnaturalfrequency_radps./(referencecontroleffectiveness_per_s2*"
            "dynamicpressureratioknots);",
            "lookupdynamicpressureratioraw=actualdynamicpressureratio;",
            "lookupdynamicpressureratioraw=1;",
            "lookupdynamicpressureratioraw=(trueairspeed_mps/"
            "referencetrueairspeed_mps)^2;",
            "lookupdynamicpressureratio=min(max("
            "lookupdynamicpressureratioraw,dynamicpressureratioknots(1)),"
            "dynamicpressureratioknots(end));",
            "lookupclamped=lookupdynamicpressureratio~="
            "lookupdynamicpressureratioraw;",
            "upperknotindex=find(dynamicpressureratioknots>="
            "lookupdynamicpressureratio,1,'first');",
            "interpolationweight=(lookupdynamicpressureratio-"
            "dynamicpressureratioknots(lowerknotindex))/("
            "dynamicpressureratioknots(upperknotindex)-"
            "dynamicpressureratioknots(lowerknotindex));",
            "rollanglegain=(1-interpolationweight)*"
            "rollanglegaintable(lowerknotindex)+interpolationweight*"
            "rollanglegaintable(upperknotindex);",
            "rollrategain_s=(1-interpolationweight)*"
            "rollrategaintable_s(lowerknotindex)+interpolationweight*"
            "rollrategaintable_s(upperknotindex);",
            "idealrollanglegain=targetnaturalfrequency_radps^2/("
            "referencecontroleffectiveness_per_s2*"
            "lookupdynamicpressureratio);",
            "idealrollrategain_s=2*targetdampingratio*"
            "targetnaturalfrequency_radps/("
            "referencecontroleffectiveness_per_s2*"
            "lookupdynamicpressureratio);",
            "rollanglegaininterpolationerrorfraction=(rollanglegain-"
            "idealrollanglegain)/idealrollanglegain;",
            "rollrategaininterpolationerrorfraction=(rollrategain_s-"
            "idealrollrategain_s)/idealrollrategain_s;",
            "actualconditionidealrollanglegain="
            "targetnaturalfrequency_radps^2/("
            "referencecontroleffectiveness_per_s2*"
            "actualdynamicpressureratio);",
            "actualconditionidealrollrategain_s=2*targetdampingratio*"
            "targetnaturalfrequency_radps/("
            "referencecontroleffectiveness_per_s2*"
            "actualdynamicpressureratio);",
            "rollanglegainactualconditionmismatchfraction=(rollanglegain-"
            "actualconditionidealrollanglegain)/"
            "actualconditionidealrollanglegain;",
            "rollrategainactualconditionmismatchfraction=(rollrategain_s-"
            "actualconditionidealrollrategain_s)/"
            "actualconditionidealrollrategain_s;",
            "rollcommand_rad=rollcommandstep_rad*double("
            "time_s>=commandsteptime_s);",
            "rollerror_rad(k)=rollcommand_rad(k)-rollangle_rad(k);",
            "aileroncommandunclamped_rad(k)=rollanglegain*"
            "rollerror_rad(k)-rollrategain_s*rollrate_radps(k);",
            "aileroncommand_rad(k)=min(max(aileroncommandunclamped_rad(k),"
            "-aileroncommandlimit_rad),aileroncommandlimit_rad);",
            "rollacceleration_radps2(k)=actualcontroleffectiveness_per_s2*"
            "aileroncommand_rad(k);",
            "rollangle_rad(k+1)=rollangle_rad(k)+sampletime_s*"
            "rollrate_radps(k);",
            "rollrate_radps(k+1)=rollrate_radps(k)+sampletime_s*"
            "rollacceleration_radps2(k);",
            "aileroncommandsaturated=abs(aileroncommandunclamped_rad)>"
            "aileroncommandlimit_rad;",
            "activemask=time_s>=commandsteptime_s;",
            "activesamplecount=sum(activemask);",
            "rolltrackingrms_deg=sqrt(mean(rollerror_deg(activemask).^2));",
            "finalrollerror_deg=rollerror_deg(end);",
            "peakrollovershoot_deg=max(max(rad2deg("
            "rollangle_rad(activemask)))-rollcommandstep_deg,0);",
            "capturetolerance_deg=0.1*rollcommandstep_deg;",
            "captureindex=find(activemask&abs(rollerror_deg)<="
            "capturetolerance_deg,1,'first');",
            "timetoninetypercent_s=time_s(captureindex)-"
            "commandsteptime_s;",
            "settlingtolerance_deg=0.02*rollcommandstep_deg;",
            "settledbyend=abs(finalrollerror_deg)<="
            "settlingtolerance_deg;",
            "outsidetolerance=find(activemask&abs(rollerror_deg)>"
            "settlingtolerance_deg,1,'last');",
            "settlingtime_s=time_s(outsidetolerance+1)-"
            "commandsteptime_s;",
            "'peakabsoluteaileroncommand_deg',max(abs(rad2deg("
            "aileroncommand_rad)))",
            "effectivenaturalfrequency_radps=sqrt("
            "actualcontroleffectiveness_per_s2*rollanglegain);",
            "effectivedampingratio=actualcontroleffectiveness_per_s2*"
            "rollrategain_s/(2*effectivenaturalfrequency_radps);",
        ):
            with self.subTest(expression=expression):
                self.assertIn(expression, compact)
        for identifier in (
            "P16:model:TrueAirspeedRange",
            "P16:model:AirDensityRange",
            "P16:model:ScheduleMode",
        ):
            self.assertIn(identifier, model)
        for field in (
            "trueAirspeed_mps",
            "airDensity_kgpm3",
            "scheduleMode",
            "referenceDynamicPressure_Pa",
            "actualDynamicPressure_Pa",
            "actualDynamicPressureRatio",
            "actualControlEffectiveness_per_s2",
            "dynamicPressureRatioKnots",
            "rollAngleGainTable",
            "rollRateGainTable_s",
            "lookupDynamicPressureRatioRaw",
            "lookupDynamicPressureRatio",
            "lookupClamped",
            "lowerKnotIndex",
            "upperKnotIndex",
            "interpolationWeight",
            "scheduleSource",
            "rollAngleGain",
            "rollRateGain_s",
            "rollAngleGainInterpolationErrorFraction",
            "rollRateGainInterpolationErrorFraction",
            "actualConditionIdealRollAngleGain",
            "actualConditionIdealRollRateGain_s",
            "rollAngleGainActualConditionMismatchFraction",
            "rollRateGainActualConditionMismatchFraction",
            "rollCommand_deg",
            "rollAngle_deg",
            "rollRate_degps",
            "rollError_deg",
            "aileronCommandUnclamped_deg",
            "aileronCommand_deg",
            "rollAcceleration_degps2",
            "rollTrackingRMS_deg",
            "peakAbsoluteAileronCommand_deg",
            "effectiveNaturalFrequency_radps",
            "effectiveDampingRatio",
            "scheduleEquation",
            "controllerEquation",
            "plantEquation",
            "brokenCaseDefinition",
            "analysisScope",
        ):
            with self.subTest(field=field):
                self.assertIn(f"'{field}'", model)
        self.assertNotIn("interp1", lower)
        for call in (
            "figure",
            "uifigure",
            "plot",
            "subplot",
            "xlabel",
            "ylabel",
            "title",
            "legend",
            "disp",
            "fprintf",
        ):
            self.assertNotRegex(lower, rf"\b{call}\s*\(")

    def test_experiment_has_two_isolated_sweeps_and_equal_q_broken_case(
        self,
    ) -> None:
        experiment = self.text["experiment.m"]
        compact = re.sub(r"\s+", "", experiment.replace("...", "")).lower()
        for expression in (
            "airspeedsweep_mps=[4552.56067.572];",
            "densityratiosweep=[0.50.7511.251.5];",
            "scheduled=model(airspeedsweep_mps(k),"
            "baseline.referenceairdensity_kgpm3,1);",
            "fixed=model(airspeedsweep_mps(k),"
            "baseline.referenceairdensity_kgpm3,0);",
            "densitysweep_kgpm3=baseline.referenceairdensity_kgpm3*"
            "densityratiosweep;",
            "scheduled=model(baseline.referencetrueairspeed_mps,"
            "densitysweep_kgpm3(k),1);",
            "equaldynamicpressureairspeed_mps=75;",
            "equaldynamicpressuredensity_kgpm3="
            "baseline.referenceairdensity_kgpm3*("
            "baseline.referencetrueairspeed_mps/"
            "equaldynamicpressureairspeed_mps)^2;",
            "equalqcorrect=model(equaldynamicpressureairspeed_mps,"
            "equaldynamicpressuredensity_kgpm3,1);",
            "broken=model(equaldynamicpressureairspeed_mps,"
            "equaldynamicpressuredensity_kgpm3,-1);",
        ):
            self.assertIn(expression, compact)
        self.assertGreaterEqual(experiment.lower().count("mechanism"), 2)
        self.assertGreaterEqual(experiment.lower().count("reset"), 1)
        figures = re.findall(r"figure\('Name','(P16 [^']+)'\)", experiment)
        self.assertEqual(len(figures), 5)
        self.assertEqual(len(figures), len(set(figures)))
        for unit_label in (
            "Time (s)",
            "Roll angle (deg)",
            "Roll error (deg)",
            "Equivalent aileron command (deg)",
            "True airspeed (m/s)",
            "Density ratio rho/rho_{ref}",
            "Frequency (rad/s) or damping ratio",
        ):
            self.assertIn(unit_label, experiment)
        self.assertNotIn("close all", experiment.lower())
        self.assertIn("clear run_checks;", experiment)
        self.assertIn("run_checks;", experiment)

    def test_interaction_checks_recovery_and_resource_bounds(self) -> None:
        interactive = self.text["interactive.m"]
        checks = self.text["run_checks.m"]
        checks_compact = re.sub(
            r"\s+", "", checks.replace("...", "")
        ).lower()
        tutor_checks = self.text["checks.md"].lower()
        for token in (
            "uifigure",
            "uigridlayout",
            "uislider",
            "uidropdown",
            "uibutton",
            "ValueChangingFcn",
            "ValueChangedFcn",
            "resetBaseline",
            "modelFcn=@model",
            "scheduleMode=-1",
            "cla(",
        ):
            self.assertIn(token, interactive)
        self.assertGreaterEqual(interactive.count("uiaxes("), 4)
        self.assertNotIn("close all", interactive.lower())
        self.assertIn("findall(groot,'Type','figure','Name',uiName)", interactive)
        for exact_reset in (
            "speedControl.Value=60",
            "densityControl.Value=referenceDensity",
            "modeControl.Value='Dynamic-pressure schedule'",
        ):
            self.assertIn(exact_reset, interactive)
        for token in (
            "repeat=modelFcn(60,referenceDensity,1)",
            "expectedReferenceQ_Pa",
            "expectedActualQ_Pa",
            "expectedAngleTable",
            "expectedRateTable",
            "independentLookup",
            "expectedError_rad",
            "expectedControl_rad",
            "expectedNextAngle",
            "expectedNextRate",
            "airspeedSweep_mps",
            "densityRatioSweep",
            "equalQ",
            "lowCorner",
            "highCorner",
            "lookupClamped",
            "rollback",
            "invalidCalls",
            "afterRejection",
            "acceptedCaseCount==12",
            "representativeCaseCount==27",
            "timeout",
            "cancellation",
            "migration",
            "backup/restore",
        ):
            self.assertIn(token, checks)
        for expression in (
            "abs(scheduledpeakaileron_deg(k)-max(abs("
            "scheduled.aileroncommand_deg)))<absolutetolerance",
            "abs(peakaileronbydensity_deg(k)-max(abs("
            "sample.aileroncommand_deg)))<absolutetolerance",
        ):
            self.assertIn(expression, checks_compact)
        self.assertIn("interpretation questions", tutor_checks)
        self.assertIn("teach-back", tutor_checks)
        self.assertIn("two sentences", tutor_checks)

    def test_no_opaque_toolbox_random_external_or_async_behavior(self) -> None:
        matlab = "\n".join(
            self.text[name].lower()
            for name in (
                "model.m",
                "experiment.m",
                "interactive.m",
                "lesson.m",
                "run_checks.m",
            )
        )
        for forbidden_call in (
            "pid",
            "pidtune",
            "tf",
            "ss",
            "feedback",
            "lsim",
            "step",
            "c2d",
            "place",
            "acker",
            "lqr",
            "interp1",
            "fsolve",
            "lsqnonlin",
            "fmincon",
            "solve",
            "vpasolve",
            "trim",
            "findop",
            "linearize",
            "ode45",
            "ode23",
            "ode15s",
            "sim",
            "load_system",
            "open_system",
            "rng",
            "rand",
            "randn",
            "fopen",
            "fread",
            "fwrite",
            "save",
            "load",
            "readtable",
            "writetable",
            "webread",
            "urlread",
            "webwrite",
            "urlwrite",
            "addpath",
            "rmpath",
            "tcpclient",
            "udpport",
            "serialport",
            "system",
            "unix",
            "dos",
            "timer",
            "parfeval",
            "parpool",
            "backgroundpool",
            "batch",
            "pause",
            "waitfor",
            "input",
            "eval",
            "feval",
            "evalin",
            "assignin",
        ):
            self.assertNotRegex(matlab, rf"\b{re.escape(forbidden_call)}\s*\(")
        self.assertNotRegex(
            matlab,
            re.compile(r"^\s*(?:global|persistent|parfor|while)\b", re.MULTILINE),
        )
        self.assertNotRegex(matlab, r"\bclose\s+all\b")
        self.assertNotIn("simulink", matlab)
        self.assertNotIn("control system toolbox", matlab)

    def test_retained_evidence_has_acceptance_map_and_claim_boundary(self) -> None:
        evidence = EVIDENCE_PATH.read_text(encoding="utf-8")
        self.assertTrue(evidence.endswith("\n"))
        self.assertFalse(evidence.endswith("\n\n"))
        for heading in (
            "## Result and claim boundary",
            "## Acceptance mapping",
            "## Exact validation performed",
            "## Independent system-risk review and sweep-demand behavioral coverage",
            "## Changed and preserved invariants",
            "## Residual risks and limitations",
            "## Rollback",
            "## Explicitly unperformed validation",
            "## Schema-aligned evidence summary",
        ):
            self.assertIn(heading, evidence)
        self.assertEqual(
            len(re.findall(r"^\| A[1-8] \|", evidence, re.MULTILINE)), 8
        )
        for boundary in (
            "static",
            "simulated",
            "matlab runtime",
            "ui",
            "numerical",
            "bench",
            "hil",
            "field",
            "production",
            "unperformed",
        ):
            self.assertIn(boundary, evidence.lower())
        match = re.search(r"```json\n(.*?)\n```", evidence, re.DOTALL)
        self.assertIsNotNone(match)
        summary = json.loads(match.group(1))
        self.assertEqual(summary["batch_id"], "P16")
        self.assertEqual(summary["status"], "pass")
        self.assertEqual(len(summary["acceptance"]), 8)
        self.assertTrue(
            all(item["status"] == "pass" for item in summary["acceptance"])
        )


class P16EquationOracleTests(unittest.TestCase):
    def test_deterministic_baseline_signature_and_fixed_shape(self) -> None:
        first = _oracle()
        second = _oracle(60.0, REFERENCE_DENSITY_KGPM3, 1)
        self.assertEqual(first, second)
        self.assertEqual(first["sample_count"], 801)
        self.assertEqual(first["interval_count"], 800)
        self.assertEqual(first["active_sample_count"], 751)
        self.assertEqual(first["time_s"][0], 0.0)
        self.assertEqual(first["time_s"][-1], 8.0)
        self.assertAlmostEqual(first["reference_dynamic_pressure_pa"], 1325.0079853184736, 10)
        self.assertEqual(first["actual_dynamic_pressure_ratio"], 1.0)
        self.assertEqual(first["lookup_dynamic_pressure_ratio"], 1.0)
        self.assertFalse(first["lookup_clamped"])
        self.assertEqual(first["lower_knot_index"], 2)
        self.assertEqual(first["upper_knot_index"], 3)
        self.assertEqual(first["interpolation_weight"], 1.0)
        self.assertAlmostEqual(first["roll_angle_gain"], 0.48, 14)
        self.assertAlmostEqual(first["roll_rate_gain_s"], 0.32, 14)
        self.assertAlmostEqual(first["effective_natural_frequency_radps"], 2.4, 14)
        self.assertAlmostEqual(first["effective_damping_ratio"], 0.8, 14)
        self.assertEqual(first["roll_error_deg"][50], 10.0)
        self.assertAlmostEqual(first["aileron_command_deg"][50], 4.8, 13)
        self.assertAlmostEqual(first["roll_acceleration_degps2"][50], 57.6, 12)
        self.assertFalse(any(first["aileron_command_saturated"]))
        self.assertTrue(first["reached_ninety_percent"])
        self.assertTrue(first["settled_by_end"])
        self.assertAlmostEqual(first["roll_tracking_rms_deg"], 2.496451737663988, 12)
        self.assertAlmostEqual(first["final_roll_error_deg"], -6.924730283266717e-06, 14)
        self.assertAlmostEqual(first["peak_roll_overshoot_deg"], 0.16154567580150037, 12)
        self.assertAlmostEqual(first["time_to_ninety_percent_s"], 1.23, 14)
        self.assertAlmostEqual(first["settling_time_s"], 1.55, 14)
        self.assertAlmostEqual(first["peak_absolute_roll_rate_degps"], 10.309297451815567, 12)
        self.assertAlmostEqual(first["peak_absolute_aileron_command_deg"], 4.8, 13)

    def test_every_schedule_controller_and_state_update_reconstructs(self) -> None:
        result = _oracle(67.5, 0.9 * REFERENCE_DENSITY_KGPM3, 1)
        expected_dynamic_pressure_pa = (
            0.5 * result["air_density_kgpm3"] * result["true_airspeed_mps"] ** 2
        )
        self.assertAlmostEqual(
            result["actual_dynamic_pressure_pa"], expected_dynamic_pressure_pa, 12
        )
        self.assertAlmostEqual(
            result["actual_dynamic_pressure_ratio"],
            expected_dynamic_pressure_pa / result["reference_dynamic_pressure_pa"],
            14,
        )
        lower = result["lower_knot_index"] - 1
        upper = result["upper_knot_index"] - 1
        weight = result["interpolation_weight"]
        expected_angle_gain = (
            (1.0 - weight) * result["roll_angle_gain_table"][lower]
            + weight * result["roll_angle_gain_table"][upper]
        )
        expected_rate_gain = (
            (1.0 - weight) * result["roll_rate_gain_table_s"][lower]
            + weight * result["roll_rate_gain_table_s"][upper]
        )
        self.assertAlmostEqual(result["roll_angle_gain"], expected_angle_gain, 15)
        self.assertAlmostEqual(result["roll_rate_gain_s"], expected_rate_gain, 15)
        dt = result["sample_time_s"]
        limit = result["aileron_command_limit_deg"]
        for index in range(result["sample_count"]):
            expected_error = (
                result["roll_command_deg"][index]
                - result["roll_angle_deg"][index]
            )
            expected_unclamped = (
                result["roll_angle_gain"] * expected_error
                - result["roll_rate_gain_s"]
                * result["roll_rate_degps"][index]
            )
            expected_command = _clamp(expected_unclamped, -limit, limit)
            expected_acceleration = (
                result["actual_control_effectiveness_per_s2"]
                * expected_command
            )
            self.assertAlmostEqual(result["roll_error_deg"][index], expected_error, 12)
            self.assertAlmostEqual(
                result["aileron_command_unclamped_deg"][index],
                expected_unclamped,
                11,
            )
            self.assertAlmostEqual(
                result["aileron_command_deg"][index], expected_command, 11
            )
            self.assertAlmostEqual(
                result["roll_acceleration_degps2"][index],
                expected_acceleration,
                10,
            )
            if index < result["sample_count"] - 1:
                self.assertAlmostEqual(
                    result["roll_angle_deg"][index + 1],
                    result["roll_angle_deg"][index]
                    + dt * result["roll_rate_degps"][index],
                    11,
                )
                self.assertAlmostEqual(
                    result["roll_rate_degps"][index + 1],
                    result["roll_rate_degps"][index]
                    + dt * result["roll_acceleration_degps2"][index],
                    10,
                )

    def test_causal_order_and_stable_baseline_capture(self) -> None:
        result = _oracle()
        command_index = 50
        self.assertTrue(all(value == 0.0 for value in result["roll_command_deg"][:command_index]))
        for key in (
            "roll_angle_deg",
            "roll_rate_degps",
            "roll_error_deg",
            "aileron_command_deg",
            "roll_acceleration_degps2",
        ):
            self.assertTrue(all(value == 0.0 for value in result[key][:command_index]), key)
        self.assertEqual(result["roll_angle_deg"][command_index], 0.0)
        self.assertEqual(result["roll_rate_degps"][command_index], 0.0)
        self.assertGreater(result["aileron_command_deg"][command_index], 0.0)
        self.assertGreater(result["roll_acceleration_degps2"][command_index], 0.0)
        self.assertEqual(result["roll_angle_deg"][command_index + 1], 0.0)
        self.assertGreater(result["roll_rate_degps"][command_index + 1], 0.0)
        self.assertGreater(result["roll_angle_deg"][command_index + 2], 0.0)
        self.assertLess(abs(result["final_roll_error_deg"]), 0.001)
        self.assertLess(result["peak_roll_overshoot_deg"], 0.2)
        self.assertLess(result["time_to_ninety_percent_s"], 1.5)
        self.assertLess(result["settling_time_s"], 3.0)

    def test_manual_table_knots_midpoints_and_clamping(self) -> None:
        knots = (0.5, 0.75, 1.0, 1.25, 1.5)
        knot_results = tuple(
            _oracle(60.0, ratio * REFERENCE_DENSITY_KGPM3, 1)
            for ratio in knots
        )
        for index, (ratio, result) in enumerate(zip(knots, knot_results)):
            with self.subTest(ratio=ratio):
                self.assertAlmostEqual(result["lookup_dynamic_pressure_ratio"], ratio, 14)
                self.assertAlmostEqual(
                    result["roll_angle_gain"],
                    result["target_natural_frequency_radps"] ** 2
                    / (
                        result["reference_control_effectiveness_per_s2"]
                        * ratio
                    ),
                    14,
                )
                self.assertAlmostEqual(
                    result["roll_rate_gain_s"],
                    2.0
                    * result["target_damping_ratio"]
                    * result["target_natural_frequency_radps"]
                    / (
                        result["reference_control_effectiveness_per_s2"]
                        * ratio
                    ),
                    14,
                )
                self.assertAlmostEqual(
                    result["roll_angle_gain_interpolation_error_fraction"], 0.0, 14
                )
                self.assertAlmostEqual(
                    result["roll_rate_gain_interpolation_error_fraction"], 0.0, 14
                )
                self.assertAlmostEqual(result["effective_natural_frequency_radps"], 2.4, 13)
                self.assertAlmostEqual(result["effective_damping_ratio"], 0.8, 13)
                if index == 0:
                    self.assertEqual(result["lower_knot_index"], 1)
                    self.assertEqual(result["upper_knot_index"], 1)
                    self.assertEqual(result["interpolation_weight"], 0.0)
                else:
                    self.assertEqual(result["lower_knot_index"], index)
                    self.assertEqual(result["upper_knot_index"], index + 1)
                    self.assertAlmostEqual(result["interpolation_weight"], 1.0, 14)

        midpoint = _oracle(60.0, 0.875 * REFERENCE_DENSITY_KGPM3, 1)
        self.assertEqual(midpoint["lower_knot_index"], 2)
        self.assertEqual(midpoint["upper_knot_index"], 3)
        self.assertAlmostEqual(midpoint["interpolation_weight"], 0.5, 14)
        self.assertAlmostEqual(
            midpoint["roll_angle_gain"],
            0.5
            * (
                midpoint["roll_angle_gain_table"][1]
                + midpoint["roll_angle_gain_table"][2]
            ),
            14,
        )
        self.assertGreater(
            midpoint["roll_angle_gain_interpolation_error_fraction"], 0.0
        )
        self.assertAlmostEqual(
            midpoint["roll_angle_gain_interpolation_error_fraction"],
            midpoint["roll_rate_gain_interpolation_error_fraction"],
            14,
        )
        self.assertAlmostEqual(
            midpoint["roll_angle_gain_actual_condition_mismatch_fraction"],
            midpoint["roll_angle_gain_interpolation_error_fraction"],
            14,
        )
        self.assertAlmostEqual(
            midpoint["roll_rate_gain_actual_condition_mismatch_fraction"],
            midpoint["roll_rate_gain_interpolation_error_fraction"],
            14,
        )

        below = _oracle(45.0, 0.5 * REFERENCE_DENSITY_KGPM3, 1)
        above = _oracle(75.0, 1.5 * REFERENCE_DENSITY_KGPM3, 1)
        self.assertLess(below["actual_dynamic_pressure_ratio"], 0.5)
        self.assertEqual(below["lookup_dynamic_pressure_ratio"], 0.5)
        self.assertTrue(below["lookup_clamped"])
        self.assertGreater(above["actual_dynamic_pressure_ratio"], 1.5)
        self.assertEqual(above["lookup_dynamic_pressure_ratio"], 1.5)
        self.assertTrue(above["lookup_clamped"])

    def test_density_sweep_hits_knots_and_retains_response(self) -> None:
        ratios = (0.5, 0.75, 1.0, 1.25, 1.5)
        results = tuple(
            _oracle(60.0, ratio * REFERENCE_DENSITY_KGPM3, 1)
            for ratio in ratios
        )
        baseline = results[2]
        for ratio, result in zip(ratios, results):
            with self.subTest(ratio=ratio):
                self.assertAlmostEqual(result["actual_dynamic_pressure_ratio"], ratio, 14)
                self.assertEqual(result["schedule_mode"], 1)
                self.assertEqual(result["true_airspeed_mps"], 60.0)
                for history in (
                    "roll_angle_deg",
                    "roll_rate_degps",
                    "roll_acceleration_degps2",
                ):
                    self.assertLess(
                        max(
                            abs(actual - expected)
                            for actual, expected in zip(
                                result[history], baseline[history]
                            )
                        ),
                        1e-12,
                        history,
                    )
                self.assertAlmostEqual(result["effective_natural_frequency_radps"], 2.4, 13)
                self.assertAlmostEqual(result["effective_damping_ratio"], 0.8, 13)
        self.assertTrue(
            all(
                earlier["roll_angle_gain"] > later["roll_angle_gain"]
                for earlier, later in zip(results, results[1:])
            )
        )
        self.assertTrue(
            all(
                earlier["roll_rate_gain_s"] > later["roll_rate_gain_s"]
                for earlier, later in zip(results, results[1:])
            )
        )

    def test_airspeed_sweep_isolates_schedule_from_fixed_gain(self) -> None:
        speeds = (45.0, 52.5, 60.0, 67.5, 72.0)
        scheduled = tuple(
            _oracle(speed, REFERENCE_DENSITY_KGPM3, 1) for speed in speeds
        )
        fixed = tuple(
            _oracle(speed, REFERENCE_DENSITY_KGPM3, 0) for speed in speeds
        )
        for speed, scheduled_result, fixed_result in zip(speeds, scheduled, fixed):
            with self.subTest(speed=speed):
                self.assertEqual(scheduled_result["true_airspeed_mps"], speed)
                self.assertEqual(fixed_result["true_airspeed_mps"], speed)
                self.assertEqual(
                    scheduled_result["air_density_kgpm3"],
                    fixed_result["air_density_kgpm3"],
                )
                self.assertEqual(scheduled_result["schedule_mode"], 1)
                self.assertEqual(fixed_result["schedule_mode"], 0)
                self.assertEqual(
                    scheduled_result["actual_control_effectiveness_per_s2"],
                    fixed_result["actual_control_effectiveness_per_s2"],
                )
                self.assertEqual(
                    scheduled_result["roll_command_deg"],
                    fixed_result["roll_command_deg"],
                )
                self.assertEqual(
                    scheduled_result["time_s"], fixed_result["time_s"]
                )
                self.assertEqual(fixed_result["lookup_dynamic_pressure_ratio"], 1.0)
        scheduled_frequency_spread = max(
            result["effective_natural_frequency_radps"] for result in scheduled
        ) - min(result["effective_natural_frequency_radps"] for result in scheduled)
        fixed_frequency_spread = max(
            result["effective_natural_frequency_radps"] for result in fixed
        ) - min(result["effective_natural_frequency_radps"] for result in fixed)
        scheduled_capture_spread = max(
            result["time_to_ninety_percent_s"] for result in scheduled
        ) - min(result["time_to_ninety_percent_s"] for result in scheduled)
        fixed_capture_spread = max(
            result["time_to_ninety_percent_s"] for result in fixed
        ) - min(result["time_to_ninety_percent_s"] for result in fixed)
        self.assertLess(scheduled_frequency_spread, 0.04)
        self.assertGreater(fixed_frequency_spread, 0.9)
        self.assertLess(scheduled_capture_spread, fixed_capture_spread)
        self.assertEqual(scheduled[2], _oracle())
        self.assertEqual(scheduled[2]["roll_angle_deg"], fixed[2]["roll_angle_deg"])

    def test_scheduled_sweeps_preserve_response_while_control_demand_falls(
        self,
    ) -> None:
        speed_results = tuple(
            _oracle(speed, REFERENCE_DENSITY_KGPM3, 1)
            for speed in (45.0, 52.5, 60.0, 67.5, 72.0)
        )
        density_results = tuple(
            _oracle(60.0, ratio * REFERENCE_DENSITY_KGPM3, 1)
            for ratio in (0.5, 0.75, 1.0, 1.25, 1.5)
        )

        for sweep_name, results in (
            ("true airspeed", speed_results),
            ("air density", density_results),
        ):
            with self.subTest(sweep=sweep_name):
                effectiveness = tuple(
                    result["actual_control_effectiveness_per_s2"]
                    for result in results
                )
                reported_peak_demand = tuple(
                    result["peak_absolute_aileron_command_deg"]
                    for result in results
                )
                history_peak_demand = tuple(
                    max(map(abs, result["aileron_command_deg"]))
                    for result in results
                )
                self.assertTrue(
                    all(
                        earlier < later
                        for earlier, later in zip(
                            effectiveness, effectiveness[1:]
                        )
                    )
                )
                self.assertTrue(
                    all(
                        earlier > later
                        for earlier, later in zip(
                            reported_peak_demand, reported_peak_demand[1:]
                        )
                    )
                )
                for reported, reconstructed in zip(
                    reported_peak_demand, history_peak_demand
                ):
                    self.assertAlmostEqual(reported, reconstructed, 14)
                self.assertTrue(
                    all(
                        result["aileron_command_saturation_fraction"] == 0.0
                        for result in results
                    )
                )

        speed_settling = tuple(
            result["settling_time_s"] for result in speed_results
        )
        self.assertLessEqual(max(speed_settling) - min(speed_settling), 0.011)
        self.assertAlmostEqual(
            speed_results[0]["peak_absolute_aileron_command_deg"], 8.8, 13
        )
        self.assertAlmostEqual(
            speed_results[-1]["peak_absolute_aileron_command_deg"],
            3.3536,
            13,
        )

        density_baseline = density_results[2]
        for result in density_results:
            self.assertLess(
                max(
                    abs(actual - expected)
                    for actual, expected in zip(
                        result["roll_angle_deg"],
                        density_baseline["roll_angle_deg"],
                    )
                ),
                1e-12,
            )
            self.assertAlmostEqual(
                result["settling_time_s"],
                density_baseline["settling_time_s"],
                14,
            )
        self.assertAlmostEqual(
            density_results[0]["peak_absolute_aileron_command_deg"], 9.6, 13
        )
        self.assertAlmostEqual(
            density_results[-1]["peak_absolute_aileron_command_deg"], 3.2, 13
        )

    def test_equal_dynamic_pressure_pair_exposes_tas_only_failure(self) -> None:
        paired_speed = 75.0
        paired_density = REFERENCE_DENSITY_KGPM3 * (60.0 / paired_speed) ** 2
        reference_scheduled = _oracle()
        pair_scheduled = _oracle(paired_speed, paired_density, 1)
        reference_broken = _oracle(60.0, REFERENCE_DENSITY_KGPM3, -1)
        pair_broken = _oracle(paired_speed, paired_density, -1)

        self.assertAlmostEqual(
            pair_scheduled["actual_dynamic_pressure_pa"],
            reference_scheduled["actual_dynamic_pressure_pa"],
            12,
        )
        self.assertAlmostEqual(pair_scheduled["actual_dynamic_pressure_ratio"], 1.0, 14)
        self.assertAlmostEqual(
            pair_scheduled["lookup_dynamic_pressure_ratio"],
            reference_scheduled["lookup_dynamic_pressure_ratio"],
            14,
        )
        self.assertAlmostEqual(
            pair_scheduled["roll_angle_gain"],
            reference_scheduled["roll_angle_gain"],
            14,
        )
        self.assertAlmostEqual(
            pair_scheduled["roll_rate_gain_s"],
            reference_scheduled["roll_rate_gain_s"],
            14,
        )
        for history in ("roll_angle_deg", "roll_rate_degps"):
            self.assertLess(
                max(
                    abs(actual - expected)
                    for actual, expected in zip(
                        pair_scheduled[history], reference_scheduled[history]
                    )
                ),
                1e-12,
                history,
            )

        self.assertEqual(reference_broken["roll_angle_deg"], reference_scheduled["roll_angle_deg"])
        self.assertAlmostEqual(pair_broken["lookup_dynamic_pressure_ratio_raw"], 1.5625, 14)
        self.assertEqual(pair_broken["lookup_dynamic_pressure_ratio"], 1.5)
        self.assertTrue(pair_broken["lookup_clamped"])
        self.assertLess(pair_broken["roll_angle_gain"], pair_scheduled["roll_angle_gain"])
        self.assertLess(pair_broken["roll_rate_gain_s"], pair_scheduled["roll_rate_gain_s"])
        self.assertAlmostEqual(
            pair_broken["actual_condition_ideal_roll_angle_gain"], 0.48, 14
        )
        self.assertAlmostEqual(
            pair_broken["actual_condition_ideal_roll_rate_gain_s"], 0.32, 14
        )
        self.assertAlmostEqual(
            pair_broken["roll_angle_gain_actual_condition_mismatch_fraction"],
            -1.0 / 3.0,
            14,
        )
        self.assertAlmostEqual(
            pair_broken["roll_rate_gain_actual_condition_mismatch_fraction"],
            -1.0 / 3.0,
            14,
        )
        self.assertLess(
            pair_broken["effective_natural_frequency_radps"],
            pair_scheduled["effective_natural_frequency_radps"],
        )
        self.assertLess(
            pair_broken["effective_damping_ratio"],
            pair_scheduled["effective_damping_ratio"],
        )
        self.assertGreater(
            pair_broken["time_to_ninety_percent_s"],
            pair_scheduled["time_to_ninety_percent_s"],
        )
        self.assertNotEqual(pair_broken["roll_angle_deg"], pair_scheduled["roll_angle_deg"])
        self.assertIn("density omitted", pair_broken["schedule_source"])

    def test_fixed_reference_mode_is_exact_nominal_limit(self) -> None:
        scheduled = _oracle()
        fixed = _oracle(60.0, REFERENCE_DENSITY_KGPM3, 0)
        broken = _oracle(60.0, REFERENCE_DENSITY_KGPM3, -1)
        self.assertEqual(scheduled["lookup_dynamic_pressure_ratio"], 1.0)
        self.assertEqual(fixed["lookup_dynamic_pressure_ratio"], 1.0)
        self.assertEqual(broken["lookup_dynamic_pressure_ratio"], 1.0)
        for key in (
            "roll_angle_gain",
            "roll_rate_gain_s",
            "roll_angle_deg",
            "roll_rate_degps",
            "roll_error_deg",
            "aileron_command_deg",
            "roll_acceleration_degps2",
            "time_to_ninety_percent_s",
            "settling_time_s",
        ):
            self.assertEqual(scheduled[key], fixed[key], key)
            self.assertEqual(scheduled[key], broken[key], key)

    def test_malformed_inputs_reject_without_poisoning_recovery(self) -> None:
        malformed = (
            (44.999999999, REFERENCE_DENSITY_KGPM3, 1),
            (75.000000001, REFERENCE_DENSITY_KGPM3, 1),
            (60.0, 0.5 * REFERENCE_DENSITY_KGPM3 - 1e-12, 1),
            (60.0, 1.5 * REFERENCE_DENSITY_KGPM3 + 1e-12, 1),
            (60.0, REFERENCE_DENSITY_KGPM3, -2),
            (60.0, REFERENCE_DENSITY_KGPM3, 2),
            ([45.0, 60.0], REFERENCE_DENSITY_KGPM3, 1),
            (60.0, [REFERENCE_DENSITY_KGPM3], 1),
            (60.0, REFERENCE_DENSITY_KGPM3, [-1, 1]),
            (60.0 + 1.0j, REFERENCE_DENSITY_KGPM3, 1),
            (60.0, REFERENCE_DENSITY_KGPM3 + 1.0j, 1),
            (float("nan"), REFERENCE_DENSITY_KGPM3, 1),
            (60.0, float("inf"), 1),
            (60.0, REFERENCE_DENSITY_KGPM3, float("nan")),
            (True, REFERENCE_DENSITY_KGPM3, 1),
        )
        for speed, density, mode in malformed:
            with self.subTest(speed=speed, density=density, mode=mode):
                with self.assertRaises(ValueError):
                    _oracle(speed, density, mode)
        self.assertEqual(_oracle(), _oracle(60.0, REFERENCE_DENSITY_KGPM3, 1))

    def test_broken_and_rejected_calls_have_exact_rollback_and_recovery(self) -> None:
        baseline = _oracle()
        broken = _oracle(
            75.0,
            REFERENCE_DENSITY_KGPM3 * (60.0 / 75.0) ** 2,
            -1,
        )
        self.assertNotEqual(broken["roll_angle_deg"], baseline["roll_angle_deg"])
        rolled_back = _oracle(60.0, REFERENCE_DENSITY_KGPM3, 1)
        self.assertEqual(rolled_back, baseline)
        with self.assertRaises(ValueError):
            _oracle(80.0, REFERENCE_DENSITY_KGPM3, 1)
        recovered = _oracle()
        self.assertEqual(recovered, baseline)

    def test_accepted_corners_and_representative_grid_are_finite_and_fixed(self) -> None:
        corners = tuple(
            _oracle(speed, density_ratio * REFERENCE_DENSITY_KGPM3, mode)
            for speed in (45.0, 75.0)
            for density_ratio in (0.5, 1.5)
            for mode in (-1, 0, 1)
        )
        self.assertEqual(len(corners), 12)
        grid = tuple(
            _oracle(speed, density_ratio * REFERENCE_DENSITY_KGPM3, mode)
            for speed in (45.0, 60.0, 75.0)
            for density_ratio in (0.5, 1.0, 1.5)
            for mode in (-1, 0, 1)
        )
        self.assertEqual(len(grid), 27)
        history_fields = (
            "roll_command_deg",
            "roll_angle_deg",
            "roll_rate_degps",
            "roll_error_deg",
            "aileron_command_unclamped_deg",
            "aileron_command_deg",
            "roll_acceleration_degps2",
        )
        for result in corners + grid:
            self.assertEqual(result["sample_count"], 801)
            self.assertEqual(result["interval_count"], 800)
            self.assertEqual(result["active_sample_count"], 751)
            for key in history_fields:
                self.assertEqual(len(result[key]), 801)
                self.assertTrue(all(math.isfinite(value) for value in result[key]), key)
            self.assertTrue(math.isfinite(result["roll_angle_gain"]))
            self.assertTrue(math.isfinite(result["roll_rate_gain_s"]))
            self.assertGreater(result["roll_angle_gain"], 0.0)
            self.assertGreater(result["roll_rate_gain_s"], 0.0)
            self.assertGreaterEqual(min(result["aileron_command_deg"]), -15.0)
            self.assertLessEqual(max(result["aileron_command_deg"]), 15.0)
            self.assertGreaterEqual(result["lookup_dynamic_pressure_ratio"], 0.5)
            self.assertLessEqual(result["lookup_dynamic_pressure_ratio"], 1.5)

    def test_synchronous_interface_bounds_timeout_and_cancellation_applicability(
        self,
    ) -> None:
        result = _oracle()
        self.assertEqual(result["sample_count"], 801)
        self.assertEqual(result["interval_count"], 800)
        self.assertEqual(len(result["dynamic_pressure_ratio_knots"]), 5)
        self.assertEqual(len(result["roll_angle_gain_table"]), 5)
        self.assertEqual(len(result["roll_rate_gain_table_s"]), 5)
        self.assertEqual(result["time_s"], tuple(index * 0.01 for index in range(801)))
        # The model performs fixed synchronous arithmetic and owns no timer,
        # future, worker, external I/O, or cancellation state. A runtime
        # timeout/cancel transition is therefore not applicable; fixed work,
        # bounded inputs, and capped 12/27-case matrices are the resource gate.


if __name__ == "__main__":
    unittest.main()
