from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_ID = "P13"
MODULE_FOLDER = ROOT / "modules/13-hold-pitch-and-altitude"
EVIDENCE_PATH = ROOT / "docs/evidence/P13-2026-08-23.md"
GUIDING_QUESTION = (
    "What inputs, observable effects, and failure modes matter when you hold "
    "Pitch and Altitude?"
)


def _bounded_scalar(name: str, value: object, lower: float, upper: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite real scalar")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite real scalar")
    if not lower <= result <= upper:
        raise ValueError(f"{name} must be between {lower} and {upper} inclusive")
    return result


def _feedback_sign(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("feedback sign must be +1 or -1")
    result = float(value)
    if not math.isfinite(result) or result not in (-1.0, 1.0):
        raise ValueError("feedback sign must be +1 or -1")
    return int(result)


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def _oracle(
    altitude_gain_rad_per_m: object = 0.004,
    pitch_natural_frequency_radps: object = 2.4,
    altitude_feedback_sign: object = 1,
) -> dict[str, object]:
    """Independent standard-library implementation of the declared equations."""
    altitude_gain = _bounded_scalar(
        "altitude-to-pitch gain", altitude_gain_rad_per_m, 0.0, 0.008
    )
    pitch_frequency = _bounded_scalar(
        "pitch natural frequency", pitch_natural_frequency_radps, 1.2, 3.6
    )
    feedback_sign = _feedback_sign(altitude_feedback_sign)

    sample_time_s = 0.02
    time_s = tuple(index * sample_time_s for index in range(1501))
    sample_count = len(time_s)
    initial_altitude_m = 1000.0
    altitude_step_m = 30.0
    command_step_time_s = 1.0
    true_airspeed_mps = 60.0
    flight_path_time_constant_s = 1.5
    pitch_damping_ratio = 0.8
    pitch_command_limit_rad = math.radians(10.0)
    pitch_control_command_limit_rad = math.radians(20.0)
    pitch_stiffness_per_s2 = 0.8
    pitch_rate_damping_per_s = 1.2
    pitch_control_effectiveness_per_s2 = 12.0

    pitch_proportional_gain = (
        pitch_frequency**2 - pitch_stiffness_per_s2
    ) / pitch_control_effectiveness_per_s2
    pitch_command_feedforward_gain = (
        pitch_stiffness_per_s2 / pitch_control_effectiveness_per_s2
    )
    pitch_rate_gain_s = (
        2.0 * pitch_damping_ratio * pitch_frequency
        - pitch_rate_damping_per_s
    ) / pitch_control_effectiveness_per_s2

    altitude_command_m = tuple(
        initial_altitude_m
        + (altitude_step_m if time >= command_step_time_s else 0.0)
        for time in time_s
    )
    altitude_m = [0.0] * sample_count
    pitch_angle_rad = [0.0] * sample_count
    pitch_rate_radps = [0.0] * sample_count
    flight_path_angle_rad = [0.0] * sample_count
    altitude_error_m = [0.0] * sample_count
    climb_rate_mps = [0.0] * sample_count
    pitch_command_unclamped_rad = [0.0] * sample_count
    pitch_command_rad = [0.0] * sample_count
    pitch_control_command_unclamped_rad = [0.0] * sample_count
    pitch_control_command_rad = [0.0] * sample_count
    pitch_acceleration_radps2 = [0.0] * sample_count
    flight_path_rate_radps = [0.0] * sample_count
    pitch_command_saturated = [False] * sample_count
    pitch_control_command_saturated = [False] * sample_count
    altitude_m[0] = initial_altitude_m

    for index in range(sample_count):
        altitude_error_m[index] = altitude_command_m[index] - altitude_m[index]
        climb_rate_mps[index] = true_airspeed_mps * math.sin(
            flight_path_angle_rad[index]
        )
        pitch_command_unclamped_rad[index] = (
            feedback_sign * altitude_gain * altitude_error_m[index]
        )
        pitch_command_rad[index] = _clamp(
            pitch_command_unclamped_rad[index],
            -pitch_command_limit_rad,
            pitch_command_limit_rad,
        )
        pitch_command_saturated[index] = (
            abs(pitch_command_unclamped_rad[index]) > pitch_command_limit_rad
        )
        pitch_control_command_unclamped_rad[index] = (
            pitch_proportional_gain
            * (pitch_command_rad[index] - pitch_angle_rad[index])
            + pitch_command_feedforward_gain * pitch_command_rad[index]
            - pitch_rate_gain_s * pitch_rate_radps[index]
        )
        pitch_control_command_rad[index] = _clamp(
            pitch_control_command_unclamped_rad[index],
            -pitch_control_command_limit_rad,
            pitch_control_command_limit_rad,
        )
        pitch_control_command_saturated[index] = (
            abs(pitch_control_command_unclamped_rad[index])
            > pitch_control_command_limit_rad
        )
        pitch_acceleration_radps2[index] = (
            -pitch_stiffness_per_s2 * pitch_angle_rad[index]
            - pitch_rate_damping_per_s * pitch_rate_radps[index]
            + pitch_control_effectiveness_per_s2
            * pitch_control_command_rad[index]
        )
        flight_path_rate_radps[index] = (
            pitch_angle_rad[index] - flight_path_angle_rad[index]
        ) / flight_path_time_constant_s

        if index < sample_count - 1:
            altitude_m[index + 1] = (
                altitude_m[index] + sample_time_s * climb_rate_mps[index]
            )
            pitch_angle_rad[index + 1] = (
                pitch_angle_rad[index] + sample_time_s * pitch_rate_radps[index]
            )
            pitch_rate_radps[index + 1] = (
                pitch_rate_radps[index]
                + sample_time_s * pitch_acceleration_radps2[index]
            )
            flight_path_angle_rad[index + 1] = (
                flight_path_angle_rad[index]
                + sample_time_s * flight_path_rate_radps[index]
            )

    down_command_m = tuple(-value for value in altitude_command_m)
    down_position_m = tuple(-value for value in altitude_m)
    down_error_m = tuple(
        command - position
        for command, position in zip(down_command_m, down_position_m)
    )
    pitch_angle_deg = tuple(map(math.degrees, pitch_angle_rad))
    pitch_rate_degps = tuple(map(math.degrees, pitch_rate_radps))
    flight_path_angle_deg = tuple(map(math.degrees, flight_path_angle_rad))
    pitch_command_unclamped_deg = tuple(
        map(math.degrees, pitch_command_unclamped_rad)
    )
    pitch_command_deg = tuple(map(math.degrees, pitch_command_rad))
    pitch_control_command_unclamped_deg = tuple(
        map(math.degrees, pitch_control_command_unclamped_rad)
    )
    pitch_control_command_deg = tuple(
        map(math.degrees, pitch_control_command_rad)
    )
    pitch_acceleration_degps2 = tuple(
        map(math.degrees, pitch_acceleration_radps2)
    )
    flight_path_rate_degps = tuple(map(math.degrees, flight_path_rate_radps))
    active_indices = tuple(
        index for index, time in enumerate(time_s) if time >= command_step_time_s
    )
    pitch_tracking_error_deg = tuple(
        command - angle
        for command, angle in zip(pitch_command_deg, pitch_angle_deg)
    )
    pitch_tracking_rms_deg = math.sqrt(
        sum(pitch_tracking_error_deg[index] ** 2 for index in active_indices)
        / len(active_indices)
    )
    peak_altitude_overshoot_m = max(
        0.0,
        max(altitude_m[index] - altitude_command_m[-1] for index in active_indices),
    )
    capture_threshold_m = initial_altitude_m + 0.9 * altitude_step_m
    capture_index = next(
        (
            index
            for index in active_indices
            if altitude_m[index] >= capture_threshold_m
        ),
        None,
    )
    reached_ninety_percent = capture_index is not None
    time_to_ninety_percent_s = (
        time_s[capture_index] - command_step_time_s
        if capture_index is not None
        else time_s[-1] - command_step_time_s
    )
    settling_tolerance_m = 0.02 * altitude_step_m
    final_altitude_error_m = altitude_error_m[-1]
    settled_by_end = abs(final_altitude_error_m) <= settling_tolerance_m

    return {
        "altitude_gain_rad_per_m": altitude_gain,
        "pitch_natural_frequency_radps": pitch_frequency,
        "altitude_feedback_sign": feedback_sign,
        "sample_time_s": sample_time_s,
        "time_s": time_s,
        "sample_count": sample_count,
        "interval_count": sample_count - 1,
        "active_sample_count": len(active_indices),
        "initial_altitude_m": initial_altitude_m,
        "altitude_step_m": altitude_step_m,
        "command_step_time_s": command_step_time_s,
        "true_airspeed_mps": true_airspeed_mps,
        "flight_path_time_constant_s": flight_path_time_constant_s,
        "pitch_damping_ratio": pitch_damping_ratio,
        "pitch_command_limit_deg": 10.0,
        "pitch_control_command_limit_deg": 20.0,
        "pitch_stiffness_per_s2": pitch_stiffness_per_s2,
        "pitch_rate_damping_per_s": pitch_rate_damping_per_s,
        "pitch_control_effectiveness_per_s2": (
            pitch_control_effectiveness_per_s2
        ),
        "pitch_proportional_gain": pitch_proportional_gain,
        "pitch_command_feedforward_gain": pitch_command_feedforward_gain,
        "pitch_rate_gain_s": pitch_rate_gain_s,
        "altitude_command_m": altitude_command_m,
        "down_command_m": down_command_m,
        "altitude_m": tuple(altitude_m),
        "down_position_m": down_position_m,
        "altitude_error_m": tuple(altitude_error_m),
        "down_error_m": down_error_m,
        "pitch_angle_deg": pitch_angle_deg,
        "pitch_rate_degps": pitch_rate_degps,
        "flight_path_angle_deg": flight_path_angle_deg,
        "climb_rate_mps": tuple(climb_rate_mps),
        "pitch_command_unclamped_deg": pitch_command_unclamped_deg,
        "pitch_command_deg": pitch_command_deg,
        "pitch_control_command_unclamped_deg": (
            pitch_control_command_unclamped_deg
        ),
        "pitch_control_command_deg": pitch_control_command_deg,
        "pitch_acceleration_degps2": pitch_acceleration_degps2,
        "flight_path_rate_degps": flight_path_rate_degps,
        "pitch_command_saturated": tuple(pitch_command_saturated),
        "pitch_control_command_saturated": tuple(
            pitch_control_command_saturated
        ),
        "pitch_command_saturation_fraction": (
            sum(pitch_command_saturated) / sample_count
        ),
        "pitch_control_command_saturation_fraction": (
            sum(pitch_control_command_saturated) / sample_count
        ),
        "pitch_tracking_error_deg": pitch_tracking_error_deg,
        "pitch_tracking_rms_deg": pitch_tracking_rms_deg,
        "final_altitude_error_m": final_altitude_error_m,
        "peak_altitude_overshoot_m": peak_altitude_overshoot_m,
        "reached_ninety_percent": reached_ninety_percent,
        "time_to_ninety_percent_s": time_to_ninety_percent_s,
        "settling_tolerance_m": settling_tolerance_m,
        "settled_by_end": settled_by_end,
        "peak_pitch_angle_deg": max(map(abs, pitch_angle_deg)),
        "peak_flight_path_angle_deg": max(map(abs, flight_path_angle_deg)),
        "peak_pitch_control_command_deg": max(
            map(abs, pitch_control_command_deg)
        ),
    }


class P13ArtifactTests(unittest.TestCase):
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
                "number": 13,
                "id": "P13",
                "title": "Hold Pitch and Altitude",
                "guiding_question": GUIDING_QUESTION,
                "phase": 4,
                "phase_title": "Autopilots",
                "slug": "hold-pitch-and-altitude",
                "folder": "modules/13-hold-pitch-and-altitude",
                "implementation_batch": "P13",
                "prerequisites": ["P12"],
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
            "p12",
            "h=-down",
            "altitude error",
            "outer loop",
            "inner loop",
            "pitch-control",
            "flight-path angle",
            "perturbation",
            "mechanism",
            "reset",
            "broken",
            "open-altitude-loop",
            "teach-back",
            "fixed speed",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, combined)
        self.assertRegex(walkthrough.lower(), r"one plot|one .* at a time")
        self.assertIn("conceptual rather than current api compatibility", combined)
        self.assertIn("does not accept a p12", combined)
        self.assertIn("does not reproduce p10 actuator dynamics", combined)
        self.assertIn("does not inherit", combined)
        self.assertIn("p15", combined)
        self.assertIn("integral-of-error state or action", combined)
        self.assertNotIn("the response is identical through the command sample", combined)
        self.assertIn("does not bound altitude error", combined)
        self.assertNotIn("bounded divergence", combined)
        root_readme = (ROOT / "README.md").read_text(encoding="utf-8").lower()
        self.assertNotIn("bounded error growth", root_readme)
        self.assertIn("altitude error is still growing", root_readme)
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
            "functionout=model(altitudegain_rad_per_m,"
            "pitchnaturalfrequency_radps,altitudefeedbacksign)",
            compact,
        )
        self.assertIn("arguments", lower)
        self.assertIn(
            "altitudegain_rad_per_m(1,1)double{mustbereal,mustbefinite}=0.004",
            compact,
        )
        self.assertIn(
            "pitchnaturalfrequency_radps(1,1)double"
            "{mustbereal,mustbefinite}=2.4",
            compact,
        )
        self.assertIn(
            "altitudefeedbacksign(1,1)double{mustbereal,mustbefinite}=1",
            compact,
        )
        self.assertIn("altitudegain_rad_per_m<0||altitudegain_rad_per_m>0.008", compact)
        self.assertIn(
            "pitchnaturalfrequency_radps<1.2||pitchnaturalfrequency_radps>3.6",
            compact,
        )
        self.assertIn("altitudefeedbacksign~=1&&altitudefeedbacksign~=-1", compact)
        for identifier in (
            "P13:model:AltitudeGainRange",
            "P13:model:PitchFrequencyRange",
            "P13:model:FeedbackSign",
        ):
            self.assertIn(identifier, model)
        for expression in (
            "sampletime_s=0.02;",
            "timehorizon_s=30;",
            "time_s=0:sampletime_s:timehorizon_s;",
            "pitchproportionalgain=(pitchnaturalfrequency_radps^2-pitchstiffness_per_s2)/pitchcontroleffectiveness_per_s2;",
            "pitchcommandfeedforwardgain=pitchstiffness_per_s2/pitchcontroleffectiveness_per_s2;",
            "pitchrategain_s=(2*pitchdampingratio*pitchnaturalfrequency_radps-pitchratedamping_per_s)/pitchcontroleffectiveness_per_s2;",
            "altitudeerror_m(k)=altitudecommand_m(k)-altitude_m(k);",
            "climbrate_mps(k)=trueairspeed_mps*sin(flightpathangle_rad(k));",
            "pitchcommandunclamped_rad(k)=altitudefeedbacksign*altitudegain_rad_per_m*altitudeerror_m(k);",
            "pitchcommand_rad(k)=min(max(pitchcommandunclamped_rad(k),-pitchcommandlimit_rad),pitchcommandlimit_rad);",
            "pitchcontrolcommandunclamped_rad(k)=pitchproportionalgain*(pitchcommand_rad(k)-pitchangle_rad(k))+pitchcommandfeedforwardgain*pitchcommand_rad(k)-pitchrategain_s*pitchrate_radps(k);",
            "pitchcontrolcommand_rad(k)=min(max(pitchcontrolcommandunclamped_rad(k),-pitchcontrolcommandlimit_rad),pitchcontrolcommandlimit_rad);",
            "pitchacceleration_radps2(k)=-pitchstiffness_per_s2*pitchangle_rad(k)-pitchratedamping_per_s*pitchrate_radps(k)+pitchcontroleffectiveness_per_s2*pitchcontrolcommand_rad(k);",
            "flightpathrate_radps(k)=(pitchangle_rad(k)-flightpathangle_rad(k))/flightpathtimeconstant_s;",
            "altitude_m(k+1)=altitude_m(k)+sampletime_s*climbrate_mps(k);",
            "pitchangle_rad(k+1)=pitchangle_rad(k)+sampletime_s*pitchrate_radps(k);",
            "pitchrate_radps(k+1)=pitchrate_radps(k)+sampletime_s*pitchacceleration_radps2(k);",
            "flightpathangle_rad(k+1)=flightpathangle_rad(k)+sampletime_s*flightpathrate_radps(k);",
            "downcommand_m=-altitudecommand_m;",
            "downposition_m=-altitude_m;",
            "downerror_m=downcommand_m-downposition_m;",
        ):
            with self.subTest(expression=expression):
                self.assertIn(expression, compact)
        for field in (
            "altitudeCommand_m",
            "downCommand_m",
            "altitude_m",
            "downPosition_m",
            "altitudeError_m",
            "downError_m",
            "pitchAngle_deg",
            "flightPathAngle_deg",
            "pitchCommand_deg",
            "pitchControlCommand_deg",
            "controllerEquation",
            "plantEquation",
            "frameConvention",
            "brokenCaseDefinition",
            "analysisScope",
        ):
            self.assertIn(f"'{field}'", model)
        for presentation_call in (
            "figure(",
            "plot(",
            "uiaxes(",
            "uifigure(",
            "disp(",
            "fprintf(",
        ):
            self.assertNotIn(presentation_call, lower)
        self.assertNotRegex(lower, re.compile(r"^\s*(?:while|parfor)\b", re.MULTILINE))

    def test_experiment_has_two_isolated_sweeps_and_broken_case(self) -> None:
        experiment = self.text["experiment.m"]
        lower = experiment.lower()
        compact = re.sub(r"\s+", "", experiment.replace("...", "")).lower()
        self.assertGreaterEqual(experiment.count("%%"), 15)
        self.assertGreaterEqual(lower.count("figure("), 5)
        self.assertGreaterEqual(lower.count("assert("), 7)
        for concept in (
            "baseline",
            "altitude-to-pitch gain",
            "pitch natural frequency",
            "changed view",
            "mechanism",
            "limiting case",
            "broken",
            "feedback sign",
            "h=-down",
        ):
            self.assertIn(concept, lower)
        for unit in ("rad/m", "rad/s", "deg", "m/s", "m"):
            self.assertIn(unit, lower)
        self.assertIn("baseline=model(0.004,2.4,1)", compact)
        self.assertIn("model(altitudegainsweep_rad_per_m(k),2.4,1)", compact)
        self.assertIn("model(0.004,pitchfrequencysweep_radps(k),1)", compact)
        self.assertIn("openaltitudeloop=model(0,2.4,1)", compact)
        self.assertIn("broken=model(0.004,2.4,-1)", compact)
        self.assertNotRegex(lower, r"\bclose\s+all\b")
        self.assertIn("findall(groot", lower)
        self.assertIn("'^p13 '", lower)
        self.assertRegex(lower, r"clear run_checks;\s*run_checks;")
        self.assertNotIn("interactive;", lower)
        assignments = re.findall(
            r"(?:altitudeGainSweep_rad_per_m|pitchFrequencySweep_radps)\s*=\s*\[([^\]]+)\]",
            experiment,
        )
        self.assertEqual(len(assignments), 2)
        for values_text in assignments:
            values = [
                float(value)
                for value in re.findall(
                    r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)", values_text
                )
            ]
            self.assertEqual(len(values), 5)
            self.assertTrue(all(math.isfinite(value) for value in values))

    def test_interaction_checks_recovery_and_resource_bounds(self) -> None:
        interactive = self.text["interactive.m"]
        checks_script = self.text["run_checks.m"]
        checks_doc = self.text["checks.md"]
        interactive_lower = interactive.lower()
        checks_lower = checks_script.lower()
        combined_checks = f"{checks_lower}\n{checks_doc.lower()}"
        interactive_compact = re.sub(
            r"\s+", "", interactive.replace("...", "")
        ).lower()
        checks_compact = re.sub(
            r"\s+", "", checks_script.replace("...", "")
        ).lower()
        self.assertTrue(interactive.startswith("function interactive\n"))
        self.assertTrue(checks_script.startswith("function run_checks\n"))
        self.assertIn("clear model;", "\n".join(interactive_lower.splitlines()[:8]))
        self.assertIn("clear model;", "\n".join(checks_lower.splitlines()[:6]))
        self.assertIn("p13 pitch and altitude hold explorer", interactive_lower)
        self.assertIn("existingui=findall(groot", interactive_compact)
        self.assertIn("close(existingui)", interactive_compact)
        self.assertEqual(interactive_lower.count("uislider("), 2)
        self.assertEqual(interactive_lower.count("uiswitch("), 1)
        self.assertEqual(interactive_lower.count("uibutton("), 1)
        self.assertIn("'limits',[00.008]", interactive_compact)
        self.assertIn("'limits',[1.23.6]", interactive_compact)
        self.assertIn("'value',0.004", interactive_compact)
        self.assertIn("'value',2.4", interactive_compact)
        self.assertIn("valuechangingfcn", interactive_lower)
        self.assertIn("valuechangedfcn", interactive_lower)
        self.assertIn("event.value", interactive_lower)
        self.assertIn("buttonpushedfcn", interactive_lower)
        self.assertIn("functionresetbaseline", interactive_compact)
        self.assertIn("gaincontrol.value=0.004", interactive_compact)
        self.assertIn("frequencycontrol.value=2.4", interactive_compact)
        self.assertIn("signcontrol.value='correct+1'", interactive_compact)
        self.assertIn("modelfcn=@model", interactive_compact)
        self.assertEqual(interactive_compact.count("out=modelfcn("), 1)
        self.assertIn("ifout.reachedninetypercent", interactive_compact)
        self.assertIn("capturetext='notreached'", interactive_compact)
        self.assertIn("90%%time%s", interactive_compact)
        self.assertGreaterEqual(interactive_compact.count("cla("), 4)
        self.assertNotIn("yyaxis", interactive_lower)

        self.assertGreaterEqual(checks_lower.count("assert("), 40)
        for concept in (
            "determinism",
            "fixed shapes",
            "finite resources",
            "signed baseline",
            "independently reconstruct",
            "every recurrence",
            "open-altitude-loop",
            "isolated parameter sweeps",
            "broken altitude/down sign",
            "malformed",
            "rejected inputs",
            "recovery",
            "rollback",
            "timeout",
            "cancellation",
            "compatibility",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, combined_checks)
        self.assertIn("samplecount==1501", checks_compact)
        self.assertIn("intervalcount==1500", checks_compact)
        self.assertIn("acceptedcornercount==8", checks_compact)
        self.assertIn("representativecasecount==18", checks_compact)
        self.assertIn("brokentailindices", checks_compact)
        self.assertIn("pitch-command saturation must not masquerade", checks_lower)
        self.assertIn("catch exception", checks_lower)
        self.assertIn("strcmp(exception.identifier,expectedidentifier)", checks_compact)
        self.assertIn("P13 checks passed", checks_script)

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
            "fsolve",
            "lsqnonlin",
            "fmincon",
            "solve",
            "vpasolve",
            "trim",
            "findop",
            "linearize",
            "sim",
            "load_system",
            "open_system",
            "ode45",
            "ode23",
            "ode15s",
            "readtable",
            "writetable",
            "webread",
            "urlread",
            "webwrite",
            "urlwrite",
            "fopen",
            "fread",
            "fwrite",
            "save",
            "load",
            "addpath",
            "rmpath",
            "rng",
            "rand",
            "randn",
            "timer",
            "parfeval",
            "parpool",
            "backgroundpool",
            "batch",
            "pause",
            "waitfor",
            "system",
            "unix",
            "dos",
            "input",
            "eval",
            "feval",
            "evalin",
            "assignin",
            "serialport",
            "tcpclient",
            "udpport",
        )
        for call in forbidden_calls:
            with self.subTest(call=call):
                self.assertNotRegex(matlab, rf"\b{re.escape(call)}\s*\(")
        self.assertNotRegex(
            matlab,
            re.compile(r"^\s*(?:global|persistent|parfor|while)\b", re.MULTILINE),
        )
        self.assertNotRegex(matlab, r"\bclose\s+all\b")
        self.assertNotIn("simulink", matlab)
        self.assertNotIn("control system toolbox", matlab)
        self.assertNotIn("aerospace toolbox", matlab)

    def test_retained_evidence_has_acceptance_map_and_claim_boundary(self) -> None:
        evidence = EVIDENCE_PATH.read_text(encoding="utf-8")
        self.assertTrue(evidence.endswith("\n"))
        self.assertFalse(evidence.endswith("\n\n"))
        self.assertNotIn("LIFECYCLE_GATE_RESULTS", evidence)
        self.assertNotIn("FINAL_VALIDATION_RESULTS", evidence)
        for heading in (
            "## Result and claim boundary",
            "## Acceptance mapping",
            "## Exact validation performed",
            "## Changed and preserved invariants",
            "## Residual risks and limitations",
            "## Rollback",
            "## Explicitly unperformed validation",
            "## Schema-aligned evidence summary",
        ):
            self.assertIn(heading, evidence)
        self.assertEqual(len(re.findall(r"^\| A[1-8] \|", evidence, re.MULTILINE)), 8)
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
        self.assertEqual(summary["batch_id"], "P13")
        self.assertEqual(summary["status"], "pass")
        self.assertEqual(len(summary["acceptance"]), 8)
        self.assertTrue(
            all(item["status"] == "pass" for item in summary["acceptance"])
        )


class P13EquationOracleTests(unittest.TestCase):
    def test_deterministic_baseline_signature_and_fixed_shape(self) -> None:
        first = _oracle()
        second = _oracle(0.004, 2.4, 1)
        self.assertEqual(first, second)
        self.assertEqual(first["sample_count"], 1501)
        self.assertEqual(first["interval_count"], 1500)
        self.assertEqual(first["active_sample_count"], 1451)
        self.assertEqual(first["time_s"][0], 0.0)
        self.assertEqual(first["time_s"][-1], 30.0)
        self.assertEqual(first["altitude_command_m"][49], 1000.0)
        self.assertEqual(first["altitude_command_m"][50], 1030.0)
        self.assertEqual(first["altitude_m"][50], 1000.0)
        self.assertAlmostEqual(first["pitch_command_deg"][50], 6.875493541569878, 12)
        self.assertAlmostEqual(
            first["pitch_control_command_deg"][50], 3.300236899953542, 12
        )
        self.assertAlmostEqual(first["pitch_angle_deg"][75], 2.590689002929238, 12)
        self.assertAlmostEqual(
            first["flight_path_angle_deg"][75], 0.292884513830748, 12
        )
        self.assertAlmostEqual(first["altitude_error_m"][250], 16.547538451269475, 10)
        self.assertAlmostEqual(first["altitude_m"][-1], 1029.9897035680115, 9)
        self.assertAlmostEqual(first["final_altitude_error_m"], 0.010296431988536, 9)
        self.assertAlmostEqual(first["peak_altitude_overshoot_m"], 1.575586453233427, 9)
        self.assertAlmostEqual(first["time_to_ninety_percent_s"], 7.28, 12)
        self.assertAlmostEqual(first["pitch_tracking_rms_deg"], 0.932364804504502, 12)

    def test_every_controller_equation_state_update_and_down_identity(self) -> None:
        result = _oracle()
        dt = result["sample_time_s"]
        for index in range(result["sample_count"]):
            with self.subTest(sample=index):
                self.assertAlmostEqual(
                    result["altitude_error_m"][index],
                    result["altitude_command_m"][index]
                    - result["altitude_m"][index],
                    places=12,
                )
                self.assertEqual(
                    result["down_command_m"][index],
                    -result["altitude_command_m"][index],
                )
                self.assertEqual(
                    result["down_position_m"][index],
                    -result["altitude_m"][index],
                )
                self.assertAlmostEqual(
                    result["down_error_m"][index],
                    -result["altitude_error_m"][index],
                    places=12,
                )
                expected_climb = result["true_airspeed_mps"] * math.sin(
                    math.radians(result["flight_path_angle_deg"][index])
                )
                self.assertAlmostEqual(
                    result["climb_rate_mps"][index], expected_climb, places=12
                )
                if index == result["interval_count"]:
                    continue
                self.assertAlmostEqual(
                    result["altitude_m"][index + 1],
                    result["altitude_m"][index]
                    + dt * result["climb_rate_mps"][index],
                    places=12,
                )
                self.assertAlmostEqual(
                    result["pitch_angle_deg"][index + 1],
                    result["pitch_angle_deg"][index]
                    + dt * result["pitch_rate_degps"][index],
                    places=12,
                )
                self.assertAlmostEqual(
                    result["pitch_rate_degps"][index + 1],
                    result["pitch_rate_degps"][index]
                    + dt * result["pitch_acceleration_degps2"][index],
                    places=11,
                )
                self.assertAlmostEqual(
                    result["flight_path_angle_deg"][index + 1],
                    result["flight_path_angle_deg"][index]
                    + dt * result["flight_path_rate_degps"][index],
                    places=12,
                )

    def test_gain_schedule_has_declared_unsaturated_closed_loop_coefficients(self) -> None:
        for frequency in (1.2, 1.8, 2.4, 3.0, 3.6):
            result = _oracle(0.004, frequency, 1)
            with self.subTest(frequency=frequency):
                b_u = result["pitch_control_effectiveness_per_s2"]
                self.assertAlmostEqual(
                    result["pitch_stiffness_per_s2"]
                    + b_u * result["pitch_proportional_gain"],
                    frequency**2,
                    places=14,
                )
                self.assertAlmostEqual(
                    b_u
                    * (
                        result["pitch_proportional_gain"]
                        + result["pitch_command_feedforward_gain"]
                    ),
                    frequency**2,
                    places=14,
                )
                self.assertAlmostEqual(
                    result["pitch_rate_damping_per_s"]
                    + b_u * result["pitch_rate_gain_s"],
                    2.0 * result["pitch_damping_ratio"] * frequency,
                    places=14,
                )

    def test_zero_outer_gain_is_exact_open_loop_limit(self) -> None:
        result = _oracle(0.0, 2.4, 1)
        self.assertTrue(all(value == 1000.0 for value in result["altitude_m"]))
        self.assertTrue(all(value == -1000.0 for value in result["down_position_m"]))
        for key in (
            "pitch_command_deg",
            "pitch_control_command_deg",
            "pitch_angle_deg",
            "pitch_rate_degps",
            "flight_path_angle_deg",
            "climb_rate_mps",
        ):
            self.assertTrue(all(value == 0.0 for value in result[key]), key)
        self.assertEqual(result["final_altitude_error_m"], 30.0)
        self.assertFalse(result["reached_ninety_percent"])
        self.assertFalse(result["settled_by_end"])

    def test_altitude_gain_sweep_isolated_tradeoffs(self) -> None:
        gains = (0.0, 0.002, 0.004, 0.006, 0.008)
        results = tuple(_oracle(gain, 2.4, 1) for gain in gains)
        errors_at_five = tuple(result["altitude_error_m"][250] for result in results)
        expected_errors = (
            30.0,
            23.045935037884874,
            16.547538451269475,
            10.699119009948845,
            9.439161747247454,
        )
        for result, expected in zip(results, expected_errors):
            self.assertAlmostEqual(result["altitude_error_m"][250], expected, 10)
            self.assertEqual(result["pitch_natural_frequency_radps"], 2.4)
            self.assertEqual(result["altitude_feedback_sign"], 1)
            self.assertEqual(result["time_s"], results[0]["time_s"])
            self.assertEqual(
                result["altitude_command_m"], results[0]["altitude_command_m"]
            )
        self.assertTrue(
            all(left > right for left, right in zip(errors_at_five, errors_at_five[1:]))
        )
        overshoot = tuple(result["peak_altitude_overshoot_m"] for result in results)
        self.assertEqual(overshoot[0], 0.0)
        self.assertGreater(overshoot[-1], overshoot[2])
        self.assertEqual(results[0]["pitch_command_saturation_fraction"], 0.0)
        self.assertGreater(results[-1]["pitch_command_saturation_fraction"], 0.0)

    def test_pitch_frequency_sweep_isolated_tracking_authority_trade(self) -> None:
        frequencies = (1.2, 1.8, 2.4, 3.0, 3.6)
        results = tuple(_oracle(0.004, frequency, 1) for frequency in frequencies)
        pitch_at_one_point_five = tuple(result["pitch_angle_deg"][75] for result in results)
        tracking_rms = tuple(result["pitch_tracking_rms_deg"] for result in results)
        peak_control = tuple(
            result["peak_pitch_control_command_deg"] for result in results
        )
        self.assertTrue(
            all(
                left < right
                for left, right in zip(
                    pitch_at_one_point_five, pitch_at_one_point_five[1:]
                )
            )
        )
        self.assertTrue(
            all(left > right for left, right in zip(tracking_rms, tracking_rms[1:]))
        )
        self.assertTrue(
            all(left < right for left, right in zip(peak_control, peak_control[1:]))
        )
        for result in results:
            self.assertEqual(result["altitude_gain_rad_per_m"], 0.004)
            self.assertEqual(result["altitude_feedback_sign"], 1)
            self.assertEqual(result["pitch_damping_ratio"], 0.8)
            self.assertEqual(result["flight_path_time_constant_s"], 1.5)

    def test_broken_feedback_sign_is_isolated_finite_and_recoverable(self) -> None:
        correct = _oracle()
        broken = _oracle(0.004, 2.4, -1)
        command_index = 50
        for key in (
            "altitude_m",
            "pitch_angle_deg",
            "pitch_rate_degps",
            "flight_path_angle_deg",
        ):
            self.assertEqual(
                broken[key][: command_index + 1], correct[key][: command_index + 1]
            )
        self.assertEqual(broken["altitude_gain_rad_per_m"], correct["altitude_gain_rad_per_m"])
        self.assertEqual(
            broken["pitch_natural_frequency_radps"],
            correct["pitch_natural_frequency_radps"],
        )
        self.assertEqual(broken["altitude_feedback_sign"], -1)
        self.assertEqual(
            broken["pitch_command_deg"][command_index],
            -correct["pitch_command_deg"][command_index],
        )
        self.assertEqual(
            broken["pitch_control_command_deg"][command_index],
            -correct["pitch_control_command_deg"][command_index],
        )
        self.assertAlmostEqual(broken["altitude_m"][-1], 728.837776591105, 9)
        self.assertAlmostEqual(broken["final_altitude_error_m"], 301.162223408895, 9)
        self.assertGreater(broken["pitch_command_saturation_fraction"], 0.8)
        self.assertLessEqual(max(map(abs, broken["pitch_command_deg"])), 10.0)
        self.assertLessEqual(
            max(map(abs, broken["pitch_control_command_deg"])), 20.0
        )
        self.assertEqual(_oracle(), correct)

    def test_broken_saturation_does_not_arrest_altitude_error_growth(self) -> None:
        broken = _oracle(0.004, 2.4, -1)
        command_index = 50
        tail_start = broken["sample_count"] - 51
        error = broken["altitude_error_m"]
        tail_error = error[tail_start:]

        self.assertTrue(
            all(
                right >= left
                for left, right in zip(
                    error[command_index:], error[command_index + 1 :]
                )
            )
        )
        self.assertTrue(all(broken["pitch_command_saturated"][tail_start:]))
        self.assertTrue(
            all(left < right for left, right in zip(tail_error, tail_error[1:]))
        )
        self.assertGreater(tail_error[-1] - tail_error[0], 10.0)
        for index in range(tail_start, broken["interval_count"]):
            self.assertAlmostEqual(
                error[index + 1] - error[index],
                -broken["sample_time_s"] * broken["climb_rate_mps"][index],
                places=11,
            )
        self.assertLess(broken["climb_rate_mps"][-1], -10.0)

    def test_malformed_inputs_reject_without_poisoning_recovery(self) -> None:
        malformed = (
            (-1e-12, 2.4, 1),
            (0.008000000001, 2.4, 1),
            (0.004, 1.199999999, 1),
            (0.004, 3.600000001, 1),
            (0.004, 2.4, 0),
            (0.004, 2.4, 2),
            ([0.003, 0.004], 2.4, 1),
            (0.004, [2.0, 2.4], 1),
            (0.004, 2.4, [1, -1]),
            (0.004 + 1.0j, 2.4, 1),
            (0.004, 2.4 + 1.0j, 1),
            (float("nan"), 2.4, 1),
            (0.004, float("inf"), 1),
            (0.004, 2.4, float("nan")),
            (True, 2.4, 1),
        )
        for gain, frequency, sign in malformed:
            with self.subTest(gain=gain, frequency=frequency, sign=sign):
                with self.assertRaises(ValueError):
                    _oracle(gain, frequency, sign)
        self.assertEqual(_oracle(), _oracle(0.004, 2.4, 1))

    def test_accepted_corners_and_representative_grid_are_finite_and_fixed(self) -> None:
        corners = tuple(
            _oracle(gain, frequency, sign)
            for gain in (0.0, 0.008)
            for frequency in (1.2, 3.6)
            for sign in (-1, 1)
        )
        self.assertEqual(len(corners), 8)
        grid = tuple(
            _oracle(gain, frequency, sign)
            for gain in (0.0, 0.004, 0.008)
            for frequency in (1.2, 2.4, 3.6)
            for sign in (-1, 1)
        )
        self.assertEqual(len(grid), 18)
        for result in corners + grid:
            self.assertEqual(result["sample_count"], 1501)
            self.assertEqual(result["interval_count"], 1500)
            self.assertFalse(any(result["pitch_control_command_saturated"]))
            for key in (
                "altitude_m",
                "pitch_angle_deg",
                "pitch_rate_degps",
                "flight_path_angle_deg",
                "pitch_command_deg",
                "pitch_control_command_deg",
            ):
                self.assertTrue(all(math.isfinite(value) for value in result[key]))
            self.assertLessEqual(max(map(abs, result["pitch_command_deg"])), 10.0)
            self.assertLessEqual(
                max(map(abs, result["pitch_control_command_deg"])), 20.0
            )


if __name__ == "__main__":
    unittest.main()
