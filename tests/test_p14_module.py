from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_ID = "P14"
MODULE_FOLDER = ROOT / "modules/14-hold-roll-and-heading"
EVIDENCE_PATH = ROOT / "docs/evidence/P14-2026-08-23.md"
GUIDING_QUESTION = (
    "What inputs, observable effects, and failure modes matter when you hold "
    "Roll and Heading?"
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


def _heading_error_mode(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("heading error mode must be 0 or 1")
    result = float(value)
    if not math.isfinite(result) or result not in (0.0, 1.0):
        raise ValueError("heading error mode must be 0 or 1")
    return int(result)


def _wrap_radians(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def _oracle(
    heading_gain_bank_per_heading: object = 0.5,
    roll_natural_frequency_radps: object = 2.4,
    heading_error_mode: object = 1,
) -> dict[str, object]:
    """Independent standard-library implementation of the declared equations."""
    heading_gain = _bounded_scalar(
        "heading-to-bank gain", heading_gain_bank_per_heading, 0.0, 1.0
    )
    roll_frequency = _bounded_scalar(
        "roll natural frequency", roll_natural_frequency_radps, 1.2, 3.6
    )
    error_mode = _heading_error_mode(heading_error_mode)

    sample_time_s = 0.02
    time_s = tuple(index * sample_time_s for index in range(3001))
    sample_count = len(time_s)
    command_step_time_s = 1.0
    initial_heading_rad = math.radians(170.0)
    commanded_heading_rad = math.radians(-170.0)
    shortest_command_step_rad = _wrap_radians(
        commanded_heading_rad - initial_heading_rad
    )
    nearest_continuous_target_rad = (
        initial_heading_rad + shortest_command_step_rad
    )
    true_airspeed_mps = 60.0
    gravity_mps2 = 9.80665
    roll_damping_ratio = 0.8
    bank_command_limit_rad = math.radians(12.0)
    bank_teaching_envelope_deg = 15.0

    heading_command_wrapped_rad = tuple(
        initial_heading_rad if time < command_step_time_s else commanded_heading_rad
        for time in time_s
    )
    heading_command_continuous_rad = tuple(
        initial_heading_rad
        if time < command_step_time_s
        else nearest_continuous_target_rad
        for time in time_s
    )
    heading_unwrapped_rad = [0.0] * sample_count
    heading_wrapped_rad = [0.0] * sample_count
    raw_heading_error_rad = [0.0] * sample_count
    shortest_heading_error_rad = [0.0] * sample_count
    heading_error_used_rad = [0.0] * sample_count
    bank_command_unclamped_rad = [0.0] * sample_count
    bank_command_rad = [0.0] * sample_count
    bank_angle_rad = [0.0] * sample_count
    bank_rate_radps = [0.0] * sample_count
    bank_acceleration_radps2 = [0.0] * sample_count
    heading_rate_radps = [0.0] * sample_count
    bank_command_saturated = [False] * sample_count
    heading_unwrapped_rad[0] = initial_heading_rad

    for index in range(sample_count):
        heading_wrapped_rad[index] = _wrap_radians(heading_unwrapped_rad[index])
        raw_heading_error_rad[index] = (
            heading_command_wrapped_rad[index] - heading_wrapped_rad[index]
        )
        shortest_heading_error_rad[index] = _wrap_radians(
            raw_heading_error_rad[index]
        )
        heading_error_used_rad[index] = (
            shortest_heading_error_rad[index]
            if error_mode == 1
            else raw_heading_error_rad[index]
        )
        bank_command_unclamped_rad[index] = (
            heading_gain * heading_error_used_rad[index]
        )
        bank_command_rad[index] = _clamp(
            bank_command_unclamped_rad[index],
            -bank_command_limit_rad,
            bank_command_limit_rad,
        )
        bank_command_saturated[index] = (
            abs(bank_command_unclamped_rad[index]) > bank_command_limit_rad
        )
        bank_acceleration_radps2[index] = (
            roll_frequency**2
            * (bank_command_rad[index] - bank_angle_rad[index])
            - 2.0
            * roll_damping_ratio
            * roll_frequency
            * bank_rate_radps[index]
        )
        heading_rate_radps[index] = (
            gravity_mps2
            / true_airspeed_mps
            * math.tan(bank_angle_rad[index])
        )

        if index < sample_count - 1:
            heading_unwrapped_rad[index + 1] = (
                heading_unwrapped_rad[index]
                + sample_time_s * heading_rate_radps[index]
            )
            bank_angle_rad[index + 1] = (
                bank_angle_rad[index] + sample_time_s * bank_rate_radps[index]
            )
            bank_rate_radps[index + 1] = (
                bank_rate_radps[index]
                + sample_time_s * bank_acceleration_radps2[index]
            )

    heading_command_wrapped_deg = tuple(
        map(math.degrees, heading_command_wrapped_rad)
    )
    heading_command_continuous_deg = tuple(
        map(math.degrees, heading_command_continuous_rad)
    )
    heading_unwrapped_deg = tuple(map(math.degrees, heading_unwrapped_rad))
    heading_wrapped_deg = tuple(map(math.degrees, heading_wrapped_rad))
    raw_heading_error_deg = tuple(map(math.degrees, raw_heading_error_rad))
    shortest_heading_error_deg = tuple(
        map(math.degrees, shortest_heading_error_rad)
    )
    heading_error_used_deg = tuple(map(math.degrees, heading_error_used_rad))
    bank_command_unclamped_deg = tuple(
        map(math.degrees, bank_command_unclamped_rad)
    )
    bank_command_deg = tuple(map(math.degrees, bank_command_rad))
    bank_angle_deg = tuple(map(math.degrees, bank_angle_rad))
    bank_rate_degps = tuple(map(math.degrees, bank_rate_radps))
    bank_acceleration_degps2 = tuple(
        map(math.degrees, bank_acceleration_radps2)
    )
    heading_rate_degps = tuple(map(math.degrees, heading_rate_radps))
    heading_change_deg = tuple(
        value - math.degrees(initial_heading_rad)
        for value in heading_unwrapped_deg
    )
    active_indices = tuple(
        index for index, time in enumerate(time_s) if time >= command_step_time_s
    )
    bank_tracking_error_deg = tuple(
        command - angle
        for command, angle in zip(bank_command_deg, bank_angle_deg)
    )
    bank_tracking_rms_deg = math.sqrt(
        sum(bank_tracking_error_deg[index] ** 2 for index in active_indices)
        / len(active_indices)
    )
    capture_tolerance_deg = 0.1 * abs(math.degrees(shortest_command_step_rad))
    capture_index = next(
        (
            index
            for index in active_indices
            if abs(shortest_heading_error_deg[index]) <= capture_tolerance_deg
        ),
        None,
    )
    reached_ninety_percent = capture_index is not None
    time_to_ninety_percent_s = (
        time_s[capture_index] - command_step_time_s
        if capture_index is not None
        else time_s[-1] - command_step_time_s
    )
    settling_tolerance_deg = 0.02 * abs(
        math.degrees(shortest_command_step_rad)
    )
    final_shortest_heading_error_deg = shortest_heading_error_deg[-1]
    settled_by_end = (
        abs(final_shortest_heading_error_deg) <= settling_tolerance_deg
    )
    signed_heading_travel_deg = heading_change_deg[-1]
    absolute_heading_travel_deg = sum(
        abs(right - left)
        for left, right in zip(heading_unwrapped_deg, heading_unwrapped_deg[1:])
    )
    correct_turn_direction = 1 if shortest_command_step_rad > 0.0 else -1
    wrong_way_travel_deg = max(
        0.0, -correct_turn_direction * signed_heading_travel_deg
    )

    return {
        "heading_gain_bank_per_heading": heading_gain,
        "roll_natural_frequency_radps": roll_frequency,
        "heading_error_mode": error_mode,
        "sample_time_s": sample_time_s,
        "time_s": time_s,
        "sample_count": sample_count,
        "interval_count": sample_count - 1,
        "active_sample_count": len(active_indices),
        "command_step_time_s": command_step_time_s,
        "initial_heading_deg": math.degrees(initial_heading_rad),
        "commanded_heading_deg": math.degrees(commanded_heading_rad),
        "shortest_command_step_deg": math.degrees(shortest_command_step_rad),
        "nearest_continuous_target_deg": math.degrees(
            nearest_continuous_target_rad
        ),
        "true_airspeed_mps": true_airspeed_mps,
        "gravity_mps2": gravity_mps2,
        "roll_damping_ratio": roll_damping_ratio,
        "bank_command_limit_deg": 12.0,
        "bank_teaching_envelope_deg": bank_teaching_envelope_deg,
        "heading_command_wrapped_deg": heading_command_wrapped_deg,
        "heading_command_continuous_deg": heading_command_continuous_deg,
        "heading_unwrapped_deg": heading_unwrapped_deg,
        "heading_wrapped_deg": heading_wrapped_deg,
        "heading_change_deg": heading_change_deg,
        "raw_heading_error_deg": raw_heading_error_deg,
        "shortest_heading_error_deg": shortest_heading_error_deg,
        "heading_error_used_deg": heading_error_used_deg,
        "bank_command_unclamped_deg": bank_command_unclamped_deg,
        "bank_command_deg": bank_command_deg,
        "bank_command_saturated": tuple(bank_command_saturated),
        "bank_command_saturation_fraction": (
            sum(bank_command_saturated) / sample_count
        ),
        "bank_angle_deg": bank_angle_deg,
        "bank_rate_degps": bank_rate_degps,
        "bank_acceleration_degps2": bank_acceleration_degps2,
        "heading_rate_degps": heading_rate_degps,
        "bank_tracking_error_deg": bank_tracking_error_deg,
        "bank_tracking_rms_deg": bank_tracking_rms_deg,
        "heading_error_at_ten_seconds_deg": shortest_heading_error_deg[500],
        "final_shortest_heading_error_deg": final_shortest_heading_error_deg,
        "capture_tolerance_deg": capture_tolerance_deg,
        "reached_ninety_percent": reached_ninety_percent,
        "time_to_ninety_percent_s": time_to_ninety_percent_s,
        "settling_tolerance_deg": settling_tolerance_deg,
        "settled_by_end": settled_by_end,
        "signed_heading_travel_deg": signed_heading_travel_deg,
        "absolute_heading_travel_deg": absolute_heading_travel_deg,
        "wrong_way_travel_deg": wrong_way_travel_deg,
        "peak_absolute_shortest_heading_error_deg": max(
            abs(shortest_heading_error_deg[index]) for index in active_indices
        ),
        "peak_bank_angle_deg": max(map(abs, bank_angle_deg)),
        "peak_bank_rate_degps": max(map(abs, bank_rate_degps)),
        "peak_bank_acceleration_degps2": max(
            map(abs, bank_acceleration_degps2)
        ),
    }


class P14ArtifactTests(unittest.TestCase):
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
                "number": 14,
                "id": "P14",
                "title": "Hold Roll and Heading",
                "guiding_question": GUIDING_QUESTION,
                "phase": 4,
                "phase_title": "Autopilots",
                "slug": "hold-roll-and-heading",
                "folder": "modules/14-hold-roll-and-heading",
                "implementation_batch": "P14",
                "prerequisites": ["P13"],
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
            "p13",
            "outer loop",
            "inner loop",
            "shortest",
            "circular heading error",
            "bank command",
            "right-wing-down",
            "coordinated",
            "mechanism",
            "reset",
            "broken",
            "open-heading-loop",
            "teach-back",
            "fixed speed",
            "branch cut",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, combined)
        self.assertRegex(walkthrough.lower(), r"one plot|one .* at a time")
        self.assertIn("conceptual rather than current api compatibility", combined)
        self.assertIn("does not accept a p13", combined)
        self.assertIn("not positive feedback", combined)
        self.assertIn("does not make the selected route correct", combined)
        self.assertIn("p15", combined)
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
            "functionout=model(headinggain_bank_per_heading,"
            "rollnaturalfrequency_radps,headingerrormode)",
            compact,
        )
        self.assertIn("arguments", lower)
        self.assertIn(
            "headinggain_bank_per_heading(1,1)double"
            "{mustbereal,mustbefinite}=0.5",
            compact,
        )
        self.assertIn(
            "rollnaturalfrequency_radps(1,1)double"
            "{mustbereal,mustbefinite}=2.4",
            compact,
        )
        self.assertIn(
            "headingerrormode(1,1)double{mustbereal,mustbefinite}=1",
            compact,
        )
        self.assertIn(
            "headinggain_bank_per_heading<0||headinggain_bank_per_heading>1",
            compact,
        )
        self.assertIn(
            "rollnaturalfrequency_radps<1.2||rollnaturalfrequency_radps>3.6",
            compact,
        )
        self.assertIn("headingerrormode~=0&&headingerrormode~=1", compact)
        for identifier in (
            "P14:model:HeadingGainRange",
            "P14:model:RollFrequencyRange",
            "P14:model:HeadingErrorMode",
        ):
            self.assertIn(identifier, model)
        for expression in (
            "sampletime_s=0.02;",
            "timehorizon_s=60;",
            "time_s=0:sampletime_s:timehorizon_s;",
            "commandsteptime_s=1;",
            "initialheading_rad=deg2rad(170);",
            "commandedheading_rad=deg2rad(-170);",
            "trueairspeed_mps=60;",
            "gravity_mps2=9.80665;",
            "rolldampingratio=0.8;",
            "bankcommandlimit_rad=deg2rad(12);",
            "bankteachingenvelope_rad=deg2rad(15);",
            "shortestcommandstep_rad=mod(commandedheading_rad-initialheading_rad+pi,2*pi)-pi;",
            "headingwrapped_rad(k)=mod(headingunwrapped_rad(k)+pi,2*pi)-pi;",
            "rawheadingerror_rad(k)=headingcommandwrapped_rad(k)-headingwrapped_rad(k);",
            "shortestheadingerror_rad(k)=mod(rawheadingerror_rad(k)+pi,2*pi)-pi;",
            "bankcommandunclamped_rad(k)=headinggain_bank_per_heading*headingerrorused_rad(k);",
            "bankcommand_rad(k)=min(max(bankcommandunclamped_rad(k),-bankcommandlimit_rad),bankcommandlimit_rad);",
            "bankacceleration_radps2(k)=rollnaturalfrequency_radps^2*(bankcommand_rad(k)-bankangle_rad(k))-2*rolldampingratio*rollnaturalfrequency_radps*bankrate_radps(k);",
            "headingrate_radps(k)=gravity_mps2/trueairspeed_mps*tan(bankangle_rad(k));",
            "headingunwrapped_rad(k+1)=headingunwrapped_rad(k)+sampletime_s*headingrate_radps(k);",
            "bankangle_rad(k+1)=bankangle_rad(k)+sampletime_s*bankrate_radps(k);",
            "bankrate_radps(k+1)=bankrate_radps(k)+sampletime_s*bankacceleration_radps2(k);",
        ):
            with self.subTest(expression=expression):
                self.assertIn(expression, compact)
        mode_routing = re.compile(
            r"if\s+headingErrorMode==1\s+"
            r"headingErrorUsed_rad\(k\)=shortestHeadingError_rad\(k\);\s+"
            r"else.*?headingErrorUsed_rad\(k\)=rawHeadingError_rad\(k\);\s+end",
            re.DOTALL,
        )
        self.assertRegex(model, mode_routing)
        for metric_expression in (
            "bankcommandsaturated=abs(bankcommandunclamped_rad)>bankcommandlimit_rad;",
            "activemask=time_s>=commandsteptime_s;",
            "banktrackingrms_deg=sqrt(mean(banktrackingerror_deg(activemask).^2));",
            "finalshortestheadingerror_deg=rad2deg(shortestheadingerror_rad(end));",
            "capturetolerance_deg=0.1*abs(rad2deg(shortestcommandstep_rad));",
            "headingchange_deg=rad2deg(headingunwrapped_rad-initialheading_rad);",
            "signedheadingtravel_deg=headingchange_deg(end);",
            "absoluteheadingtravel_deg=sum(abs(diff(rad2deg(headingunwrapped_rad))));",
            "wrongwaytravel_deg=max(0,-correctturndirection*signedheadingtravel_deg);",
        ):
            with self.subTest(metric=metric_expression):
                self.assertIn(metric_expression, compact)
        for field in (
            "headingCommandWrapped_deg",
            "headingCommandContinuous_deg",
            "headingUnwrapped_deg",
            "headingWrapped_deg",
            "rawHeadingError_deg",
            "shortestHeadingError_deg",
            "headingErrorUsed_deg",
            "bankCommand_deg",
            "bankAngle_deg",
            "bankRate_degps",
            "bankAcceleration_degps2",
            "headingRate_degps",
            "controllerEquation",
            "rollEquation",
            "turnEquation",
            "headingConvention",
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
        self.assertNotRegex(
            lower, re.compile(r"^\s*(?:while|parfor)\b", re.MULTILINE)
        )

    def test_experiment_has_two_isolated_sweeps_and_broken_case(self) -> None:
        experiment = self.text["experiment.m"]
        lower = experiment.lower()
        compact = re.sub(r"\s+", "", experiment.replace("...", "")).lower()
        self.assertGreaterEqual(experiment.count("%%"), 15)
        self.assertGreaterEqual(lower.count("figure("), 5)
        self.assertGreaterEqual(lower.count("assert("), 7)
        for concept in (
            "baseline",
            "heading-to-bank gain",
            "roll natural frequency",
            "changed view",
            "mechanism",
            "limiting case",
            "broken",
            "raw subtraction",
            "shortest path",
        ):
            self.assertIn(concept, lower)
        for unit in ("rad/rad", "rad/s", "deg/s^2", "deg/s", "m/s", "deg"):
            self.assertIn(unit, lower)
        self.assertIn("baseline=model(0.5,2.4,1)", compact)
        self.assertIn(
            "model(headinggainsweep_bank_per_heading(k),2.4,1)", compact
        )
        self.assertIn("model(0.5,rollfrequencysweep_radps(k),1)", compact)
        self.assertIn("openheadingloop=model(0,2.4,1)", compact)
        self.assertIn("broken=model(0.5,2.4,0)", compact)
        self.assertNotRegex(lower, r"\bclose\s+all\b")
        self.assertIn("findall(groot", lower)
        self.assertIn("'^p14 '", lower)
        self.assertRegex(lower, r"clear run_checks;\s*run_checks;")
        self.assertNotIn("interactive;", lower)
        assignments = re.findall(
            r"(?:headingGainSweep_bank_per_heading|rollFrequencySweep_radps)"
            r"\s*=\s*\[([^\]]+)\]",
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
        self.assertIn("p14 roll and heading hold explorer", interactive_lower)
        self.assertIn("existingui=findall(groot", interactive_compact)
        self.assertIn("close(existingui)", interactive_compact)
        self.assertEqual(interactive_lower.count("uislider("), 2)
        self.assertEqual(interactive_lower.count("uiswitch("), 1)
        self.assertEqual(interactive_lower.count("uibutton("), 1)
        self.assertIn("'limits',[01]", interactive_compact)
        self.assertIn("'limits',[1.23.6]", interactive_compact)
        self.assertIn("'value',0.5", interactive_compact)
        self.assertIn("'value',2.4", interactive_compact)
        self.assertIn("valuechangingfcn", interactive_lower)
        self.assertIn("valuechangedfcn", interactive_lower)
        self.assertIn("event.value", interactive_lower)
        self.assertIn("buttonpushedfcn", interactive_lower)
        self.assertIn("functionresetbaseline", interactive_compact)
        self.assertIn("gaincontrol.value=0.5", interactive_compact)
        self.assertIn("frequencycontrol.value=2.4", interactive_compact)
        self.assertIn(
            "modecontrol.value='wrappedshortestpath'", interactive_compact
        )
        self.assertIn("modelfcn=@model", interactive_compact)
        self.assertEqual(interactive_compact.count("out=modelfcn("), 1)
        self.assertIn("ifout.reachedninetypercent", interactive_compact)
        self.assertIn("capturetext='notreached'", interactive_compact)
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
            "branch-cut continuity",
            "open-heading-loop",
            "isolated parameter sweeps",
            "broken raw subtraction",
            "malformed",
            "rejected inputs",
            "recovery",
            "rollback",
            "timeout",
            "cancellation",
            "compatibility",
            "migration",
            "backup",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, combined_checks)
        self.assertIn("samplecount==3001", checks_compact)
        self.assertIn("intervalcount==3000", checks_compact)
        self.assertIn("acceptedcornercount==8", checks_compact)
        self.assertIn("representativecasecount==18", checks_compact)
        self.assertIn("brokentailindices", checks_compact)
        self.assertIn("brokenheadingtailintervals", checks_compact)
        self.assertIn("catch exception", checks_lower)
        self.assertIn(
            "strcmp(exception.identifier,expectedidentifier)", checks_compact
        )
        self.assertIn("P14 checks passed", checks_script)

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
            "wrapto180",
            "wraptopi",
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
        self.assertNotIn("mapping toolbox", matlab)

    def test_retained_evidence_has_acceptance_map_and_claim_boundary(self) -> None:
        evidence = EVIDENCE_PATH.read_text(encoding="utf-8")
        self.assertTrue(evidence.endswith("\n"))
        self.assertFalse(evidence.endswith("\n\n"))
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
        self.assertEqual(summary["batch_id"], "P14")
        self.assertEqual(summary["status"], "pass")
        self.assertEqual(len(summary["acceptance"]), 8)
        self.assertTrue(
            all(item["status"] == "pass" for item in summary["acceptance"])
        )


class P14EquationOracleTests(unittest.TestCase):
    def test_deterministic_baseline_signature_and_fixed_shape(self) -> None:
        first = _oracle()
        second = _oracle(0.5, 2.4, 1)
        self.assertEqual(first, second)
        self.assertEqual(first["sample_count"], 3001)
        self.assertEqual(first["interval_count"], 3000)
        self.assertEqual(first["active_sample_count"], 2951)
        self.assertEqual(first["time_s"][0], 0.0)
        self.assertEqual(first["time_s"][-1], 60.0)
        self.assertAlmostEqual(first["shortest_command_step_deg"], 20.0, 12)
        self.assertAlmostEqual(first["nearest_continuous_target_deg"], 190.0, 12)
        self.assertAlmostEqual(first["raw_heading_error_deg"][50], -340.0, 12)
        self.assertAlmostEqual(
            first["shortest_heading_error_deg"][50], 20.0, 12
        )
        self.assertAlmostEqual(first["bank_command_deg"][50], 10.0, 12)
        self.assertAlmostEqual(
            first["bank_acceleration_degps2"][50], 57.6, 11
        )
        self.assertAlmostEqual(first["bank_angle_deg"][75], 3.765653541420135, 10)
        self.assertAlmostEqual(
            first["heading_rate_degps"][75], 0.616361823037713, 10
        )
        self.assertAlmostEqual(
            first["heading_error_at_ten_seconds_deg"],
            9.693404960655482,
            9,
        )
        self.assertAlmostEqual(
            first["final_shortest_heading_error_deg"],
            0.126913314957683,
            9,
        )
        self.assertAlmostEqual(first["time_to_ninety_percent_s"], 27.2, 12)
        self.assertAlmostEqual(
            first["bank_tracking_rms_deg"], 0.905378979330061, 9
        )
        self.assertAlmostEqual(first["peak_bank_angle_deg"], 9.691004143934801, 9)
        self.assertTrue(first["settled_by_end"])
        self.assertFalse(any(first["bank_command_saturated"]))

    def test_every_wrap_controller_turn_and_state_update(self) -> None:
        result = _oracle()
        dt = result["sample_time_s"]
        for index in range(result["sample_count"]):
            with self.subTest(sample=index):
                expected_wrapped = math.degrees(
                    _wrap_radians(math.radians(result["heading_unwrapped_deg"][index]))
                )
                self.assertAlmostEqual(
                    result["heading_wrapped_deg"][index], expected_wrapped, places=12
                )
                expected_raw = (
                    result["heading_command_wrapped_deg"][index]
                    - result["heading_wrapped_deg"][index]
                )
                expected_shortest = math.degrees(
                    _wrap_radians(math.radians(expected_raw))
                )
                self.assertAlmostEqual(
                    result["raw_heading_error_deg"][index], expected_raw, places=12
                )
                self.assertAlmostEqual(
                    result["shortest_heading_error_deg"][index],
                    expected_shortest,
                    places=12,
                )
                self.assertAlmostEqual(
                    result["heading_error_used_deg"][index],
                    expected_shortest,
                    places=12,
                )
                expected_bank_command = _clamp(
                    result["heading_gain_bank_per_heading"] * expected_shortest,
                    -result["bank_command_limit_deg"],
                    result["bank_command_limit_deg"],
                )
                self.assertAlmostEqual(
                    result["bank_command_deg"][index],
                    expected_bank_command,
                    places=12,
                )
                expected_acceleration = (
                    result["roll_natural_frequency_radps"] ** 2
                    * (expected_bank_command - result["bank_angle_deg"][index])
                    - 2.0
                    * result["roll_damping_ratio"]
                    * result["roll_natural_frequency_radps"]
                    * result["bank_rate_degps"][index]
                )
                self.assertAlmostEqual(
                    result["bank_acceleration_degps2"][index],
                    expected_acceleration,
                    places=10,
                )
                expected_heading_rate = math.degrees(
                    result["gravity_mps2"]
                    / result["true_airspeed_mps"]
                    * math.tan(math.radians(result["bank_angle_deg"][index]))
                )
                self.assertAlmostEqual(
                    result["heading_rate_degps"][index],
                    expected_heading_rate,
                    places=12,
                )
                if index == result["interval_count"]:
                    continue
                self.assertAlmostEqual(
                    result["heading_unwrapped_deg"][index + 1],
                    result["heading_unwrapped_deg"][index]
                    + dt * result["heading_rate_degps"][index],
                    places=11,
                )
                self.assertAlmostEqual(
                    result["bank_angle_deg"][index + 1],
                    result["bank_angle_deg"][index]
                    + dt * result["bank_rate_degps"][index],
                    places=12,
                )
                self.assertAlmostEqual(
                    result["bank_rate_degps"][index + 1],
                    result["bank_rate_degps"][index]
                    + dt * result["bank_acceleration_degps2"][index],
                    places=11,
                )

    def test_branch_cut_is_display_only_and_small_bank_limit_closes(self) -> None:
        result = _oracle()
        wrapped_jumps = tuple(
            right - left
            for left, right in zip(
                result["heading_wrapped_deg"], result["heading_wrapped_deg"][1:]
            )
        )
        continuous_steps = tuple(
            right - left
            for left, right in zip(
                result["heading_unwrapped_deg"],
                result["heading_unwrapped_deg"][1:],
            )
        )
        self.assertGreater(max(map(abs, wrapped_jumps)), 350.0)
        self.assertLess(max(map(abs, continuous_steps)), 0.04)
        for bank_deg, exact_rate_degps in zip(
            result["bank_angle_deg"], result["heading_rate_degps"]
        ):
            linear_rate_radps = (
                result["gravity_mps2"]
                / result["true_airspeed_mps"]
                * math.radians(bank_deg)
            )
            self.assertLess(
                abs(math.radians(exact_rate_degps) - linear_rate_radps), 3e-4
            )
        self.assertGreater(result["signed_heading_travel_deg"], 19.8)
        self.assertLess(result["signed_heading_travel_deg"], 20.0)

    def test_zero_outer_gain_is_exact_open_loop_limit(self) -> None:
        result = _oracle(0.0, 2.4, 1)
        self.assertTrue(
            all(value == 170.0 for value in result["heading_unwrapped_deg"])
        )
        self.assertTrue(all(value == 0.0 for value in result["heading_change_deg"]))
        for key in (
            "bank_command_deg",
            "bank_angle_deg",
            "bank_rate_degps",
            "bank_acceleration_degps2",
            "heading_rate_degps",
        ):
            self.assertTrue(all(value == 0.0 for value in result[key]), key)
        self.assertAlmostEqual(result["final_shortest_heading_error_deg"], 20.0, 12)
        self.assertFalse(result["reached_ninety_percent"])
        self.assertFalse(result["settled_by_end"])

    def test_heading_gain_sweep_isolated_capture_authority_trade(self) -> None:
        gains = (0.0, 0.25, 0.5, 0.75, 1.0)
        results = tuple(_oracle(gain, 2.4, 1) for gain in gains)
        errors_at_ten = tuple(
            result["heading_error_at_ten_seconds_deg"] for result in results
        )
        expected_errors = (
            20.0,
            14.087257521807203,
            9.693404960655482,
            6.828117306718742,
            5.403664247613667,
        )
        for result, expected in zip(results, expected_errors):
            self.assertAlmostEqual(
                result["heading_error_at_ten_seconds_deg"], expected, 9
            )
            self.assertEqual(result["roll_natural_frequency_radps"], 2.4)
            self.assertEqual(result["heading_error_mode"], 1)
            self.assertEqual(result["time_s"], results[0]["time_s"])
            self.assertEqual(
                result["heading_command_wrapped_deg"],
                results[0]["heading_command_wrapped_deg"],
            )
        self.assertTrue(
            all(left > right for left, right in zip(errors_at_ten, errors_at_ten[1:]))
        )
        capture_times = tuple(
            result["time_to_ninety_percent_s"] for result in results[1:]
        )
        self.assertTrue(
            all(left > right for left, right in zip(capture_times, capture_times[1:]))
        )
        self.assertEqual(results[0]["peak_bank_angle_deg"], 0.0)
        self.assertGreater(
            results[2]["peak_bank_angle_deg"], results[1]["peak_bank_angle_deg"]
        )
        self.assertEqual(results[2]["bank_command_saturation_fraction"], 0.0)
        self.assertGreater(results[3]["bank_command_saturation_fraction"], 0.0)
        self.assertGreater(
            results[4]["bank_command_saturation_fraction"],
            results[3]["bank_command_saturation_fraction"],
        )

    def test_roll_frequency_sweep_isolated_tracking_acceleration_trade(self) -> None:
        frequencies = (1.2, 1.8, 2.4, 3.0, 3.6)
        results = tuple(_oracle(0.5, frequency, 1) for frequency in frequencies)
        bank_at_one_point_five = tuple(
            result["bank_angle_deg"][75] for result in results
        )
        tracking_rms = tuple(result["bank_tracking_rms_deg"] for result in results)
        peak_acceleration = tuple(
            result["peak_bank_acceleration_degps2"] for result in results
        )
        expected_bank = (
            1.280716133992437,
            2.472623001030339,
            3.765653541420135,
            5.034393487542499,
            6.198635785562148,
        )
        expected_acceleration = (14.4, 32.4, 57.6, 90.0, 129.6)
        for actual, expected in zip(bank_at_one_point_five, expected_bank):
            self.assertAlmostEqual(actual, expected, 9)
        for actual, expected in zip(peak_acceleration, expected_acceleration):
            self.assertAlmostEqual(actual, expected, 9)
        self.assertTrue(
            all(
                left < right
                for left, right in zip(
                    bank_at_one_point_five, bank_at_one_point_five[1:]
                )
            )
        )
        self.assertTrue(
            all(left > right for left, right in zip(tracking_rms, tracking_rms[1:]))
        )
        self.assertTrue(
            all(
                left < right
                for left, right in zip(peak_acceleration, peak_acceleration[1:])
            )
        )
        for result in results:
            self.assertEqual(result["heading_gain_bank_per_heading"], 0.5)
            self.assertEqual(result["heading_error_mode"], 1)
            self.assertEqual(result["roll_damping_ratio"], 0.8)
            self.assertEqual(result["true_airspeed_mps"], 60.0)

    def test_broken_raw_subtraction_is_isolated_wrong_way_and_recoverable(self) -> None:
        correct = _oracle()
        broken = _oracle(0.5, 2.4, 0)
        command_index = 50
        for key in (
            "heading_unwrapped_deg",
            "heading_wrapped_deg",
            "bank_angle_deg",
            "bank_rate_degps",
            "heading_rate_degps",
        ):
            self.assertEqual(
                broken[key][: command_index + 1], correct[key][: command_index + 1]
            )
        self.assertEqual(
            broken["heading_gain_bank_per_heading"],
            correct["heading_gain_bank_per_heading"],
        )
        self.assertEqual(
            broken["roll_natural_frequency_radps"],
            correct["roll_natural_frequency_radps"],
        )
        self.assertEqual(broken["heading_error_mode"], 0)
        self.assertEqual(
            broken["raw_heading_error_deg"][: command_index + 1],
            correct["raw_heading_error_deg"][: command_index + 1],
        )
        self.assertEqual(
            broken["shortest_heading_error_deg"][: command_index + 1],
            correct["shortest_heading_error_deg"][: command_index + 1],
        )
        self.assertAlmostEqual(
            broken["heading_error_used_deg"][command_index], -340.0, 12
        )
        self.assertAlmostEqual(broken["bank_command_deg"][command_index], -12.0, 12)
        self.assertAlmostEqual(
            broken["bank_acceleration_degps2"][command_index], -69.12, 10
        )
        proper_error = broken["shortest_heading_error_deg"][command_index:]
        raw_magnitude = tuple(
            abs(value) for value in broken["raw_heading_error_deg"][command_index:]
        )
        self.assertTrue(
            all(left <= right for left, right in zip(proper_error, proper_error[1:]))
        )
        self.assertTrue(
            all(left >= right for left, right in zip(raw_magnitude, raw_magnitude[1:]))
        )
        self.assertAlmostEqual(
            broken["signed_heading_travel_deg"], -116.10501891177968, 8
        )
        self.assertAlmostEqual(
            broken["final_shortest_heading_error_deg"], 136.10501891177967, 8
        )
        self.assertGreater(broken["wrong_way_travel_deg"], 110.0)
        self.assertGreater(broken["bank_command_saturation_fraction"], 0.98)
        self.assertLess(
            broken["peak_bank_angle_deg"], broken["bank_teaching_envelope_deg"]
        )
        self.assertFalse(broken["reached_ninety_percent"])
        self.assertFalse(broken["settled_by_end"])
        self.assertEqual(_oracle(), correct)

    def test_broken_saturation_does_not_arrest_wrong_way_turn(self) -> None:
        broken = _oracle(0.5, 2.4, 0)
        tail_start = broken["sample_count"] - 51
        tail_heading = broken["heading_unwrapped_deg"][tail_start:]
        tail_shortest_error = broken["shortest_heading_error_deg"][tail_start:]
        tail_raw_error = broken["raw_heading_error_deg"][tail_start:]

        self.assertEqual(broken["time_s"][tail_start], 59.0)
        self.assertTrue(all(broken["bank_command_saturated"][tail_start:]))
        self.assertTrue(
            all(left > right for left, right in zip(tail_heading, tail_heading[1:]))
        )
        self.assertTrue(
            all(
                left < right
                for left, right in zip(
                    tail_shortest_error, tail_shortest_error[1:]
                )
            )
        )
        self.assertTrue(
            all(
                abs(left) > abs(right)
                for left, right in zip(tail_raw_error, tail_raw_error[1:])
            )
        )
        self.assertGreater(tail_heading[0] - tail_heading[-1], 1.9)
        self.assertGreater(tail_shortest_error[-1] - tail_shortest_error[0], 1.9)
        for index in range(tail_start, broken["interval_count"]):
            self.assertAlmostEqual(
                broken["heading_unwrapped_deg"][index + 1],
                broken["heading_unwrapped_deg"][index]
                + broken["sample_time_s"] * broken["heading_rate_degps"][index],
                places=11,
            )
        self.assertLess(broken["heading_rate_degps"][-1], -1.9)

    def test_malformed_inputs_reject_without_poisoning_recovery(self) -> None:
        malformed = (
            (-1e-12, 2.4, 1),
            (1.000000000001, 2.4, 1),
            (0.5, 1.199999999, 1),
            (0.5, 3.600000001, 1),
            (0.5, 2.4, -1),
            (0.5, 2.4, 2),
            ([0.25, 0.5], 2.4, 1),
            (0.5, [1.8, 2.4], 1),
            (0.5, 2.4, [0, 1]),
            (0.5 + 1.0j, 2.4, 1),
            (0.5, 2.4 + 1.0j, 1),
            (float("nan"), 2.4, 1),
            (0.5, float("inf"), 1),
            (0.5, 2.4, float("nan")),
            (True, 2.4, 1),
        )
        for gain, frequency, mode in malformed:
            with self.subTest(gain=gain, frequency=frequency, mode=mode):
                with self.assertRaises(ValueError):
                    _oracle(gain, frequency, mode)
        self.assertEqual(_oracle(), _oracle(0.5, 2.4, 1))

    def test_accepted_corners_and_representative_grid_are_finite_and_fixed(self) -> None:
        corners = tuple(
            _oracle(gain, frequency, mode)
            for gain in (0.0, 1.0)
            for frequency in (1.2, 3.6)
            for mode in (0, 1)
        )
        self.assertEqual(len(corners), 8)
        grid = tuple(
            _oracle(gain, frequency, mode)
            for gain in (0.0, 0.5, 1.0)
            for frequency in (1.2, 2.4, 3.6)
            for mode in (0, 1)
        )
        self.assertEqual(len(grid), 18)
        for result in corners + grid:
            self.assertEqual(result["sample_count"], 3001)
            self.assertEqual(result["interval_count"], 3000)
            for key in (
                "heading_unwrapped_deg",
                "heading_wrapped_deg",
                "shortest_heading_error_deg",
                "bank_angle_deg",
                "bank_rate_degps",
                "bank_acceleration_degps2",
                "heading_rate_degps",
            ):
                self.assertTrue(all(math.isfinite(value) for value in result[key]))
            self.assertLessEqual(
                max(map(abs, result["bank_command_deg"])), 12.0 + 1e-12
            )
            self.assertLess(
                result["peak_bank_angle_deg"], result["bank_teaching_envelope_deg"]
            )


if __name__ == "__main__":
    unittest.main()
