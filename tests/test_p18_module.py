from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_ID = "P18"
MODULE_FOLDER = ROOT / "modules/18-follow-waypoints"
EVIDENCE_PATH = ROOT / "docs/evidence/P18-2026-08-23.md"
GUIDING_QUESTION = (
    "What inputs, observable effects, and failure modes matter when you follow "
    "Waypoints?"
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


def _bearing_mode(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("bearing mode must be -1 or +1")
    result = float(value)
    if not math.isfinite(result) or result not in (-1.0, 1.0):
        raise ValueError("bearing mode must be -1 or +1")
    return int(result)


def _wrap_radians(value: float) -> float:
    return (value + math.pi) % (2.0 * math.pi) - math.pi


def _oracle(
    arrival_radius_m: object = 30.0,
    course_response_gain_per_s: object = 0.8,
    bearing_mode: object = 1,
) -> dict[str, object]:
    """Independent standard-library implementation of the declared equations."""
    radius = _bounded_scalar("arrival radius", arrival_radius_m, 10.0, 80.0)
    gain = _bounded_scalar(
        "course-response gain", course_response_gain_per_s, 0.0, 1.2
    )
    mode = _bearing_mode(bearing_mode)

    sample_time_s = 0.1
    time_horizon_s = 100.0
    time_s = tuple(index * sample_time_s for index in range(1001))
    sample_count = len(time_s)
    interval_count = sample_count - 1
    ground_speed_mps = 25.0
    maximum_course_rate_radps = math.radians(12.0)
    initial_course_rad = 0.0
    waypoint_north_m = (0.0, 400.0, 400.0, 100.0, 100.0)
    waypoint_east_m = (0.0, 0.0, 300.0, 300.0, 650.0)
    waypoint_count = len(waypoint_north_m)
    leg_north_m = tuple(
        waypoint_north_m[index + 1] - waypoint_north_m[index]
        for index in range(waypoint_count - 1)
    )
    leg_east_m = tuple(
        waypoint_east_m[index + 1] - waypoint_east_m[index]
        for index in range(waypoint_count - 1)
    )
    leg_length_m = tuple(
        math.hypot(north, east)
        for north, east in zip(leg_north_m, leg_east_m)
    )

    north_position_m = [0.0] * sample_count
    east_position_m = [0.0] * sample_count
    course_unwrapped_rad = [initial_course_rad] * sample_count
    course_wrapped_rad = [initial_course_rad] * sample_count
    course_command_rad = [0.0] * sample_count
    course_error_rad = [0.0] * sample_count
    course_rate_command_unclamped_radps = [0.0] * sample_count
    course_rate_radps = [0.0] * sample_count
    course_rate_saturated = [False] * sample_count
    range_to_active_waypoint_m = [0.0] * sample_count
    along_track_distance_m = [0.0] * sample_count
    cross_track_error_m = [0.0] * sample_count
    active_waypoint_index = [0] * sample_count
    active_leg_index = [0] * sample_count
    motion_active = [False] * sample_count

    waypoint_captured = [True, False, False, False, False]
    waypoint_capture_index = [1, 0, 0, 0, 0]
    waypoint_capture_time_s = [0.0] * waypoint_count
    waypoint_capture_north_m = [0.0] * waypoint_count
    waypoint_capture_east_m = [0.0] * waypoint_count
    waypoint_capture_range_m = [0.0] * waypoint_count
    current_waypoint_index = 1
    route_complete = False
    mission_completion_index = 0
    mission_completion_time_s = time_horizon_s + sample_time_s

    for index in range(sample_count):
        if route_complete:
            active_waypoint_index[index] = waypoint_count + 1
            course_command_rad[index] = course_wrapped_rad[index]
            if index < interval_count:
                north_position_m[index + 1] = north_position_m[index]
                east_position_m[index + 1] = east_position_m[index]
                course_unwrapped_rad[index + 1] = course_unwrapped_rad[index]
                course_wrapped_rad[index + 1] = course_wrapped_rad[index]
            continue

        while current_waypoint_index < waypoint_count:
            delta_north = (
                waypoint_north_m[current_waypoint_index] - north_position_m[index]
            )
            delta_east = (
                waypoint_east_m[current_waypoint_index] - east_position_m[index]
            )
            waypoint_range = math.hypot(delta_north, delta_east)
            if waypoint_range > radius:
                break
            waypoint_captured[current_waypoint_index] = True
            waypoint_capture_index[current_waypoint_index] = index + 1
            waypoint_capture_time_s[current_waypoint_index] = time_s[index]
            waypoint_capture_north_m[current_waypoint_index] = north_position_m[index]
            waypoint_capture_east_m[current_waypoint_index] = east_position_m[index]
            waypoint_capture_range_m[current_waypoint_index] = waypoint_range
            current_waypoint_index += 1
            if current_waypoint_index >= waypoint_count:
                route_complete = True
                mission_completion_index = index + 1
                mission_completion_time_s = time_s[index]
                break

        if route_complete:
            active_waypoint_index[index] = waypoint_count + 1
            course_command_rad[index] = course_wrapped_rad[index]
            if index < interval_count:
                north_position_m[index + 1] = north_position_m[index]
                east_position_m[index + 1] = east_position_m[index]
                course_unwrapped_rad[index + 1] = course_unwrapped_rad[index]
                course_wrapped_rad[index + 1] = course_wrapped_rad[index]
            continue

        active_waypoint_index[index] = current_waypoint_index + 1
        active_leg_index[index] = current_waypoint_index
        delta_north = (
            waypoint_north_m[current_waypoint_index] - north_position_m[index]
        )
        delta_east = waypoint_east_m[current_waypoint_index] - east_position_m[index]
        range_to_active_waypoint_m[index] = math.hypot(delta_north, delta_east)
        leg_index = current_waypoint_index - 1
        unit_north = leg_north_m[leg_index] / leg_length_m[leg_index]
        unit_east = leg_east_m[leg_index] / leg_length_m[leg_index]
        relative_north = north_position_m[index] - waypoint_north_m[leg_index]
        relative_east = east_position_m[index] - waypoint_east_m[leg_index]
        along_track_distance_m[index] = (
            relative_north * unit_north + relative_east * unit_east
        )
        cross_track_error_m[index] = (
            relative_north * -unit_east + relative_east * unit_north
        )

        if mode == 1:
            course_command_rad[index] = math.atan2(delta_east, delta_north)
        else:
            course_command_rad[index] = math.atan2(delta_north, delta_east)
        course_error_rad[index] = _wrap_radians(
            course_command_rad[index] - course_wrapped_rad[index]
        )
        course_rate_command_unclamped_radps[index] = (
            gain * course_error_rad[index]
        )
        course_rate_radps[index] = max(
            -maximum_course_rate_radps,
            min(
                maximum_course_rate_radps,
                course_rate_command_unclamped_radps[index],
            ),
        )
        course_rate_saturated[index] = (
            abs(course_rate_command_unclamped_radps[index])
            > maximum_course_rate_radps
        )
        motion_active[index] = True

        if index < interval_count:
            north_position_m[index + 1] = (
                north_position_m[index]
                + sample_time_s
                * ground_speed_mps
                * math.cos(course_unwrapped_rad[index])
            )
            east_position_m[index + 1] = (
                east_position_m[index]
                + sample_time_s
                * ground_speed_mps
                * math.sin(course_unwrapped_rad[index])
            )
            course_unwrapped_rad[index + 1] = (
                course_unwrapped_rad[index]
                + sample_time_s * course_rate_radps[index]
            )
            course_wrapped_rad[index + 1] = _wrap_radians(
                course_unwrapped_rad[index + 1]
            )

    step_distance_m = tuple(
        math.hypot(
            north_position_m[index + 1] - north_position_m[index],
            east_position_m[index + 1] - east_position_m[index],
        )
        for index in range(interval_count)
    )
    active_course_errors_deg = tuple(
        math.degrees(course_error_rad[index])
        for index in range(sample_count)
        if motion_active[index]
    )
    active_cross_track_m = tuple(
        cross_track_error_m[index]
        for index in range(sample_count)
        if motion_active[index]
    )
    minimum_range_to_waypoint_m = tuple(
        min(
            math.hypot(north - waypoint_north, east - waypoint_east)
            for north, east in zip(north_position_m, east_position_m)
        )
        for waypoint_north, waypoint_east in zip(
            waypoint_north_m, waypoint_east_m
        )
    )
    final_target_distance_m = math.hypot(
        north_position_m[-1] - waypoint_north_m[-1],
        east_position_m[-1] - waypoint_east_m[-1],
    )

    return {
        "arrival_radius_m": radius,
        "course_response_gain_per_s": gain,
        "bearing_mode": mode,
        "sample_time_s": sample_time_s,
        "time_horizon_s": time_horizon_s,
        "time_s": time_s,
        "sample_count": sample_count,
        "interval_count": interval_count,
        "ground_speed_mps": ground_speed_mps,
        "maximum_course_rate_degps": 12.0,
        "initial_course_deg": math.degrees(initial_course_rad),
        "waypoint_north_m": waypoint_north_m,
        "waypoint_east_m": waypoint_east_m,
        "waypoint_count": waypoint_count,
        "leg_north_m": leg_north_m,
        "leg_east_m": leg_east_m,
        "leg_length_m": leg_length_m,
        "minimum_leg_length_m": min(leg_length_m),
        "planned_route_length_m": sum(leg_length_m),
        "north_position_m": tuple(north_position_m),
        "east_position_m": tuple(east_position_m),
        "course_unwrapped_deg": tuple(map(math.degrees, course_unwrapped_rad)),
        "course_wrapped_deg": tuple(map(math.degrees, course_wrapped_rad)),
        "course_command_deg": tuple(map(math.degrees, course_command_rad)),
        "course_error_deg": tuple(map(math.degrees, course_error_rad)),
        "course_rate_command_unclamped_degps": tuple(
            map(math.degrees, course_rate_command_unclamped_radps)
        ),
        "course_rate_degps": tuple(map(math.degrees, course_rate_radps)),
        "course_rate_saturated": tuple(course_rate_saturated),
        "range_to_active_waypoint_m": tuple(range_to_active_waypoint_m),
        "along_track_distance_m": tuple(along_track_distance_m),
        "cross_track_error_m": tuple(cross_track_error_m),
        "active_waypoint_index": tuple(active_waypoint_index),
        "active_leg_index": tuple(active_leg_index),
        "motion_active": tuple(motion_active),
        "motion_active_sample_count": sum(motion_active),
        "motion_active_interval_count": sum(motion_active[:-1]),
        "step_distance_m": step_distance_m,
        "waypoint_captured": tuple(waypoint_captured),
        "waypoint_capture_index": tuple(waypoint_capture_index),
        "waypoint_capture_time_s": tuple(waypoint_capture_time_s),
        "waypoint_capture_north_m": tuple(waypoint_capture_north_m),
        "waypoint_capture_east_m": tuple(waypoint_capture_east_m),
        "waypoint_capture_range_m": tuple(waypoint_capture_range_m),
        "waypoint_captured_count": sum(waypoint_captured),
        "target_waypoint_captured_count": sum(waypoint_captured[1:]),
        "route_complete": route_complete,
        "mission_completion_index": mission_completion_index,
        "mission_completion_time_s": mission_completion_time_s,
        "flown_distance_m": sum(step_distance_m),
        "course_error_rms_deg": math.sqrt(
            sum(value * value for value in active_course_errors_deg)
            / len(active_course_errors_deg)
        ),
        "peak_absolute_course_error_deg": max(map(abs, active_course_errors_deg)),
        "course_rate_saturation_count": sum(course_rate_saturated),
        "course_rate_saturation_fraction": (
            sum(course_rate_saturated) / sum(motion_active)
        ),
        "cross_track_rms_m": math.sqrt(
            sum(value * value for value in active_cross_track_m)
            / len(active_cross_track_m)
        ),
        "peak_absolute_cross_track_error_m": max(map(abs, active_cross_track_m)),
        "minimum_range_to_waypoint_m": minimum_range_to_waypoint_m,
        "final_target_distance_m": final_target_distance_m,
    }


class P18ArtifactTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        manifest = json.loads(
            (ROOT / "curriculum/modules.json").read_text(encoding="utf-8")
        )
        cls.modules = {module["id"]: module for module in manifest["modules"]}
        cls.module = cls.modules[MODULE_ID]
        cls.text = {
            path.name: path.read_text(encoding="utf-8")
            for path in MODULE_FOLDER.iterdir()
            if path.is_file()
        }

    def test_permanent_manifest_identity_and_complete_artifacts(self) -> None:
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
                "number": 18,
                "id": "P18",
                "title": "Follow Waypoints",
                "guiding_question": GUIDING_QUESTION,
                "phase": 5,
                "phase_title": "Navigation and guidance",
                "slug": "follow-waypoints",
                "folder": "modules/18-follow-waypoints",
                "implementation_batch": "P18",
                "prerequisites": ["P17"],
            },
        )
        self.assertEqual(self.module["status"], "implemented")
        self.assertEqual(self.module["evidence_level"], "simulated")
        self.assertEqual(self.modules["P17"]["status"], "implemented")
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
        self.assertLessEqual(required, set(self.text))
        for name in required:
            with self.subTest(file=name):
                self.assertTrue(self.text[name].strip(), name)
                self.assertTrue(self.text[name].endswith("\n"), name)
                self.assertFalse(self.text[name].endswith("\n\n"), name)

    def test_learning_slice_is_concept_first_and_bounded(self) -> None:
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
            "p17",
            "north/east",
            "stationary",
            "active waypoint",
            "clockwise from north",
            "arrival radius",
            "course-response gain",
            "bounded",
            "deterministic",
            "mechanism",
            "reset",
            "broken",
            "teach-back",
            "conceptual",
            "p19",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, combined)
        self.assertRegex(walkthrough.lower(), r"one plot|one .* at a time")
        self.assertIn("does not consume p17", combined)
        self.assertIn("conceptual rather than current api compatibility", combined)
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
        for expression in (
            "functionout=model(arrivalradius_m,courseresponsegain_per_s,bearingmode)",
            "arrivalradius_m(1,1)double{mustbereal,mustbefinite}=30",
            "courseresponsegain_per_s(1,1)double{mustbereal,mustbefinite}=0.8",
            "bearingmode(1,1)double{mustbereal,mustbefinite}=1",
            "minimumarrivalradius_m=10;",
            "maximumarrivalradius_m=80;",
            "minimumcourseresponsegain_per_s=0;",
            "maximumcourseresponsegain_per_s=1.2;",
            "bearingmode~=1&&bearingmode~=-1",
            "sampletime_s=0.1;",
            "timehorizon_s=100;",
            "groundspeed_mps=25;",
            "maximumcourserate_radps=deg2rad(12);",
            "waypointnorth_m=[0400400100100];",
            "waypointeast_m=[00300300650];",
            "waypointrange_m=hypot(deltanorth_m,deltaeast_m);",
            "waypointrange_m>arrivalradius_m",
            "coursecommand_rad(k)=atan2(deltaeast_m,deltanorth_m);",
            "coursecommand_rad(k)=atan2(deltanorth_m,deltaeast_m);",
            "courseerror_rad(k)=mod(coursecommand_rad(k)-coursewrapped_rad(k)+pi,2*pi)-pi;",
            "courseratecommandunclamped_radps(k)=courseresponsegain_per_s*courseerror_rad(k);",
            "northposition_m(k+1)=northposition_m(k)+sampletime_s*groundspeed_mps*cos(courseunwrapped_rad(k));",
            "eastposition_m(k+1)=eastposition_m(k)+sampletime_s*groundspeed_mps*sin(courseunwrapped_rad(k));",
            "activelegindex(k)=currentwaypointindex-1;",
            "alongtrackdistance_m(k)=relativenorth_m*legunitnorth+relativeeast_m*leguniteast;",
            "crosstrackerror_m(k)=relativenorth_m*(-leguniteast)+relativeeast_m*legunitnorth;",
            "crosstrackrms_m=sqrt(mean(activecrosstrackerror_m.^2));",
            "peakabsolutecrosstrackerror_m=max(abs(activecrosstrackerror_m));",
        ):
            with self.subTest(expression=expression):
                self.assertIn(expression, compact)
        for presentation in (
            "figure(",
            "uifigure(",
            "plot(",
            "subplot(",
            "uiaxes(",
            "uilabel(",
            "fprintf(",
        ):
            self.assertNotIn(presentation, lower)

    def test_experiment_has_two_isolated_sweeps_and_broken_case(self) -> None:
        experiment = self.text["experiment.m"]
        compact = re.sub(r"\s+", "", experiment.replace("...", "")).lower()
        for expression in (
            "baseline=model(30,0.8,1);",
            "arrivalradiussweep_m=[1020305080];",
            "sample=model(arrivalradiussweep_m(k),baseline.courseresponsegain_per_s,1);",
            "courseresponsegainsweep_per_s=[00.20.40.81.2];",
            "sample=model(baseline.arrivalradius_m,courseresponsegainsweep_per_s(k),1);",
            "zeroresponse=model(baseline.arrivalradius_m,0,1);",
            "broken=model(baseline.arrivalradius_m,baseline.courseresponsegain_per_s,-1);",
            "100*baseline.activewaypointindex(activepath)",
            "yyaxisleft;",
            "yyaxisright;",
            "clearrun_checks;run_checks;",
        ):
            with self.subTest(expression=expression):
                self.assertIn(expression, compact)
        self.assertGreaterEqual(experiment.count("%%"), 8)
        for label in (
            "East position (m)",
            "North position (m)",
            "Time (s)",
            "Course (deg)",
            "Waypoint arrival radius (m)",
            "Course-response gain (1/s)",
        ):
            self.assertIn(label, experiment)
        self.assertIn("Mechanism:", experiment)
        self.assertIn("Deliberately broken", experiment)

    def test_interaction_and_checks_cover_recovery_resources(self) -> None:
        interactive = self.text["interactive.m"]
        checks = self.text["checks.md"] + "\n" + self.text["run_checks.m"]
        compact_ui = re.sub(r"\s+", "", interactive.replace("...", "")).lower()
        compact_checks = re.sub(r"\s+", "", checks.replace("...", "")).lower()
        for token in (
            "uifigure(",
            "uislider(",
            "uidropdown(",
            "valuechangingfcn",
            "valuechangedfcn",
            "buttonpushedfcn",
            "resetbaseline",
            "radiuscontrol.value=30;",
            "gaincontrol.value=0.8;",
            "modecontrol.value='correctatan2(east,north)';",
            "modelfcn=@model;",
        ):
            self.assertIn(token, compact_ui)
        self.assertEqual(compact_ui.count("uislider("), 2)
        self.assertIn("findall(groot,'type','figure','name',uiname)", compact_ui)
        for token in (
            "malformed",
            "recovery",
            "rollback",
            "timeout",
            "cancellation",
            "fixed 1001-sample",
            "capped 8/18-case",
            "migration",
            "backup/restore",
            "p17",
            "p19",
        ):
            self.assertIn(token, checks.lower())
        for expression in (
            "remainingalongtrack_m=baseline.leglength_m(activeleg)-baseline.alongtrackdistance_m(baseline.motionactive);",
            "rangefromlegcoordinates_m=hypot(remainingalongtrack_m,baseline.crosstrackerror_m(baseline.motionactive));",
            "abs(baseline.crosstrackrms_m-52.26952755669431)<1e-8",
            "abs(baseline.peakabsolutecrosstrackerror_m-112.03266029039757)<1e-8",
        ):
            self.assertIn(expression, compact_checks)

    def test_no_opaque_toolbox_random_external_or_async_behavior(self) -> None:
        matlab = "\n".join(
            self.text[name]
            for name in ("model.m", "experiment.m", "interactive.m", "run_checks.m")
        )
        lower = matlab.lower()
        banned_shortcuts = (
            "waypointfollower",
            "waypointtrajectory",
            "controllerpurepursuit",
            "navpath",
            "wraptopi",
            "proportionalnavigation",
            "aerospacetoolbox",
        )
        for token in banned_shortcuts:
            self.assertNotIn(token, lower)
        calls = re.compile(
            r"\b(rand|randn|rng|load|save|readtable|writetable|fopen|webread|"
            r"tcpclient|udpport|timer|parfeval|parpool)\s*\(",
            re.IGNORECASE,
        )
        self.assertIsNone(calls.search(matlab))

    def test_retained_evidence_acceptance_and_claim_boundary(self) -> None:
        evidence = EVIDENCE_PATH.read_text(encoding="utf-8")
        self.assertTrue(evidence.endswith("\n"))
        self.assertFalse(evidence.endswith("\n\n"))
        for token in (
            "Acceptance mapping",
            "Figure, control, metric, unit, and runtime inventory",
            "Exact validation performed",
            "Changed and preserved invariants",
            "Residual risks and limitations",
            "Rollback",
            "Explicitly unperformed validation",
            "static",
            "independent simulated",
            "MATLAB runtime",
            "UI",
            "numerical-fidelity",
            "bench",
            "HIL",
            "field",
            "RT1/RT2",
            "Unreal",
            "signing",
            "deployment",
            "production",
        ):
            self.assertIn(token, evidence)
        self.assertEqual(len(re.findall(r"^\| A[1-8] \|", evidence, re.MULTILINE)), 8)
        match = re.search(r"```json\n(.*?)\n```", evidence, re.DOTALL)
        self.assertIsNotNone(match)
        summary = json.loads(match.group(1))
        self.assertEqual(summary["batch_id"], "P18")
        self.assertEqual(summary["status"], "pass")
        self.assertEqual(len(summary["acceptance"]), 8)
        self.assertTrue(all(item["status"] == "pass" for item in summary["acceptance"]))


class P18EquationOracleTests(unittest.TestCase):
    def test_deterministic_baseline_signature_and_fixed_shape(self) -> None:
        first = _oracle()
        second = _oracle()
        self.assertEqual(first, second)
        self.assertEqual(first["sample_count"], 1001)
        self.assertEqual(first["interval_count"], 1000)
        self.assertEqual(first["waypoint_north_m"], (0.0, 400.0, 400.0, 100.0, 100.0))
        self.assertEqual(first["waypoint_east_m"], (0.0, 0.0, 300.0, 300.0, 650.0))
        self.assertEqual(first["leg_length_m"], (400.0, 300.0, 300.0, 350.0))
        self.assertEqual(first["planned_route_length_m"], 1350.0)
        self.assertLess(2.0 * 80.0, first["minimum_leg_length_m"])
        self.assertTrue(first["route_complete"])
        self.assertEqual(first["waypoint_capture_index"], (1, 149, 295, 418, 593))
        for observed, expected in zip(
            first["waypoint_capture_time_s"], (0.0, 14.8, 29.4, 41.7, 59.2)
        ):
            self.assertAlmostEqual(observed, expected, 12)
        expected_ranges = (
            0.0,
            30.0,
            28.61267913300444,
            29.413712525799518,
            27.827713892891673,
        )
        for observed, expected in zip(first["waypoint_capture_range_m"], expected_ranges):
            self.assertAlmostEqual(observed, expected, 11)
        self.assertEqual(first["mission_completion_index"], 593)
        self.assertAlmostEqual(first["mission_completion_time_s"], 59.2, 12)
        self.assertEqual(first["motion_active_sample_count"], 592)
        self.assertEqual(first["motion_active_interval_count"], 592)
        self.assertAlmostEqual(first["flown_distance_m"], 1480.0, 10)
        self.assertAlmostEqual(first["course_error_rms_deg"], 33.914338204209656, 11)
        self.assertAlmostEqual(first["peak_absolute_course_error_deg"], 95.04123761897495, 11)
        self.assertEqual(first["course_rate_saturation_count"], 223)
        self.assertAlmostEqual(first["course_rate_saturation_fraction"], 223 / 592, 14)

    def test_active_leg_coordinates_and_cross_track_metrics_reconstruct(self) -> None:
        result = _oracle()
        waypoints = tuple(zip(result["waypoint_north_m"], result["waypoint_east_m"]))
        leg_north = result["leg_north_m"]
        leg_east = result["leg_east_m"]
        leg_length = result["leg_length_m"]
        reconstructed_cross_track = []

        for index in range(result["sample_count"]):
            if not result["motion_active"][index]:
                self.assertEqual(result["active_leg_index"][index], 0)
                self.assertEqual(result["along_track_distance_m"][index], 0.0)
                self.assertEqual(result["cross_track_error_m"][index], 0.0)
                continue

            active_leg = result["active_leg_index"][index]
            self.assertEqual(active_leg, result["active_waypoint_index"][index] - 1)
            leg_offset = active_leg - 1
            origin_north, origin_east = waypoints[leg_offset]
            relative_north = result["north_position_m"][index] - origin_north
            relative_east = result["east_position_m"][index] - origin_east
            unit_north = leg_north[leg_offset] / leg_length[leg_offset]
            unit_east = leg_east[leg_offset] / leg_length[leg_offset]
            expected_along_track = (
                relative_north * unit_north + relative_east * unit_east
            )
            expected_cross_track = (
                relative_north * -unit_east + relative_east * unit_north
            )
            self.assertAlmostEqual(
                result["along_track_distance_m"][index], expected_along_track, 11
            )
            self.assertAlmostEqual(
                result["cross_track_error_m"][index], expected_cross_track, 11
            )
            self.assertAlmostEqual(
                result["range_to_active_waypoint_m"][index],
                math.hypot(
                    leg_length[leg_offset] - expected_along_track,
                    expected_cross_track,
                ),
                11,
            )
            reconstructed_cross_track.append(expected_cross_track)

        expected_rms = math.sqrt(
            sum(value * value for value in reconstructed_cross_track)
            / len(reconstructed_cross_track)
        )
        expected_peak = max(map(abs, reconstructed_cross_track))
        self.assertTrue(any(value > 0.0 for value in reconstructed_cross_track))
        self.assertTrue(any(value < 0.0 for value in reconstructed_cross_track))
        self.assertAlmostEqual(result["cross_track_rms_m"], expected_rms, 11)
        self.assertAlmostEqual(
            result["peak_absolute_cross_track_error_m"], expected_peak, 11
        )
        self.assertAlmostEqual(result["cross_track_rms_m"], 52.26952755669431, 11)
        self.assertAlmostEqual(
            result["peak_absolute_cross_track_error_m"], 112.03266029039757, 11
        )

    def test_every_bearing_response_motion_and_event_reconstructs(self) -> None:
        result = _oracle()
        waypoints = tuple(zip(result["waypoint_north_m"], result["waypoint_east_m"]))
        for index in range(result["sample_count"]):
            if result["motion_active"][index]:
                target = result["active_waypoint_index"][index] - 1
                delta_north = waypoints[target][0] - result["north_position_m"][index]
                delta_east = waypoints[target][1] - result["east_position_m"][index]
                expected_bearing = math.atan2(delta_east, delta_north)
                expected_error = _wrap_radians(
                    expected_bearing
                    - math.radians(result["course_wrapped_deg"][index])
                )
                expected_raw = result["course_response_gain_per_s"] * expected_error
                limit = math.radians(result["maximum_course_rate_degps"])
                expected_rate = max(-limit, min(limit, expected_raw))
                self.assertAlmostEqual(
                    result["range_to_active_waypoint_m"][index],
                    math.hypot(delta_north, delta_east),
                    11,
                )
                self.assertAlmostEqual(
                    math.radians(result["course_command_deg"][index]), expected_bearing, 12
                )
                self.assertAlmostEqual(
                    math.radians(result["course_error_deg"][index]), expected_error, 12
                )
                self.assertAlmostEqual(
                    math.radians(result["course_rate_command_unclamped_degps"][index]),
                    expected_raw,
                    12,
                )
                self.assertAlmostEqual(
                    math.radians(result["course_rate_degps"][index]), expected_rate, 12
                )
                self.assertEqual(
                    result["course_rate_saturated"][index], abs(expected_raw) > limit
                )
            else:
                self.assertEqual(result["active_waypoint_index"][index], 6)
                self.assertEqual(result["course_error_deg"][index], 0.0)
                self.assertEqual(result["course_rate_degps"][index], 0.0)

        for index in range(result["interval_count"]):
            if result["motion_active"][index]:
                course = math.radians(result["course_unwrapped_deg"][index])
                self.assertAlmostEqual(
                    result["north_position_m"][index + 1],
                    result["north_position_m"][index]
                    + 2.5 * math.cos(course),
                    10,
                )
                self.assertAlmostEqual(
                    result["east_position_m"][index + 1],
                    result["east_position_m"][index]
                    + 2.5 * math.sin(course),
                    10,
                )
                self.assertAlmostEqual(result["step_distance_m"][index], 2.5, 10)
            else:
                self.assertEqual(result["step_distance_m"][index], 0.0)

    def test_capture_boundary_wrap_and_zero_response_limits(self) -> None:
        radius = 30.0
        self.assertLessEqual(radius, radius)
        self.assertFalse(math.nextafter(radius, math.inf) <= radius)
        self.assertEqual(_wrap_radians(0.0), 0.0)
        self.assertEqual(_wrap_radians(math.pi), -math.pi)
        turn_radius = 25.0 / math.radians(12.0)
        self.assertAlmostEqual(turn_radius, 119.36620731892151, 12)
        result = _oracle(course_response_gain_per_s=0.0)
        self.assertFalse(result["route_complete"])
        self.assertEqual(result["target_waypoint_captured_count"], 1)
        self.assertEqual(result["waypoint_captured"], (True, True, False, False, False))
        self.assertEqual(result["mission_completion_index"], 0)
        self.assertEqual(result["mission_completion_time_s"], 100.1)
        self.assertTrue(all(value == 0.0 for value in result["east_position_m"]))
        self.assertTrue(all(value == 0.0 for value in result["course_unwrapped_deg"]))
        self.assertTrue(all(value == 0.0 for value in result["course_rate_degps"]))
        for north, time in zip(result["north_position_m"], result["time_s"]):
            self.assertAlmostEqual(north, 25.0 * time, 10)
        self.assertAlmostEqual(result["flown_distance_m"], 2500.0, 10)

    def test_arrival_radius_sweep_isolates_switching_geometry(self) -> None:
        radii = (10.0, 20.0, 30.0, 50.0, 80.0)
        results = tuple(_oracle(radius, 0.8, 1) for radius in radii)
        expected_times = (63.1, 61.0, 59.2, 55.5, 50.0)
        expected_distances = (1577.5, 1525.0, 1480.0, 1387.5, 1250.0)
        baseline = results[2]
        for radius, result, time, distance in zip(
            radii, results, expected_times, expected_distances
        ):
            with self.subTest(radius=radius):
                self.assertEqual(result["course_response_gain_per_s"], 0.8)
                self.assertEqual(result["bearing_mode"], 1)
                self.assertEqual(result["waypoint_north_m"], baseline["waypoint_north_m"])
                self.assertEqual(result["waypoint_east_m"], baseline["waypoint_east_m"])
                self.assertEqual(result["time_s"], baseline["time_s"])
                self.assertTrue(result["route_complete"])
                self.assertEqual(result["target_waypoint_captured_count"], 4)
                self.assertAlmostEqual(result["mission_completion_time_s"], time, 11)
                self.assertAlmostEqual(result["flown_distance_m"], distance, 10)
                for capture_range in result["waypoint_capture_range_m"][1:]:
                    self.assertLessEqual(capture_range, radius + 1e-12)
                    self.assertGreater(capture_range, radius - 2.5 - 1e-12)
                self.assertAlmostEqual(
                    result["flown_distance_m"],
                    result["ground_speed_mps"] * result["mission_completion_time_s"],
                    10,
                )
        self.assertTrue(all(a > b for a, b in zip(expected_times, expected_times[1:])))
        self.assertTrue(
            all(a > b for a, b in zip(expected_distances, expected_distances[1:]))
        )

    def test_course_gain_sweep_isolates_response_and_rate_authority(self) -> None:
        gains = (0.0, 0.2, 0.4, 0.8, 1.2)
        results = tuple(_oracle(30.0, gain, 1) for gain in gains)
        expected_complete = (False, True, True, True, True)
        expected_captured = (1, 4, 4, 4, 4)
        expected_times = (100.1, 65.6, 60.0, 59.2, 59.1)
        expected_distances = (2500.0, 1640.0, 1500.0, 1480.0, 1477.5)
        expected_saturation_counts = (0, 109, 177, 223, 239)
        baseline = results[3]
        for gain, result, complete, captured, time, distance, saturated in zip(
            gains,
            results,
            expected_complete,
            expected_captured,
            expected_times,
            expected_distances,
            expected_saturation_counts,
        ):
            with self.subTest(gain=gain):
                self.assertEqual(result["arrival_radius_m"], 30.0)
                self.assertEqual(result["bearing_mode"], 1)
                self.assertEqual(result["waypoint_north_m"], baseline["waypoint_north_m"])
                self.assertEqual(result["waypoint_east_m"], baseline["waypoint_east_m"])
                self.assertEqual(result["route_complete"], complete)
                self.assertEqual(result["target_waypoint_captured_count"], captured)
                self.assertAlmostEqual(result["mission_completion_time_s"], time, 11)
                self.assertAlmostEqual(result["flown_distance_m"], distance, 10)
                self.assertEqual(result["course_rate_saturation_count"], saturated)
        fractions = tuple(result["course_rate_saturation_fraction"] for result in results)
        self.assertTrue(all(a <= b for a, b in zip(fractions, fractions[1:])))

    def test_swapped_bearing_isolated_failure_and_rollback(self) -> None:
        correct = _oracle()
        broken = _oracle(bearing_mode=-1)
        for field in (
            "arrival_radius_m",
            "course_response_gain_per_s",
            "sample_time_s",
            "time_horizon_s",
            "time_s",
            "ground_speed_mps",
            "maximum_course_rate_degps",
            "initial_course_deg",
            "waypoint_north_m",
            "waypoint_east_m",
            "leg_length_m",
            "planned_route_length_m",
        ):
            self.assertEqual(correct[field], broken[field], field)
        self.assertEqual(correct["course_command_deg"][0], 0.0)
        self.assertAlmostEqual(broken["course_command_deg"][0], 90.0, 12)
        self.assertFalse(broken["route_complete"])
        self.assertEqual(broken["target_waypoint_captured_count"], 0)
        self.assertEqual(broken["waypoint_captured"], (True, False, False, False, False))
        self.assertTrue(all(index == 2 for index in broken["active_waypoint_index"]))
        self.assertAlmostEqual(
            broken["minimum_range_to_waypoint_m"][1], 296.8687930873698, 10
        )
        self.assertAlmostEqual(broken["north_position_m"][-1], -1429.641274946602, 9)
        self.assertAlmostEqual(broken["east_position_m"][-1], 1817.972989561882, 9)
        self.assertAlmostEqual(broken["flown_distance_m"], 2500.0, 9)
        self.assertLess(broken["course_error_rms_deg"], correct["course_error_rms_deg"])
        self.assertEqual(_oracle(), correct)

    def test_malformed_inputs_reject_without_poisoning_recovery(self) -> None:
        malformed = (
            (9.999999, 0.8, 1),
            (80.000001, 0.8, 1),
            ([30.0], 0.8, 1),
            (30.0 + 1.0j, 0.8, 1),
            (float("nan"), 0.8, 1),
            (float("inf"), 0.8, 1),
            (30.0, -1e-12, 1),
            (30.0, 1.200001, 1),
            (30.0, [0.8], 1),
            (30.0, 0.8 + 1.0j, 1),
            (30.0, float("nan"), 1),
            (30.0, float("inf"), 1),
            (30.0, 0.8, -2),
            (30.0, 0.8, 0),
            (30.0, 0.8, 2),
            (30.0, 0.8, [1]),
            (30.0, 0.8, 1.0 + 1.0j),
            (30.0, 0.8, float("nan")),
            (True, 0.8, 1),
        )
        baseline = _oracle()
        for radius, gain, mode in malformed:
            with self.subTest(radius=radius, gain=gain, mode=mode):
                with self.assertRaises(ValueError):
                    _oracle(radius, gain, mode)
                self.assertEqual(_oracle(), baseline)

    def test_broken_and_rejected_calls_exact_rollback_recovery(self) -> None:
        baseline = _oracle()
        broken = _oracle(30.0, 0.8, -1)
        self.assertNotEqual(broken["north_position_m"], baseline["north_position_m"])
        self.assertEqual(_oracle(30.0, 0.8, 1), baseline)
        with self.assertRaises(ValueError):
            _oracle(81.0, 0.8, 1)
        self.assertEqual(_oracle(), baseline)

    def test_corners_and_capped_grid_finite_fixed_bounded(self) -> None:
        corners = tuple(
            _oracle(radius, gain, mode)
            for radius in (10.0, 80.0)
            for gain in (0.0, 1.2)
            for mode in (-1, 1)
        )
        self.assertEqual(len(corners), 8)
        grid = tuple(
            _oracle(radius, gain, mode)
            for radius in (10.0, 30.0, 80.0)
            for gain in (0.0, 0.8, 1.2)
            for mode in (-1, 1)
        )
        self.assertEqual(len(grid), 18)
        history_fields = (
            "time_s",
            "north_position_m",
            "east_position_m",
            "course_unwrapped_deg",
            "course_wrapped_deg",
            "course_command_deg",
            "course_error_deg",
            "course_rate_command_unclamped_degps",
            "course_rate_degps",
            "range_to_active_waypoint_m",
            "along_track_distance_m",
            "cross_track_error_m",
            "active_waypoint_index",
            "active_leg_index",
        )
        for result in corners + grid:
            self.assertEqual(result["sample_count"], 1001)
            self.assertEqual(result["interval_count"], 1000)
            for field in history_fields:
                self.assertEqual(len(result[field]), 1001, field)
                self.assertTrue(all(math.isfinite(value) for value in result[field]), field)
            self.assertLessEqual(max(map(abs, result["course_wrapped_deg"])), 180.0)
            self.assertLessEqual(max(map(abs, result["course_error_deg"])), 180.0)
            self.assertLessEqual(max(map(abs, result["course_rate_degps"])), 12.0 + 1e-12)
            self.assertLessEqual(result["flown_distance_m"], 2500.0 + 1e-9)
            self.assertTrue(
                all(
                    math.hypot(north, east) <= 2500.0 + 1e-9
                    for north, east in zip(
                        result["north_position_m"], result["east_position_m"]
                    )
                )
            )
            active = result["active_waypoint_index"]
            self.assertTrue(all(2 <= index <= 6 for index in active))
            self.assertTrue(all(0 <= b - a <= 1 for a, b in zip(active, active[1:])))
            captured = tuple(map(int, result["waypoint_captured"]))
            self.assertTrue(all(b - a <= 0 for a, b in zip(captured, captured[1:])))

    def test_sync_resource_timeout_cancellation_compatibility_disposition(self) -> None:
        result = _oracle()
        self.assertEqual(result["time_s"], tuple(index * 0.1 for index in range(1001)))
        self.assertEqual(result["waypoint_count"], 5)
        self.assertEqual(result["planned_route_length_m"], 1350.0)
        self.assertEqual(result["ground_speed_mps"], 25.0)
        self.assertEqual(result["maximum_course_rate_degps"], 12.0)
        # This API performs fixed synchronous arithmetic and owns no timer,
        # future, worker, external I/O, cancellation state, or input-sized
        # allocation. Runtime timeout/cancel transitions are not applicable;
        # bounded inputs plus fixed histories and capped matrices are the gate.


if __name__ == "__main__":
    unittest.main()
