from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path
from typing import Iterable

ROOT = Path(__file__).resolve().parents[1]
MODULE_ID = "P02"
MODULE_FOLDER = ROOT / "modules/02-transform-between-aerospace-frames"
GUIDING_QUESTION = (
    "What inputs, observable effects, and failure modes matter when you transform "
    "Between Aerospace Frames?"
)

Vector = tuple[float, float, float]
Matrix = tuple[Vector, Vector, Vector]


def _finite_scalar(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite scalar")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite scalar")
    return result


def _mat_vec(matrix: Matrix, vector: Vector) -> Vector:
    return tuple(  # type: ignore[return-value]
        sum(row[index] * vector[index] for index in range(3)) for row in matrix
    )


def _mat_mul(left: Matrix, right: Matrix) -> Matrix:
    return tuple(  # type: ignore[return-value]
        tuple(
            sum(left[row][index] * right[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def _transpose(matrix: Matrix) -> Matrix:
    return tuple(  # type: ignore[return-value]
        tuple(matrix[row][column] for row in range(3)) for column in range(3)
    )


def _norm(vector: Iterable[float]) -> float:
    return math.sqrt(sum(value * value for value in vector))


def _vector_error(left: Vector, right: Vector) -> float:
    return _norm(left[index] - right[index] for index in range(3))


def _matrix_error(left: Matrix, right: Matrix) -> float:
    return _norm(
        left[row][column] - right[row][column]
        for row in range(3)
        for column in range(3)
    )


def _determinant(matrix: Matrix) -> float:
    a, b, c = matrix[0]
    d, e, f = matrix[1]
    g, h, i = matrix[2]
    return a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g)


def _wind_to_body(alpha_deg: float, beta_deg: float) -> Matrix:
    """Independent passive wind-to-body DCM for x-forward/y-right/z-down axes."""
    alpha = math.radians(alpha_deg)
    beta = math.radians(beta_deg)
    ca, sa = math.cos(alpha), math.sin(alpha)
    cb, sb = math.cos(beta), math.sin(beta)
    return (
        (ca * cb, -ca * sb, -sa),
        (sb, cb, 0.0),
        (sa * cb, -sa * sb, ca),
    )


def _body_to_ned(roll_deg: float, pitch_deg: float, yaw_deg: float) -> Matrix:
    """Independent passive body-to-NED 3-2-1 yaw-pitch-roll DCM."""
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy),
        (cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy),
        (-sp, sr * cp, cr * cp),
    )


def _oracle(
    speed: object = 70.0,
    alpha_deg: object = 6.0,
    beta_deg: object = 0.0,
    roll_deg: object = 0.0,
    pitch_deg: object = 9.0,
    yaw_deg: object = 30.0,
) -> dict[str, object]:
    """Pure-Python numerical oracle; it does not execute or translate MATLAB."""
    speed_value = _finite_scalar("speed", speed)
    alpha_value = _finite_scalar("alpha", alpha_deg)
    beta_value = _finite_scalar("beta", beta_deg)
    roll_value = _finite_scalar("roll", roll_deg)
    pitch_value = _finite_scalar("pitch", pitch_deg)
    yaw_value = _finite_scalar("yaw", yaw_deg)
    if speed_value <= 0.0:
        raise ValueError("speed must be positive")
    if abs(beta_value) >= 90.0:
        raise ValueError("sideslip must be strictly between -90 and 90 degrees")
    if abs(pitch_value) >= 90.0:
        raise ValueError("pitch must be strictly between -90 and 90 degrees")

    wind_to_body = _wind_to_body(alpha_value, beta_value)
    body_to_ned = _body_to_ned(roll_value, pitch_value, yaw_value)
    wind_to_ned = _mat_mul(body_to_ned, wind_to_body)
    velocity_wind = (speed_value, 0.0, 0.0)
    velocity_body = _mat_vec(wind_to_body, velocity_wind)
    velocity_ned = _mat_vec(body_to_ned, velocity_body)
    recovered_body = _mat_vec(_transpose(body_to_ned), velocity_ned)
    horizontal_speed = math.hypot(velocity_ned[0], velocity_ned[1])
    track_deg = math.degrees(math.atan2(velocity_ned[1], velocity_ned[0]))
    flight_path_deg = math.degrees(math.atan2(-velocity_ned[2], horizontal_speed))
    identity: Matrix = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

    return {
        "wind_to_body": wind_to_body,
        "body_to_ned": body_to_ned,
        "wind_to_ned": wind_to_ned,
        "velocity_wind": velocity_wind,
        "velocity_body": velocity_body,
        "velocity_ned": velocity_ned,
        "recovered_body": recovered_body,
        "track_deg": track_deg,
        "flight_path_deg": flight_path_deg,
        "orthogonality_error": max(
            _matrix_error(_mat_mul(_transpose(wind_to_body), wind_to_body), identity),
            _matrix_error(_mat_mul(_transpose(body_to_ned), body_to_ned), identity),
        ),
        "determinant_error": max(
            abs(_determinant(wind_to_body) - 1.0),
            abs(_determinant(body_to_ned) - 1.0),
        ),
        "round_trip_error": _vector_error(recovered_body, velocity_body),
        "norm_error": abs(_norm(velocity_ned) - speed_value),
    }


class P02ArtifactTests(unittest.TestCase):
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
                "number": 2,
                "id": "P02",
                "title": "Transform Between Aerospace Frames",
                "guiding_question": GUIDING_QUESTION,
                "phase": 1,
                "phase_title": "Point-mass flight",
                "slug": "transform-between-aerospace-frames",
                "folder": "modules/02-transform-between-aerospace-frames",
                "implementation_batch": "P02",
                "prerequisites": ["P01"],
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
        lesson = self.text["lesson.md"]
        walkthrough = self.text["walkthrough.md"]
        checks = self.text["checks.md"]
        combined = "\n".join((readme, lesson_script, lesson, walkthrough, checks)).lower()

        for name, text in {
            "README.md": readme,
            "lesson.m": lesson_script,
            "lesson.md": lesson,
            "walkthrough.md": walkthrough,
            "checks.md": checks,
        }.items():
            with self.subTest(file=name):
                self.assertIn(GUIDING_QUESTION, text)

        self.assertIn("p01", combined)
        self.assertRegex(combined, r"wind(?:[- ]axis| frame| axes)")
        self.assertRegex(combined, r"body(?:[- ]axis| frame| axes)")
        self.assertIn("north-east-down", combined)
        self.assertIn("read", walkthrough.lower())
        self.assertIn("baseline", walkthrough.lower())
        self.assertRegex(walkthrough.lower(), r"one lever|move yaw alone|one .* at a time")
        self.assertRegex(walkthrough.lower(), r"changed|delta|change")
        self.assertIn("mechanism", combined)
        self.assertIn("teach-back", combined)
        for placeholder in (
            "curriculum-scaffolded",
            "not implemented yet",
            "intentionally refuses",
            "activate its governed implementation batch",
            "todo",
            "tbd",
        ):
            self.assertNotIn(placeholder, combined)

    def test_model_is_transparent_directional_and_guarded(self) -> None:
        model = self.text["model.m"]
        compact = re.sub(r"\s+", "", model.replace("...", "")).lower()

        self.assertIn(
            "functionout=model(speed,alphadeg,betadeg,rolldeg,pitchdeg,yawdeg)",
            compact,
        )
        self.assertIn("arguments", model.lower())
        self.assertGreaterEqual(model.count("(1,1) double {mustBeFinite}"), 5)
        self.assertIn("(1,1) double {mustBeFinite,mustBePositive}", model)
        self.assertIn("abs(betaDeg)>=90", model)
        self.assertIn("abs(pitchDeg)>=90", model)
        self.assertIn("P02:model:SideslipRange", model)
        self.assertIn("P02:model:PitchSingularity", model)

        self.assertIn(
            "c_wind_to_body=[ca*cb,-ca*sb,-sa;sb,cb,0;sa*cb,-sa*sb,ca];",
            compact,
        )
        self.assertIn(
            "c_body_to_ned=[cp*cy,sr*sp*cy-cr*sy,cr*sp*cy+sr*sy;"
            "cp*sy,sr*sp*sy+cr*cy,cr*sp*sy-sr*cy;-sp,sr*cp,cr*cp];",
            compact,
        )
        self.assertIn("c_wind_to_ned=c_body_to_ned*c_wind_to_body;", compact)
        self.assertIn("velocitybody_mps=c_wind_to_body*velocitywind_mps;", compact)
        self.assertIn("velocityned_mps=c_body_to_ned*velocitybody_mps;", compact)
        self.assertIn("recoveredbody_mps=c_body_to_ned.'*velocityned_mps;", compact)

        for presentation_call in ("figure(", "plot(", "uiaxes(", "uifigure(", "disp(", "fprintf("):
            self.assertNotIn(presentation_call, model.lower())
        self.assertNotRegex(model.lower(), r"\b(?:while|parfor)\b")
        self.assertIn("eye(3)", compact)
        self.assertIn("[speed;0;0]", compact)

    def test_experiment_has_two_sweeps_metrics_and_transpose_failure(self) -> None:
        experiment = self.text["experiment.m"]
        lower = experiment.lower()
        self.assertGreaterEqual(experiment.count("%%"), 5)
        self.assertIn("baseline", lower)
        self.assertGreaterEqual(lower.count("sweep"), 2)
        self.assertRegex(lower, r"sweep[^\n]*yaw|yaw[^\n]*sweep")
        self.assertRegex(lower, r"sweep[^\n]*(?:beta|sideslip)|(?:beta|sideslip)[^\n]*sweep")
        self.assertIn("broken", lower)
        self.assertIn("transpose", lower)
        self.assertRegex(lower, r"wrong|incorrect|revers")
        self.assertGreaterEqual(lower.count("figure("), 3)
        self.assertIn("xlabel", lower)
        self.assertIn("ylabel", lower)
        self.assertIn("m/s", lower)
        self.assertIn("deg", lower)
        self.assertIn("fprintf", lower)
        self.assertIn("assert(", lower)
        self.assertRegex(lower, r"model\([^\n]*yaw")
        self.assertRegex(lower, r"model\([^\n]*(?:beta|sideslip)")
        self.assertNotRegex(lower, r"\bclose\s+all\b")
        self.assertIn("findall(groot", lower)
        self.assertIn("'^p[0-9][0-9] '", lower)

        for variable in ("yawSweepDeg", "betaSweepDeg"):
            match = re.search(rf"{variable}\s*=\s*\[([^\]]+)\]", experiment)
            self.assertIsNotNone(match, variable)
            values = [float(value) for value in match.group(1).split()]
            self.assertGreaterEqual(len(values), 3, variable)
            self.assertLessEqual(len(values), 25, variable)
            self.assertTrue(all(math.isfinite(value) for value in values), variable)

    def test_interaction_and_checks_cover_controls_limits_and_recovery(self) -> None:
        experiment = self.text["experiment.m"]
        interactive = self.text["interactive.m"].lower()
        checks_script = self.text["run_checks.m"]
        checks_lower = checks_script.lower()
        checks_compact = re.sub(r"\s+", "", checks_script.replace("...", ""))
        interactive_compact = re.sub(r"\s+", "", interactive.replace("...", ""))

        self.assertIn("clear model;", "\n".join(experiment.splitlines()[:10]).lower())
        self.assertIn("clear model;", "\n".join(interactive.splitlines()[:5]))
        self.assertIn("clear model;", "\n".join(checks_lower.splitlines()[:5]))
        self.assertRegex(experiment.lower(), r"clear run_checks;\s*run_checks;")

        self.assertIn("uifigure(", interactive)
        self.assertGreaterEqual(interactive.count("uislider("), 2)
        self.assertIn("yaw", interactive)
        self.assertRegex(interactive, r"beta|sideslip")
        self.assertIn("valuechangingfcn", interactive)
        self.assertIn("valuechangedfcn", interactive)
        self.assertIn("modelfcn=@model;", interactive_compact)
        self.assertIn("out=modelfcn(", interactive_compact)
        self.assertIn("m/s", interactive)
        self.assertIn("deg", interactive)

        self.assertGreaterEqual(checks_lower.count("assert("), 8)
        for concept in (
            "orthogonality",
            "determinant",
            "roundtrip",
            "norm",
            "yaw",
            "beta",
            "rollcase",
            "broken",
            "sidesliprange",
            "pitchsingularity",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, checks_lower.replace("-", ""))
        self.assertIn("catch exception", checks_lower)
        self.assertIn(
            "strcmp(exception.identifier,expectedIdentifier)",
            checks_compact,
        )
        self.assertIn("'P02:model:SideslipRange'", checks_script)
        self.assertIn("'P02:model:PitchSingularity'", checks_script)
        self.assertIn("P02 checks passed", checks_script)

    def test_no_opaque_toolbox_randomness_or_external_io(self) -> None:
        matlab = "\n".join(
            self.text[name]
            for name in ("model.m", "experiment.m", "interactive.m", "lesson.m", "run_checks.m")
        ).lower()
        forbidden_calls = (
            "angle2dcm",
            "dcm2angle",
            "eul2rotm",
            "rotm2eul",
            "quatrotate",
            "quaternion",
            "rotateframe",
            "rotatepoint",
            "aer2ned",
            "ned2body",
            "rotx",
            "roty",
            "rotz",
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
            "input",
            "eval",
            "feval",
        )
        for call in forbidden_calls:
            with self.subTest(call=call):
                self.assertNotRegex(matlab, rf"\b{call}\s*\(")
        self.assertNotRegex(matlab, r"\brand(?:n|i)?\s*\(")
        self.assertNotRegex(matlab, r"\brng\s*\(")
        self.assertNotRegex(matlab, r"\b(?:load|save)\s*\(")
        self.assertNotRegex(matlab, re.compile(r"^\s*(?:while|parfor)\b", re.MULTILINE))


class P02IndependentOracleTests(unittest.TestCase):
    def test_baseline_is_deterministic_bounded_and_physically_interpretable(self) -> None:
        first = _oracle()
        second = _oracle()
        self.assertEqual(first, second)
        self.assertAlmostEqual(first["track_deg"], 30.0, places=12)
        self.assertAlmostEqual(first["flight_path_deg"], 3.0, places=12)
        self.assertLess(first["orthogonality_error"], 1e-12)
        self.assertLess(first["determinant_error"], 1e-12)
        self.assertLess(first["round_trip_error"], 1e-12)
        self.assertLess(first["norm_error"], 1e-12)

    def test_rotation_norm_round_trip_and_limiting_cases(self) -> None:
        identity = _oracle(80.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertEqual(identity["velocity_wind"], (80.0, 0.0, 0.0))
        self.assertEqual(identity["velocity_body"], (80.0, 0.0, 0.0))
        self.assertEqual(identity["velocity_ned"], (80.0, 0.0, 0.0))

        east = _oracle(80.0, 0.0, 0.0, 0.0, 0.0, 90.0)
        self.assertLess(_vector_error(east["velocity_ned"], (0.0, 80.0, 0.0)), 1e-12)
        level = _oracle(80.0, 12.0, 0.0, 0.0, 12.0, -25.0)
        self.assertAlmostEqual(level["track_deg"], -25.0, places=12)
        self.assertAlmostEqual(level["flight_path_deg"], 0.0, places=12)

        for angles in ((-30.0, 10.0, 20.0, -15.0, 170.0), (45.0, -20.0, -35.0, 40.0, -120.0)):
            alpha, beta, roll, pitch, yaw = angles
            with self.subTest(angles=angles):
                result = _oracle(137.5, alpha, beta, roll, pitch, yaw)
                self.assertLess(result["orthogonality_error"], 2e-12)
                self.assertLess(result["determinant_error"], 2e-12)
                self.assertLess(result["round_trip_error"], 3e-12)
                self.assertLess(result["norm_error"], 3e-12)

    def test_yaw_and_sideslip_sweeps_change_distinct_observables(self) -> None:
        yaw_values = (-60.0, -30.0, 0.0, 30.0, 60.0)
        yaw_results = [_oracle(70.0, 0.0, 0.0, 0.0, 0.0, yaw) for yaw in yaw_values]
        self.assertEqual(
            [result["velocity_body"] for result in yaw_results],
            [(70.0, 0.0, 0.0)] * len(yaw_values),
        )
        for expected, result in zip(yaw_values, yaw_results):
            self.assertAlmostEqual(result["track_deg"], expected, places=12)

        beta_values = (-20.0, -10.0, 0.0, 10.0, 20.0)
        beta_results = [_oracle(70.0, 0.0, beta, 0.0, 0.0, 0.0) for beta in beta_values]
        body_lateral = [result["velocity_body"][1] for result in beta_results]
        self.assertEqual(body_lateral, sorted(body_lateral))
        for expected, result in zip(beta_values, beta_results):
            self.assertAlmostEqual(result["track_deg"], expected, places=12)
            self.assertAlmostEqual(_norm(result["velocity_body"]), 70.0, places=12)

    def test_positive_roll_maps_body_right_velocity_toward_down(self) -> None:
        result = _oracle(70.0, 0.0, 30.0, 90.0, 0.0, 0.0)
        expected_body = (
            70.0 * math.cos(math.radians(30.0)),
            70.0 * math.sin(math.radians(30.0)),
            0.0,
        )
        expected_ned = (expected_body[0], 0.0, expected_body[1])

        self.assertLess(_vector_error(result["velocity_body"], expected_body), 1e-12)
        self.assertLess(_vector_error(result["velocity_ned"], expected_ned), 1e-12)
        self.assertGreater(result["velocity_ned"][2], 0.0)

    def test_broken_transpose_is_detectable_but_correct_inverse_recovers(self) -> None:
        result = _oracle(90.0, 7.0, -6.0, -8.0, 12.0, 35.0)
        body_to_ned = result["body_to_ned"]
        velocity_body = result["velocity_body"]
        correct_ned = result["velocity_ned"]
        wrong_ned = _mat_vec(_transpose(body_to_ned), velocity_body)

        self.assertGreater(_vector_error(wrong_ned, correct_ned), 20.0)
        recovered = _mat_vec(_transpose(body_to_ned), correct_ned)
        self.assertLess(_vector_error(recovered, velocity_body), 2e-14)

    def test_malformed_inputs_fail_before_calculation(self) -> None:
        malformed_cases = (
            ("zero speed", (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
            ("negative speed", (-1.0, 0.0, 0.0, 0.0, 0.0, 0.0)),
            ("nan speed", (math.nan, 0.0, 0.0, 0.0, 0.0, 0.0)),
            ("infinite alpha", (70.0, math.inf, 0.0, 0.0, 0.0, 0.0)),
            ("vector beta", (70.0, 0.0, [1.0], 0.0, 0.0, 0.0)),
            ("beta lower boundary", (70.0, 0.0, -90.0, 0.0, 0.0, 0.0)),
            ("beta upper boundary", (70.0, 0.0, 90.0, 0.0, 0.0, 0.0)),
            ("pitch lower boundary", (70.0, 0.0, 0.0, 0.0, -90.0, 0.0)),
            ("pitch upper boundary", (70.0, 0.0, 0.0, 0.0, 90.0, 0.0)),
            ("text yaw", (70.0, 0.0, 0.0, 0.0, 0.0, "north")),
        )
        for name, values in malformed_cases:
            with self.subTest(case=name), self.assertRaises(ValueError):
                _oracle(*values)

    def test_representative_grid_remains_fixed_size_and_bounded(self) -> None:
        yaw_values = range(-180, 181, 15)
        beta_values = range(-60, 61, 10)
        case_count = 0
        for yaw in yaw_values:
            for beta in beta_values:
                result = _oracle(70.0, 6.0, float(beta), 5.0, 9.0, float(yaw))
                case_count += 1
                for matrix_name in ("wind_to_body", "body_to_ned", "wind_to_ned"):
                    matrix = result[matrix_name]
                    self.assertEqual(len(matrix), 3)
                    self.assertEqual([len(row) for row in matrix], [3, 3, 3])
                for vector_name in (
                    "velocity_wind",
                    "velocity_body",
                    "velocity_ned",
                    "recovered_body",
                ):
                    self.assertEqual(len(result[vector_name]), 3)
                self.assertLess(result["norm_error"], 3e-14)
        self.assertEqual(case_count, 325)
        self.assertLessEqual(case_count, 400)


if __name__ == "__main__":
    unittest.main()
