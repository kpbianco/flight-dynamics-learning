from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_ID = "P11"
MODULE_FOLDER = ROOT / "modules/11-model-flight-sensors"
GUIDING_QUESTION = (
    "What inputs, observable effects, and failure modes matter when you model "
    "Flight Sensors?"
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


def _integrate(
    time_s: tuple[float, ...], rate_deg_s: tuple[float, ...]
) -> tuple[float, ...]:
    angle = [0.0] * len(time_s)
    for index in range(len(time_s) - 1):
        step_s = time_s[index + 1] - time_s[index]
        angle[index + 1] = angle[index] + 0.5 * step_s * (
            rate_deg_s[index] + rate_deg_s[index + 1]
        )
    return tuple(angle)


def _transpose(
    matrix: tuple[tuple[float, ...], ...]
) -> tuple[tuple[float, ...], ...]:
    return tuple(tuple(row[column] for row in matrix) for column in range(3))


def _mat_vec(
    matrix: tuple[tuple[float, ...], ...], vector: tuple[float, ...]
) -> tuple[float, ...]:
    return tuple(
        sum(coefficient * component for coefficient, component in zip(row, vector))
        for row in matrix
    )


def _mat_mul(
    left: tuple[tuple[float, ...], ...],
    right: tuple[tuple[float, ...], ...],
) -> tuple[tuple[float, ...], ...]:
    right_transpose = _transpose(right)
    return tuple(
        tuple(sum(a * b for a, b in zip(row, column)) for column in right_transpose)
        for row in left
    )


def _determinant(matrix: tuple[tuple[float, ...], ...]) -> float:
    return (
        matrix[0][0]
        * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1]
        * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2]
        * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def _vector_add(
    left: tuple[float, ...], right: tuple[float, ...]
) -> tuple[float, ...]:
    return tuple(a + b for a, b in zip(left, right))


def _vector_subtract(
    left: tuple[float, ...], right: tuple[float, ...]
) -> tuple[float, ...]:
    return tuple(a - b for a, b in zip(left, right))


def _norm(vector: tuple[float, ...]) -> float:
    return math.sqrt(sum(value * value for value in vector))


def _max_difference(left: object, right: object) -> float:
    if isinstance(left, tuple) and isinstance(right, tuple):
        return max(
            (_max_difference(a, b) for a, b in zip(left, right)), default=0.0
        )
    return abs(float(left) - float(right))


def _oracle(
    gyro_bias_deg_s: object = 0.20,
    accelerometer_noise_rms_mps2: object = 0.15,
) -> dict[str, object]:
    """Pure-stdlib equation oracle independent of the MATLAB source."""
    bias = _bounded_scalar("gyro bias", gyro_bias_deg_s, -0.5, 0.5)
    noise_rms = _bounded_scalar(
        "accelerometer noise vector RMS",
        accelerometer_noise_rms_mps2,
        0.0,
        0.5,
    )
    sample_time_s = 0.01
    time_s = tuple(index * sample_time_s for index in range(801))
    pitch_rate_truth = tuple(
        10.0 * math.sin(2.0 * math.pi * (time - 1.0) / 4.0)
        if 1.0 < time < 5.0
        else 0.0
        for time in time_s
    )
    pitch_angle_truth = _integrate(time_s, pitch_rate_truth)
    acceleration_ned = tuple(
        (
            2.0 * math.sin(math.pi * (time - 2.0) / 2.0)
            if 2.0 < time < 4.0
            else 0.0,
            0.0,
            0.0,
        )
        for time in time_s
    )
    gravity_ned = (0.0, 0.0, 9.80665)

    body_to_ned = []
    ideal_specific_force = []
    broken_ideal_specific_force = []
    for angle_deg, acceleration in zip(pitch_angle_truth, acceleration_ned):
        pitch_rad = math.radians(angle_deg)
        cosine_pitch = math.cos(pitch_rad)
        sine_pitch = math.sin(pitch_rad)
        rotation = (
            (cosine_pitch, 0.0, sine_pitch),
            (0.0, 1.0, 0.0),
            (-sine_pitch, 0.0, cosine_pitch),
        )
        body_to_ned.append(rotation)
        ned_to_body = _transpose(rotation)
        ideal_specific_force.append(
            _mat_vec(ned_to_body, _vector_subtract(acceleration, gravity_ned))
        )
        broken_ideal_specific_force.append(
            _mat_vec(ned_to_body, acceleration)
        )

    raw_axes = (
        tuple(
            math.sin(2.0 * math.pi * 1.7 * time)
            + 0.35 * math.cos(2.0 * math.pi * 0.43 * time)
            for time in time_s
        ),
        tuple(
            math.cos(2.0 * math.pi * 1.1 * time)
            - 0.25 * math.sin(2.0 * math.pi * 0.61 * time)
            for time in time_s
        ),
        tuple(
            math.sin(2.0 * math.pi * 2.3 * time + 0.4)
            + 0.30 * math.cos(2.0 * math.pi * 0.29 * time)
            for time in time_s
        ),
    )
    centered_axes = tuple(
        tuple(value - sum(axis) / len(axis) for value in axis) for axis in raw_axes
    )
    raw_vector_rms = math.sqrt(
        sum(
            sum(axis[index] * axis[index] for axis in centered_axes)
            for index in range(len(time_s))
        )
        / len(time_s)
    )
    unit_noise = tuple(
        tuple(axis[index] / raw_vector_rms for axis in centered_axes)
        for index in range(len(time_s))
    )
    accelerometer_noise = tuple(
        tuple(noise_rms * component for component in vector)
        for vector in unit_noise
    )
    measured_specific_force = tuple(
        _vector_add(ideal, noise)
        for ideal, noise in zip(ideal_specific_force, accelerometer_noise)
    )
    broken_measured_specific_force = tuple(
        _vector_add(ideal, noise)
        for ideal, noise in zip(broken_ideal_specific_force, accelerometer_noise)
    )

    pitch_rate_measured = tuple(value + bias for value in pitch_rate_truth)
    pitch_angle_measured = _integrate(time_s, pitch_rate_measured)
    pitch_angle_error = tuple(
        measured - truth
        for measured, truth in zip(pitch_angle_measured, pitch_angle_truth)
    )
    gravity_omission = tuple(
        _vector_subtract(complete, broken)
        for complete, broken in zip(
            measured_specific_force, broken_measured_specific_force
        )
    )
    noise_vector_rms_measured = math.sqrt(
        sum(_norm(vector) ** 2 for vector in accelerometer_noise) / len(time_s)
    )

    return {
        "gyro_bias_deg_s": bias,
        "accelerometer_noise_rms_mps2": noise_rms,
        "sample_time_s": sample_time_s,
        "time_horizon_s": 8.0,
        "time_s": time_s,
        "sample_count": len(time_s),
        "update_count": len(time_s) - 1,
        "gravity_ned_mps2": gravity_ned,
        "pitch_rate_truth_deg_s": pitch_rate_truth,
        "pitch_angle_truth_deg": pitch_angle_truth,
        "acceleration_ned_mps2": acceleration_ned,
        "body_to_ned": tuple(body_to_ned),
        "ideal_specific_force_body_mps2": tuple(ideal_specific_force),
        "broken_ideal_specific_force_body_mps2": tuple(
            broken_ideal_specific_force
        ),
        "unit_noise_shape": unit_noise,
        "accelerometer_noise_body_mps2": accelerometer_noise,
        "accelerometer_measured_body_mps2": measured_specific_force,
        "broken_accelerometer_measured_body_mps2": (
            broken_measured_specific_force
        ),
        "pitch_rate_measured_deg_s": pitch_rate_measured,
        "pitch_angle_measured_deg": pitch_angle_measured,
        "pitch_angle_error_deg": pitch_angle_error,
        "gravity_omission_body_mps2": gravity_omission,
        "gravity_omission_magnitude_mps2": tuple(
            _norm(vector) for vector in gravity_omission
        ),
        "noise_vector_rms_measured_mps2": noise_vector_rms_measured,
        "noise_mean_body_mps2": tuple(
            sum(vector[axis] for vector in accelerometer_noise) / len(time_s)
            for axis in range(3)
        ),
        "peak_pitch_rate_truth_deg_s": max(map(abs, pitch_rate_truth)),
        "peak_pitch_angle_truth_deg": max(map(abs, pitch_angle_truth)),
        "final_pitch_angle_error_deg": pitch_angle_error[-1],
    }


class P11ArtifactTests(unittest.TestCase):
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
                "number": 11,
                "id": "P11",
                "title": "Model Flight Sensors",
                "guiding_question": GUIDING_QUESTION,
                "phase": 3,
                "phase_title": "Six-degree-of-freedom simulation",
                "slug": "model-flight-sensors",
                "folder": "modules/11-model-flight-sensors",
                "implementation_batch": "P11",
                "prerequisites": ["P10"],
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

    def test_learning_slice_is_concept_first_complete_and_bounded(self) -> None:
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
            "p10",
            "truth",
            "gyro",
            "accelerometer",
            "specific force",
            "north-east-down",
            "body z",
            "deg/s",
            "m/s^2",
            "mechanism",
            "reset",
            "broken",
            "gravity",
            "teach-back",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, combined)
        self.assertRegex(walkthrough.lower(), r"one plot|one .* at a time")
        self.assertIn("not current api compatibility", combined)
        self.assertIn("not white gaussian", combined)
        self.assertIn("does not implement fusion", combined)
        experiment_lower = experiment.lower()
        self.assertEqual(experiment_lower.count("predict once:"), 1)
        self.assertLess(
            experiment_lower.index("predict once:"),
            experiment_lower.index("baseline=model("),
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
            "functionout=model(gyrobias_deg_s,accelerometernoiserms_mps2)", compact
        )
        self.assertIn("arguments", lower)
        self.assertIn(
            "gyrobias_deg_s(1,1)double{mustbereal,mustbefinite}=0.20", compact
        )
        self.assertIn(
            "accelerometernoiserms_mps2(1,1)double{mustbereal,mustbefinite}=0.15",
            compact,
        )
        self.assertIn("gyrobias_deg_s<-0.5||gyrobias_deg_s>0.5", compact)
        self.assertIn(
            "accelerometernoiserms_mps2<0||accelerometernoiserms_mps2>0.5",
            compact,
        )
        for identifier in (
            "P11:model:GyroBiasRange",
            "P11:model:AccelerometerNoiseRange",
        ):
            self.assertIn(identifier, model)

        for expression in (
            "sampletime_s=0.01;",
            "timehorizon_s=8;",
            "time_s=0:sampletime_s:timehorizon_s;",
            "gravityned_mps2=[0;0;gravity_mps2];",
            "pitchactive=time_s>1&time_s<5;",
            "accelerationactive=time_s>2&time_s<4;",
            "idealspecificforcebody_mps2(:,k)=nedtobody*(accelerationned_mps2(:,k)-gravityned_mps2);",
            "pitchratemeasured_deg_s=pitchratetruth_deg_s+gyrobias_deg_s;",
            "angle_deg(k+1)=angle_deg(k)+0.5*step_s*(rate_deg_s(k)+rate_deg_s(k+1));",
        ):
            with self.subTest(expression=expression):
                self.assertIn(expression, compact)
        self.assertIn("rawnoiseshape=rawnoiseshape-mean(rawnoiseshape,2);", compact)
        self.assertIn("unitnoiseshape=rawnoiseshape/rawnoisevectorrms;", compact)

        for field in (
            "pitchRateTruth_deg_s",
            "pitchAngleTruth_deg",
            "accelerationNED_mps2",
            "bodyToNED",
            "pitchRateMeasured_deg_s",
            "pitchAngleError_deg",
            "idealSpecificForceBody_mps2",
            "accelerometerNoiseBody_mps2",
            "accelerometerMeasuredBody_mps2",
            "specificForceEquationResidualNED_mps2",
            "brokenIdealSpecificForceBody_mps2",
            "brokenAccelerometerMeasuredBody_mps2",
            "gravityOmissionMagnitude_mps2",
            "frameConvention",
            "noiseDefinition",
            "brokenCaseDefinition",
            "analysisScope",
        ):
            with self.subTest(field=field):
                self.assertIn(f"'{field}'", model)
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
            lower, re.compile(r"^\s*(?:while|parfor)\b", re.MULTILINE)
        )

    def test_experiment_has_two_isolated_sweeps_and_broken_case(self) -> None:
        experiment = self.text["experiment.m"]
        lower = experiment.lower()
        compact = re.sub(r"\s+", "", experiment.replace("...", "")).lower()
        self.assertGreaterEqual(experiment.count("%%"), 14)
        self.assertGreaterEqual(lower.count("figure("), 5)
        self.assertGreaterEqual(lower.count("assert("), 7)
        for concept in (
            "baseline",
            "gyro bias",
            "accelerometer",
            "specific force",
            "changed view",
            "mechanism",
            "limiting cases",
            "broken",
            "gravity",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, lower)
        for unit in ("deg/s", "deg", "m/s^2", "s"):
            self.assertIn(unit, lower)
        self.assertIn("model(gyrobiassweep_deg_s(k),0.15)", compact)
        self.assertIn("model(0.20,accelerometerrmssweep_mps2(k))", compact)
        self.assertIn("ideal=model(0,0)", compact)
        self.assertIn("baseline.brokenidealspecificforcebody_mps2", compact)
        self.assertNotRegex(lower, r"\bclose\s+all\b")
        self.assertIn("findall(groot", lower)
        self.assertIn("'^p11 '", lower)
        self.assertRegex(lower, r"clear run_checks;\s*run_checks;")
        self.assertNotIn("interactive;", lower)

        assignments = re.findall(
            r"(?:gyroBiasSweep_deg_s|accelerometerRmsSweep_mps2)\s*=\s*\[([^\]]+)\]",
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
            self.assertGreaterEqual(len(values), 5)
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
        self.assertIn("clear model;", "\n".join(interactive_lower.splitlines()[:8]))
        self.assertIn("clear model;", "\n".join(checks_lower.splitlines()[:6]))
        self.assertIn("uifigure(", interactive_lower)
        self.assertIn("p11", interactive_lower)
        self.assertIn("existingui=findall(groot", interactive_compact)
        self.assertIn("close(existingui)", interactive_compact)
        self.assertEqual(interactive_lower.count("uislider("), 2)
        self.assertEqual(interactive_lower.count("uibutton("), 1)
        self.assertIn("'limits',[-0.50.5]", interactive_compact)
        self.assertIn("'limits',[00.5]", interactive_compact)
        self.assertIn("'value',0.20", interactive_compact)
        self.assertIn("'value',0.15", interactive_compact)
        self.assertIn("valuechangingfcn", interactive_lower)
        self.assertIn("valuechangedfcn", interactive_lower)
        self.assertIn("event.value", interactive_lower)
        self.assertIn("buttonpushedfcn", interactive_lower)
        self.assertIn("functionresetbaseline", interactive_compact)
        self.assertIn("biascontrol.value=0.20", interactive_compact)
        self.assertIn("noisecontrol.value=0.15", interactive_compact)
        self.assertIn("modelfcn=@model;", interactive_compact)
        self.assertEqual(interactive_compact.count("out=modelfcn("), 1)
        self.assertGreaterEqual(interactive_compact.count("cla("), 4)
        self.assertNotIn("yyaxis", interactive_lower)
        for unit in ("deg/s", "deg", "m/s^2", "s"):
            self.assertIn(unit, interactive_lower)

        self.assertGreaterEqual(checks_lower.count("assert("), 30)
        for concept in (
            "determinism",
            "fixed shape",
            "finite resources",
            "independently reconstruct truth",
            "every integration update",
            "supported level rest",
            "nonlevel frame-sign signature",
            "ideal limits",
            "isolated parameter sweeps",
            "broken specific-force equation",
            "malformed inputs",
            "rejected inputs",
            "recovery",
            "rollback",
            "timeout",
            "cancellation",
            "compatibility",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, combined_checks)
        self.assertIn("samplecount==801", checks_compact)
        self.assertIn("updatecount==800", checks_compact)
        self.assertRegex(checks_compact, r"representativecasecount==9\b")
        self.assertIn("catch exception", checks_lower)
        self.assertIn(
            "strcmp(exception.identifier,expectedidentifier)", checks_compact
        )
        self.assertIn("P11 checks passed", checks_script)

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
            "quadprog",
            "optimproblem",
            "solve",
            "vpasolve",
            "trim",
            "findop",
            "linearize",
            "sim",
            "load_system",
            "open_system",
            "ss",
            "tf",
            "lsim",
            "initial",
            "impulse",
            "step",
            "ode45",
            "ode23",
            "ode15s",
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

    def test_retained_evidence_maps_acceptance_and_claim_boundaries(self) -> None:
        evidence_path = ROOT / "docs/evidence/P11-2026-08-23.md"
        evidence = evidence_path.read_text(encoding="utf-8")
        self.assertTrue(evidence.endswith("\n"))
        self.assertFalse(evidence.endswith("\n\n"))
        for heading in (
            "## Acceptance mapping",
            "## Figure, control, metric, and unit inventory",
            "## Changed and preserved invariants",
            "## Residual risks and limitations",
            "## Rollback",
            "## Explicitly unperformed validation",
        ):
            self.assertIn(heading, evidence)
        for boundary in (
            "does not establish MATLAB execution",
            "physical hardware",
            "hardware-in-the-loop (HIL)",
            "field",
            "deployment",
            "production validation",
        ):
            self.assertIn(boundary, evidence)
        match = re.search(r"```json\n(.*?)\n```", evidence, re.DOTALL)
        self.assertIsNotNone(match)
        summary = json.loads(match.group(1))
        self.assertEqual(summary["schema_version"], 1)
        self.assertEqual(summary["batch_id"], "P11")
        self.assertEqual(summary["status"], "pass")
        self.assertEqual(len(summary["acceptance"]), 8)
        self.assertTrue(
            all(item["status"] == "pass" for item in summary["acceptance"])
        )


class P11IndependentOracleTests(unittest.TestCase):
    def test_baseline_is_deterministic_and_recognizable(self) -> None:
        first = _oracle()
        second = _oracle()
        self.assertEqual(first, second)
        self.assertEqual(first["sample_count"], 801)
        self.assertEqual(first["update_count"], 800)
        self.assertEqual(first["time_s"][0], 0.0)
        self.assertEqual(first["time_s"][-1], 8.0)
        self.assertEqual(first["sample_time_s"], 0.01)
        self.assertEqual(first["peak_pitch_rate_truth_deg_s"], 10.0)
        self.assertAlmostEqual(
            first["peak_pitch_angle_truth_deg"], 12.732133646887217
        )
        self.assertAlmostEqual(first["pitch_angle_truth_deg"][-1], 0.0)
        self.assertAlmostEqual(first["final_pitch_angle_error_deg"], 1.6)
        self.assertAlmostEqual(first["noise_vector_rms_measured_mps2"], 0.15)
        self.assertEqual(max(vector[0] for vector in first["acceleration_ned_mps2"]), 2.0)

    def test_shapes_finite_values_and_resource_boundary(self) -> None:
        result = _oracle()
        self.assertEqual(len(result["time_s"]), 801)
        scalar_histories = (
            "pitch_rate_truth_deg_s",
            "pitch_angle_truth_deg",
            "pitch_rate_measured_deg_s",
            "pitch_angle_measured_deg",
            "pitch_angle_error_deg",
            "gravity_omission_magnitude_mps2",
        )
        for name in scalar_histories:
            history = result[name]
            self.assertEqual(len(history), 801, name)
            self.assertTrue(all(math.isfinite(value) for value in history), name)
        vector_histories = (
            "acceleration_ned_mps2",
            "ideal_specific_force_body_mps2",
            "broken_ideal_specific_force_body_mps2",
            "unit_noise_shape",
            "accelerometer_noise_body_mps2",
            "accelerometer_measured_body_mps2",
            "broken_accelerometer_measured_body_mps2",
            "gravity_omission_body_mps2",
        )
        for name in vector_histories:
            history = result[name]
            self.assertEqual(len(history), 801, name)
            self.assertTrue(all(len(vector) == 3 for vector in history), name)
            self.assertTrue(
                all(math.isfinite(value) for vector in history for value in vector),
                name,
            )
        self.assertEqual(len(result["body_to_ned"]), 801)

    def test_truth_schedule_and_every_integration_update_close(self) -> None:
        result = _oracle()
        time_s = result["time_s"]
        truth_rate = result["pitch_rate_truth_deg_s"]
        measured_rate = result["pitch_rate_measured_deg_s"]
        truth_angle = result["pitch_angle_truth_deg"]
        measured_angle = result["pitch_angle_measured_deg"]
        for index in range(result["update_count"]):
            step_s = time_s[index + 1] - time_s[index]
            with self.subTest(index=index):
                self.assertAlmostEqual(
                    truth_angle[index + 1],
                    truth_angle[index]
                    + 0.5 * step_s * (truth_rate[index] + truth_rate[index + 1]),
                )
                self.assertAlmostEqual(
                    measured_angle[index + 1],
                    measured_angle[index]
                    + 0.5
                    * step_s
                    * (measured_rate[index] + measured_rate[index + 1]),
                )
        for time, rate in zip(time_s, truth_rate):
            expected = (
                10.0 * math.sin(2.0 * math.pi * (time - 1.0) / 4.0)
                if 1.0 < time < 5.0
                else 0.0
            )
            self.assertEqual(rate, expected)
        for time, angle_error in zip(time_s, result["pitch_angle_error_deg"]):
            self.assertAlmostEqual(angle_error, 0.20 * time, places=12)

    def test_frames_specific_force_rest_and_closure(self) -> None:
        result = _oracle()
        gravity = result["gravity_ned_mps2"]
        self.assertEqual(result["acceleration_ned_mps2"][0], (0.0, 0.0, 0.0))
        self.assertEqual(result["body_to_ned"][0], ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))
        self.assertEqual(
            result["ideal_specific_force_body_mps2"][0],
            (0.0, 0.0, -9.80665),
        )
        self.assertEqual(
            result["broken_ideal_specific_force_body_mps2"][0],
            (0.0, 0.0, 0.0),
        )
        for rotation, acceleration, specific_force in zip(
            result["body_to_ned"],
            result["acceleration_ned_mps2"],
            result["ideal_specific_force_body_mps2"],
        ):
            with self.subTest(rotation=rotation):
                identity = _mat_mul(_transpose(rotation), rotation)
                self.assertLess(
                    _max_difference(
                        identity,
                        ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
                    ),
                    1e-14,
                )
                self.assertAlmostEqual(_determinant(rotation), 1.0)
                reconstructed = _vector_add(
                    _mat_vec(rotation, specific_force), gravity
                )
                self.assertLess(_max_difference(reconstructed, acceleration), 1e-12)

    def test_nonlevel_pitch_sign_sets_dynamic_body_force_directions(self) -> None:
        result = _oracle(0.0, 0.0)
        index = result["time_s"].index(3.0)
        angle_deg = result["pitch_angle_truth_deg"][index]
        self.assertAlmostEqual(angle_deg, 12.732133646887217)
        self.assertEqual(result["acceleration_ned_mps2"][index], (2.0, 0.0, 0.0))

        angle_rad = math.radians(angle_deg)
        cosine_pitch = math.cos(angle_rad)
        sine_pitch = math.sin(angle_rad)
        expected_rotation = (
            (cosine_pitch, 0.0, sine_pitch),
            (0.0, 1.0, 0.0),
            (-sine_pitch, 0.0, cosine_pitch),
        )
        expected_ideal = (
            2.0 * cosine_pitch + 9.80665 * sine_pitch,
            0.0,
            2.0 * sine_pitch - 9.80665 * cosine_pitch,
        )
        expected_broken = (
            2.0 * cosine_pitch,
            0.0,
            2.0 * sine_pitch,
        )
        expected_omission = (
            9.80665 * sine_pitch,
            0.0,
            -9.80665 * cosine_pitch,
        )

        self.assertLess(
            _max_difference(result["body_to_ned"][index], expected_rotation),
            1e-14,
        )
        self.assertLess(
            _max_difference(
                result["ideal_specific_force_body_mps2"][index], expected_ideal
            ),
            1e-14,
        )
        self.assertLess(
            _max_difference(
                result["broken_ideal_specific_force_body_mps2"][index],
                expected_broken,
            ),
            1e-14,
        )
        self.assertLess(
            _max_difference(
                result["gravity_omission_body_mps2"][index], expected_omission
            ),
            1e-14,
        )
        self.assertGreater(expected_ideal[0], 2.0)
        self.assertLess(expected_ideal[2], 0.0)

    def test_deterministic_noise_has_declared_mean_and_vector_rms(self) -> None:
        result = _oracle()
        for mean in result["noise_mean_body_mps2"]:
            self.assertAlmostEqual(mean, 0.0, places=14)
        self.assertAlmostEqual(result["noise_vector_rms_measured_mps2"], 0.15)
        unit_rms = math.sqrt(
            sum(_norm(vector) ** 2 for vector in result["unit_noise_shape"])
            / result["sample_count"]
        )
        self.assertAlmostEqual(unit_rms, 1.0)
        self.assertEqual(_oracle(), result)

    def test_two_sweeps_change_only_the_selected_sensor_error(self) -> None:
        biases = (-0.50, -0.25, 0.0, 0.25, 0.50)
        bias_results = [_oracle(value, 0.15) for value in biases]
        expected_final_errors = (-4.0, -2.0, 0.0, 2.0, 4.0)
        for expected, result in zip(expected_final_errors, bias_results):
            self.assertAlmostEqual(result["final_pitch_angle_error_deg"], expected)
            for fixed_field in (
                "pitch_rate_truth_deg_s",
                "pitch_angle_truth_deg",
                "acceleration_ned_mps2",
                "body_to_ned",
                "ideal_specific_force_body_mps2",
                "accelerometer_noise_body_mps2",
                "accelerometer_measured_body_mps2",
                "broken_accelerometer_measured_body_mps2",
            ):
                self.assertEqual(result[fixed_field], bias_results[0][fixed_field])

        noise_values = (0.0, 0.05, 0.15, 0.30, 0.50)
        noise_results = [_oracle(0.20, value) for value in noise_values]
        for expected, result in zip(noise_values, noise_results):
            self.assertAlmostEqual(result["noise_vector_rms_measured_mps2"], expected)
            for fixed_field in (
                "pitch_rate_truth_deg_s",
                "pitch_angle_truth_deg",
                "acceleration_ned_mps2",
                "body_to_ned",
                "ideal_specific_force_body_mps2",
                "pitch_rate_measured_deg_s",
                "pitch_angle_measured_deg",
            ):
                self.assertEqual(result[fixed_field], noise_results[0][fixed_field])
        self.assertTrue(
            all(
                left < right
                for left, right in zip(noise_values, noise_values[1:])
            )
        )

    def test_zero_error_limits_are_exact(self) -> None:
        zero_bias = _oracle(0.0, 0.15)
        self.assertEqual(
            zero_bias["pitch_rate_measured_deg_s"],
            zero_bias["pitch_rate_truth_deg_s"],
        )
        self.assertEqual(
            zero_bias["pitch_angle_measured_deg"],
            zero_bias["pitch_angle_truth_deg"],
        )
        zero_noise = _oracle(0.20, 0.0)
        self.assertTrue(
            all(
                all(component == 0.0 for component in vector)
                for vector in zero_noise["accelerometer_noise_body_mps2"]
            )
        )
        self.assertEqual(
            zero_noise["accelerometer_measured_body_mps2"],
            zero_noise["ideal_specific_force_body_mps2"],
        )

    def test_broken_case_omits_only_gravity(self) -> None:
        result = _oracle()
        for rotation, complete, broken, complete_ideal, broken_ideal, noise in zip(
            result["body_to_ned"],
            result["accelerometer_measured_body_mps2"],
            result["broken_accelerometer_measured_body_mps2"],
            result["ideal_specific_force_body_mps2"],
            result["broken_ideal_specific_force_body_mps2"],
            result["accelerometer_noise_body_mps2"],
        ):
            expected_omission = _mat_vec(
                _transpose(rotation), (0.0, 0.0, -9.80665)
            )
            self.assertLess(
                _max_difference(_vector_subtract(complete, broken), expected_omission),
                1e-14,
            )
            self.assertLess(
                _max_difference(_vector_subtract(complete_ideal, broken_ideal), expected_omission),
                1e-14,
            )
            self.assertLess(_max_difference(_vector_subtract(complete, complete_ideal), noise), 1e-14)
            self.assertLess(_max_difference(_vector_subtract(broken, broken_ideal), noise), 1e-14)
        for magnitude in result["gravity_omission_magnitude_mps2"]:
            self.assertAlmostEqual(magnitude, 9.80665)

    def test_malformed_inputs_fail_without_poisoning_recovery(self) -> None:
        baseline = _oracle()
        malformed = (
            ("bias below", (-0.5001, 0.15)),
            ("bias above", (0.5001, 0.15)),
            ("noise below", (0.20, -0.001)),
            ("noise above", (0.20, 0.5001)),
            ("nan bias", (math.nan, 0.15)),
            ("infinite noise", (0.20, math.inf)),
            ("list bias", ([0.20], 0.15)),
            ("complex noise", (0.20, 0.15 + 1j)),
            ("text bias", ("biased", 0.15)),
            ("boolean noise", (0.20, True)),
        )
        for name, values in malformed:
            with self.subTest(case=name), self.assertRaises(ValueError):
                _oracle(*values)
        self.assertEqual(_oracle(), baseline)

    def test_accepted_corners_and_representative_grid_are_bounded(self) -> None:
        corner_count = 0
        for bias in (-0.5, 0.5):
            for noise_rms in (0.0, 0.5):
                result = _oracle(bias, noise_rms)
                corner_count += 1
                self.assertEqual(result["sample_count"], 801)
                self.assertEqual(result["update_count"], 800)
                self.assertAlmostEqual(
                    result["final_pitch_angle_error_deg"], bias * 8.0
                )
                self.assertAlmostEqual(
                    result["noise_vector_rms_measured_mps2"], noise_rms
                )
                self.assertTrue(
                    all(
                        math.isfinite(value)
                        for vector in result["accelerometer_measured_body_mps2"]
                        for value in vector
                    )
                )
        self.assertEqual(corner_count, 4)

        representative_count = 0
        for bias in (-0.5, 0.20, 0.5):
            for noise_rms in (0.0, 0.15, 0.5):
                result = _oracle(bias, noise_rms)
                representative_count += 1
                self.assertEqual(len(result["time_s"]), 801)
        self.assertEqual(representative_count, 9)
        self.assertLessEqual(representative_count, 10)


if __name__ == "__main__":
    unittest.main()
