from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_ID = "P17"
MODULE_FOLDER = ROOT / "modules/17-fuse-ins-and-gps"
EVIDENCE_PATH = ROOT / "docs/evidence/P17-2026-08-23.md"
GUIDING_QUESTION = (
    "What inputs, observable effects, and failure modes matter when you fuse "
    "INS and GPS?"
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


def _fusion_mode(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("fusion mode must be -1, 0, or +1")
    result = float(value)
    if not math.isfinite(result) or result not in (-1.0, 0.0, 1.0):
        raise ValueError("fusion mode must be -1, 0, or +1")
    return int(result)


def _oracle(
    ins_acceleration_bias_mps2: object = 0.04,
    gps_position_error_rms_m: object = 1.0,
    fusion_mode: object = 1,
) -> dict[str, object]:
    """Independent standard-library implementation of the declared equations."""
    bias = _bounded_scalar(
        "INS acceleration bias", ins_acceleration_bias_mps2, -0.08, 0.08
    )
    gps_rms = _bounded_scalar(
        "GPS position-error RMS", gps_position_error_rms_m, 0.0, 4.0
    )
    mode = _fusion_mode(fusion_mode)

    sample_time_s = 0.02
    time_horizon_s = 60.0
    time_s = tuple(
        index * sample_time_s
        for index in range(round(time_horizon_s / sample_time_s) + 1)
    )
    sample_count = len(time_s)
    interval_count = sample_count - 1
    initial_north_position_m = 0.0
    initial_north_velocity_mps = 20.0
    gps_update_period_s = 1.0
    gps_update_step_count = round(gps_update_period_s / sample_time_s)
    gps_update_indices = tuple(
        range(gps_update_step_count, sample_count, gps_update_step_count)
    )
    gps_update_mask = tuple(
        index in set(gps_update_indices) for index in range(sample_count)
    )
    gps_position_gain = 0.45
    gps_velocity_gain = 0.12
    innovation_gate_m = 25.0
    outlier_time_s = 30.0
    outlier_index = round(outlier_time_s / sample_time_s)
    outlier_magnitude_m = 80.0

    north_acceleration_truth_mps2 = tuple(
        0.5
        if 5.0 <= time < 15.0
        else -0.5
        if 25.0 <= time < 35.0
        else 0.0
        for time in time_s
    )
    north_velocity_truth_mps = [initial_north_velocity_mps] * sample_count
    north_position_truth_m = [initial_north_position_m] * sample_count
    for index in range(interval_count):
        acceleration = north_acceleration_truth_mps2[index]
        north_position_truth_m[index + 1] = (
            north_position_truth_m[index]
            + sample_time_s * north_velocity_truth_mps[index]
            + 0.5 * sample_time_s**2 * acceleration
        )
        north_velocity_truth_mps[index + 1] = (
            north_velocity_truth_mps[index] + sample_time_s * acceleration
        )

    north_acceleration_ins_mps2 = tuple(
        acceleration + bias for acceleration in north_acceleration_truth_mps2
    )
    north_velocity_ins_only_mps = [initial_north_velocity_mps] * sample_count
    north_position_ins_only_m = [initial_north_position_m] * sample_count
    for index in range(interval_count):
        acceleration = north_acceleration_ins_mps2[index]
        north_position_ins_only_m[index + 1] = (
            north_position_ins_only_m[index]
            + sample_time_s * north_velocity_ins_only_mps[index]
            + 0.5 * sample_time_s**2 * acceleration
        )
        north_velocity_ins_only_mps[index + 1] = (
            north_velocity_ins_only_mps[index] + sample_time_s * acceleration
        )

    gps_update_time_s = tuple(time_s[index] for index in gps_update_indices)
    raw_gps_position_error_shape = tuple(
        math.sin(2.0 * math.pi * time / 17.0)
        + 0.35 * math.cos(2.0 * math.pi * time / 7.0 + 0.2)
        for time in gps_update_time_s
    )
    raw_mean = sum(raw_gps_position_error_shape) / len(
        raw_gps_position_error_shape
    )
    mean_removed_shape = tuple(
        value - raw_mean for value in raw_gps_position_error_shape
    )
    raw_rms = math.sqrt(
        sum(value**2 for value in mean_removed_shape) / len(mean_removed_shape)
    )
    unit_gps_position_error_shape = tuple(
        value / raw_rms for value in mean_removed_shape
    )
    gps_nominal_position_error_m = [0.0] * sample_count
    for index, shape in zip(gps_update_indices, unit_gps_position_error_shape):
        gps_nominal_position_error_m[index] = gps_rms * shape
    gps_outlier_m = [0.0] * sample_count
    gps_outlier_m[outlier_index] = outlier_magnitude_m
    gps_position_measurement_m = [0.0] * sample_count
    for index in gps_update_indices:
        gps_position_measurement_m[index] = (
            north_position_truth_m[index]
            + gps_nominal_position_error_m[index]
            + gps_outlier_m[index]
        )

    north_position_predicted_m = [initial_north_position_m] * sample_count
    north_velocity_predicted_mps = [initial_north_velocity_mps] * sample_count
    north_position_fused_m = [initial_north_position_m] * sample_count
    north_velocity_fused_mps = [initial_north_velocity_mps] * sample_count
    gps_innovation_m = [0.0] * sample_count
    gps_position_correction_m = [0.0] * sample_count
    gps_velocity_correction_mps = [0.0] * sample_count
    gps_accepted = [False] * sample_count
    gps_rejected = [False] * sample_count
    gps_ignored = [False] * sample_count
    update_indices = set(gps_update_indices)

    for index in range(1, sample_count):
        acceleration = north_acceleration_ins_mps2[index - 1]
        north_position_predicted_m[index] = (
            north_position_fused_m[index - 1]
            + sample_time_s * north_velocity_fused_mps[index - 1]
            + 0.5 * sample_time_s**2 * acceleration
        )
        north_velocity_predicted_mps[index] = (
            north_velocity_fused_mps[index - 1]
            + sample_time_s * acceleration
        )
        if index in update_indices:
            gps_innovation_m[index] = (
                gps_position_measurement_m[index]
                - north_position_predicted_m[index]
            )
            if mode == -1:
                gps_accepted[index] = True
            elif mode == 1:
                gps_accepted[index] = (
                    abs(gps_innovation_m[index]) <= innovation_gate_m
                )
                gps_rejected[index] = not gps_accepted[index]
            else:
                gps_ignored[index] = True
            if gps_accepted[index]:
                gps_position_correction_m[index] = (
                    gps_position_gain * gps_innovation_m[index]
                )
                gps_velocity_correction_mps[index] = (
                    gps_velocity_gain
                    / gps_update_period_s
                    * gps_innovation_m[index]
                )
        north_position_fused_m[index] = (
            north_position_predicted_m[index]
            + gps_position_correction_m[index]
        )
        north_velocity_fused_mps[index] = (
            north_velocity_predicted_mps[index]
            + gps_velocity_correction_mps[index]
        )

    north_position_ins_only_error_m = tuple(
        estimated - truth
        for estimated, truth in zip(
            north_position_ins_only_m, north_position_truth_m
        )
    )
    north_velocity_ins_only_error_mps = tuple(
        estimated - truth
        for estimated, truth in zip(
            north_velocity_ins_only_mps, north_velocity_truth_mps
        )
    )
    north_position_fused_error_m = tuple(
        estimated - truth
        for estimated, truth in zip(north_position_fused_m, north_position_truth_m)
    )
    north_velocity_fused_error_mps = tuple(
        estimated - truth
        for estimated, truth in zip(north_velocity_fused_mps, north_velocity_truth_mps)
    )
    expected_ins_only_velocity_error_mps = tuple(bias * time for time in time_s)
    expected_ins_only_position_error_m = tuple(
        0.5 * bias * time**2 for time in time_s
    )
    gps_position_error_rms_measured_m = math.sqrt(
        sum(gps_nominal_position_error_m[index] ** 2 for index in gps_update_indices)
        / len(gps_update_indices)
    )

    return {
        "ins_acceleration_bias_mps2": bias,
        "gps_position_error_rms_m": gps_rms,
        "fusion_mode": mode,
        "sample_time_s": sample_time_s,
        "time_horizon_s": time_horizon_s,
        "time_s": time_s,
        "sample_count": sample_count,
        "interval_count": interval_count,
        "initial_north_position_m": initial_north_position_m,
        "initial_north_velocity_mps": initial_north_velocity_mps,
        "gps_update_period_s": gps_update_period_s,
        "gps_update_step_count": gps_update_step_count,
        "gps_update_indices": gps_update_indices,
        "gps_update_mask": gps_update_mask,
        "gps_update_count": len(gps_update_indices),
        "gps_position_gain": gps_position_gain,
        "gps_velocity_gain": gps_velocity_gain,
        "innovation_gate_m": innovation_gate_m,
        "outlier_time_s": outlier_time_s,
        "outlier_index": outlier_index,
        "outlier_magnitude_m": outlier_magnitude_m,
        "north_acceleration_truth_mps2": north_acceleration_truth_mps2,
        "north_velocity_truth_mps": tuple(north_velocity_truth_mps),
        "north_position_truth_m": tuple(north_position_truth_m),
        "north_acceleration_ins_mps2": north_acceleration_ins_mps2,
        "north_velocity_ins_only_mps": tuple(north_velocity_ins_only_mps),
        "north_position_ins_only_m": tuple(north_position_ins_only_m),
        "north_velocity_ins_only_error_mps": north_velocity_ins_only_error_mps,
        "north_position_ins_only_error_m": north_position_ins_only_error_m,
        "expected_ins_only_velocity_error_mps": expected_ins_only_velocity_error_mps,
        "expected_ins_only_position_error_m": expected_ins_only_position_error_m,
        "unit_gps_position_error_shape": unit_gps_position_error_shape,
        "gps_nominal_position_error_m": tuple(gps_nominal_position_error_m),
        "gps_outlier_m": tuple(gps_outlier_m),
        "gps_position_measurement_m": tuple(gps_position_measurement_m),
        "gps_position_error_rms_measured_m": gps_position_error_rms_measured_m,
        "north_position_predicted_m": tuple(north_position_predicted_m),
        "north_velocity_predicted_mps": tuple(north_velocity_predicted_mps),
        "gps_innovation_m": tuple(gps_innovation_m),
        "gps_position_correction_m": tuple(gps_position_correction_m),
        "gps_velocity_correction_mps": tuple(gps_velocity_correction_mps),
        "gps_accepted": tuple(gps_accepted),
        "gps_rejected": tuple(gps_rejected),
        "gps_ignored": tuple(gps_ignored),
        "gps_accepted_count": sum(gps_accepted),
        "gps_rejected_count": sum(gps_rejected),
        "gps_ignored_count": sum(gps_ignored),
        "north_position_fused_m": tuple(north_position_fused_m),
        "north_velocity_fused_mps": tuple(north_velocity_fused_mps),
        "north_position_fused_error_m": north_position_fused_error_m,
        "north_velocity_fused_error_mps": north_velocity_fused_error_mps,
        "ins_only_final_position_error_m": north_position_ins_only_error_m[-1],
        "ins_only_final_velocity_error_mps": north_velocity_ins_only_error_mps[-1],
        "fused_position_rms_m": math.sqrt(
            sum(value**2 for value in north_position_fused_error_m) / sample_count
        ),
        "fused_velocity_rms_mps": math.sqrt(
            sum(value**2 for value in north_velocity_fused_error_mps) / sample_count
        ),
        "fused_peak_absolute_position_error_m": max(
            map(abs, north_position_fused_error_m)
        ),
        "fused_peak_absolute_velocity_error_mps": max(
            map(abs, north_velocity_fused_error_mps)
        ),
        "fused_final_position_error_m": north_position_fused_error_m[-1],
        "fused_final_velocity_error_mps": north_velocity_fused_error_mps[-1],
        "outlier_innovation_m": gps_innovation_m[outlier_index],
        "outlier_position_correction_m": gps_position_correction_m[outlier_index],
        "outlier_velocity_correction_mps": gps_velocity_correction_mps[outlier_index],
    }


class P17ArtifactTests(unittest.TestCase):
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
                "number": 17,
                "id": "P17",
                "title": "Fuse INS and GPS",
                "guiding_question": GUIDING_QUESTION,
                "phase": 5,
                "phase_title": "Navigation and guidance",
                "slug": "fuse-ins-and-gps",
                "folder": "modules/17-fuse-ins-and-gps",
                "implementation_batch": "P17",
                "prerequisites": ["P16"],
            },
        )
        self.assertEqual(self.module["status"], "implemented")
        self.assertEqual(self.module["evidence_level"], "simulated")
        self.assertEqual(self.modules["P16"]["status"], "implemented")
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
            "p16",
            "p11",
            "p12",
            "north",
            "gravity-compensated",
            "ins",
            "gps",
            "prediction",
            "innovation",
            "alpha-beta",
            "bias",
            "deterministic",
            "mechanism",
            "reset",
            "broken",
            "gate",
            "outlier",
            "teach-back",
            "conceptual",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, combined)
        self.assertRegex(walkthrough.lower(), r"one plot|one .* at a time")
        self.assertIn("does not consume p16", combined)
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
            "functionout=model(insaccelerationbias_mps2,gpspositionerrorrms_m,fusionmode)",
            "insaccelerationbias_mps2(1,1)double{mustbereal,mustbefinite}=0.04",
            "gpspositionerrorrms_m(1,1)double{mustbereal,mustbefinite}=1",
            "fusionmode(1,1)double{mustbereal,mustbefinite}=1",
            "minimuminsaccelerationbias_mps2=-0.08;",
            "maximuminsaccelerationbias_mps2=0.08;",
            "minimumgpspositionerrorrms_m=0;",
            "maximumgpspositionerrorrms_m=4;",
            "fusionmode~=1&&fusionmode~=0&&fusionmode~=-1",
            "sampletime_s=0.02;",
            "timehorizon_s=60;",
            "time_s=0:sampletime_s:timehorizon_s;",
            "initialnorthvelocity_mps=20;",
            "gpsupdateperiod_s=1;",
            "gpsupdatestepcount=round(gpsupdateperiod_s/sampletime_s);",
            "gpsupdateindices=1+gpsupdatestepcount:gpsupdatestepcount:samplecount;",
            "gpspositiongain=0.45;",
            "gpsvelocitygain=0.12;",
            "innovationgate_m=25;",
            "outliertime_s=30;",
            "outliermagnitude_m=80;",
            "northaccelerationtruth_mps2(time_s>=5&time_s<15)=0.5;",
            "northaccelerationtruth_mps2(time_s>=25&time_s<35)=-0.5;",
            "northaccelerationins_mps2=northaccelerationtruth_mps2+insaccelerationbias_mps2;",
            "rawgpspositionerrorshape=sin(2*pi*gpsupdatetime_s/17)+0.35*cos(2*pi*gpsupdatetime_s/7+0.2);",
            "rawgpspositionerrorshape=rawgpspositionerrorshape-mean(rawgpspositionerrorshape);",
            "unitgpspositionerrorshape=rawgpspositionerrorshape/rawgpspositionerrorrms;",
            "gpsoutlier_m(outlierindex)=outliermagnitude_m;",
            "northpositionpredicted_m(k)=northpositionfused_m(k-1)+sampletime_s*northvelocityfused_mps(k-1)+0.5*sampletime_s^2*acceleration;",
            "northvelocitypredicted_mps(k)=northvelocityfused_mps(k-1)+sampletime_s*acceleration;",
            "gpsinnovation_m(k)=gpspositionmeasurement_m(k)-northpositionpredicted_m(k);",
            "gpsaccepted(k)=abs(gpsinnovation_m(k))<=innovationgate_m;",
            "gpspositioncorrection_m(k)=gpspositiongain*gpsinnovation_m(k);",
            "gpsvelocitycorrection_mps(k)=(gpsvelocitygain/gpsupdateperiod_s)*gpsinnovation_m(k);",
            "northpositionfused_m(k)=northpositionpredicted_m(k)+gpspositioncorrection_m(k);",
            "northvelocityfused_mps(k)=northvelocitypredicted_mps(k)+gpsvelocitycorrection_mps(k);",
            "expectedinsonlypositionerror_m=0.5*insaccelerationbias_mps2*time_s.^2;",
            "expectedinsonlyvelocityerror_mps=insaccelerationbias_mps2*time_s;",
        ):
            with self.subTest(expression=expression):
                self.assertIn(expression, compact)
        for identifier in (
            "P17:model:InsAccelerationBiasRange",
            "P17:model:GpsPositionErrorRange",
            "P17:model:FusionMode",
        ):
            self.assertIn(identifier, model)
        for field in (
            "insAccelerationBias_mps2",
            "gpsPositionErrorRms_m",
            "fusionMode",
            "time_s",
            "gpsUpdateIndices",
            "gpsUpdateMask",
            "northAccelerationTruth_mps2",
            "northPositionTruth_m",
            "northAccelerationINS_mps2",
            "northPositionINSOnly_m",
            "gpsNominalPositionError_m",
            "gpsOutlier_m",
            "gpsPositionMeasurement_m",
            "northPositionPredicted_m",
            "gpsInnovation_m",
            "gpsPositionCorrection_m",
            "gpsVelocityCorrection_mps",
            "gpsAccepted",
            "gpsRejected",
            "gpsIgnored",
            "northPositionFused_m",
            "northVelocityFused_mps",
            "fusedPositionRMS_m",
            "outlierInnovation_m",
            "predictionEquation",
            "innovationEquation",
            "correctionEquation",
            "gateEquation",
            "sensorDefinition",
            "brokenCaseDefinition",
            "analysisScope",
        ):
            with self.subTest(field=field):
                self.assertIn(f"'{field}'", model)
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

    def test_experiment_has_two_isolated_sweeps_and_outlier_case(self) -> None:
        experiment = self.text["experiment.m"]
        compact = re.sub(r"\s+", "", experiment.replace("...", "")).lower()
        for expression in (
            "baseline=model(0.04,1,1);",
            "insbiassweep_mps2=[00.020.040.060.08];",
            "sample=model(insbiassweep_mps2(k),baseline.gpspositionerrorrms_m,1);",
            "gpspositionerrorsweep_m=[00.5124];",
            "sample=model(baseline.insaccelerationbias_mps2,gpspositionerrorsweep_m(k),1);",
            "ideal=model(0,0,1);",
            "insonly=model(baseline.insaccelerationbias_mps2,baseline.gpspositionerrorrms_m,0);",
            "broken=model(baseline.insaccelerationbias_mps2,baseline.gpspositionerrorrms_m,-1);",
        ):
            self.assertIn(expression, compact)
        self.assertGreaterEqual(experiment.lower().count("mechanism"), 2)
        self.assertGreaterEqual(experiment.lower().count("reset"), 2)
        figures = re.findall(r"figure\('Name','(P17 [^']+)'\)", experiment)
        self.assertEqual(len(figures), 5)
        self.assertEqual(len(figures), len(set(figures)))
        for unit_label in (
            "Time (s)",
            "North position (m)",
            "North position error (m)",
            "GPS innovation (m)",
            "INS acceleration bias (m/s^2)",
            "Selected GPS position-error RMS (m)",
        ):
            self.assertIn(unit_label, experiment)
        self.assertNotIn("close all", experiment.lower())
        self.assertIn("clear run_checks;", experiment)
        self.assertIn("run_checks;", experiment)

    def test_interaction_and_checks_cover_recovery_resources(self) -> None:
        interactive = self.text["interactive.m"]
        checks = self.text["run_checks.m"]
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
            "fusionMode=-1",
            "cla(",
        ):
            self.assertIn(token, interactive)
        self.assertGreaterEqual(interactive.count("uiaxes("), 4)
        self.assertNotIn("close all", interactive.lower())
        self.assertIn("findall(groot,'Type','figure','Name',uiName)", interactive)
        for exact_reset in (
            "biasControl.Value=0.04",
            "gpsControl.Value=1",
            "modeControl.Value='Gated INS/GPS fusion'",
        ):
            self.assertIn(exact_reset, interactive)
        for token in (
            "repeat=modelFcn(0.04,1,1)",
            "expectedAcceleration",
            "expectedTruthPosition",
            "unitGpsShape",
            "expectedPredictedPosition",
            "expectedInnovation",
            "expectedAccepted",
            "insBiasSweep_mps2",
            "gpsPositionErrorSweep_m",
            "gateBoundary",
            "idealINSOnly",
            "negativeBias",
            "positiveNoiseFreeFusion",
            "nominalAcceptedMask",
            "broken",
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
        self.assertIn("interpretation questions", tutor_checks)
        self.assertIn("teach-back", tutor_checks)
        self.assertIn("two sentences", tutor_checks)
        self.assertIn("not applicable", tutor_checks)

    def test_no_opaque_toolbox_random_external_async_behavior(self) -> None:
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
            "kalman",
            "trackingkf",
            "insfilter",
            "ahrsfilter",
            "imufilter",
            "extendedkalmanfilter",
            "particlefilter",
            "fuse",
            "pid",
            "tf",
            "ss",
            "feedback",
            "lsim",
            "c2d",
            "lqr",
            "interp1",
            "fsolve",
            "ode45",
            "sim",
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
        self.assertNotIn("navigation toolbox", matlab)
        self.assertNotIn("sensor fusion and tracking toolbox", matlab)

    def test_retained_evidence_acceptance_and_claim_boundary(self) -> None:
        evidence = EVIDENCE_PATH.read_text(encoding="utf-8")
        self.assertTrue(evidence.endswith("\n"))
        self.assertFalse(evidence.endswith("\n\n"))
        for heading in (
            "## Result and claim boundary",
            "## Acceptance mapping",
            "## Exact validation performed",
            "## Independent audits",
            "## Independent system-risk review and signed-bias behavioral coverage",
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
        self.assertEqual(summary["batch_id"], "P17")
        self.assertEqual(summary["status"], "pass")
        self.assertEqual(len(summary["acceptance"]), 8)
        self.assertTrue(all(item["status"] == "pass" for item in summary["acceptance"]))


class P17EquationOracleTests(unittest.TestCase):
    def test_deterministic_baseline_signature_and_fixed_shape(self) -> None:
        first = _oracle()
        second = _oracle(0.04, 1.0, 1)
        self.assertEqual(first, second)
        self.assertEqual(first["sample_count"], 3001)
        self.assertEqual(first["interval_count"], 3000)
        self.assertEqual(first["gps_update_count"], 60)
        self.assertEqual(first["gps_accepted_count"], 59)
        self.assertEqual(first["gps_rejected_count"], 1)
        self.assertEqual(first["gps_ignored_count"], 0)
        self.assertAlmostEqual(first["fused_position_rms_m"], 1.180877601392873, 12)
        self.assertAlmostEqual(
            first["fused_peak_absolute_position_error_m"],
            2.385486692971881,
            12,
        )
        self.assertAlmostEqual(first["fused_final_position_error_m"], 0.136263825022525, 12)
        self.assertAlmostEqual(
            first["fused_final_velocity_error_mps"], -0.115398118713124, 12
        )

    def test_truth_and_sensor_histories_reconstruct(self) -> None:
        result = _oracle()
        dt = result["sample_time_s"]
        position = result["north_position_truth_m"]
        velocity = result["north_velocity_truth_mps"]
        acceleration = result["north_acceleration_truth_mps2"]
        for index in range(result["interval_count"]):
            self.assertAlmostEqual(
                position[index + 1],
                position[index] + dt * velocity[index] + 0.5 * dt**2 * acceleration[index],
                12,
            )
            self.assertAlmostEqual(
                velocity[index + 1], velocity[index] + dt * acceleration[index], 12
            )
        self.assertAlmostEqual(position[-1], 1300.0, 9)
        self.assertAlmostEqual(velocity[-1], 20.0, 12)
        shape = result["unit_gps_position_error_shape"]
        self.assertAlmostEqual(sum(shape) / len(shape), 0.0, 14)
        self.assertAlmostEqual(math.sqrt(sum(x * x for x in shape) / len(shape)), 1.0, 14)
        self.assertAlmostEqual(result["gps_position_error_rms_measured_m"], 1.0, 14)
        updates = set(result["gps_update_indices"])
        for index, measurement in enumerate(result["gps_position_measurement_m"]):
            if index in updates:
                self.assertAlmostEqual(
                    measurement,
                    position[index]
                    + result["gps_nominal_position_error_m"][index]
                    + result["gps_outlier_m"][index],
                    12,
                )
            else:
                self.assertEqual(measurement, 0.0)
        self.assertEqual(sum(value != 0.0 for value in result["gps_outlier_m"]), 1)
        self.assertEqual(result["gps_outlier_m"][result["outlier_index"]], 80.0)

    def test_every_prediction_innovation_gate_and_correction_reconstructs(self) -> None:
        result = _oracle()
        dt = result["sample_time_s"]
        updates = set(result["gps_update_indices"])
        for index in range(1, result["sample_count"]):
            expected_position = (
                result["north_position_fused_m"][index - 1]
                + dt * result["north_velocity_fused_mps"][index - 1]
                + 0.5 * dt**2 * result["north_acceleration_ins_mps2"][index - 1]
            )
            expected_velocity = (
                result["north_velocity_fused_mps"][index - 1]
                + dt * result["north_acceleration_ins_mps2"][index - 1]
            )
            self.assertAlmostEqual(result["north_position_predicted_m"][index], expected_position, 12)
            self.assertAlmostEqual(result["north_velocity_predicted_mps"][index], expected_velocity, 12)
            if index in updates:
                innovation = result["gps_position_measurement_m"][index] - expected_position
                accepted = abs(innovation) <= result["innovation_gate_m"]
                self.assertAlmostEqual(result["gps_innovation_m"][index], innovation, 12)
                self.assertEqual(result["gps_accepted"][index], accepted)
                self.assertEqual(result["gps_rejected"][index], not accepted)
                expected_p_correction = 0.45 * innovation if accepted else 0.0
                expected_v_correction = 0.12 * innovation if accepted else 0.0
                self.assertAlmostEqual(result["gps_position_correction_m"][index], expected_p_correction, 12)
                self.assertAlmostEqual(result["gps_velocity_correction_mps"][index], expected_v_correction, 12)
            else:
                self.assertEqual(result["gps_innovation_m"][index], 0.0)
                self.assertFalse(result["gps_accepted"][index])
                self.assertFalse(result["gps_rejected"][index])
            self.assertAlmostEqual(
                result["north_position_fused_m"][index],
                expected_position + result["gps_position_correction_m"][index],
                12,
            )
            self.assertAlmostEqual(
                result["north_velocity_fused_mps"][index],
                expected_velocity + result["gps_velocity_correction_mps"][index],
                12,
            )

    def test_gps_fix_causality_and_gate_boundary(self) -> None:
        result = _oracle()
        self.assertFalse(result["gps_update_mask"][0])
        self.assertEqual(result["gps_update_indices"], tuple(range(50, 3001, 50)))
        self.assertEqual(tuple(result["time_s"][i] for i in result["gps_update_indices"]), tuple(float(i) for i in range(1, 61)))
        self.assertTrue(abs(25.0) <= result["innovation_gate_m"])
        self.assertFalse(abs(math.nextafter(25.0, math.inf)) <= result["innovation_gate_m"])
        outlier = result["outlier_index"]
        self.assertTrue(result["gps_rejected"][outlier])
        self.assertFalse(result["gps_accepted"][outlier])
        self.assertAlmostEqual(result["outlier_innovation_m"], 79.33779956347905, 11)
        self.assertEqual(result["outlier_position_correction_m"], 0.0)
        self.assertEqual(result["outlier_velocity_correction_mps"], 0.0)

    def test_bias_sweep_isolates_inertial_drift(self) -> None:
        biases = (0.0, 0.02, 0.04, 0.06, 0.08)
        results = tuple(_oracle(bias, 1.0, 1) for bias in biases)
        expected_position = tuple(0.5 * bias * 60.0**2 for bias in biases)
        expected_velocity = tuple(bias * 60.0 for bias in biases)
        expected_rms = (
            1.163493836900810,
            1.165304641533275,
            1.180877601392873,
            1.209681327532636,
            1.250802111330508,
        )
        baseline = results[2]
        for bias, result, position_error, velocity_error, rms in zip(
            biases, results, expected_position, expected_velocity, expected_rms
        ):
            with self.subTest(bias=bias):
                self.assertEqual(result["gps_position_error_rms_m"], 1.0)
                self.assertEqual(result["north_position_truth_m"], baseline["north_position_truth_m"])
                self.assertEqual(result["gps_position_measurement_m"], baseline["gps_position_measurement_m"])
                self.assertAlmostEqual(result["ins_only_final_position_error_m"], position_error, 9)
                self.assertAlmostEqual(result["ins_only_final_velocity_error_mps"], velocity_error, 9)
                self.assertAlmostEqual(result["fused_position_rms_m"], rms, 11)
        self.assertTrue(all(a < b for a, b in zip(expected_rms, expected_rms[1:])))

    def test_signed_bias_reverses_noise_free_drift_and_corrections(self) -> None:
        positive = _oracle(0.08, 0.0, 1)
        negative = _oracle(-0.08, 0.0, 1)

        for field in (
            "time_s",
            "north_acceleration_truth_mps2",
            "north_position_truth_m",
            "north_velocity_truth_mps",
            "gps_update_mask",
            "gps_nominal_position_error_m",
            "gps_outlier_m",
            "gps_position_measurement_m",
        ):
            self.assertEqual(positive[field], negative[field], field)

        for positive_value, negative_value, truth_value in zip(
            positive["north_acceleration_ins_mps2"],
            negative["north_acceleration_ins_mps2"],
            positive["north_acceleration_truth_mps2"],
        ):
            self.assertAlmostEqual(
                positive_value - truth_value,
                -(negative_value - truth_value),
                13,
            )

        for field in (
            "north_position_ins_only_error_m",
            "north_velocity_ins_only_error_mps",
            "north_position_fused_error_m",
            "north_velocity_fused_error_mps",
            "gps_position_correction_m",
            "gps_velocity_correction_mps",
        ):
            with self.subTest(field=field):
                self.assertTrue(
                    all(
                        math.isclose(
                            positive_value,
                            -negative_value,
                            rel_tol=0.0,
                            abs_tol=1e-9,
                        )
                        for positive_value, negative_value in zip(
                            positive[field], negative[field]
                        )
                    )
                )

        self.assertEqual(positive["gps_accepted"], negative["gps_accepted"])
        self.assertEqual(positive["gps_rejected"], negative["gps_rejected"])
        self.assertEqual(positive["gps_accepted_count"], 59)
        self.assertEqual(positive["gps_rejected_count"], 1)
        self.assertTrue(positive["gps_rejected"][positive["outlier_index"]])

        accepted_indices = tuple(
            index
            for index in positive["gps_update_indices"]
            if positive["gps_accepted"][index]
        )
        self.assertEqual(len(accepted_indices), 59)
        for index in accepted_indices:
            with self.subTest(time_s=positive["time_s"][index]):
                self.assertAlmostEqual(
                    positive["gps_innovation_m"][index],
                    -negative["gps_innovation_m"][index],
                    9,
                )
                for result in (positive, negative):
                    predicted_error = (
                        result["north_position_predicted_m"][index]
                        - result["north_position_truth_m"][index]
                    )
                    corrected_error = result["north_position_fused_error_m"][index]
                    self.assertLess(abs(corrected_error), abs(predicted_error))
                    self.assertAlmostEqual(
                        corrected_error,
                        (1.0 - result["gps_position_gain"]) * predicted_error,
                        9,
                    )

        self.assertGreater(positive["fused_final_position_error_m"], 0.0)
        self.assertLess(negative["fused_final_position_error_m"], 0.0)
        self.assertAlmostEqual(
            positive["fused_position_rms_m"], negative["fused_position_rms_m"], 9
        )
        self.assertAlmostEqual(
            positive["fused_velocity_rms_mps"], negative["fused_velocity_rms_mps"], 9
        )

    def test_noise_sweep_scales_nominal_gps_error_and_fused_rms(self) -> None:
        levels = (0.0, 0.5, 1.0, 2.0, 4.0)
        results = tuple(_oracle(0.04, level, 1) for level in levels)
        expected_rms = (
            0.254243988397741,
            0.625401055665580,
            1.180877601392873,
            2.330609283066586,
            4.650653988745627,
        )
        baseline = results[2]
        for level, result, rms in zip(levels, results, expected_rms):
            with self.subTest(level=level):
                self.assertEqual(result["north_position_truth_m"], baseline["north_position_truth_m"])
                self.assertEqual(result["north_acceleration_ins_mps2"], baseline["north_acceleration_ins_mps2"])
                self.assertEqual(result["north_position_ins_only_m"], baseline["north_position_ins_only_m"])
                self.assertEqual(result["gps_outlier_m"], baseline["gps_outlier_m"])
                self.assertAlmostEqual(result["gps_position_error_rms_measured_m"], level, 13)
                self.assertAlmostEqual(result["fused_position_rms_m"], rms, 11)
                reconstructed = math.sqrt(
                    sum(value**2 for value in result["north_position_fused_error_m"])
                    / result["sample_count"]
                )
                self.assertAlmostEqual(result["fused_position_rms_m"], reconstructed, 14)
        self.assertTrue(all(a < b for a, b in zip(expected_rms, expected_rms[1:])))

    def test_zero_bias_zero_noise_and_ins_only_limits(self) -> None:
        ideal = _oracle(0.0, 0.0, 1)
        ideal_ins_only = _oracle(0.0, 0.0, 0)
        self.assertEqual(ideal["north_position_fused_m"], ideal["north_position_truth_m"])
        self.assertEqual(ideal["north_velocity_fused_mps"], ideal["north_velocity_truth_mps"])
        self.assertEqual(ideal["fused_position_rms_m"], 0.0)
        self.assertEqual(ideal["gps_accepted_count"], 59)
        self.assertEqual(ideal["gps_rejected_count"], 1)
        self.assertEqual(ideal_ins_only["north_position_fused_m"], ideal_ins_only["north_position_truth_m"])
        self.assertEqual(ideal_ins_only["gps_accepted_count"], 0)
        self.assertEqual(ideal_ins_only["gps_rejected_count"], 0)
        self.assertEqual(ideal_ins_only["gps_ignored_count"], 60)
        ins_only = _oracle(0.04, 1.0, 0)
        self.assertEqual(ins_only["north_position_fused_m"], ins_only["north_position_ins_only_m"])
        self.assertEqual(ins_only["north_velocity_fused_mps"], ins_only["north_velocity_ins_only_mps"])
        for error, expected in zip(
            ins_only["north_position_ins_only_error_m"],
            ins_only["expected_ins_only_position_error_m"],
        ):
            self.assertAlmostEqual(error, expected, 9)
        for error, expected in zip(
            ins_only["north_velocity_ins_only_error_mps"],
            ins_only["expected_ins_only_velocity_error_mps"],
        ):
            self.assertAlmostEqual(error, expected, 9)

    def test_outlier_gate_vs_accept_all_isolated_failure(self) -> None:
        correct = _oracle()
        broken = _oracle(0.04, 1.0, -1)
        for field in (
            "time_s",
            "north_acceleration_truth_mps2",
            "north_position_truth_m",
            "north_velocity_truth_mps",
            "north_acceleration_ins_mps2",
            "north_position_ins_only_m",
            "north_velocity_ins_only_mps",
            "gps_update_mask",
            "gps_nominal_position_error_m",
            "gps_outlier_m",
            "gps_position_measurement_m",
        ):
            self.assertEqual(correct[field], broken[field], field)
        outlier = correct["outlier_index"]
        self.assertEqual(correct["north_position_fused_m"][:outlier], broken["north_position_fused_m"][:outlier])
        self.assertEqual(correct["north_velocity_fused_mps"][:outlier], broken["north_velocity_fused_mps"][:outlier])
        self.assertEqual(broken["gps_accepted_count"], 60)
        self.assertEqual(broken["gps_rejected_count"], 0)
        self.assertAlmostEqual(broken["outlier_position_correction_m"], 35.70200980356557, 11)
        self.assertAlmostEqual(broken["outlier_velocity_correction_mps"], 9.520535947617486, 11)
        self.assertAlmostEqual(broken["north_position_fused_error_m"][outlier], 34.727795781358964, 11)
        self.assertAlmostEqual(broken["fused_peak_absolute_position_error_m"], 43.95460633859625, 11)
        self.assertAlmostEqual(broken["fused_position_rms_m"], 6.519034268309427, 11)
        next_fix = outlier + broken["gps_update_step_count"]
        self.assertGreater(
            max(abs(value) for value in broken["north_position_fused_error_m"][outlier:next_fix]),
            abs(broken["north_position_fused_error_m"][outlier]),
        )

    def test_malformed_inputs_reject_without_poisoning_recovery(self) -> None:
        malformed = (
            (-0.080000001, 1.0, 1),
            (0.080000001, 1.0, 1),
            ([0.04], 1.0, 1),
            (0.04 + 1.0j, 1.0, 1),
            (float("nan"), 1.0, 1),
            (float("inf"), 1.0, 1),
            (0.04, -1e-12, 1),
            (0.04, 4.000000001, 1),
            (0.04, [1.0], 1),
            (0.04, 1.0 + 1.0j, 1),
            (0.04, float("nan"), 1),
            (0.04, float("inf"), 1),
            (0.04, 1.0, -2),
            (0.04, 1.0, 2),
            (0.04, 1.0, 0.5),
            (0.04, 1.0, [1]),
            (0.04, 1.0, 1.0 + 1.0j),
            (0.04, 1.0, float("nan")),
            (True, 1.0, 1),
        )
        baseline = _oracle()
        for bias, gps_rms, mode in malformed:
            with self.subTest(bias=bias, gps_rms=gps_rms, mode=mode):
                with self.assertRaises(ValueError):
                    _oracle(bias, gps_rms, mode)
                self.assertEqual(_oracle(), baseline)

    def test_broken_and_rejected_calls_exact_rollback_recovery(self) -> None:
        baseline = _oracle()
        broken = _oracle(0.04, 1.0, -1)
        self.assertNotEqual(broken["north_position_fused_m"], baseline["north_position_fused_m"])
        self.assertEqual(_oracle(0.04, 1.0, 1), baseline)
        with self.assertRaises(ValueError):
            _oracle(0.09, 1.0, 1)
        self.assertEqual(_oracle(), baseline)

    def test_corners_and_capped_grid_finite_fixed_bounded(self) -> None:
        corners = tuple(
            _oracle(bias, gps_rms, mode)
            for bias in (-0.08, 0.08)
            for gps_rms in (0.0, 4.0)
            for mode in (-1, 0, 1)
        )
        self.assertEqual(len(corners), 12)
        grid = tuple(
            _oracle(bias, gps_rms, mode)
            for bias in (-0.08, 0.0, 0.08)
            for gps_rms in (0.0, 1.0, 4.0)
            for mode in (-1, 0, 1)
        )
        self.assertEqual(len(grid), 27)
        history_fields = (
            "north_position_truth_m",
            "north_velocity_truth_mps",
            "north_acceleration_ins_mps2",
            "north_position_ins_only_m",
            "north_velocity_ins_only_mps",
            "gps_position_measurement_m",
            "north_position_predicted_m",
            "north_velocity_predicted_mps",
            "gps_innovation_m",
            "gps_position_correction_m",
            "gps_velocity_correction_mps",
            "north_position_fused_m",
            "north_velocity_fused_mps",
        )
        for result in corners + grid:
            self.assertEqual(result["sample_count"], 3001)
            self.assertEqual(result["interval_count"], 3000)
            self.assertEqual(result["gps_update_count"], 60)
            for field in history_fields:
                self.assertEqual(len(result[field]), 3001, field)
                self.assertTrue(all(math.isfinite(value) for value in result[field]), field)
            self.assertLess(result["fused_peak_absolute_position_error_m"], 160.0)
            self.assertLess(max(map(abs, result["gps_position_correction_m"])), 50.0)
            self.assertLess(max(map(abs, result["gps_velocity_correction_mps"])), 15.0)
            self.assertEqual(
                result["gps_accepted_count"]
                + result["gps_rejected_count"]
                + result["gps_ignored_count"],
                60,
            )
            if result["fusion_mode"] == 1:
                self.assertEqual(result["gps_accepted_count"], 59)
                self.assertEqual(result["gps_rejected_count"], 1)

    def test_sync_resource_timeout_cancellation_compatibility_disposition(self) -> None:
        result = _oracle()
        self.assertEqual(result["time_s"], tuple(index * 0.02 for index in range(3001)))
        self.assertEqual(result["gps_update_indices"], tuple(range(50, 3001, 50)))
        self.assertEqual(len(result["unit_gps_position_error_shape"]), 60)
        self.assertEqual(result["gps_position_gain"], 0.45)
        self.assertEqual(result["gps_velocity_gain"], 0.12)
        # This API performs fixed synchronous arithmetic and owns no timer,
        # future, worker, external I/O, cancellation state, or input-sized
        # allocation. Runtime timeout/cancel transitions are not applicable;
        # bounded inputs plus fixed histories and capped matrices are the gate.


if __name__ == "__main__":
    unittest.main()
