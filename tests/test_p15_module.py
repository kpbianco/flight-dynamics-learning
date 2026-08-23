from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_ID = "P15"
MODULE_FOLDER = ROOT / "modules/15-control-speed-with-throttle"
EVIDENCE_PATH = ROOT / "docs/evidence/P15-2026-08-23.md"
GUIDING_QUESTION = (
    "What inputs, observable effects, and failure modes matter when you control "
    "Speed with Throttle?"
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
        raise ValueError("feedback sign must be -1 or +1")
    result = float(value)
    if not math.isfinite(result) or result not in (-1.0, 1.0):
        raise ValueError("feedback sign must be -1 or +1")
    return int(result)


def _clamp(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def _oracle(
    speed_gain_per_s: object = 0.15,
    throttle_time_constant_s: object = 0.8,
    speed_feedback_sign: object = 1,
) -> dict[str, object]:
    """Independent standard-library implementation of the declared equations."""
    speed_gain = _bounded_scalar("speed gain", speed_gain_per_s, 0.0, 0.3)
    throttle_time_constant = _bounded_scalar(
        "throttle time constant", throttle_time_constant_s, 0.2, 1.4
    )
    feedback_sign = _feedback_sign(speed_feedback_sign)

    sample_time_s = 0.02
    time_horizon_s = 30.0
    time_s = tuple(
        index * sample_time_s
        for index in range(round(time_horizon_s / sample_time_s) + 1)
    )
    sample_count = len(time_s)
    command_step_time_s = 1.0
    initial_true_airspeed_mps = 60.0
    commanded_true_airspeed_mps = 70.0
    speed_command_step_mps = (
        commanded_true_airspeed_mps - initial_true_airspeed_mps
    )

    air_density_kgpm3 = 0.736115547399152
    wing_area_m2 = 16.2
    parasite_drag_coefficient = 0.025
    induced_drag_factor = 0.045
    mass_kg = 1200.0
    gravity_mps2 = 9.80665
    maximum_lift_coefficient = 1.4
    maximum_thrust_n = 4000.0
    weight_n = mass_kg * gravity_mps2
    parasite_drag_scale_n_per_mps2 = (
        0.5
        * air_density_kgpm3
        * wing_area_m2
        * parasite_drag_coefficient
    )
    induced_drag_scale_n_mps2 = (
        2.0
        * induced_drag_factor
        * weight_n**2
        / (air_density_kgpm3 * wing_area_m2)
    )
    stall_speed_mps = math.sqrt(
        2.0
        * weight_n
        / (
            air_density_kgpm3
            * wing_area_m2
            * maximum_lift_coefficient
        )
    )

    def drag_at(speed_mps: float) -> float:
        return (
            parasite_drag_scale_n_per_mps2 * speed_mps**2
            + induced_drag_scale_n_mps2 / speed_mps**2
        )

    initial_drag_n = drag_at(initial_true_airspeed_mps)
    trim_throttle = initial_drag_n / maximum_thrust_n
    commanded_drag_n = drag_at(commanded_true_airspeed_mps)
    commanded_equilibrium_throttle = commanded_drag_n / maximum_thrust_n
    speed_command_mps = tuple(
        initial_true_airspeed_mps
        if time < command_step_time_s
        else commanded_true_airspeed_mps
        for time in time_s
    )

    true_airspeed_mps = [0.0] * sample_count
    speed_error_mps = [0.0] * sample_count
    speed_error_used_mps = [0.0] * sample_count
    parasite_drag_n = [0.0] * sample_count
    induced_drag_n = [0.0] * sample_count
    drag_n = [0.0] * sample_count
    requested_acceleration_mps2 = [0.0] * sample_count
    thrust_command_unclamped_n = [0.0] * sample_count
    thrust_command_n = [0.0] * sample_count
    thrust_command_saturated = [False] * sample_count
    throttle_command = [0.0] * sample_count
    throttle_actual = [0.0] * sample_count
    throttle_rate_per_s = [0.0] * sample_count
    thrust_actual_n = [0.0] * sample_count
    net_forward_force_n = [0.0] * sample_count
    longitudinal_acceleration_mps2 = [0.0] * sample_count
    true_airspeed_mps[0] = initial_true_airspeed_mps
    throttle_actual[0] = trim_throttle

    for index in range(sample_count):
        speed_error_mps[index] = (
            speed_command_mps[index] - true_airspeed_mps[index]
        )
        speed_error_used_mps[index] = (
            feedback_sign * speed_error_mps[index]
        )
        parasite_drag_n[index] = (
            parasite_drag_scale_n_per_mps2
            * true_airspeed_mps[index] ** 2
        )
        induced_drag_n[index] = (
            induced_drag_scale_n_mps2 / true_airspeed_mps[index] ** 2
        )
        drag_n[index] = parasite_drag_n[index] + induced_drag_n[index]
        requested_acceleration_mps2[index] = (
            speed_gain * speed_error_used_mps[index]
        )
        thrust_command_unclamped_n[index] = (
            drag_n[index]
            + mass_kg * requested_acceleration_mps2[index]
        )
        thrust_command_n[index] = _clamp(
            thrust_command_unclamped_n[index], 0.0, maximum_thrust_n
        )
        thrust_command_saturated[index] = (
            thrust_command_unclamped_n[index] < 0.0
            or thrust_command_unclamped_n[index] > maximum_thrust_n
        )
        throttle_command[index] = (
            thrust_command_n[index] / maximum_thrust_n
        )
        throttle_rate_per_s[index] = (
            throttle_command[index] - throttle_actual[index]
        ) / throttle_time_constant
        thrust_actual_n[index] = maximum_thrust_n * throttle_actual[index]
        net_forward_force_n[index] = (
            thrust_actual_n[index] - drag_n[index]
        )
        longitudinal_acceleration_mps2[index] = (
            net_forward_force_n[index] / mass_kg
        )
        if index < sample_count - 1:
            true_airspeed_mps[index + 1] = (
                true_airspeed_mps[index]
                + sample_time_s * longitudinal_acceleration_mps2[index]
            )
            throttle_actual[index + 1] = (
                throttle_actual[index]
                + sample_time_s * throttle_rate_per_s[index]
            )

    active_indices = tuple(
        index
        for index, time in enumerate(time_s)
        if time >= command_step_time_s
    )
    throttle_tracking_error = tuple(
        command - actual
        for command, actual in zip(throttle_command, throttle_actual)
    )
    throttle_tracking_rms = math.sqrt(
        sum(throttle_tracking_error[index] ** 2 for index in active_indices)
        / len(active_indices)
    )
    speed_tracking_rms_mps = math.sqrt(
        sum(speed_error_mps[index] ** 2 for index in active_indices)
        / len(active_indices)
    )
    capture_tolerance_mps = 0.1 * abs(speed_command_step_mps)
    capture_index = next(
        (
            index
            for index in active_indices
            if abs(speed_error_mps[index]) <= capture_tolerance_mps
        ),
        None,
    )
    reached_ninety_percent = capture_index is not None
    time_to_ninety_percent_s = (
        time_s[capture_index] - command_step_time_s
        if capture_index is not None
        else time_horizon_s - command_step_time_s
    )
    settling_tolerance_mps = 0.02 * abs(speed_command_step_mps)
    final_speed_error_mps = speed_error_mps[-1]
    settled_by_end = abs(final_speed_error_mps) <= settling_tolerance_mps
    outside_tolerance = tuple(
        index
        for index in active_indices
        if abs(speed_error_mps[index]) > settling_tolerance_mps
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

    return {
        "speed_gain_per_s": speed_gain,
        "throttle_time_constant_s": throttle_time_constant,
        "speed_feedback_sign": feedback_sign,
        "sample_time_s": sample_time_s,
        "time_horizon_s": time_horizon_s,
        "time_s": time_s,
        "sample_count": sample_count,
        "interval_count": sample_count - 1,
        "active_sample_count": len(active_indices),
        "command_step_time_s": command_step_time_s,
        "initial_true_airspeed_mps": initial_true_airspeed_mps,
        "commanded_true_airspeed_mps": commanded_true_airspeed_mps,
        "speed_command_step_mps": speed_command_step_mps,
        "air_density_kgpm3": air_density_kgpm3,
        "wing_area_m2": wing_area_m2,
        "parasite_drag_coefficient": parasite_drag_coefficient,
        "induced_drag_factor": induced_drag_factor,
        "mass_kg": mass_kg,
        "gravity_mps2": gravity_mps2,
        "maximum_lift_coefficient": maximum_lift_coefficient,
        "maximum_thrust_n": maximum_thrust_n,
        "weight_n": weight_n,
        "parasite_drag_scale_n_per_mps2": (
            parasite_drag_scale_n_per_mps2
        ),
        "induced_drag_scale_n_mps2": induced_drag_scale_n_mps2,
        "stall_speed_mps": stall_speed_mps,
        "initial_drag_n": initial_drag_n,
        "trim_throttle": trim_throttle,
        "commanded_drag_n": commanded_drag_n,
        "commanded_equilibrium_throttle": commanded_equilibrium_throttle,
        "speed_command_mps": speed_command_mps,
        "true_airspeed_mps": tuple(true_airspeed_mps),
        "speed_error_mps": tuple(speed_error_mps),
        "speed_error_used_mps": tuple(speed_error_used_mps),
        "parasite_drag_n": tuple(parasite_drag_n),
        "induced_drag_n": tuple(induced_drag_n),
        "drag_n": tuple(drag_n),
        "requested_acceleration_mps2": tuple(
            requested_acceleration_mps2
        ),
        "thrust_command_unclamped_n": tuple(
            thrust_command_unclamped_n
        ),
        "thrust_command_n": tuple(thrust_command_n),
        "thrust_command_saturated": tuple(thrust_command_saturated),
        "thrust_command_saturation_fraction": (
            sum(thrust_command_saturated) / sample_count
        ),
        "throttle_command": tuple(throttle_command),
        "throttle_actual": tuple(throttle_actual),
        "throttle_rate_per_s": tuple(throttle_rate_per_s),
        "thrust_actual_n": tuple(thrust_actual_n),
        "net_forward_force_n": tuple(net_forward_force_n),
        "longitudinal_acceleration_mps2": tuple(
            longitudinal_acceleration_mps2
        ),
        "throttle_tracking_error": throttle_tracking_error,
        "throttle_tracking_rms": throttle_tracking_rms,
        "speed_tracking_rms_mps": speed_tracking_rms_mps,
        "speed_at_five_seconds_mps": true_airspeed_mps[250],
        "speed_error_at_ten_seconds_mps": speed_error_mps[500],
        "final_speed_error_mps": final_speed_error_mps,
        "capture_tolerance_mps": capture_tolerance_mps,
        "reached_ninety_percent": reached_ninety_percent,
        "time_to_ninety_percent_s": time_to_ninety_percent_s,
        "settling_tolerance_mps": settling_tolerance_mps,
        "settled_by_end": settled_by_end,
        "settling_time_s": settling_time_s,
        "peak_speed_overshoot_mps": max(
            0.0,
            max(
                true_airspeed_mps[index] - commanded_true_airspeed_mps
                for index in active_indices
            ),
        ),
        "minimum_true_airspeed_mps": min(true_airspeed_mps),
        "minimum_stall_margin_mps": (
            min(true_airspeed_mps) - stall_speed_mps
        ),
        "stall_envelope_maintained": (
            min(true_airspeed_mps) >= stall_speed_mps
        ),
        "peak_throttle_command": max(throttle_command),
        "peak_throttle_actual": max(throttle_actual),
        "peak_absolute_throttle_rate_per_s": max(
            map(abs, throttle_rate_per_s)
        ),
        "peak_absolute_acceleration_mps2": max(
            map(abs, longitudinal_acceleration_mps2)
        ),
        "final_net_forward_force_n": net_forward_force_n[-1],
    }


class P15ArtifactTests(unittest.TestCase):
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
                "number": 15,
                "id": "P15",
                "title": "Control Speed with Throttle",
                "guiding_question": GUIDING_QUESTION,
                "phase": 4,
                "phase_title": "Autopilots",
                "slug": "control-speed-with-throttle",
                "folder": "modules/15-control-speed-with-throttle",
                "implementation_batch": "P15",
                "prerequisites": ["P14"],
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
            "p14",
            "p04",
            "p10",
            "speed error",
            "thrust command",
            "delivered throttle",
            "thrust-minus-drag",
            "mechanism",
            "reset",
            "broken",
            "feedback-open",
            "teach-back",
            "stall boundary",
            "positive feedback",
            "fixed horizon",
            "conceptual",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, combined)
        self.assertRegex(walkthrough.lower(), r"one plot|one .* at a time")
        self.assertIn("conceptual rather than current api compatibility", combined)
        self.assertIn("does not accept a p14", combined)
        self.assertIn("not integrator windup", combined)
        self.assertIn("p16", combined)
        self.assertIn("subsequent requests differ through feedback", combined)
        self.assertNotIn("unchanged request", combined)
        self.assertNotIn("delivers the same request", combined)
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
            "functionout=model(speedgain_per_s,"
            "throttletimeconstant_s,speedfeedbacksign)",
            compact,
        )
        self.assertIn(
            "speedgain_per_s(1,1)double"
            "{mustbereal,mustbefinite}=0.15",
            compact,
        )
        self.assertIn(
            "throttletimeconstant_s(1,1)double"
            "{mustbereal,mustbefinite}=0.8",
            compact,
        )
        self.assertIn(
            "speedfeedbacksign(1,1)double"
            "{mustbereal,mustbefinite}=1",
            compact,
        )
        for expression in (
            "speedgain_per_s<0||speedgain_per_s>0.3",
            "throttletimeconstant_s<0.2||throttletimeconstant_s>1.4",
            "speedfeedbacksign~=1&&speedfeedbacksign~=-1",
            "sampletime_s=0.02;",
            "timehorizon_s=30;",
            "initialtrueairspeed_mps=60;",
            "commandedtrueairspeed_mps=70;",
            "commandsteptime_s=1;",
            "time_s=0:sampletime_s:timehorizon_s;",
            "speedcommandstep_mps=commandedtrueairspeed_mps-"
            "initialtrueairspeed_mps;",
            "airdensity_kgpm3=0.736115547399152;",
            "wingarea_m2=16.2;",
            "parasitedragcoefficient=0.025;",
            "induceddragfactor=0.045;",
            "mass_kg=1200;",
            "gravity_mps2=9.80665;",
            "maximumliftcoefficient=1.4;",
            "maximumthrust_n=4000;",
            "weight_n=mass_kg*gravity_mps2;",
            "parasitedragscale_n_per_mps2=0.5*airdensity_kgpm3*"
            "wingarea_m2*parasitedragcoefficient;",
            "induceddragscale_n_mps2=2*induceddragfactor*weight_n^2/("
            "airdensity_kgpm3*wingarea_m2);",
            "stallspeed_mps=sqrt(2*weight_n/(airdensity_kgpm3*"
            "wingarea_m2*maximumliftcoefficient));",
            "initialdrag_n=parasitedragscale_n_per_mps2*"
            "initialtrueairspeed_mps^2+induceddragscale_n_mps2/"
            "initialtrueairspeed_mps^2;",
            "trimthrottle=initialdrag_n/maximumthrust_n;",
            "commandeddrag_n=parasitedragscale_n_per_mps2*"
            "commandedtrueairspeed_mps^2+induceddragscale_n_mps2/"
            "commandedtrueairspeed_mps^2;",
            "commandedequilibriumthrottle=commandeddrag_n/maximumthrust_n;",
            "speedcommand_mps=initialtrueairspeed_mps+"
            "speedcommandstep_mps*double(time_s>=commandsteptime_s);",
            "trueairspeed_mps(1)=initialtrueairspeed_mps;",
            "throttleactual(1)=trimthrottle;",
            "speederror_mps(k)=speedcommand_mps(k)-trueairspeed_mps(k);",
            "speederrorused_mps(k)=speedfeedbacksign*speederror_mps(k);",
            "parasitedrag_n(k)=parasitedragscale_n_per_mps2*"
            "trueairspeed_mps(k)^2;",
            "induceddrag_n(k)=induceddragscale_n_mps2/"
            "trueairspeed_mps(k)^2;",
            "drag_n(k)=parasitedrag_n(k)+induceddrag_n(k);",
            "requestedacceleration_mps2(k)=speedgain_per_s*"
            "speederrorused_mps(k);",
            "thrustcommandunclamped_n(k)=drag_n(k)+mass_kg*"
            "requestedacceleration_mps2(k);",
            "thrustcommand_n(k)=min(max(thrustcommandunclamped_n(k),0),"
            "maximumthrust_n);",
            "throttlecommand(k)=thrustcommand_n(k)/maximumthrust_n;",
            "throttlerate_per_s(k)=(throttlecommand(k)-"
            "throttleactual(k))/throttletimeconstant_s;",
            "thrustactual_n(k)=maximumthrust_n*throttleactual(k);",
            "netforwardforce_n(k)=thrustactual_n(k)-drag_n(k);",
            "longitudinalacceleration_mps2(k)=netforwardforce_n(k)/mass_kg;",
            "trueairspeed_mps(k+1)=trueairspeed_mps(k)+"
            "sampletime_s*longitudinalacceleration_mps2(k);",
            "throttleactual(k+1)=throttleactual(k)+"
            "sampletime_s*throttlerate_per_s(k);",
            "thrustcommandsaturated=thrustcommandunclamped_n<0|"
            "thrustcommandunclamped_n>maximumthrust_n;",
            "activemask=time_s>=commandsteptime_s;",
            "activesamplecount=sum(activemask);",
            "throttletrackingerror=throttlecommand-throttleactual;",
            "throttletrackingrms=sqrt(mean("
            "throttletrackingerror(activemask).^2));",
            "speedtrackingrms_mps=sqrt(mean(speederror_mps(activemask).^2));",
            "finalspeederror_mps=speederror_mps(end);",
            "speederrorattenseconds_mps=speederror_mps("
            "find(time_s>=10,1,'first'));",
            "speedatfiveseconds_mps=trueairspeed_mps("
            "find(time_s>=5,1,'first'));",
            "capturetolerance_mps=0.1*abs(speedcommandstep_mps);",
            "captureindex=find(activemask&abs(speederror_mps)<="
            "capturetolerance_mps,1,'first');",
            "reachedninetypercent=false;",
            "timetoninetypercent_s=timehorizon_s-commandsteptime_s;",
            "reachedninetypercent=true;",
            "timetoninetypercent_s=time_s(captureindex)-commandsteptime_s;",
            "settlingtolerance_mps=0.02*abs(speedcommandstep_mps);",
            "settledbyend=abs(finalspeederror_mps)<=settlingtolerance_mps;",
            "outsidetolerance=find(activemask&abs(speederror_mps)>"
            "settlingtolerance_mps,1,'last');",
            "settlingtime_s=time_s(outsidetolerance+1)-commandsteptime_s;",
            "settlingtime_s=timehorizon_s-commandsteptime_s;",
            "peakspeedovershoot_mps=max(0,max(trueairspeed_mps(activemask)-"
            "commandedtrueairspeed_mps));",
            "minimumstallmargin_mps=min(trueairspeed_mps)-stallspeed_mps;",
            "'thrustcommandsaturationfraction',sum("
            "thrustcommandsaturated)/samplecount,",
            "'stallenvelopemaintained',minimumstallmargin_mps>=0,",
        ):
            with self.subTest(expression=expression):
                self.assertIn(expression, compact)
        for identifier in (
            "P15:model:SpeedGainRange",
            "P15:model:ThrottleTimeConstantRange",
            "P15:model:FeedbackSign",
        ):
            self.assertIn(identifier, model)
        for field in (
            "speedGain_per_s",
            "throttleTimeConstant_s",
            "speedFeedbackSign",
            "parasiteDragScale_N_per_mps2",
            "inducedDragScale_N_mps2",
            "stallSpeed_mps",
            "speedCommand_mps",
            "trueAirspeed_mps",
            "speedError_mps",
            "speedErrorUsed_mps",
            "parasiteDrag_N",
            "inducedDrag_N",
            "drag_N",
            "thrustCommandUnclamped_N",
            "thrustCommand_N",
            "throttleCommand",
            "throttleActual",
            "throttleRate_per_s",
            "thrustActual_N",
            "netForwardForce_N",
            "longitudinalAcceleration_mps2",
            "throttleTrackingRMS",
            "speedTrackingRMS_mps",
            "finalSpeedError_mps",
            "minimumStallMargin_mps",
            "controllerEquation",
            "dragEquation",
            "throttleEquation",
            "speedEquation",
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

    def test_experiment_has_two_isolated_sweeps_and_broken_case(self) -> None:
        experiment = self.text["experiment.m"]
        compact = re.sub(r"\s+", "", experiment.replace("...", "")).lower()
        for expression in (
            "speedgainsweep_per_s=[00.0750.150.2250.3];",
            "throttletimesweep_s=[0.20.50.81.11.4];",
            "sample=model(speedgainsweep_per_s(k),0.8,1);",
            "sample=model(0.15,throttletimesweep_s(k),1);",
            "openspeedloop=model(0,0.8,1);",
            "broken=model(0.15,0.8,-1);",
            "broken.thrustcommand_n(commandindex)==0",
            "broken.thrustcommandsaturationfraction>0.96",
        ):
            self.assertIn(expression, compact)
        self.assertGreaterEqual(experiment.lower().count("mechanism"), 3)
        self.assertGreaterEqual(experiment.lower().count("reset"), 3)
        figures = re.findall(r"figure\('Name','(P15 [^']+)'\)", experiment)
        self.assertEqual(len(figures), 5)
        self.assertEqual(len(figures), len(set(figures)))
        for unit_label in (
            "Time (s)",
            "True airspeed (m/s)",
            "Speed error (m/s)",
            "Throttle (%)",
            "Force (N)",
            "Longitudinal acceleration (m/s^2)",
            "Speed gain (1/s)",
            "Throttle time constant (s)",
            "Throttle tracking RMS (fraction)",
            "Peak throttle rate (1/s)",
        ):
            self.assertIn(unit_label, experiment)
        self.assertNotIn("close all", experiment.lower())
        self.assertIn("clear run_checks;", experiment)
        self.assertIn("run_checks;", experiment)

    def test_interaction_checks_recovery_and_resource_bounds(self) -> None:
        interactive = self.text["interactive.m"]
        checks = self.text["run_checks.m"]
        tutor_checks = self.text["checks.md"].lower()
        for token in (
            "uifigure",
            "uigridlayout",
            "uislider",
            "uiswitch",
            "uibutton",
            "ValueChangingFcn",
            "ValueChangedFcn",
            "resetBaseline",
            "gainControl.Value=0.15",
            "timeControl.Value=0.8",
            "Correct negative feedback",
            "speedFeedbackSign=-1",
            "modelFcn(speedGain_per_s",
            "cla(axSpeed)",
            "cla(axError)",
            "cla(axThrottle)",
            "cla(axForce)",
        ):
            self.assertIn(token, interactive)
        self.assertEqual(interactive.count("uiaxes("), 4)
        self.assertNotIn("close all", interactive.lower())
        for exact_reset in (
            "gainControl.Value=0.15",
            "timeControl.Value=0.8",
            "modeControl.Value='Correct negative feedback'",
        ):
            self.assertIn(exact_reset, interactive)
        self.assertIn("modelFcn=@model", interactive)
        self.assertIn(
            "findall(groot,'Type','figure','Name',uiName)", interactive
        )
        for token in (
            "isequaln(baseline,repeatedBaseline)",
            "expectedParasiteDrag_N",
            "expectedInducedDrag_N",
            "expectedThrustCommand_N",
            "expectedThrottleRate_per_s",
            "expectedSpeedNext_mps",
            "expectedThrottleNext",
            "speedGainSweep_per_s",
            "throttleTimeSweep_s",
            "brokenTailIndices",
            "rolledBackBaseline",
            "malformedCalls",
            "recoveredBaseline",
            "firstDeliveryIndex",
            "firstSpeedResponseIndex",
            "acceptedCornerCount==8",
            "representativeCaseCount==18",
            "timeout",
            "cancellation",
            "migration",
            "backup/restore",
        ):
            self.assertIn(token, checks)
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
            self.assertNotRegex(
                matlab, rf"\b{re.escape(forbidden_call)}\s*\("
            )
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
            "## Independent system-risk review and causal-capture coverage",
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
        self.assertEqual(summary["batch_id"], "P15")
        self.assertEqual(summary["status"], "pass")
        self.assertEqual(len(summary["acceptance"]), 8)
        self.assertTrue(
            all(item["status"] == "pass" for item in summary["acceptance"])
        )


class P15EquationOracleTests(unittest.TestCase):
    def test_deterministic_baseline_signature_and_fixed_shape(self) -> None:
        first = _oracle()
        second = _oracle(0.15, 0.8, 1)
        self.assertEqual(first, second)
        self.assertEqual(first["sample_count"], 1501)
        self.assertEqual(first["interval_count"], 1500)
        self.assertEqual(first["active_sample_count"], 1451)
        self.assertEqual(first["time_s"][0], 0.0)
        self.assertEqual(first["time_s"][-1], 30.0)
        self.assertAlmostEqual(first["initial_drag_n"], 826.9521725905026, 9)
        self.assertAlmostEqual(first["trim_throttle"], 0.20673804314762567, 12)
        self.assertAlmostEqual(first["stall_speed_mps"], 37.546671093409294, 10)
        self.assertEqual(first["speed_error_mps"][50], 10.0)
        self.assertEqual(first["speed_error_used_mps"][50], 10.0)
        self.assertEqual(first["requested_acceleration_mps2"][50], 1.5)
        self.assertAlmostEqual(
            first["thrust_command_n"][50], 2626.9521725905024, 9
        )
        self.assertAlmostEqual(
            first["throttle_command"][50], 0.6567380431476256, 12
        )
        self.assertEqual(first["longitudinal_acceleration_mps2"][50], 0.0)
        self.assertAlmostEqual(
            first["throttle_actual"][75], 0.4163656596679233, 10
        )
        self.assertAlmostEqual(
            first["longitudinal_acceleration_mps2"][75],
            0.6974716229756095,
            10,
        )
        self.assertAlmostEqual(
            first["true_airspeed_mps"][100], 60.62890154528778, 10
        )
        self.assertAlmostEqual(
            first["speed_at_five_seconds_mps"], 64.07175584861862, 10
        )
        self.assertAlmostEqual(
            first["speed_error_at_ten_seconds_mps"],
            2.509387815834927,
            10,
        )
        self.assertAlmostEqual(
            first["final_speed_error_mps"], 0.0799245300887037, 10
        )
        self.assertAlmostEqual(first["time_to_ninety_percent_s"], 14.34, 12)
        self.assertAlmostEqual(first["settling_time_s"], 23.68, 12)
        self.assertAlmostEqual(
            first["throttle_tracking_rms"], 0.053024251502165075, 10
        )
        self.assertTrue(first["settled_by_end"])
        self.assertTrue(first["stall_envelope_maintained"])
        self.assertFalse(any(first["thrust_command_saturated"]))

    def test_every_drag_controller_throttle_force_and_state_update(self) -> None:
        result = _oracle()
        dt = result["sample_time_s"]
        for index in range(result["sample_count"]):
            with self.subTest(sample=index):
                speed = result["true_airspeed_mps"][index]
                expected_parasite = (
                    result["parasite_drag_scale_n_per_mps2"] * speed**2
                )
                expected_induced = (
                    result["induced_drag_scale_n_mps2"] / speed**2
                )
                expected_drag = expected_parasite + expected_induced
                expected_error = result["speed_command_mps"][index] - speed
                expected_used = result["speed_feedback_sign"] * expected_error
                expected_requested_acceleration = (
                    result["speed_gain_per_s"] * expected_used
                )
                expected_raw_thrust = (
                    expected_drag
                    + result["mass_kg"] * expected_requested_acceleration
                )
                expected_thrust_command = _clamp(
                    expected_raw_thrust, 0.0, result["maximum_thrust_n"]
                )
                expected_throttle_command = (
                    expected_thrust_command / result["maximum_thrust_n"]
                )
                expected_throttle_rate = (
                    expected_throttle_command
                    - result["throttle_actual"][index]
                ) / result["throttle_time_constant_s"]
                expected_thrust = (
                    result["maximum_thrust_n"]
                    * result["throttle_actual"][index]
                )
                expected_net_force = expected_thrust - expected_drag
                expected_acceleration = (
                    expected_net_force / result["mass_kg"]
                )
                self.assertAlmostEqual(
                    result["parasite_drag_n"][index], expected_parasite, 10
                )
                self.assertAlmostEqual(
                    result["induced_drag_n"][index], expected_induced, 10
                )
                self.assertAlmostEqual(result["drag_n"][index], expected_drag, 10)
                self.assertAlmostEqual(
                    result["speed_error_mps"][index], expected_error, 12
                )
                self.assertAlmostEqual(
                    result["speed_error_used_mps"][index], expected_used, 12
                )
                self.assertAlmostEqual(
                    result["requested_acceleration_mps2"][index],
                    expected_requested_acceleration,
                    12,
                )
                self.assertAlmostEqual(
                    result["thrust_command_unclamped_n"][index],
                    expected_raw_thrust,
                    9,
                )
                self.assertAlmostEqual(
                    result["thrust_command_n"][index],
                    expected_thrust_command,
                    9,
                )
                self.assertAlmostEqual(
                    result["throttle_command"][index],
                    expected_throttle_command,
                    12,
                )
                self.assertAlmostEqual(
                    result["throttle_rate_per_s"][index],
                    expected_throttle_rate,
                    12,
                )
                self.assertAlmostEqual(
                    result["thrust_actual_n"][index], expected_thrust, 10
                )
                self.assertAlmostEqual(
                    result["net_forward_force_n"][index],
                    expected_net_force,
                    10,
                )
                self.assertAlmostEqual(
                    result["longitudinal_acceleration_mps2"][index],
                    expected_acceleration,
                    12,
                )
                if index == result["interval_count"]:
                    continue
                self.assertAlmostEqual(
                    result["true_airspeed_mps"][index + 1],
                    speed + dt * expected_acceleration,
                    12,
                )
                self.assertAlmostEqual(
                    result["throttle_actual"][index + 1],
                    result["throttle_actual"][index]
                    + dt * expected_throttle_rate,
                    12,
                )

    def test_correct_feedback_preserves_causal_order_and_monotonic_capture(
        self,
    ) -> None:
        result = _oracle()
        command_index = 50
        first_delivery_index = command_index + 1
        first_speed_response_index = command_index + 2
        trim_throttle = result["trim_throttle"]

        self.assertGreater(
            result["throttle_command"][command_index], trim_throttle
        )
        self.assertEqual(
            result["throttle_actual"][command_index], trim_throttle
        )
        self.assertEqual(
            result["longitudinal_acceleration_mps2"][command_index], 0.0
        )
        self.assertEqual(
            result["true_airspeed_mps"][command_index],
            result["initial_true_airspeed_mps"],
        )

        expected_first_delivery = trim_throttle + result["sample_time_s"] * (
            result["throttle_command"][command_index] - trim_throttle
        ) / result["throttle_time_constant_s"]
        self.assertAlmostEqual(
            result["throttle_actual"][first_delivery_index],
            expected_first_delivery,
            12,
        )
        self.assertEqual(
            result["true_airspeed_mps"][first_delivery_index],
            result["initial_true_airspeed_mps"],
        )
        self.assertGreater(
            result["longitudinal_acceleration_mps2"][first_delivery_index],
            0.0,
        )
        self.assertGreater(
            result["true_airspeed_mps"][first_speed_response_index],
            result["initial_true_airspeed_mps"],
        )

        active_speed = result["true_airspeed_mps"][command_index:]
        active_error = result["speed_error_mps"][command_index:]
        self.assertTrue(
            all(
                left <= right
                for left, right in zip(active_speed, active_speed[1:])
            )
        )
        self.assertTrue(
            all(
                left < right
                for left, right in zip(
                    active_speed[1:], active_speed[2:]
                )
            )
        )
        self.assertTrue(
            all(
                left >= right
                for left, right in zip(active_error, active_error[1:])
            )
        )
        self.assertTrue(
            all(
                left > right
                for left, right in zip(
                    active_error[1:], active_error[2:]
                )
            )
        )
        self.assertTrue(
            all(
                force >= 0.0
                for force in result["net_forward_force_n"][command_index:]
            )
        )
        self.assertLess(
            max(active_speed), result["commanded_true_airspeed_mps"]
        )

    def test_zero_speed_gain_is_exact_feedback_open_trim_limit(self) -> None:
        result = _oracle(0.0, 0.8, 1)
        self.assertTrue(
            all(value == 60.0 for value in result["true_airspeed_mps"])
        )
        self.assertTrue(
            all(
                value == result["trim_throttle"]
                for value in result["throttle_command"]
            )
        )
        self.assertTrue(
            all(
                value == result["trim_throttle"]
                for value in result["throttle_actual"]
            )
        )
        for key in (
            "requested_acceleration_mps2",
            "throttle_rate_per_s",
            "net_forward_force_n",
            "longitudinal_acceleration_mps2",
        ):
            self.assertTrue(all(value == 0.0 for value in result[key]), key)
        self.assertEqual(result["final_speed_error_mps"], 10.0)
        self.assertFalse(result["reached_ninety_percent"])
        self.assertFalse(result["settled_by_end"])

    def test_speed_gain_sweep_isolated_capture_authority_trade(self) -> None:
        gains = (0.0, 0.075, 0.15, 0.225, 0.3)
        results = tuple(_oracle(gain, 0.8, 1) for gain in gains)
        expected_speed_at_five = (
            60.0,
            62.207733344641035,
            64.07175584861862,
            65.6318383365179,
            66.8066107392886,
        )
        expected_error_at_ten = (
            10.0,
            5.238802235073322,
            2.509387815834927,
            1.0411921959026245,
            0.3380680265113938,
        )
        expected_capture_times = (29.0, 29.0, 14.34, 9.14, 6.66)
        for result, speed, error, capture in zip(
            results,
            expected_speed_at_five,
            expected_error_at_ten,
            expected_capture_times,
        ):
            self.assertAlmostEqual(result["speed_at_five_seconds_mps"], speed, 9)
            self.assertAlmostEqual(
                result["speed_error_at_ten_seconds_mps"], error, 9
            )
            self.assertAlmostEqual(
                result["time_to_ninety_percent_s"], capture, 12
            )
            self.assertEqual(result["throttle_time_constant_s"], 0.8)
            self.assertEqual(result["speed_feedback_sign"], 1)
            self.assertEqual(result["time_s"], results[0]["time_s"])
            self.assertEqual(
                result["speed_command_mps"], results[0]["speed_command_mps"]
            )
        speed_at_five = tuple(
            result["speed_at_five_seconds_mps"] for result in results
        )
        error_at_ten = tuple(
            result["speed_error_at_ten_seconds_mps"] for result in results
        )
        peak_throttle = tuple(
            result["peak_throttle_actual"] for result in results
        )
        self.assertTrue(
            all(
                left < right
                for left, right in zip(speed_at_five, speed_at_five[1:])
            )
        )
        self.assertTrue(
            all(
                left > right
                for left, right in zip(error_at_ten, error_at_ten[1:])
            )
        )
        self.assertTrue(
            all(
                left < right
                for left, right in zip(peak_throttle, peak_throttle[1:])
            )
        )
        self.assertTrue(
            all(
                left > right
                for left, right in zip(
                    expected_capture_times[1:], expected_capture_times[2:]
                )
            )
        )
        self.assertEqual(results[2]["thrust_command_saturation_fraction"], 0.0)
        self.assertAlmostEqual(
            results[-1]["thrust_command_saturation_fraction"],
            0.03530979347101932,
            12,
        )

    def test_throttle_time_sweep_isolated_tracking_rate_trade(self) -> None:
        time_constants = (0.2, 0.5, 0.8, 1.1, 1.4)
        results = tuple(
            _oracle(0.15, time_constant, 1)
            for time_constant in time_constants
        )
        expected_speed_at_two = (
            61.15429458821988,
            60.82932695729019,
            60.62890154528778,
            60.50331257970016,
            60.41865656026334,
        )
        expected_tracking_rms = (
            0.027113290496212472,
            0.04214430659160602,
            0.053024251502165075,
            0.06197382340441243,
            0.06973981035435724,
        )
        expected_peak_rate = (
            2.25,
            0.9,
            0.5625,
            0.409090909090909,
            0.3214285714285714,
        )
        for result, speed, tracking, peak_rate in zip(
            results,
            expected_speed_at_two,
            expected_tracking_rms,
            expected_peak_rate,
        ):
            self.assertAlmostEqual(result["true_airspeed_mps"][100], speed, 9)
            self.assertAlmostEqual(result["throttle_tracking_rms"], tracking, 9)
            self.assertAlmostEqual(
                result["peak_absolute_throttle_rate_per_s"], peak_rate, 9
            )
            self.assertEqual(result["speed_gain_per_s"], 0.15)
            self.assertEqual(result["speed_feedback_sign"], 1)
            self.assertEqual(result["mass_kg"], 1200.0)
            self.assertEqual(result["maximum_thrust_n"], 4000.0)
        speed_at_two = tuple(
            result["true_airspeed_mps"][100] for result in results
        )
        tracking_rms = tuple(
            result["throttle_tracking_rms"] for result in results
        )
        peak_rate = tuple(
            result["peak_absolute_throttle_rate_per_s"]
            for result in results
        )
        self.assertTrue(
            all(
                left > right
                for left, right in zip(speed_at_two, speed_at_two[1:])
            )
        )
        self.assertTrue(
            all(
                left < right
                for left, right in zip(tracking_rms, tracking_rms[1:])
            )
        )
        self.assertTrue(
            all(left > right for left, right in zip(peak_rate, peak_rate[1:]))
        )

    def test_broken_reversed_feedback_is_isolated_and_recoverable(self) -> None:
        correct = _oracle()
        broken = _oracle(0.15, 0.8, -1)
        command_index = 50
        for key in (
            "true_airspeed_mps",
            "throttle_actual",
            "thrust_actual_n",
            "drag_n",
            "net_forward_force_n",
            "longitudinal_acceleration_mps2",
        ):
            self.assertEqual(
                broken[key][: command_index + 1],
                correct[key][: command_index + 1],
                key,
            )
        self.assertEqual(broken["speed_gain_per_s"], correct["speed_gain_per_s"])
        self.assertEqual(
            broken["throttle_time_constant_s"],
            correct["throttle_time_constant_s"],
        )
        self.assertEqual(broken["speed_feedback_sign"], -1)
        self.assertEqual(
            broken["speed_error_mps"][: command_index + 1],
            correct["speed_error_mps"][: command_index + 1],
        )
        self.assertEqual(broken["speed_error_used_mps"][command_index], -10.0)
        self.assertAlmostEqual(
            broken["thrust_command_unclamped_n"][command_index],
            -973.0478274094974,
            9,
        )
        self.assertEqual(broken["thrust_command_n"][command_index], 0.0)
        self.assertEqual(broken["throttle_command"][command_index], 0.0)
        proper_error = broken["speed_error_mps"][command_index:]
        speed = broken["true_airspeed_mps"][command_index:]
        self.assertTrue(
            all(
                left <= right
                for left, right in zip(proper_error, proper_error[1:])
            )
        )
        self.assertTrue(
            all(left >= right for left, right in zip(speed, speed[1:]))
        )
        self.assertTrue(
            all(value == 0.0 for value in broken["throttle_command"][50:])
        )
        self.assertTrue(all(broken["thrust_command_saturated"][50:]))
        self.assertAlmostEqual(
            broken["minimum_true_airspeed_mps"], 40.98971839975168, 8
        )
        self.assertAlmostEqual(
            broken["final_speed_error_mps"], 29.01028160024832, 8
        )
        self.assertAlmostEqual(
            broken["thrust_command_saturation_fraction"],
            0.966688874083944,
            12,
        )
        self.assertGreater(broken["minimum_stall_margin_mps"], 3.4)
        self.assertTrue(broken["stall_envelope_maintained"])
        self.assertFalse(broken["reached_ninety_percent"])
        self.assertFalse(broken["settled_by_end"])
        self.assertEqual(_oracle(), correct)

    def test_broken_idle_saturation_does_not_arrest_tail_failure(self) -> None:
        broken = _oracle(0.15, 0.8, -1)
        tail_start = broken["sample_count"] - 51
        tail_speed = broken["true_airspeed_mps"][tail_start:]
        tail_error = broken["speed_error_mps"][tail_start:]
        self.assertEqual(broken["time_s"][tail_start], 29.0)
        self.assertTrue(all(broken["thrust_command_saturated"][tail_start:]))
        self.assertTrue(
            all(value == 0.0 for value in broken["throttle_command"][tail_start:])
        )
        self.assertTrue(
            all(left > right for left, right in zip(tail_speed, tail_speed[1:]))
        )
        self.assertTrue(
            all(left < right for left, right in zip(tail_error, tail_error[1:]))
        )
        self.assertGreater(tail_speed[0] - tail_speed[-1], 0.72)
        self.assertGreater(tail_error[-1] - tail_error[0], 0.72)
        for index in range(tail_start, broken["interval_count"]):
            self.assertAlmostEqual(
                broken["true_airspeed_mps"][index + 1],
                broken["true_airspeed_mps"][index]
                + broken["sample_time_s"]
                * broken["longitudinal_acceleration_mps2"][index],
                12,
            )
        self.assertLess(
            broken["longitudinal_acceleration_mps2"][-1], -0.72
        )

    def test_malformed_inputs_reject_without_poisoning_recovery(self) -> None:
        malformed = (
            (-1e-12, 0.8, 1),
            (0.300000000001, 0.8, 1),
            (0.15, 0.199999999, 1),
            (0.15, 1.400000001, 1),
            (0.15, 0.8, 0),
            (0.15, 0.8, 2),
            ([0.075, 0.15], 0.8, 1),
            (0.15, [0.5, 0.8], 1),
            (0.15, 0.8, [-1, 1]),
            (0.15 + 1.0j, 0.8, 1),
            (0.15, 0.8 + 1.0j, 1),
            (float("nan"), 0.8, 1),
            (0.15, float("inf"), 1),
            (0.15, 0.8, float("nan")),
            (True, 0.8, 1),
        )
        for gain, time_constant, sign in malformed:
            with self.subTest(
                gain=gain, time_constant=time_constant, sign=sign
            ):
                with self.assertRaises(ValueError):
                    _oracle(gain, time_constant, sign)
        self.assertEqual(_oracle(), _oracle(0.15, 0.8, 1))

    def test_accepted_corners_and_representative_grid_are_finite_and_fixed(self) -> None:
        corners = tuple(
            _oracle(gain, time_constant, sign)
            for gain in (0.0, 0.3)
            for time_constant in (0.2, 1.4)
            for sign in (-1, 1)
        )
        self.assertEqual(len(corners), 8)
        grid = tuple(
            _oracle(gain, time_constant, sign)
            for gain in (0.0, 0.15, 0.3)
            for time_constant in (0.2, 0.8, 1.4)
            for sign in (-1, 1)
        )
        self.assertEqual(len(grid), 18)
        for result in corners + grid:
            self.assertEqual(result["sample_count"], 1501)
            self.assertEqual(result["interval_count"], 1500)
            for key in (
                "true_airspeed_mps",
                "speed_error_mps",
                "parasite_drag_n",
                "induced_drag_n",
                "drag_n",
                "thrust_command_n",
                "throttle_command",
                "throttle_actual",
                "throttle_rate_per_s",
                "thrust_actual_n",
                "net_forward_force_n",
                "longitudinal_acceleration_mps2",
            ):
                self.assertTrue(
                    all(math.isfinite(value) for value in result[key]), key
                )
            self.assertTrue(result["stall_envelope_maintained"])
            self.assertGreater(min(result["true_airspeed_mps"]), 0.0)
            self.assertGreaterEqual(min(result["throttle_command"]), 0.0)
            self.assertLessEqual(max(result["throttle_command"]), 1.0)
            self.assertGreaterEqual(min(result["throttle_actual"]), 0.0)
            self.assertLessEqual(max(result["throttle_actual"]), 1.0)


if __name__ == "__main__":
    unittest.main()
