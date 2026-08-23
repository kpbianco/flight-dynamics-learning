from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_ID = "P12"
MODULE_FOLDER = ROOT / "modules/12-validate-energy-and-frame-conventions"
EVIDENCE_PATH = ROOT / "docs/evidence/P12-2026-08-23.md"
GUIDING_QUESTION = (
    "What inputs, observable effects, and failure modes matter when you validate "
    "Energy and Frame Conventions?"
)


Vector = tuple[float, float, float]
Matrix = tuple[Vector, Vector, Vector]


def _bounded_scalar(name: str, value: object, lower: float, upper: float) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite real scalar")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite real scalar")
    if not lower <= result <= upper:
        raise ValueError(f"{name} must be between {lower} and {upper} inclusive")
    return result


def _transpose(matrix: Matrix) -> Matrix:
    return tuple(
        tuple(matrix[row][column] for row in range(3))
        for column in range(3)
    )  # type: ignore[return-value]


def _mat_vec(matrix: Matrix, vector: Vector) -> Vector:
    return tuple(
        sum(coefficient * component for coefficient, component in zip(row, vector))
        for row in matrix
    )  # type: ignore[return-value]


def _mat_mul(left: Matrix, right: Matrix) -> Matrix:
    right_transpose = _transpose(right)
    return tuple(
        tuple(
            sum(a * b for a, b in zip(row, column))
            for column in right_transpose
        )
        for row in left
    )  # type: ignore[return-value]


def _determinant(matrix: Matrix) -> float:
    return (
        matrix[0][0]
        * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1]
        * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2]
        * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def _dot(left: Vector, right: Vector) -> float:
    return sum(a * b for a, b in zip(left, right))


def _norm(vector: Vector) -> float:
    return math.sqrt(_dot(vector, vector))


def _max_difference(left: object, right: object) -> float:
    if isinstance(left, tuple) and isinstance(right, tuple):
        return max(
            (_max_difference(a, b) for a, b in zip(left, right)), default=0.0
        )
    return abs(float(left) - float(right))


def _trapezoidal_integral(
    time_s: tuple[float, ...], values: tuple[float, ...]
) -> tuple[float, ...]:
    integral = [0.0] * len(time_s)
    for index in range(len(time_s) - 1):
        step_s = time_s[index + 1] - time_s[index]
        integral[index + 1] = integral[index] + 0.5 * step_s * (
            values[index] + values[index + 1]
        )
    return tuple(integral)


def _oracle(
    forward_specific_force_mps2: object = 1.5,
    heading_angle_deg: object = 30.0,
) -> dict[str, object]:
    """Pure-stdlib equation oracle independent of the MATLAB source."""
    force = _bounded_scalar(
        "forward non-gravity specific force",
        forward_specific_force_mps2,
        0.0,
        3.0,
    )
    heading_deg = _bounded_scalar(
        "heading angle", heading_angle_deg, -180.0, 180.0
    )
    mass_kg = 1200.0
    gravity_mps2 = 9.80665
    initial_altitude_m = 1000.0
    initial_speed_mps = 60.0
    pitch_deg = 30.0
    sample_time_s = 0.02
    time_s = tuple(index * sample_time_s for index in range(301))

    pitch_rad = math.radians(pitch_deg)
    heading_rad = math.radians(heading_deg)
    cosine_pitch = math.cos(pitch_rad)
    sine_pitch = math.sin(pitch_rad)
    cosine_heading = math.cos(heading_rad)
    sine_heading = math.sin(heading_rad)
    body_to_ned: Matrix = (
        (
            cosine_pitch * cosine_heading,
            -sine_heading,
            sine_pitch * cosine_heading,
        ),
        (
            cosine_pitch * sine_heading,
            cosine_heading,
            sine_pitch * sine_heading,
        ),
        (-sine_pitch, 0.0, cosine_pitch),
    )
    ned_to_body = _transpose(body_to_ned)
    initial_position_ned_m: Vector = (0.0, 0.0, -initial_altitude_m)
    initial_velocity_body_mps: Vector = (initial_speed_mps, 0.0, 0.0)
    initial_velocity_ned_mps = _mat_vec(
        body_to_ned, initial_velocity_body_mps
    )
    specific_force_body_mps2: Vector = (force, 0.0, 0.0)
    specific_force_ned_mps2 = _mat_vec(
        body_to_ned, specific_force_body_mps2
    )
    acceleration_ned_mps2: Vector = (
        specific_force_ned_mps2[0],
        specific_force_ned_mps2[1],
        specific_force_ned_mps2[2] + gravity_mps2,
    )
    acceleration_body_components_mps2 = _mat_vec(
        ned_to_body, acceleration_ned_mps2
    )

    position_ned_m: list[Vector] = []
    velocity_ned_mps: list[Vector] = []
    velocity_body_mps: list[Vector] = []
    kinetic_energy_ned_j: list[float] = []
    kinetic_energy_body_j: list[float] = []
    potential_energy_j: list[float] = []
    power_body_w: list[float] = []
    power_ned_w: list[float] = []
    for time in time_s:
        velocity_ned: Vector = tuple(
            initial_velocity_ned_mps[axis]
            + acceleration_ned_mps2[axis] * time
            for axis in range(3)
        )  # type: ignore[assignment]
        position_ned: Vector = tuple(
            initial_position_ned_m[axis]
            + initial_velocity_ned_mps[axis] * time
            + 0.5 * acceleration_ned_mps2[axis] * time * time
            for axis in range(3)
        )  # type: ignore[assignment]
        velocity_body = _mat_vec(ned_to_body, velocity_ned)
        position_ned_m.append(position_ned)
        velocity_ned_mps.append(velocity_ned)
        velocity_body_mps.append(velocity_body)
        kinetic_energy_ned_j.append(0.5 * mass_kg * _dot(velocity_ned, velocity_ned))
        kinetic_energy_body_j.append(
            0.5 * mass_kg * _dot(velocity_body, velocity_body)
        )
        potential_energy_j.append(-mass_kg * gravity_mps2 * position_ned[2])
        power_body_w.append(mass_kg * _dot(specific_force_body_mps2, velocity_body))
        power_ned_w.append(mass_kg * _dot(specific_force_ned_mps2, velocity_ned))

    mechanical_energy_j = tuple(
        kinetic + potential
        for kinetic, potential in zip(kinetic_energy_ned_j, potential_energy_j)
    )
    work_trapezoidal_j = _trapezoidal_integral(time_s, tuple(power_body_w))
    work_closed_form_j = tuple(
        mass_kg
        * force
        * (
            initial_speed_mps * time
            + 0.5 * acceleration_body_components_mps2[0] * time * time
        )
        for time in time_s
    )
    energy_balance_residual_j = tuple(
        energy - mechanical_energy_j[0] - work
        for energy, work in zip(mechanical_energy_j, work_trapezoidal_j)
    )
    down_change_m = tuple(
        position[2] - initial_position_ned_m[2] for position in position_ned_m
    )
    broken_residual_j = tuple(
        2.0 * mass_kg * gravity_mps2 * change for change in down_change_m
    )
    altitude_m = tuple(-position[2] for position in position_ned_m)
    speed_ned_mps = tuple(_norm(vector) for vector in velocity_ned_mps)
    speed_body_mps = tuple(_norm(vector) for vector in velocity_body_mps)
    apex_time_s = -initial_velocity_ned_mps[2] / acceleration_ned_mps2[2]

    return {
        "force_mps2": force,
        "heading_deg": heading_deg,
        "mass_kg": mass_kg,
        "gravity_mps2": gravity_mps2,
        "pitch_deg": pitch_deg,
        "sample_time_s": sample_time_s,
        "time_s": time_s,
        "sample_count": len(time_s),
        "interval_count": len(time_s) - 1,
        "body_to_ned": body_to_ned,
        "ned_to_body": ned_to_body,
        "initial_position_ned_m": initial_position_ned_m,
        "initial_velocity_ned_mps": initial_velocity_ned_mps,
        "specific_force_body_mps2": specific_force_body_mps2,
        "specific_force_ned_mps2": specific_force_ned_mps2,
        "acceleration_ned_mps2": acceleration_ned_mps2,
        "acceleration_body_components_mps2": acceleration_body_components_mps2,
        "position_ned_m": tuple(position_ned_m),
        "velocity_ned_mps": tuple(velocity_ned_mps),
        "velocity_body_mps": tuple(velocity_body_mps),
        "altitude_m": altitude_m,
        "speed_ned_mps": speed_ned_mps,
        "speed_body_mps": speed_body_mps,
        "kinetic_energy_ned_j": tuple(kinetic_energy_ned_j),
        "kinetic_energy_body_j": tuple(kinetic_energy_body_j),
        "potential_energy_j": tuple(potential_energy_j),
        "mechanical_energy_j": mechanical_energy_j,
        "power_body_w": tuple(power_body_w),
        "power_ned_w": tuple(power_ned_w),
        "work_trapezoidal_j": work_trapezoidal_j,
        "work_closed_form_j": work_closed_form_j,
        "energy_balance_residual_j": energy_balance_residual_j,
        "down_change_m": down_change_m,
        "broken_residual_j": broken_residual_j,
        "apex_time_s": apex_time_s,
        "apex_altitude_gain_m": (
            initial_velocity_ned_mps[2] ** 2
            / (2.0 * acceleration_ned_mps2[2])
        ),
        "horizontal_range_m": math.hypot(
            position_ned_m[-1][0], position_ned_m[-1][1]
        ),
    }


class P12ArtifactTests(unittest.TestCase):
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
                "number": 12,
                "id": "P12",
                "title": "Validate Energy and Frame Conventions",
                "guiding_question": GUIDING_QUESTION,
                "phase": 3,
                "phase_title": "Six-degree-of-freedom simulation",
                "slug": "validate-energy-and-frame-conventions",
                "folder": "modules/12-validate-energy-and-frame-conventions",
                "implementation_batch": "P12",
                "prerequisites": ["P11"],
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
            "p11",
            "specific force",
            "north-east-down",
            "body-to-ned",
            "h=-down",
            "kinetic energy",
            "potential energy",
            "mechanical energy",
            "work",
            "mechanism",
            "reset",
            "broken",
            "free fall",
            "teach-back",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, combined)
        self.assertRegex(walkthrough.lower(), r"one plot|one .* at a time")
        self.assertIn("not current api compatibility", combined)
        self.assertIn("p12 does not accept those", combined)
        self.assertIn("active yaw", combined)
        self.assertIn("fixed ned", combined)
        self.assertIn("not a passive", combined)
        self.assertIn("horizontal asymmetry", combined)
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
            "functionout=model(forwardspecificforce_mps2,headingangle_deg)",
            compact,
        )
        self.assertIn("arguments", lower)
        self.assertIn(
            "forwardspecificforce_mps2(1,1)double{mustbereal,mustbefinite}=1.5",
            compact,
        )
        self.assertIn(
            "headingangle_deg(1,1)double{mustbereal,mustbefinite}=30", compact
        )
        self.assertIn(
            "forwardspecificforce_mps2<0||forwardspecificforce_mps2>3", compact
        )
        self.assertIn("headingangle_deg<-180||headingangle_deg>180", compact)
        self.assertIn("P12:model:SpecificForceRange", model)
        self.assertIn("P12:model:HeadingRange", model)
        self.assertIn("active body yaw relative to fixed ned", lower)
        self.assertIn("no wind or horizontal asymmetry", lower)

        for expression in (
            "sampletime_s=0.02;",
            "timehorizon_s=6;",
            "time_s=0:sampletime_s:timehorizon_s;",
            "bodytoned=[cp*cy,-sy,sp*cy;cp*sy,cy,sp*sy;-sp,0,cp];",
            "nedtobody=bodytoned.';",
            "gravityned_mps2=[0;0;gravity_mps2];",
            "initialvelocityned_mps=bodytoned*initialvelocitybody_mps;",
            "specificforcened_mps2=bodytoned*specificforcebody_mps2;",
            "accelerationned_mps2=specificforcened_mps2+gravityned_mps2;",
            "velocityned_mps=repmat(initialvelocityned_mps,1,samplecount)+accelerationned_mps2*time_s;",
            "positionned_m=repmat(initialpositionned_m,1,samplecount)+initialvelocityned_mps*time_s+0.5*accelerationned_mps2*(time_s.^2);",
            "velocitybody_mps=nedtobody*velocityned_mps;",
            "altitude_m=-positionned_m(3,:);",
            "kineticenergyned_j=0.5*mass_kg*sum(velocityned_mps.^2,1);",
            "kineticenergybody_j=0.5*mass_kg*sum(velocitybody_mps.^2,1);",
            "potentialenergy_j=mass_kg*gravity_mps2*altitude_m;",
            "mechanicalenergy_j=kineticenergyned_j+potentialenergy_j;",
            "specificpowerbody_w_per_kg=sum(repmat(specificforcebody_mps2,1,samplecount).*velocitybody_mps,1);",
            "specificpowerned_w_per_kg=sum(repmat(specificforcened_mps2,1,samplecount).*velocityned_mps,1);",
            "powerbody_w=mass_kg*specificpowerbody_w_per_kg;",
            "powerned_w=mass_kg*specificpowerned_w_per_kg;",
            "workinput_j=mass_kg*forwardspecificforce_mps2*(initialspeed_mps*time_s+0.5*bodyforwardacceleration_mps2*time_s.^2);",
            "energybalanceresidual_j=mechanicalenergy_j-mechanicalenergy_j(1)-workinput_j;",
            "brokenheight_m=positionned_m(3,:);",
            "brokenpotentialenergy_j=mass_kg*gravity_mps2*brokenheight_m;",
            "brokenmechanicalenergy_j=kineticenergyned_j+brokenpotentialenergy_j;",
            "expectedbrokenenergybalanceresidual_j=2*mass_kg*gravity_mps2*downchange_m;",
        ):
            with self.subTest(expression=expression):
                self.assertIn(expression, compact)
        for field in (
            "bodyToNED",
            "nedToBody",
            "positionNED_m",
            "velocityNED_mps",
            "velocityBody_mps",
            "kineticEnergyNED_J",
            "kineticEnergyBody_J",
            "potentialEnergy_J",
            "mechanicalEnergy_J",
            "powerBody_W",
            "powerNED_W",
            "workInput_J",
            "energyBalanceResidual_J",
            "brokenEnergyBalanceResidual_J",
            "frameConvention",
            "specificForceEquation",
            "energyEquation",
            "brokenCaseDefinition",
            "analysisScope",
        ):
            with self.subTest(field=field):
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
        self.assertGreaterEqual(experiment.count("%%"), 14)
        self.assertGreaterEqual(lower.count("figure("), 5)
        self.assertGreaterEqual(lower.count("assert("), 7)
        for concept in (
            "baseline",
            "specific force",
            "heading",
            "changed view",
            "mechanism",
            "limiting",
            "free fall",
            "broken",
            "down-as-height",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, lower)
        for unit in ("m/s^2", "deg", "m/s", "kj", "j", "w"):
            self.assertIn(unit, lower)
        self.assertIn("model(specificforcesweep_mps2(k),30)", compact)
        self.assertIn("model(1.5,headingsweep_deg(k))", compact)
        self.assertIn("freefall=model(0,30)", compact)
        self.assertIn("dueeast=model(1.5,90)", compact)
        self.assertIn("baseline.brokenenergybalanceresidual_j", compact)
        self.assertNotIn("j or w", lower)
        self.assertNotIn("kj) or apex gain", lower)
        self.assertIn("ylabel('energyresidual(j)')", compact)
        self.assertIn("ylabel('powerresidual(w)')", compact)
        self.assertIn("ylabel('finalnon-gravitywork(kj)')", compact)
        self.assertIn("ylabel('apexaltitudegain(m)')", compact)
        self.assertNotRegex(lower, r"\bclose\s+all\b")
        self.assertIn("findall(groot", lower)
        self.assertIn("'^p12 '", lower)
        self.assertRegex(lower, r"clear run_checks;\s*run_checks;")
        self.assertNotIn("interactive;", lower)

        assignments = re.findall(
            r"(?:specificForceSweep_mps2|headingSweep_deg)\s*=\s*\[([^\]]+)\]",
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
        self.assertIn("clear model;", "\n".join(interactive_lower.splitlines()[:8]))
        self.assertIn("clear model;", "\n".join(checks_lower.splitlines()[:6]))
        self.assertIn("p12 energy and frame convention explorer", interactive_lower)
        self.assertIn("existingui=findall(groot", interactive_compact)
        self.assertIn("close(existingui)", interactive_compact)
        self.assertEqual(interactive_lower.count("uislider("), 2)
        self.assertEqual(interactive_lower.count("uibutton("), 1)
        self.assertIn("'limits',[03]", interactive_compact)
        self.assertIn("'limits',[-180180]", interactive_compact)
        self.assertIn("'value',1.5", interactive_compact)
        self.assertIn("'value',30", interactive_compact)
        self.assertIn("valuechangingfcn", interactive_lower)
        self.assertIn("valuechangedfcn", interactive_lower)
        self.assertIn("event.value", interactive_lower)
        self.assertIn("buttonpushedfcn", interactive_lower)
        self.assertIn("functionresetbaseline", interactive_compact)
        self.assertIn("forcecontrol.value=1.5", interactive_compact)
        self.assertIn("headingcontrol.value=30", interactive_compact)
        self.assertIn("modelfcn=@model;", interactive_compact)
        self.assertEqual(interactive_compact.count("out=modelfcn("), 1)
        self.assertGreaterEqual(interactive_compact.count("cla("), 4)
        self.assertNotIn("yyaxis", interactive_lower)
        for unit in ("m/s^2", "deg", "m/s", "kj", "j", "w"):
            self.assertIn(unit, interactive_lower)

        self.assertGreaterEqual(checks_lower.count("assert("), 30)
        for concept in (
            "determinism",
            "fixed shapes",
            "finite resources",
            "independently reconstruct",
            "every trapezoidal",
            "signed baseline",
            "free-fall",
            "isolated parameter sweeps",
            "broken down-as-height",
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
        self.assertIn("samplecount==301", checks_compact)
        self.assertIn("intervalcount==300", checks_compact)
        self.assertRegex(checks_compact, r"representativecasecount==9\b")
        self.assertIn("acceptedcornercount==4", checks_compact)
        for expression in (
            "expectedhorizontalposition_m=[cos(headingradians);sin(headingradians)]*duenorth.positionned_m(1,:);",
            "expectedhorizontalvelocity_mps=[cos(headingradians);sin(headingradians)]*duenorth.velocityned_mps(1,:);",
            "expectedhorizontalspecificforce_mps2=[cos(headingradians);sin(headingradians)]*duenorth.specificforcened_mps2(1);",
        ):
            with self.subTest(expression=expression):
                self.assertIn(expression, checks_compact)
        self.assertIn("catch exception", checks_lower)
        self.assertIn(
            "strcmp(exception.identifier,expectedidentifier)", checks_compact
        )
        self.assertIn("P12 checks passed", checks_script)

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
        self.assertNotRegex(matlab, re.compile(r"^\s*(?:global|persistent|parfor|while)\b", re.MULTILINE))
        self.assertNotRegex(matlab, r"\bclose\s+all\b")
        self.assertNotIn("simulink", matlab)
        self.assertNotIn("aerospace toolbox", matlab)

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
        self.assertEqual(summary["batch_id"], "P12")
        self.assertEqual(summary["status"], "pass")
        self.assertEqual(len(summary["acceptance"]), 8)
        self.assertTrue(
            all(item["status"] == "pass" for item in summary["acceptance"])
        )


class P12EquationOracleTests(unittest.TestCase):
    def test_deterministic_baseline_signed_signature_and_shape(self) -> None:
        first = _oracle()
        second = _oracle()
        self.assertEqual(first, second)
        self.assertEqual(first["sample_count"], 301)
        self.assertEqual(first["interval_count"], 300)
        self.assertEqual(first["time_s"][0], 0.0)
        self.assertEqual(first["time_s"][-1], 6.0)
        self.assertEqual(len(first["position_ned_m"]), 301)
        self.assertEqual(len(first["velocity_body_mps"]), 301)

        self.assertLess(
            _max_difference(
                first["body_to_ned"][0][0], 0.75
            ),
            1e-14,
        )
        self.assertLess(
            _max_difference(
                tuple(row[0] for row in first["body_to_ned"]),
                (0.75, math.sqrt(3.0) / 4.0, -0.5),
            ),
            1e-14,
        )
        self.assertLess(
            _max_difference(
                first["initial_velocity_ned_mps"],
                (45.0, 15.0 * math.sqrt(3.0), -30.0),
            ),
            1e-12,
        )
        self.assertLess(
            _max_difference(
                first["specific_force_ned_mps2"],
                (1.125, 0.375 * math.sqrt(3.0), -0.75),
            ),
            1e-14,
        )
        self.assertLess(
            _max_difference(
                first["acceleration_ned_mps2"],
                (1.125, 0.375 * math.sqrt(3.0), 9.05665),
            ),
            1e-14,
        )

        at_two = 100
        self.assertLess(
            _max_difference(
                first["velocity_ned_mps"][at_two],
                (47.25, 27.279800219209815, -11.8867),
            ),
            1e-12,
        )
        self.assertLess(
            _max_difference(
                first["velocity_body_mps"][at_two],
                (53.19335, 0.0, 16.98561605204533),
            ),
            1e-12,
        )
        self.assertLess(
            _max_difference(
                first["position_ned_m"][-1],
                (290.25, 167.575915632289, -1016.9803),
            ),
            1e-9,
        )
        self.assertLess(
            _max_difference(
                first["velocity_ned_mps"][-1],
                (51.75, 29.87787643056313, 24.3399),
            ),
            1e-10,
        )
        self.assertAlmostEqual(first["apex_time_s"], 3.3124830925342152, 12)
        self.assertAlmostEqual(first["apex_altitude_gain_m"], 49.68724638801323, 11)
        self.assertAlmostEqual(first["work_closed_form_j"][-1], 537732.27, 7)

    def test_proper_rotation_frame_invariants_and_every_work_interval(self) -> None:
        result = _oracle()
        identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        self.assertLess(
            _max_difference(
                _mat_mul(result["ned_to_body"], result["body_to_ned"]),
                identity,
            ),
            1e-14,
        )
        self.assertAlmostEqual(_determinant(result["body_to_ned"]), 1.0, 14)
        for index in range(result["sample_count"]):
            with self.subTest(sample=index):
                velocity_ned = result["velocity_ned_mps"][index]
                velocity_body = result["velocity_body_mps"][index]
                reconstructed = _mat_vec(result["body_to_ned"], velocity_body)
                self.assertLess(_max_difference(reconstructed, velocity_ned), 2e-14)
                self.assertAlmostEqual(
                    result["speed_ned_mps"][index],
                    result["speed_body_mps"][index],
                    places=12,
                )
                self.assertAlmostEqual(
                    result["kinetic_energy_ned_j"][index],
                    result["kinetic_energy_body_j"][index],
                    places=7,
                )
                self.assertAlmostEqual(
                    result["power_ned_w"][index],
                    result["power_body_w"][index],
                    places=9,
                )
                self.assertAlmostEqual(
                    result["work_trapezoidal_j"][index],
                    result["work_closed_form_j"][index],
                    places=7,
                )
                self.assertAlmostEqual(
                    result["energy_balance_residual_j"][index], 0.0, places=7
                )

    def test_free_fall_and_heading_signed_limits(self) -> None:
        free_fall = _oracle(0.0, 30.0)
        self.assertEqual(free_fall["specific_force_body_mps2"], (0.0, 0.0, 0.0))
        self.assertEqual(free_fall["specific_force_ned_mps2"], (0.0, 0.0, 0.0))
        self.assertTrue(all(value == 0.0 for value in free_fall["power_body_w"]))
        self.assertTrue(
            all(value == 0.0 for value in free_fall["work_trapezoidal_j"])
        )
        initial_energy = free_fall["mechanical_energy_j"][0]
        self.assertLess(
            max(
                abs(value - initial_energy)
                for value in free_fall["mechanical_energy_j"]
            ),
            1e-7,
        )

        due_north = _oracle(1.5, 0.0)
        due_east = _oracle(1.5, 90.0)
        due_west = _oracle(1.5, -90.0)
        self.assertTrue(all(position[1] == 0.0 for position in due_north["position_ned_m"]))
        self.assertLess(abs(due_east["initial_velocity_ned_mps"][0]), 1e-14)
        self.assertGreater(due_east["initial_velocity_ned_mps"][1], 0.0)
        self.assertLess(due_west["initial_velocity_ned_mps"][1], 0.0)
        self.assertLess(
            max(
                abs(east[0] - west[0])
                for east, west in zip(
                    due_east["position_ned_m"], due_west["position_ned_m"]
                )
            ),
            1e-12,
        )
        self.assertLess(
            max(
                abs(east[1] + west[1])
                for east, west in zip(
                    due_east["position_ned_m"], due_west["position_ned_m"]
                )
            ),
            1e-12,
        )

    def test_force_sweep_isolated_reference_outputs_and_trends(self) -> None:
        forces = (0.0, 0.75, 1.5, 2.25, 3.0)
        results = tuple(_oracle(force, 30.0) for force in forces)
        expected_work_j = (
            0.0,
            256716.135,
            537732.27,
            843048.405,
            1172664.54,
        )
        expected_apex_gain_m = (
            45.887229584006775,
            47.71169413623279,
            49.68724638801323,
            51.833464836753386,
            54.173463429902554,
        )
        baseline = results[2]
        for index, result in enumerate(results):
            with self.subTest(force=forces[index]):
                self.assertEqual(result["body_to_ned"], baseline["body_to_ned"])
                self.assertEqual(
                    result["initial_position_ned_m"],
                    baseline["initial_position_ned_m"],
                )
                self.assertEqual(
                    result["initial_velocity_ned_mps"],
                    baseline["initial_velocity_ned_mps"],
                )
                self.assertAlmostEqual(
                    result["work_closed_form_j"][-1], expected_work_j[index], 6
                )
                self.assertAlmostEqual(
                    result["apex_altitude_gain_m"], expected_apex_gain_m[index], 10
                )
        self.assertTrue(
            all(
                left < right
                for left, right in zip(expected_work_j, expected_work_j[1:])
            )
        )
        ranges = tuple(result["horizontal_range_m"] for result in results)
        self.assertTrue(all(left < right for left, right in zip(ranges, ranges[1:])))

    def test_heading_sweep_rotates_only_horizontal_ned_components(self) -> None:
        baseline = _oracle(1.5, 30.0)
        for heading in (-90.0, -30.0, 0.0, 30.0, 90.0):
            result = _oracle(1.5, heading)
            with self.subTest(heading=heading):
                for key in (
                    "velocity_body_mps",
                    "altitude_m",
                    "speed_ned_mps",
                    "power_body_w",
                    "work_trapezoidal_j",
                    "mechanical_energy_j",
                    "broken_residual_j",
                ):
                    self.assertLess(_max_difference(result[key], baseline[key]), 1e-7)
                self.assertAlmostEqual(
                    result["horizontal_range_m"],
                    baseline["horizontal_range_m"],
                    places=10,
                )
                self.assertAlmostEqual(
                    result["apex_altitude_gain_m"],
                    baseline["apex_altitude_gain_m"],
                    places=12,
                )

    def test_heading_lever_rotates_intermediate_paths_by_commanded_angle(self) -> None:
        due_north = _oracle(1.5, 0.0)
        for heading in (-90.0, -30.0, 0.0, 30.0, 90.0):
            result = _oracle(1.5, heading)
            heading_rad = math.radians(heading)
            cosine_heading = math.cos(heading_rad)
            sine_heading = math.sin(heading_rad)
            expected_force = (
                due_north["specific_force_ned_mps2"][0] * cosine_heading,
                due_north["specific_force_ned_mps2"][0] * sine_heading,
            )
            with self.subTest(heading=heading):
                self.assertLess(
                    _max_difference(
                        result["specific_force_ned_mps2"][:2], expected_force
                    ),
                    1e-14,
                )
                for index, (position, velocity) in enumerate(
                    zip(result["position_ned_m"], result["velocity_ned_mps"])
                ):
                    with self.subTest(heading=heading, sample=index):
                        radial_position = due_north["position_ned_m"][index][0]
                        radial_velocity = due_north["velocity_ned_mps"][index][0]
                        self.assertLess(
                            _max_difference(
                                position[:2],
                                (
                                    radial_position * cosine_heading,
                                    radial_position * sine_heading,
                                ),
                            ),
                            1e-12,
                        )
                        self.assertLess(
                            _max_difference(
                                velocity[:2],
                                (
                                    radial_velocity * cosine_heading,
                                    radial_velocity * sine_heading,
                                ),
                            ),
                            1e-12,
                        )

    def test_broken_sign_is_exact_isolated_and_not_visible_at_datum(self) -> None:
        result = _oracle()
        residual = result["broken_residual_j"]
        self.assertEqual(residual[0], 0.0)
        self.assertGreater(max(map(abs, residual)), 1_000_000.0)
        for actual, down_change in zip(residual, result["down_change_m"]):
            self.assertAlmostEqual(
                actual,
                2.0 * result["mass_kg"] * result["gravity_mps2"] * down_change,
                places=7,
            )
        apex_index = min(
            range(result["sample_count"]),
            key=lambda index: abs(result["velocity_ned_mps"][index][2]),
        )
        self.assertLess(result["down_change_m"][apex_index], 0.0)
        self.assertLess(residual[apex_index], -1_000_000.0)
        self.assertAlmostEqual(
            result["energy_balance_residual_j"][apex_index], 0.0, places=7
        )

    def test_malformed_inputs_reject_and_recovery_is_deterministic(self) -> None:
        malformed = (
            (-0.001, 30.0),
            (3.001, 30.0),
            (1.5, -180.001),
            (1.5, 180.001),
            ([1.0, 2.0], 30.0),
            (1.5, [20.0, 30.0]),
            (1.5 + 1.0j, 30.0),
            (1.5, 30.0 + 1.0j),
            (float("nan"), 30.0),
            (1.5, float("inf")),
            (True, 30.0),
        )
        for force, heading in malformed:
            with self.subTest(force=force, heading=heading):
                with self.assertRaises(ValueError):
                    _oracle(force, heading)
        self.assertEqual(_oracle(), _oracle(1.5, 30.0))

    def test_accepted_corners_and_representative_grid_are_finite_and_fixed(self) -> None:
        corners = tuple(
            _oracle(force, heading)
            for force in (0.0, 3.0)
            for heading in (-180.0, 180.0)
        )
        self.assertEqual(len(corners), 4)
        grid = tuple(
            _oracle(force, heading)
            for force in (0.0, 1.5, 3.0)
            for heading in (-90.0, 30.0, 90.0)
        )
        self.assertEqual(len(grid), 9)
        for result in corners + grid:
            self.assertEqual(result["sample_count"], 301)
            self.assertEqual(result["interval_count"], 300)
            for history in (
                result["position_ned_m"],
                result["velocity_ned_mps"],
                result["mechanical_energy_j"],
                result["work_trapezoidal_j"],
                result["broken_residual_j"],
            ):
                self.assertTrue(
                    all(
                        math.isfinite(component)
                        for value in history
                        for component in (value if isinstance(value, tuple) else (value,))
                    )
                )


if __name__ == "__main__":
    unittest.main()
