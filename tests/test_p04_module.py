from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULE_ID = "P04"
MODULE_FOLDER = ROOT / "modules/04-balance-forces-in-trim"
GUIDING_QUESTION = (
    "What inputs, observable effects, and failure modes matter when you balance "
    "Forces in Trim?"
)
BASELINE_DENSITY_KGPM3 = 0.736115547399152


def _finite_real_scalar(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite real scalar")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite real scalar")
    return result


def _oracle(
    air_density_kgpm3: object = BASELINE_DENSITY_KGPM3,
    true_airspeed_mps: object = 60.0,
    mass_kg: object = 1200.0,
    flight_path_angle_deg: object = 0.0,
) -> dict[str, object]:
    """Independent Python oracle; it does not execute or translate MATLAB."""
    density = _finite_real_scalar("air density", air_density_kgpm3)
    airspeed = _finite_real_scalar("true airspeed", true_airspeed_mps)
    mass = _finite_real_scalar("mass", mass_kg)
    path_angle = _finite_real_scalar("flight-path angle", flight_path_angle_deg)
    if density <= 0.0:
        raise ValueError("air density must be positive")
    if not 0.04 <= density <= 2.0:
        raise ValueError("air density outside the learning range")
    if airspeed <= 0.0:
        raise ValueError("true airspeed must be positive")
    if not 20.0 <= airspeed <= 200.0:
        raise ValueError("true airspeed outside the learning range")
    if mass <= 0.0:
        raise ValueError("mass must be positive")
    if not 400.0 <= mass <= 3000.0:
        raise ValueError("mass outside the learning range")
    if abs(path_angle) > 10.0:
        raise ValueError("flight-path angle outside the learning range")

    gravity_mps2 = 9.80665
    wing_area_m2 = 16.2
    zero_lift_coefficient = 0.25
    lift_curve_slope_per_rad = 5.0
    parasite_drag_coefficient = 0.025
    induced_drag_factor = 0.045
    maximum_lift_coefficient = 1.4
    maximum_thrust_n = 4000.0

    path_angle_rad = math.radians(path_angle)
    weight_n = mass * gravity_mps2
    dynamic_pressure_pa = 0.5 * density * airspeed**2
    normal_force_required_n = weight_n * math.cos(path_angle_rad)
    weight_along_path_n = weight_n * math.sin(path_angle_rad)
    lift_coefficient = normal_force_required_n / (dynamic_pressure_pa * wing_area_m2)
    angle_of_attack_rad = (
        lift_coefficient - zero_lift_coefficient
    ) / lift_curve_slope_per_rad
    induced_drag_coefficient = induced_drag_factor * lift_coefficient**2
    drag_coefficient = parasite_drag_coefficient + induced_drag_coefficient
    lift_n = dynamic_pressure_pa * wing_area_m2 * lift_coefficient
    parasite_drag_n = dynamic_pressure_pa * wing_area_m2 * parasite_drag_coefficient
    induced_drag_n = dynamic_pressure_pa * wing_area_m2 * induced_drag_coefficient
    drag_n = parasite_drag_n + induced_drag_n
    thrust_required_n = drag_n + weight_along_path_n
    normal_force_residual_n = lift_n - normal_force_required_n
    axial_force_residual_n = thrust_required_n - drag_n - weight_along_path_n
    stall_speed_mps = math.sqrt(
        2.0
        * normal_force_required_n
        / (density * wing_area_m2 * maximum_lift_coefficient)
    )
    minimum_drag_speed_mps = math.sqrt(
        2.0
        * normal_force_required_n
        / (density * wing_area_m2)
        * math.sqrt(induced_drag_factor / parasite_drag_coefficient)
    )
    lift_coefficient_margin = maximum_lift_coefficient * (
        1.0 - (stall_speed_mps / airspeed) ** 2
    )
    lift_feasible = lift_coefficient_margin >= 0.0
    thrust_feasible = 0.0 <= thrust_required_n <= maximum_thrust_n

    return {
        "air_density_kgpm3": density,
        "true_airspeed_mps": airspeed,
        "mass_kg": mass,
        "flight_path_angle_deg": path_angle,
        "flight_path_angle_rad": path_angle_rad,
        "weight_n": weight_n,
        "dynamic_pressure_pa": dynamic_pressure_pa,
        "normal_force_required_n": normal_force_required_n,
        "weight_along_path_n": weight_along_path_n,
        "lift_coefficient": lift_coefficient,
        "angle_of_attack_rad": angle_of_attack_rad,
        "angle_of_attack_deg": math.degrees(angle_of_attack_rad),
        "induced_drag_coefficient": induced_drag_coefficient,
        "drag_coefficient": drag_coefficient,
        "lift_n": lift_n,
        "parasite_drag_n": parasite_drag_n,
        "induced_drag_n": induced_drag_n,
        "drag_n": drag_n,
        "thrust_required_n": thrust_required_n,
        "required_thrust_fraction": thrust_required_n / maximum_thrust_n,
        "normal_force_residual_n": normal_force_residual_n,
        "axial_force_residual_n": axial_force_residual_n,
        "force_residual_magnitude_n": math.hypot(
            normal_force_residual_n, axial_force_residual_n
        ),
        "stall_speed_mps": stall_speed_mps,
        "minimum_drag_speed_mps": minimum_drag_speed_mps,
        "lift_coefficient_margin": lift_coefficient_margin,
        "thrust_lower_margin_n": thrust_required_n,
        "thrust_upper_margin_n": maximum_thrust_n - thrust_required_n,
        "lift_feasible": lift_feasible,
        "thrust_feasible": thrust_feasible,
        "trim_feasible": lift_feasible and thrust_feasible,
        "gravity_mps2": gravity_mps2,
        "wing_area_m2": wing_area_m2,
        "zero_lift_coefficient": zero_lift_coefficient,
        "lift_curve_slope_per_rad": lift_curve_slope_per_rad,
        "parasite_drag_coefficient": parasite_drag_coefficient,
        "induced_drag_factor": induced_drag_factor,
        "maximum_lift_coefficient": maximum_lift_coefficient,
        "maximum_thrust_n": maximum_thrust_n,
    }


class P04ArtifactTests(unittest.TestCase):
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
                "number": 4,
                "id": "P04",
                "title": "Balance Forces in Trim",
                "guiding_question": GUIDING_QUESTION,
                "phase": 1,
                "phase_title": "Point-mass flight",
                "slug": "balance-forces-in-trim",
                "folder": "modules/04-balance-forces-in-trim",
                "implementation_batch": "P04",
                "prerequisites": ["P03"],
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

        self.assertIn("p03", combined)
        self.assertIn("air density", combined)
        self.assertIn("true airspeed", combined)
        self.assertIn("point-mass force trim", combined)
        self.assertIn("q = 0.5 rho v^2", combined)
        self.assertIn("l = w cos(gamma)", combined)
        self.assertIn("t = d + w sin(gamma)", combined)
        self.assertIn("read", walkthrough.lower())
        self.assertIn("baseline", walkthrough.lower())
        self.assertRegex(walkthrough.lower(), r"one lever|true airspeed alone")
        self.assertRegex(walkthrough.lower(), r"visual transition|observe|inspect")
        self.assertIn("mechanism", combined)
        self.assertIn("reset", combined)
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

    def test_model_is_transparent_guarded_and_presentation_free(self) -> None:
        model = self.text["model.m"]
        compact = re.sub(r"\s+", "", model.replace("...", "")).lower()

        self.assertIn(
            "functionout=model(airdensity_kgpm3,trueairspeed_mps,mass_kg,"
            "flightpathangle_deg)",
            compact,
        )
        self.assertIn("arguments", model.lower())
        self.assertEqual(
            model.count("(1,1) double {mustBeReal,mustBeFinite}"), 4
        )
        for expression in (
            "airdensity_kgpm3<=0",
            "airdensity_kgpm3<0.04||airdensity_kgpm3>2",
            "trueairspeed_mps<=0",
            "trueairspeed_mps<20||trueairspeed_mps>200",
            "mass_kg<=0",
            "mass_kg<400||mass_kg>3000",
            "abs(flightpathangle_deg)>10",
        ):
            self.assertIn(expression, compact)
        for identifier in (
            "P04:model:PositiveDensity",
            "P04:model:DensityRange",
            "P04:model:PositiveAirspeed",
            "P04:model:AirspeedRange",
            "P04:model:PositiveMass",
            "P04:model:MassRange",
            "P04:model:FlightPathAngleRange",
        ):
            self.assertIn(identifier, model)

        for formula in (
            "weight_n=mass_kg*gravity_mps2;",
            "dynamicpressure_pa=0.5*airdensity_kgpm3*trueairspeed_mps^2;",
            "normalforcerequired_n=weight_n*cos(flightpathangle_rad);",
            "weightalongpath_n=weight_n*sin(flightpathangle_rad);",
            "liftcoefficient=normalforcerequired_n/(dynamicpressure_pa*wingarea_m2);",
            "angleofattack_rad=(liftcoefficient-zeroliftcoefficient)/liftcurveslope_perrad;",
            "induceddragcoefficient=induceddragfactor*liftcoefficient^2;",
            "dragcoefficient=parasitedragcoefficient+induceddragcoefficient;",
            "thrustrequired_n=drag_n+weightalongpath_n;",
            "liftcoefficientmargin=maximumliftcoefficient*(1-(stallspeed_mps/"
            "trueairspeed_mps)^2);",
            "liftfeasible=liftcoefficientmargin>=0;",
            "trimfeasible=liftfeasible&&thrustfeasible;",
        ):
            self.assertIn(formula, compact)
        self.assertIn("requiredThrustFraction", model)
        self.assertIn("normalForceResidual_N", model)
        self.assertIn("axialForceResidual_N", model)

        for presentation_call in (
            "figure(",
            "plot(",
            "bar(",
            "uiaxes(",
            "uifigure(",
            "disp(",
            "fprintf(",
        ):
            self.assertNotIn(presentation_call, model.lower())
        self.assertNotRegex(
            model.lower(), re.compile(r"^\s*(?:for|while|parfor)\b", re.MULTILINE)
        )

    def test_experiment_has_two_sweeps_metrics_and_broken_case(self) -> None:
        experiment = self.text["experiment.m"]
        lower = experiment.lower()
        compact = re.sub(r"\s+", "", experiment.replace("...", "")).lower()
        self.assertGreaterEqual(experiment.count("%%"), 12)
        self.assertIn("baseline", lower)
        self.assertGreaterEqual(lower.count("sweep"), 2)
        self.assertRegex(lower, r"sweep[^\n]*true airspeed|airspeed[^\n]*sweep")
        self.assertRegex(lower, r"sweep[^\n]*mass|mass[^\n]*sweep")
        self.assertIn("broken", lower)
        self.assertIn("omit the one-half", lower)
        self.assertIn("normal residual", lower)
        self.assertGreaterEqual(lower.count("figure("), 4)
        self.assertIn("xlabel", lower)
        self.assertIn("ylabel", lower)
        for unit in ("m/s", "kg/m^3", "pa", "n)", "deg"):
            with self.subTest(unit=unit):
                self.assertIn(unit, lower)
        self.assertIn("fprintf", lower)
        self.assertIn("assert(", lower)
        self.assertIn(
            "model(0.736115547399152,speedsweep_mps(k),1200,0)", compact
        )
        self.assertIn(
            "model(0.736115547399152,60,masssweep_kg(k),0)", compact
        )
        self.assertNotRegex(lower, r"\bclose\s+all\b")
        self.assertIn("findall(groot", lower)
        self.assertIn("'^p[0-9][0-9] '", lower)
        self.assertIn("-baseline.normalforcerequired_n", compact)
        self.assertRegex(lower, r"clear run_checks;\s*run_checks;")

        for variable in ("speedSweep_mps", "massSweep_kg"):
            match = re.search(rf"{variable}\s*=\s*\[([^\]]+)\]", experiment)
            self.assertIsNotNone(match, variable)
            values = [float(value) for value in match.group(1).split()]
            self.assertGreaterEqual(len(values), 3, variable)
            self.assertLessEqual(len(values), 25, variable)
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
        )

        self.assertIn("clear model;", "\n".join(experiment.splitlines()[:10]).lower())
        self.assertIn("clear model;", "\n".join(interactive_lower.splitlines()[:5]))
        self.assertIn("clear model;", "\n".join(checks_lower.splitlines()[:5]))
        self.assertIn("uifigure(", interactive_lower)
        self.assertGreaterEqual(interactive_lower.count("uislider("), 4)
        for control in ("density", "airspeed", "mass", "path"):
            self.assertIn(control, interactive_lower)
        self.assertIn("valuechangingfcn", interactive_lower)
        self.assertIn("valuechangedfcn", interactive_lower)
        self.assertIn("event.value", interactive_lower)
        self.assertIn("modelfcn=@model;", interactive_compact)
        self.assertIn("out=modelfcn(", interactive_compact)
        self.assertIn("speedgrid_mps=20:5:160", interactive_compact)
        self.assertIn("forindex=1:numel(speedgrid_mps)", interactive_compact)
        self.assertIn(
            "yyaxis(axmargins,'left');cla(axmargins);"
            "yyaxis(axmargins,'right');cla(axmargins);"
            "yyaxis(axmargins,'left');",
            interactive_compact,
        )
        self.assertIn("-out.normalforcerequired_n", interactive_compact)
        self.assertIn("requiredthrustfraction", interactive_compact)
        for unit in ("m/s", "kg/m^3", "pa", "n)", "deg"):
            self.assertIn(unit, interactive_lower)

        self.assertGreaterEqual(checks_lower.count("assert("), 28)
        for concept in (
            "determin",
            "force-balance",
            "stall",
            "minimum drag",
            "doubling speed",
            "doubling mass",
            "doubling density",
            "sweep regressions",
            "path-angle symmetry",
            "negative-thrust",
            "cl limit",
            "thrust cap",
            "broken",
            "densityrange",
            "airspeedrange",
            "massrange",
            "flightpathanglerange",
            "rejected inputs",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept.replace("-", ""), checks_lower.replace("-", ""))
        self.assertIn("edgecasecount==16", checks_compact.lower())
        self.assertIn("catch exception", checks_lower)
        self.assertIn(
            "strcmp(exception.identifier,expectedIdentifier)", checks_compact
        )
        self.assertIn("P04 checks passed", checks_script)

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
            "atmosisa",
            "atmoscoesa",
            "fzero",
            "roots",
            "ode45",
            "integral",
            "arrayfun",
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
        self.assertNotRegex(matlab, re.compile(r"^\s*(?:while|parfor)\b", re.MULTILINE))
        self.assertNotRegex(matlab, re.compile(r"^\s*(?:global|persistent)\b", re.MULTILINE))
        self.assertNotRegex(matlab, r"\bclose\s+all\b")


class P04IndependentOracleTests(unittest.TestCase):
    def test_baseline_is_deterministic_and_physically_interpretable(self) -> None:
        first = _oracle()
        second = _oracle()
        self.assertEqual(first, second)
        self.assertAlmostEqual(first["dynamic_pressure_pa"], 1325.00798531847, places=9)
        self.assertAlmostEqual(first["weight_n"], 11767.98, places=9)
        self.assertAlmostEqual(first["lift_coefficient"], 0.548237087298700, places=12)
        self.assertAlmostEqual(first["angle_of_attack_deg"], 3.41754527929804, places=12)
        self.assertAlmostEqual(first["parasite_drag_n"], 536.628234053982, places=9)
        self.assertAlmostEqual(first["induced_drag_n"], 290.323938536521, places=9)
        self.assertAlmostEqual(first["thrust_required_n"], 826.952172590503, places=9)
        self.assertAlmostEqual(first["stall_speed_mps"], 37.5466710934093, places=11)
        self.assertAlmostEqual(first["minimum_drag_speed_mps"], 51.4580805430069, places=11)
        self.assertTrue(first["trim_feasible"])

    def test_force_lift_and_drag_identities_close_independently(self) -> None:
        result = _oracle(0.9, 73.0, 1350.0, 4.0)
        q = 0.5 * result["air_density_kgpm3"] * result["true_airspeed_mps"] ** 2
        weight = result["mass_kg"] * result["gravity_mps2"]
        normal = weight * math.cos(result["flight_path_angle_rad"])
        along = weight * math.sin(result["flight_path_angle_rad"])
        lift_coefficient = normal / (q * result["wing_area_m2"])
        angle = (
            lift_coefficient - result["zero_lift_coefficient"]
        ) / result["lift_curve_slope_per_rad"]
        drag_coefficient = result["parasite_drag_coefficient"] + result[
            "induced_drag_factor"
        ] * lift_coefficient**2
        drag = q * result["wing_area_m2"] * drag_coefficient

        self.assertAlmostEqual(result["dynamic_pressure_pa"], q, places=12)
        self.assertAlmostEqual(result["normal_force_required_n"], normal, places=11)
        self.assertAlmostEqual(result["weight_along_path_n"], along, places=11)
        self.assertAlmostEqual(result["lift_coefficient"], lift_coefficient, places=14)
        self.assertAlmostEqual(result["angle_of_attack_rad"], angle, places=14)
        self.assertAlmostEqual(result["drag_n"], drag, places=11)
        self.assertAlmostEqual(result["thrust_required_n"], drag + along, places=11)
        self.assertLess(result["force_residual_magnitude_n"], 1e-10)

    def test_stall_and_minimum_drag_limits(self) -> None:
        baseline = _oracle()
        at_stall = _oracle(true_airspeed_mps=baseline["stall_speed_mps"])
        below_stall = _oracle(true_airspeed_mps=0.999 * baseline["stall_speed_mps"])
        above_stall = _oracle(true_airspeed_mps=1.001 * baseline["stall_speed_mps"])
        self.assertAlmostEqual(
            at_stall["lift_coefficient"], at_stall["maximum_lift_coefficient"], places=13
        )
        self.assertFalse(below_stall["lift_feasible"])
        self.assertTrue(above_stall["lift_feasible"])

        at_minimum_drag = _oracle(
            true_airspeed_mps=baseline["minimum_drag_speed_mps"]
        )
        self.assertAlmostEqual(
            at_minimum_drag["parasite_drag_n"],
            at_minimum_drag["induced_drag_n"],
            places=10,
        )
        expected_minimum_drag = 2.0 * at_minimum_drag[
            "normal_force_required_n"
        ] * math.sqrt(
            at_minimum_drag["parasite_drag_coefficient"]
            * at_minimum_drag["induced_drag_factor"]
        )
        self.assertAlmostEqual(at_minimum_drag["drag_n"], expected_minimum_drag, places=10)

    def test_inclined_stall_boundary_is_inclusive_without_gate_tolerance(self) -> None:
        reference = _oracle(flight_path_angle_deg=5.0)
        stall_speed = reference["stall_speed_mps"]
        at_stall = _oracle(
            true_airspeed_mps=stall_speed, flight_path_angle_deg=5.0
        )
        just_below = _oracle(
            true_airspeed_mps=math.nextafter(stall_speed, -math.inf),
            flight_path_angle_deg=5.0,
        )
        just_above = _oracle(
            true_airspeed_mps=math.nextafter(stall_speed, math.inf),
            flight_path_angle_deg=5.0,
        )

        self.assertAlmostEqual(
            at_stall["lift_coefficient"], at_stall["maximum_lift_coefficient"], places=14
        )
        self.assertEqual(at_stall["lift_coefficient_margin"], 0.0)
        self.assertTrue(at_stall["lift_feasible"])
        self.assertLess(just_below["lift_coefficient_margin"], 0.0)
        self.assertFalse(just_below["lift_feasible"])
        self.assertGreater(just_above["lift_coefficient_margin"], 0.0)
        self.assertTrue(just_above["lift_feasible"])

    def test_speed_mass_and_density_scaling(self) -> None:
        slow = _oracle(true_airspeed_mps=40.0)
        fast = _oracle(true_airspeed_mps=80.0)
        self.assertAlmostEqual(fast["dynamic_pressure_pa"] / slow["dynamic_pressure_pa"], 4.0)
        self.assertAlmostEqual(fast["lift_coefficient"] / slow["lift_coefficient"], 0.25)
        self.assertAlmostEqual(fast["parasite_drag_n"] / slow["parasite_drag_n"], 4.0)
        self.assertAlmostEqual(fast["induced_drag_n"] / slow["induced_drag_n"], 0.25)

        light = _oracle(mass_kg=600.0)
        heavy = _oracle(mass_kg=1200.0)
        self.assertAlmostEqual(heavy["lift_coefficient"] / light["lift_coefficient"], 2.0)
        self.assertAlmostEqual(heavy["parasite_drag_n"] / light["parasite_drag_n"], 1.0)
        self.assertAlmostEqual(heavy["induced_drag_n"] / light["induced_drag_n"], 4.0)

        thin = _oracle(air_density_kgpm3=0.5)
        dense = _oracle(air_density_kgpm3=1.0)
        self.assertAlmostEqual(dense["dynamic_pressure_pa"] / thin["dynamic_pressure_pa"], 2.0)
        self.assertAlmostEqual(dense["lift_coefficient"] / thin["lift_coefficient"], 0.5)
        self.assertAlmostEqual(dense["parasite_drag_n"] / thin["parasite_drag_n"], 2.0)
        self.assertAlmostEqual(dense["induced_drag_n"] / thin["induced_drag_n"], 0.5)

    def test_two_sweeps_change_only_the_intended_inputs_and_observables(self) -> None:
        speeds = (40.0, 50.0, 60.0, 80.0, 100.0)
        speed_results = [_oracle(true_airspeed_mps=speed) for speed in speeds]
        self.assertEqual(
            [result["mass_kg"] for result in speed_results],
            [1200.0] * len(speeds),
        )
        self.assertTrue(
            all(
                left > right
                for left, right in zip(
                    [result["lift_coefficient"] for result in speed_results],
                    [result["lift_coefficient"] for result in speed_results][1:],
                )
            )
        )
        self.assertTrue(
            all(
                left < right
                for left, right in zip(
                    [result["parasite_drag_n"] for result in speed_results],
                    [result["parasite_drag_n"] for result in speed_results][1:],
                )
            )
        )
        self.assertEqual(
            min(range(len(speed_results)), key=lambda index: speed_results[index]["drag_n"]),
            1,
        )

        masses = (800.0, 1000.0, 1200.0, 1400.0, 1600.0)
        mass_results = [_oracle(mass_kg=mass) for mass in masses]
        self.assertEqual(
            [result["true_airspeed_mps"] for result in mass_results],
            [60.0] * len(masses),
        )
        for key in ("lift_coefficient", "angle_of_attack_deg", "induced_drag_n", "drag_n"):
            values = [result[key] for result in mass_results]
            self.assertTrue(all(left < right for left, right in zip(values, values[1:])), key)
        parasite = [result["parasite_drag_n"] for result in mass_results]
        self.assertEqual(parasite, [parasite[0]] * len(parasite))

    def test_path_angle_symmetry_and_distinct_feasibility_failures(self) -> None:
        climb = _oracle(flight_path_angle_deg=5.0)
        descent = _oracle(flight_path_angle_deg=-5.0)
        baseline = _oracle()
        self.assertAlmostEqual(
            climb["normal_force_required_n"], descent["normal_force_required_n"], places=11
        )
        self.assertAlmostEqual(climb["drag_n"], descent["drag_n"], places=11)
        self.assertAlmostEqual(
            climb["thrust_required_n"] - descent["thrust_required_n"],
            2.0 * baseline["weight_n"] * math.sin(math.radians(5.0)),
            places=10,
        )

        negative_thrust = _oracle(flight_path_angle_deg=-10.0)
        lift_limited = _oracle(true_airspeed_mps=30.0)
        thrust_limited = _oracle(true_airspeed_mps=170.0)
        self.assertTrue(negative_thrust["lift_feasible"])
        self.assertFalse(negative_thrust["thrust_feasible"])
        self.assertLess(negative_thrust["thrust_required_n"], 0.0)
        self.assertFalse(lift_limited["lift_feasible"])
        self.assertTrue(lift_limited["thrust_feasible"])
        self.assertTrue(thrust_limited["lift_feasible"])
        self.assertFalse(thrust_limited["thrust_feasible"])
        self.assertGreater(
            thrust_limited["thrust_required_n"], thrust_limited["maximum_thrust_n"]
        )

    def test_broken_dynamic_pressure_factor_leaves_half_weight_residual(self) -> None:
        baseline = _oracle()
        broken_lift_coefficient = baseline["weight_n"] / (
            baseline["air_density_kgpm3"]
            * baseline["true_airspeed_mps"] ** 2
            * baseline["wing_area_m2"]
        )
        broken_lift_n = (
            baseline["dynamic_pressure_pa"]
            * baseline["wing_area_m2"]
            * broken_lift_coefficient
        )
        broken_residual_n = broken_lift_n - baseline["normal_force_required_n"]
        self.assertAlmostEqual(
            broken_lift_n / baseline["normal_force_required_n"], 0.5, places=14
        )
        self.assertAlmostEqual(
            broken_residual_n / baseline["normal_force_required_n"], -0.5, places=14
        )

    def test_malformed_inputs_fail_without_poisoning_recovery(self) -> None:
        baseline = _oracle()
        malformed_cases = (
            ("zero density", (0.0, 60.0, 1200.0, 0.0)),
            ("negative density", (-1.0, 60.0, 1200.0, 0.0)),
            ("below density range", (0.039, 60.0, 1200.0, 0.0)),
            ("above density range", (2.001, 60.0, 1200.0, 0.0)),
            ("zero speed", (BASELINE_DENSITY_KGPM3, 0.0, 1200.0, 0.0)),
            ("negative speed", (BASELINE_DENSITY_KGPM3, -1.0, 1200.0, 0.0)),
            ("below speed range", (BASELINE_DENSITY_KGPM3, 19.9, 1200.0, 0.0)),
            ("above speed range", (BASELINE_DENSITY_KGPM3, 200.1, 1200.0, 0.0)),
            ("zero mass", (BASELINE_DENSITY_KGPM3, 60.0, 0.0, 0.0)),
            ("negative mass", (BASELINE_DENSITY_KGPM3, 60.0, -1.0, 0.0)),
            ("below mass range", (BASELINE_DENSITY_KGPM3, 60.0, 399.0, 0.0)),
            ("above mass range", (BASELINE_DENSITY_KGPM3, 60.0, 3001.0, 0.0)),
            ("path range", (BASELINE_DENSITY_KGPM3, 60.0, 1200.0, 10.1)),
            ("negative path range", (BASELINE_DENSITY_KGPM3, 60.0, 1200.0, -10.1)),
            ("nan density", (math.nan, 60.0, 1200.0, 0.0)),
            ("infinite speed", (BASELINE_DENSITY_KGPM3, math.inf, 1200.0, 0.0)),
            ("vector mass", (BASELINE_DENSITY_KGPM3, 60.0, [1200.0], 0.0)),
            ("complex path", (BASELINE_DENSITY_KGPM3, 60.0, 1200.0, 1.0j)),
            ("text density", ("thin", 60.0, 1200.0, 0.0)),
            ("boolean speed", (BASELINE_DENSITY_KGPM3, True, 1200.0, 0.0)),
        )
        for name, values in malformed_cases:
            with self.subTest(case=name), self.assertRaises(ValueError):
                _oracle(*values)
        self.assertEqual(_oracle(), baseline)

    def test_all_accepted_input_boundaries_remain_finite(self) -> None:
        numeric_keys = (
            "weight_n",
            "dynamic_pressure_pa",
            "normal_force_required_n",
            "weight_along_path_n",
            "lift_coefficient",
            "angle_of_attack_deg",
            "drag_coefficient",
            "parasite_drag_n",
            "induced_drag_n",
            "drag_n",
            "thrust_required_n",
            "required_thrust_fraction",
            "force_residual_magnitude_n",
            "stall_speed_mps",
            "minimum_drag_speed_mps",
        )
        case_count = 0
        for density in (0.04, 2.0):
            for speed in (20.0, 200.0):
                for mass in (400.0, 3000.0):
                    for path_angle in (-10.0, 10.0):
                        result = _oracle(density, speed, mass, path_angle)
                        case_count += 1
                        for key in numeric_keys:
                            self.assertTrue(
                                math.isfinite(result[key]),
                                (density, speed, mass, path_angle, key),
                            )
        self.assertEqual(case_count, 16)

    def test_representative_grid_is_finite_and_resource_bounded(self) -> None:
        densities = (0.04, 0.1, 0.5, 1.0, 2.0)
        speeds = tuple(float(value) for value in range(20, 201, 20))
        masses = (400.0, 800.0, 1200.0, 2000.0, 3000.0)
        path_angles = (-10.0, -5.0, 0.0, 5.0, 10.0)
        case_count = 0
        for density in densities:
            for speed in speeds:
                for mass in masses:
                    for path_angle in path_angles:
                        result = _oracle(density, speed, mass, path_angle)
                        case_count += 1
                        self.assertTrue(math.isfinite(result["lift_coefficient"]))
                        self.assertTrue(math.isfinite(result["drag_n"]))
                        self.assertTrue(math.isfinite(result["thrust_required_n"]))
                        self.assertLess(result["force_residual_magnitude_n"], 1e-8)
        self.assertEqual(case_count, 1250)
        self.assertLessEqual(case_count, 1300)


if __name__ == "__main__":
    unittest.main()
