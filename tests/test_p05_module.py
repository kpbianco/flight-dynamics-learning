from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULE_ID = "P05"
MODULE_FOLDER = ROOT / "modules/05-see-longitudinal-static-stability"
GUIDING_QUESTION = (
    "What inputs, observable effects, and failure modes matter when you see "
    "Longitudinal Static Stability?"
)


def _finite_real_scalar(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite real scalar")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite real scalar")
    return result


def _oracle(
    cg_position_percent_mac: object = 30.0,
    tail_area_ratio_percent: object = 20.0,
    angle_of_attack_perturbation_deg: object = 2.0,
    elevator_perturbation_deg: object = 0.0,
) -> dict[str, object]:
    """Independent Python oracle; it does not execute or translate MATLAB."""
    cg_percent = _finite_real_scalar("CG position", cg_position_percent_mac)
    tail_percent = _finite_real_scalar("tail-area ratio", tail_area_ratio_percent)
    alpha_deg = _finite_real_scalar(
        "angle-of-attack perturbation", angle_of_attack_perturbation_deg
    )
    elevator_deg = _finite_real_scalar(
        "elevator perturbation", elevator_perturbation_deg
    )
    if not 15.0 <= cg_percent <= 65.0:
        raise ValueError("CG position outside the learning range")
    if tail_percent < 0.0:
        raise ValueError("tail-area ratio must be nonnegative")
    if tail_percent > 30.0:
        raise ValueError("tail-area ratio outside the learning range")
    if abs(alpha_deg) > 5.0:
        raise ValueError("angle-of-attack perturbation outside the learning range")
    if abs(elevator_deg) > 15.0:
        raise ValueError("elevator perturbation outside the learning range")

    reference_dynamic_pressure_pa = 1325.00798531847
    reference_trim_angle_of_attack_deg = 3.41754527929804
    wing_area_m2 = 16.2
    mean_aerodynamic_chord_m = 1.5
    wing_aerodynamic_center = 0.25
    tail_aerodynamic_center_x_over_mac = 3.5
    wing_lift_curve_slope_per_rad = 5.0
    tail_lift_curve_slope_per_rad = 4.0
    tail_dynamic_pressure_ratio = 0.9
    downwash_gradient = 0.4
    elevator_effectiveness = 0.6
    stability_tolerance_per_rad = 1e-12

    cg = cg_percent / 100.0
    tail_ratio = tail_percent / 100.0
    alpha_rad = math.radians(alpha_deg)
    elevator_rad = math.radians(elevator_deg)
    tail_lift_contribution = (
        tail_dynamic_pressure_ratio
        * tail_ratio
        * tail_lift_curve_slope_per_rad
        * (1.0 - downwash_gradient)
    )
    aircraft_lift_curve_slope = wing_lift_curve_slope_per_rad + tail_lift_contribution
    neutral_point = (
        wing_lift_curve_slope_per_rad * wing_aerodynamic_center
        + tail_lift_contribution * tail_aerodynamic_center_x_over_mac
    ) / aircraft_lift_curve_slope
    static_margin = neutral_point - cg
    wing_moment_slope = wing_lift_curve_slope_per_rad * (
        cg - wing_aerodynamic_center
    )
    tail_moment_slope = tail_lift_contribution * (
        cg - tail_aerodynamic_center_x_over_mac
    )
    pitching_moment_slope = wing_moment_slope + tail_moment_slope
    neutral_point_moment_slope = -aircraft_lift_curve_slope * static_margin
    elevator_derivative = (
        -tail_dynamic_pressure_ratio
        * tail_ratio
        * tail_lift_curve_slope_per_rad
        * elevator_effectiveness
        * (tail_aerodynamic_center_x_over_mac - cg)
    )
    alpha_moment_coefficient = pitching_moment_slope * alpha_rad
    elevator_moment_coefficient = elevator_derivative * elevator_rad
    pitching_moment_coefficient = (
        alpha_moment_coefficient + elevator_moment_coefficient
    )
    moment_scale_nm = (
        reference_dynamic_pressure_pa * wing_area_m2 * mean_aerodynamic_chord_m
    )

    return {
        "cg_position_percent_mac": cg_percent,
        "cg_position": cg,
        "tail_area_ratio_percent": tail_percent,
        "tail_area_ratio": tail_ratio,
        "angle_of_attack_perturbation_deg": alpha_deg,
        "angle_of_attack_perturbation_rad": alpha_rad,
        "elevator_perturbation_deg": elevator_deg,
        "elevator_perturbation_rad": elevator_rad,
        "absolute_angle_of_attack_deg": reference_trim_angle_of_attack_deg + alpha_deg,
        "tail_lift_contribution_per_rad": tail_lift_contribution,
        "aircraft_lift_curve_slope_per_rad": aircraft_lift_curve_slope,
        "neutral_point": neutral_point,
        "neutral_point_percent_mac": 100.0 * neutral_point,
        "static_margin": static_margin,
        "static_margin_percent_mac": 100.0 * static_margin,
        "wing_moment_slope_per_rad": wing_moment_slope,
        "tail_moment_slope_per_rad": tail_moment_slope,
        "pitching_moment_slope_per_rad": pitching_moment_slope,
        "neutral_point_moment_slope_per_rad": neutral_point_moment_slope,
        "elevator_control_derivative_per_rad": elevator_derivative,
        "alpha_moment_coefficient": alpha_moment_coefficient,
        "elevator_moment_coefficient": elevator_moment_coefficient,
        "pitching_moment_coefficient": pitching_moment_coefficient,
        "moment_scale_nm": moment_scale_nm,
        "alpha_moment_nm": moment_scale_nm * alpha_moment_coefficient,
        "elevator_moment_nm": moment_scale_nm * elevator_moment_coefficient,
        "pitching_moment_nm": moment_scale_nm * pitching_moment_coefficient,
        "is_statically_stable": pitching_moment_slope < -stability_tolerance_per_rad,
        "is_neutral": abs(pitching_moment_slope) <= stability_tolerance_per_rad,
        "is_statically_unstable": pitching_moment_slope > stability_tolerance_per_rad,
        "reference_dynamic_pressure_pa": reference_dynamic_pressure_pa,
        "reference_trim_angle_of_attack_deg": reference_trim_angle_of_attack_deg,
        "wing_area_m2": wing_area_m2,
        "mean_aerodynamic_chord_m": mean_aerodynamic_chord_m,
        "wing_aerodynamic_center": wing_aerodynamic_center,
        "tail_aerodynamic_center_x_over_mac": tail_aerodynamic_center_x_over_mac,
        "wing_lift_curve_slope_per_rad": wing_lift_curve_slope_per_rad,
        "tail_lift_curve_slope_per_rad": tail_lift_curve_slope_per_rad,
        "tail_dynamic_pressure_ratio": tail_dynamic_pressure_ratio,
        "downwash_gradient": downwash_gradient,
        "elevator_effectiveness": elevator_effectiveness,
        "stability_tolerance_per_rad": stability_tolerance_per_rad,
    }


class P05ArtifactTests(unittest.TestCase):
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
                "number": 5,
                "id": "P05",
                "title": "See Longitudinal Static Stability",
                "guiding_question": GUIDING_QUESTION,
                "phase": 2,
                "phase_title": "Stability and modes",
                "slug": "see-longitudinal-static-stability",
                "folder": "modules/05-see-longitudinal-static-stability",
                "implementation_batch": "P05",
                "prerequisites": ["P04"],
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

        self.assertIn("p04", combined)
        self.assertIn("dynamic pressure", combined)
        self.assertRegex(combined, r"locally retrimmed|local reference")
        self.assertIn("static margin", combined)
        self.assertIn("neutral point", combined)
        self.assertIn("nose-up", combined)
        self.assertIn("nose-down", combined)
        self.assertIn("stick-fixed", combined)
        self.assertIn("read", walkthrough.lower())
        self.assertIn("baseline", walkthrough.lower())
        self.assertRegex(walkthrough.lower(), r"one lever|move cg alone")
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
            "functionout=model(cgposition_percentmac,tailarearatio_percent,"
            "angleofattackperturbation_deg,elevatorperturbation_deg)",
            compact,
        )
        self.assertIn("arguments", model.lower())
        self.assertEqual(
            model.count("(1,1) double {mustBeReal,mustBeFinite}"), 4
        )
        for expression in (
            "cgposition_percentmac<15||cgposition_percentmac>65",
            "tailarearatio_percent<0",
            "tailarearatio_percent>30",
            "abs(angleofattackperturbation_deg)>5",
            "abs(elevatorperturbation_deg)>15",
        ):
            self.assertIn(expression, compact)
        for identifier in (
            "P05:model:CgRange",
            "P05:model:NegativeTailAreaRatio",
            "P05:model:TailAreaRatioRange",
            "P05:model:AlphaPerturbationRange",
            "P05:model:ElevatorPerturbationRange",
        ):
            self.assertIn(identifier, model)

        for formula in (
            "cgposition_fractionmac=cgposition_percentmac/100;",
            "tailarearatio=tailarearatio_percent/100;",
            "tailliftcontribution_perrad=taildynamicpressureratio*tailarearatio*"
            "tailliftcurveslope_perrad*(1-downwashgradient);",
            "aircraftliftcurveslope_perrad=wingliftcurveslope_perrad+"
            "tailliftcontribution_perrad;",
            "staticmargin_fractionmac=(neutralpoint_percentmac-"
            "cgposition_percentmac)/100;",
            "wingmomentslope_perrad=wingliftcurveslope_perrad*("
            "cgposition_fractionmac-wingaerodynamiccenter_fractionmac);",
            "tailmomentslope_perrad=tailliftcontribution_perrad*("
            "cgposition_fractionmac-tailaerodynamiccenter_xovermac);",
            "pitchingmomentslope_perrad=wingmomentslope_perrad+tailmomentslope_perrad;",
            "neutralpointmomentslope_perrad=-aircraftliftcurveslope_perrad*"
            "staticmargin_fractionmac;",
            "absoluteangleofattack_deg=referencetrimangleofattack_deg+"
            "angleofattackperturbation_deg;",
            "pitchingmomentcoefficient=alphamomentcoefficient+elevatormomentcoefficient;",
            "momentscale_nm=referencedynamicpressure_pa*wingarea_m2*"
            "meanaerodynamicchord_m;",
        ):
            self.assertIn(formula, compact)
        self.assertIn("tailAerodynamicCenter_xOverMAC=3.5", model)
        self.assertIn("stabilityTolerance_perRad=1e-12", model)
        self.assertIn("positive C_m and pitching moment are nose-up", model)

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
        self.assertRegex(lower, r"sweep[^\n]*cg|cg[^\n]*sweep")
        self.assertRegex(lower, r"sweep[^\n]*tail|tail[^\n]*sweep")
        self.assertIn("broken", lower)
        self.assertIn("reverse the static-margin sign", lower)
        self.assertIn("brokenstaticmargin_fractionmac", lower)
        self.assertGreaterEqual(lower.count("figure("), 4)
        self.assertIn("xlabel", lower)
        self.assertIn("ylabel", lower)
        for unit in ("% mac", "1/rad", "n*m", "deg", "pa"):
            with self.subTest(unit=unit):
                self.assertIn(unit, lower)
        self.assertIn("fprintf", lower)
        self.assertIn("assert(", lower)
        self.assertIn("model(cgsweep_percentmac(k),20,2,0)", compact)
        self.assertIn("model(30,tailareasweep_percent(k),2,0)", compact)
        self.assertNotRegex(lower, r"\bclose\s+all\b")
        self.assertIn("findall(groot", lower)
        self.assertIn("'^p[0-9][0-9] '", lower)
        self.assertRegex(lower, r"clear run_checks;\s*run_checks;")

        for variable in ("cgSweep_percentMAC", "tailAreaSweep_percent"):
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

        self.assertIn("clear model;", "\n".join(experiment.splitlines()[:12]).lower())
        self.assertIn("clear model;", "\n".join(interactive_lower.splitlines()[:5]))
        self.assertIn("clear model;", "\n".join(checks_lower.splitlines()[:5]))
        self.assertIn("uifigure(", interactive_lower)
        self.assertIn(
            "existingui=findall(groot,'type','figure','name',"
            "'p05longitudinalstaticstability')",
            interactive_compact,
        )
        self.assertIn("close(existingui)", interactive_compact)
        self.assertGreaterEqual(interactive_lower.count("uislider("), 4)
        for control in ("cg", "tail", "alpha", "elevator"):
            self.assertIn(control, interactive_lower)
        self.assertIn("valuechangingfcn", interactive_lower)
        self.assertIn("valuechangedfcn", interactive_lower)
        self.assertIn("event.value", interactive_lower)
        self.assertIn("modelfcn=@model;", interactive_compact)
        self.assertIn("out=modelfcn(", interactive_compact)
        self.assertIn("alphagrid_deg=-5:0.5:5", interactive_compact)
        self.assertIn("cggrid_percentmac=15:2:65", interactive_compact)
        self.assertIn("forindex=1:numel(alphagrid_deg)", interactive_compact)
        self.assertIn("forindex=1:numel(cggrid_percentmac)", interactive_compact)
        self.assertNotIn("yyaxis", interactive_lower)
        self.assertIn("absolute alpha", interactive_lower)
        self.assertNotIn("absolute reference alpha", interactive_lower)
        for unit in ("% mac", "1/rad", "n*m", "deg"):
            self.assertIn(unit, interactive_lower)

        self.assertGreaterEqual(checks_lower.count("assert("), 28)
        for concept in (
            "determin",
            "component and neutral-point",
            "alpha symmetry",
            "stability limiting cases",
            "elevator changes",
            "sweep regressions",
            "broken static-margin",
            "cgrange",
            "tailarearatiorange",
            "alphaperturbationrange",
            "elevatorperturbationrange",
            "rejected inputs",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept.replace("-", ""), checks_lower.replace("-", ""))
        self.assertIn("edgecasecount==16", checks_compact.lower())
        self.assertIn("catch exception", checks_lower)
        self.assertIn(
            "strcmp(exception.identifier,expectedIdentifier)", checks_compact
        )
        self.assertIn("P05 checks passed", checks_script)

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
            "ss",
            "tf",
            "damp",
            "pole",
            "ode45",
            "roots",
            "polyfit",
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


class P05IndependentOracleTests(unittest.TestCase):
    def test_baseline_is_deterministic_and_physically_interpretable(self) -> None:
        first = _oracle()
        second = _oracle()
        self.assertEqual(first, second)
        self.assertAlmostEqual(first["tail_lift_contribution_per_rad"], 0.432, places=14)
        self.assertAlmostEqual(first["aircraft_lift_curve_slope_per_rad"], 5.432, places=14)
        self.assertAlmostEqual(first["neutral_point"], 0.5084683357879235, places=14)
        self.assertAlmostEqual(first["static_margin"], 0.2084683357879235, places=14)
        self.assertAlmostEqual(first["pitching_moment_slope_per_rad"], -1.1324, places=14)
        self.assertAlmostEqual(
            first["elevator_control_derivative_per_rad"], -1.3824, places=14
        )
        self.assertAlmostEqual(first["pitching_moment_coefficient"], -0.03952821689916759, places=14)
        self.assertAlmostEqual(first["pitching_moment_nm"], -1272.7174337941804, places=9)
        self.assertTrue(first["is_statically_stable"])
        self.assertLess(first["alpha_moment_nm"], 0.0)

    def test_component_neutral_point_and_dimensional_identities(self) -> None:
        result = _oracle(37.0, 12.0, 1.7, -3.0)
        tail_contribution = (
            result["tail_dynamic_pressure_ratio"]
            * result["tail_area_ratio"]
            * result["tail_lift_curve_slope_per_rad"]
            * (1.0 - result["downwash_gradient"])
        )
        total_lift_slope = result["wing_lift_curve_slope_per_rad"] + tail_contribution
        neutral_point = (
            result["wing_lift_curve_slope_per_rad"]
            * result["wing_aerodynamic_center"]
            + tail_contribution * result["tail_aerodynamic_center_x_over_mac"]
        ) / total_lift_slope
        component_slope = result["wing_lift_curve_slope_per_rad"] * (
            result["cg_position"] - result["wing_aerodynamic_center"]
        ) + tail_contribution * (
            result["cg_position"] - result["tail_aerodynamic_center_x_over_mac"]
        )
        neutral_slope = -total_lift_slope * (neutral_point - result["cg_position"])
        self.assertAlmostEqual(component_slope, neutral_slope, places=14)
        self.assertAlmostEqual(result["pitching_moment_slope_per_rad"], component_slope, places=14)
        self.assertAlmostEqual(
            result["neutral_point_moment_slope_per_rad"], neutral_slope, places=14
        )
        self.assertAlmostEqual(
            result["pitching_moment_nm"],
            result["reference_dynamic_pressure_pa"]
            * result["wing_area_m2"]
            * result["mean_aerodynamic_chord_m"]
            * result["pitching_moment_coefficient"],
            places=10,
        )

    def test_local_reference_and_alpha_symmetry(self) -> None:
        for cg_percent, tail_percent in ((15.0, 0.0), (30.0, 20.0), (65.0, 30.0)):
            reference = _oracle(cg_percent, tail_percent, 0.0, 0.0)
            self.assertEqual(reference["pitching_moment_coefficient"], 0.0)
            self.assertEqual(reference["pitching_moment_nm"], 0.0)

        positive = _oracle(30.0, 20.0, 2.0, 0.0)
        negative = _oracle(30.0, 20.0, -2.0, 0.0)
        self.assertAlmostEqual(
            positive["alpha_moment_coefficient"],
            -negative["alpha_moment_coefficient"],
            places=14,
        )
        self.assertAlmostEqual(positive["alpha_moment_nm"], -negative["alpha_moment_nm"], places=10)

    def test_p04_reference_alpha_stays_fixed_while_absolute_alpha_tracks_perturbation(
        self,
    ) -> None:
        expected_reference_deg = 3.41754527929804
        cases = (
            _oracle(30.0, 20.0, 0.0, 0.0),
            _oracle(30.0, 20.0, 2.0, 0.0),
            _oracle(30.0, 20.0, -2.0, 0.0),
            _oracle(45.0, 5.0, 2.0, 0.0),
        )
        for result in cases:
            with self.subTest(
                cg=result["cg_position_percent_mac"],
                tail=result["tail_area_ratio_percent"],
                delta_alpha=result["angle_of_attack_perturbation_deg"],
            ):
                self.assertEqual(
                    result["reference_trim_angle_of_attack_deg"],
                    expected_reference_deg,
                )
                self.assertAlmostEqual(
                    result["absolute_angle_of_attack_deg"],
                    expected_reference_deg
                    + result["angle_of_attack_perturbation_deg"],
                    places=14,
                )

    def test_neutral_no_tail_and_unstable_limiting_cases(self) -> None:
        baseline = _oracle()
        neutral_cg = _oracle(baseline["neutral_point_percent_mac"], 20.0, 2.0, 0.0)
        self.assertTrue(neutral_cg["is_neutral"])
        self.assertLessEqual(
            abs(neutral_cg["pitching_moment_slope_per_rad"]),
            neutral_cg["stability_tolerance_per_rad"],
        )
        self.assertLess(abs(neutral_cg["alpha_moment_nm"]), 1e-8)

        aft = _oracle(56.0, 20.0, 2.0, 0.0)
        self.assertTrue(aft["is_statically_unstable"])
        self.assertAlmostEqual(aft["pitching_moment_slope_per_rad"], 0.27992, places=14)
        self.assertAlmostEqual(aft["pitching_moment_nm"], 314.6053197347816, places=9)

        no_tail = _oracle(30.0, 0.0, 2.0, 5.0)
        self.assertEqual(no_tail["neutral_point_percent_mac"], 25.0)
        self.assertAlmostEqual(no_tail["pitching_moment_slope_per_rad"], 0.25)
        self.assertEqual(no_tail["elevator_control_derivative_per_rad"], 0.0)
        self.assertAlmostEqual(no_tail["pitching_moment_nm"], 280.97788630214137, places=9)

        wing_only_neutral = _oracle(25.0, 0.0, 2.0, 5.0)
        self.assertTrue(wing_only_neutral["is_neutral"])
        self.assertEqual(wing_only_neutral["pitching_moment_slope_per_rad"], 0.0)
        self.assertEqual(wing_only_neutral["pitching_moment_nm"], 0.0)

        neutral_tail_percent = 100.0 * 5.0 * (0.30 - 0.25) / (
            0.9 * 4.0 * (1.0 - 0.4) * (3.5 - 0.30)
        )
        neutral_tail = _oracle(30.0, neutral_tail_percent, 2.0, 0.0)
        self.assertAlmostEqual(neutral_tail_percent, 3.616898148148147, places=14)
        self.assertTrue(neutral_tail["is_neutral"])
        self.assertLess(abs(neutral_tail["alpha_moment_nm"]), 1e-8)

    def test_elevator_changes_intercept_not_stability_slope(self) -> None:
        down = _oracle(30.0, 20.0, 0.0, 5.0)
        up = _oracle(30.0, 20.0, 0.0, -5.0)
        self.assertLess(down["elevator_moment_nm"], 0.0)
        self.assertGreater(up["elevator_moment_nm"], 0.0)
        self.assertAlmostEqual(down["elevator_moment_nm"], -up["elevator_moment_nm"], places=10)
        self.assertEqual(
            down["pitching_moment_slope_per_rad"], up["pitching_moment_slope_per_rad"]
        )
        self.assertEqual(down["static_margin"], up["static_margin"])

    def test_two_sweeps_isolate_inputs_and_cross_neutral(self) -> None:
        cg_values = (20.0, 28.0, 36.0, 44.0, 50.8468335787924, 56.0)
        cg_results = [_oracle(cg, 20.0, 2.0, 0.0) for cg in cg_values]
        self.assertEqual(
            [result["tail_area_ratio_percent"] for result in cg_results],
            [20.0] * len(cg_results),
        )
        neutral_points = [result["neutral_point"] for result in cg_results]
        self.assertEqual(neutral_points, [neutral_points[0]] * len(neutral_points))
        for key in ("static_margin", "pitching_moment_slope_per_rad", "pitching_moment_nm"):
            values = [result[key] for result in cg_results]
            direction = -1.0 if key == "static_margin" else 1.0
            self.assertTrue(
                all(direction * left < direction * right for left, right in zip(values, values[1:])),
                key,
            )
        self.assertTrue(cg_results[0]["is_statically_stable"])
        self.assertTrue(cg_results[-1]["is_statically_unstable"])

        tail_values = (0.0, 3.61689814814815, 5.0, 10.0, 15.0, 20.0, 25.0)
        tail_results = [_oracle(30.0, tail, 2.0, 0.0) for tail in tail_values]
        self.assertEqual(
            [result["cg_position_percent_mac"] for result in tail_results],
            [30.0] * len(tail_results),
        )
        for key in ("neutral_point", "static_margin"):
            values = [result[key] for result in tail_results]
            self.assertTrue(all(left < right for left, right in zip(values, values[1:])), key)
        for key in ("pitching_moment_slope_per_rad", "pitching_moment_nm"):
            values = [result[key] for result in tail_results]
            self.assertTrue(all(left > right for left, right in zip(values, values[1:])), key)
        self.assertTrue(tail_results[0]["is_statically_unstable"])
        self.assertTrue(tail_results[-1]["is_statically_stable"])

    def test_broken_static_margin_reverses_restoring_moment(self) -> None:
        baseline = _oracle()
        broken_static_margin = (
            baseline["cg_position_percent_mac"] - baseline["neutral_point_percent_mac"]
        ) / 100.0
        broken_slope = -baseline["aircraft_lift_curve_slope_per_rad"] * broken_static_margin
        broken_coefficient = broken_slope * baseline["angle_of_attack_perturbation_rad"]
        broken_moment_nm = baseline["moment_scale_nm"] * broken_coefficient
        self.assertAlmostEqual(
            broken_slope, -baseline["pitching_moment_slope_per_rad"], places=14
        )
        self.assertAlmostEqual(broken_moment_nm, -baseline["alpha_moment_nm"], places=10)
        self.assertLess(baseline["alpha_moment_nm"], 0.0)
        self.assertGreater(broken_moment_nm, 0.0)

    def test_malformed_inputs_fail_without_poisoning_recovery(self) -> None:
        baseline = _oracle()
        malformed_cases = (
            ("CG below range", (14.9, 20.0, 2.0, 0.0)),
            ("CG above range", (65.1, 20.0, 2.0, 0.0)),
            ("negative tail", (30.0, -0.1, 2.0, 0.0)),
            ("tail above range", (30.0, 30.1, 2.0, 0.0)),
            ("alpha below range", (30.0, 20.0, -5.1, 0.0)),
            ("alpha above range", (30.0, 20.0, 5.1, 0.0)),
            ("elevator below range", (30.0, 20.0, 2.0, -15.1)),
            ("elevator above range", (30.0, 20.0, 2.0, 15.1)),
            ("nan CG", (math.nan, 20.0, 2.0, 0.0)),
            ("infinite tail", (30.0, math.inf, 2.0, 0.0)),
            ("vector alpha", (30.0, 20.0, [2.0], 0.0)),
            ("complex elevator", (30.0, 20.0, 2.0, 1.0j)),
            ("text CG", ("forward", 20.0, 2.0, 0.0)),
            ("boolean tail", (30.0, True, 2.0, 0.0)),
        )
        for name, values in malformed_cases:
            with self.subTest(case=name), self.assertRaises(ValueError):
                _oracle(*values)
        self.assertEqual(_oracle(), baseline)

    def test_all_accepted_input_boundaries_remain_finite(self) -> None:
        numeric_keys = (
            "tail_lift_contribution_per_rad",
            "aircraft_lift_curve_slope_per_rad",
            "neutral_point",
            "static_margin",
            "pitching_moment_slope_per_rad",
            "elevator_control_derivative_per_rad",
            "pitching_moment_coefficient",
            "pitching_moment_nm",
        )
        case_count = 0
        for cg in (15.0, 65.0):
            for tail in (0.0, 30.0):
                for alpha in (-5.0, 5.0):
                    for elevator in (-15.0, 15.0):
                        result = _oracle(cg, tail, alpha, elevator)
                        case_count += 1
                        for key in numeric_keys:
                            self.assertTrue(math.isfinite(result[key]), (cg, tail, alpha, elevator, key))
        self.assertEqual(case_count, 16)

    def test_representative_grid_is_finite_and_resource_bounded(self) -> None:
        cgs = (15.0, 27.5, 40.0, 52.5, 65.0)
        tails = (0.0, 7.5, 15.0, 22.5, 30.0)
        alphas = (-5.0, -2.5, 0.0, 2.5, 5.0)
        elevators = (-15.0, -7.5, 0.0, 7.5, 15.0)
        case_count = 0
        for cg in cgs:
            for tail in tails:
                for alpha in alphas:
                    for elevator in elevators:
                        result = _oracle(cg, tail, alpha, elevator)
                        case_count += 1
                        for key in (
                            "neutral_point",
                            "static_margin",
                            "pitching_moment_slope_per_rad",
                            "pitching_moment_coefficient",
                            "pitching_moment_nm",
                        ):
                            self.assertTrue(
                                math.isfinite(result[key]), (cg, tail, alpha, elevator, key)
                            )
        self.assertEqual(case_count, 625)
        self.assertLessEqual(case_count, 700)


if __name__ == "__main__":
    unittest.main()
