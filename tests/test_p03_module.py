from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULE_ID = "P03"
MODULE_FOLDER = ROOT / "modules/03-build-an-atmosphere-model"
GUIDING_QUESTION = (
    "What inputs, observable effects, and failure modes matter when you build "
    "an Atmosphere Model?"
)


def _finite_real_scalar(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite real scalar")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite real scalar")
    return result


def _oracle(
    pressure_altitude_m: object = 5000.0,
    temperature_offset_k: object = 0.0,
    true_airspeed_mps: object = 150.0,
) -> dict[str, object]:
    """Independent Python oracle; it does not execute or translate MATLAB."""
    altitude = _finite_real_scalar("pressure altitude", pressure_altitude_m)
    offset = _finite_real_scalar("temperature offset", temperature_offset_k)
    airspeed = _finite_real_scalar("true airspeed", true_airspeed_mps)
    if not 0.0 <= altitude <= 20000.0:
        raise ValueError("pressure altitude outside 0 to 20 km")
    if not -100.0 <= offset <= 100.0:
        raise ValueError("temperature offset outside -100 to 100 K")
    if airspeed < 0.0:
        raise ValueError("true airspeed must be nonnegative")
    if airspeed > 1000.0:
        raise ValueError("true airspeed above learning-model range")

    sea_level_temperature_k = 288.15
    sea_level_pressure_pa = 101325.0
    gravity_mps2 = 9.80665
    gas_constant_jpkgk = 287.05287
    heat_capacity_ratio = 1.4
    lapse_rate_kpm = -0.0065
    tropopause_altitude_m = 11000.0
    tropopause_temperature_k = (
        sea_level_temperature_k + lapse_rate_kpm * tropopause_altitude_m
    )
    tropopause_pressure_pa = sea_level_pressure_pa * (
        tropopause_temperature_k / sea_level_temperature_k
    ) ** (-gravity_mps2 / (lapse_rate_kpm * gas_constant_jpkgk))

    if altitude <= tropopause_altitude_m:
        standard_temperature_k = sea_level_temperature_k + lapse_rate_kpm * altitude
        pressure_pa = sea_level_pressure_pa * (
            standard_temperature_k / sea_level_temperature_k
        ) ** (-gravity_mps2 / (lapse_rate_kpm * gas_constant_jpkgk))
        layer = "gradient troposphere"
    else:
        standard_temperature_k = tropopause_temperature_k
        pressure_pa = tropopause_pressure_pa * math.exp(
            -gravity_mps2
            * (altitude - tropopause_altitude_m)
            / (gas_constant_jpkgk * tropopause_temperature_k)
        )
        layer = "isothermal lower stratosphere"

    temperature_k = standard_temperature_k + offset
    if temperature_k <= 0.0:
        raise ValueError("absolute temperature must be positive")
    density_kgpm3 = pressure_pa / (gas_constant_jpkgk * temperature_k)
    speed_of_sound_mps = math.sqrt(
        heat_capacity_ratio * gas_constant_jpkgk * temperature_k
    )
    mach = airspeed / speed_of_sound_mps
    dynamic_pressure_pa = 0.5 * density_kgpm3 * airspeed**2
    sea_level_density_kgpm3 = sea_level_pressure_pa / (
        gas_constant_jpkgk * sea_level_temperature_k
    )
    equivalent_airspeed_mps = airspeed * math.sqrt(
        density_kgpm3 / sea_level_density_kgpm3
    )

    return {
        "pressure_altitude_m": altitude,
        "temperature_offset_k": offset,
        "true_airspeed_mps": airspeed,
        "standard_temperature_k": standard_temperature_k,
        "temperature_k": temperature_k,
        "pressure_pa": pressure_pa,
        "density_kgpm3": density_kgpm3,
        "speed_of_sound_mps": speed_of_sound_mps,
        "mach": mach,
        "dynamic_pressure_pa": dynamic_pressure_pa,
        "equivalent_airspeed_mps": equivalent_airspeed_mps,
        "sea_level_density_kgpm3": sea_level_density_kgpm3,
        "gas_constant_jpkgk": gas_constant_jpkgk,
        "heat_capacity_ratio": heat_capacity_ratio,
        "layer": layer,
    }


class P03ArtifactTests(unittest.TestCase):
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
                "number": 3,
                "id": "P03",
                "title": "Build an Atmosphere Model",
                "guiding_question": GUIDING_QUESTION,
                "phase": 1,
                "phase_title": "Point-mass flight",
                "slug": "build-an-atmosphere-model",
                "folder": "modules/03-build-an-atmosphere-model",
                "implementation_batch": "P03",
                "prerequisites": ["P02"],
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

        self.assertIn("p02", combined)
        self.assertIn("air-relative", combined)
        self.assertIn("pressure altitude", combined)
        self.assertIn("positive up", combined.replace("positive-up", "positive up"))
        self.assertIn("rho = p/(r t)", combined)
        self.assertIn("q = 0.5 rho v^2", combined)
        self.assertIn("read", walkthrough.lower())
        self.assertIn("baseline", walkthrough.lower())
        self.assertRegex(walkthrough.lower(), r"one lever|move altitude alone")
        self.assertRegex(walkthrough.lower(), r"changed|transition|observe")
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

    def test_model_is_transparent_layered_and_guarded(self) -> None:
        model = self.text["model.m"]
        compact = re.sub(r"\s+", "", model.replace("...", "")).lower()

        self.assertIn(
            "functionout=model(pressurealtitude_m,temperatureoffset_k,trueairspeed_mps)",
            compact,
        )
        self.assertIn("arguments", model.lower())
        self.assertEqual(compact.count("(1,1)double{mustbereal,mustbefinite}"), 3)
        self.assertIn("pressurealtitude_m<0||pressurealtitude_m>20000", compact)
        self.assertIn("temperatureoffset_k<-100||temperatureoffset_k>100", compact)
        self.assertIn("trueairspeed_mps<0", compact)
        self.assertIn("trueairspeed_mps>1000", compact)
        self.assertIn("temperature_k<=0", compact)
        for identifier in (
            "P03:model:AltitudeRange",
            "P03:model:TemperatureOffsetRange",
            "P03:model:NegativeAirspeed",
            "P03:model:AirspeedRange",
            "P03:model:AbsoluteTemperature",
        ):
            self.assertIn(identifier, model)

        self.assertIn("lapseRate_Kpm=-0.0065", model)
        self.assertIn("tropopauseAltitude_m=11000", model)
        self.assertIn("pressureAltitude_m<=tropopauseAltitude_m", model)
        self.assertIn("pressure_Pa=seaLevelPressure_Pa*", model)
        self.assertIn("pressure_Pa=tropopausePressure_Pa*exp(", model)
        self.assertIn("density_kgpm3=pressure_Pa/(gasConstant_JpkgK*temperature_K)", model)
        self.assertIn(
            "speedOfSound_mps=sqrt(heatCapacityRatio*gasConstant_JpkgK*temperature_K)",
            model,
        )
        self.assertIn("dynamicPressure_Pa=0.5*density_kgpm3*trueAirspeed_mps^2", model)
        self.assertIn(
            "equivalentairspeed_mps=trueairspeed_mps*sqrt("
            "density_kgpm3/sealeveldensity_kgpm3)",
            compact,
        )

        for presentation_call in (
            "figure(",
            "plot(",
            "uiaxes(",
            "uifigure(",
            "disp(",
            "fprintf(",
        ):
            self.assertNotIn(presentation_call, model.lower())
        self.assertNotRegex(model.lower(), r"\b(?:for|while|parfor)\b")

    def test_experiment_has_two_sweeps_metrics_and_density_failure(self) -> None:
        experiment = self.text["experiment.m"]
        lower = experiment.lower()
        self.assertGreaterEqual(experiment.count("%%"), 10)
        self.assertIn("baseline", lower)
        self.assertGreaterEqual(lower.count("sweep"), 2)
        self.assertRegex(lower, r"sweep[^\n]*altitude|altitude[^\n]*sweep")
        self.assertRegex(lower, r"sweep[^\n]*temperature|temperature[^\n]*sweep")
        self.assertIn("broken", lower)
        self.assertIn("sea-level density", lower)
        self.assertIn("constant-density", lower)
        self.assertRegex(lower, r"overpredict|wrong")
        self.assertGreaterEqual(lower.count("figure("), 4)
        self.assertIn("xlabel", lower)
        self.assertIn("ylabel", lower)
        for unit in ("m/s", "kg/m^3", "kpa", "km", "k"):
            with self.subTest(unit=unit):
                self.assertIn(unit, lower)
        self.assertIn("fprintf", lower)
        self.assertIn("assert(", lower)
        self.assertRegex(lower, r"model\(altitudesweep_m\(k\),0,150\)")
        self.assertRegex(lower, r"model\(5000,temperatureoffsetsweep_k\(k\),150\)")
        self.assertNotRegex(lower, r"\bclose\s+all\b")
        self.assertIn("findall(groot", lower)
        self.assertIn("'^p[0-9][0-9] '", lower)

        for variable in ("altitudeSweep_m", "temperatureOffsetSweep_K"):
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
        interactive_compact = re.sub(r"\s+", "", interactive.replace("...", "")).lower()
        checks_compact = re.sub(r"\s+", "", checks_script.replace("...", ""))

        self.assertIn("clear model;", "\n".join(experiment.splitlines()[:10]).lower())
        self.assertIn("clear model;", "\n".join(interactive_lower.splitlines()[:5]))
        self.assertIn("clear model;", "\n".join(checks_lower.splitlines()[:5]))
        self.assertRegex(experiment.lower(), r"clear run_checks;\s*run_checks;")

        self.assertIn("uifigure(", interactive_lower)
        self.assertGreaterEqual(interactive_lower.count("uislider("), 3)
        for control in ("altitude", "temperature", "airspeed"):
            self.assertIn(control, interactive_lower)
        self.assertIn("valuechangingfcn", interactive_lower)
        self.assertIn("valuechangedfcn", interactive_lower)
        self.assertIn("event.value", interactive_lower)
        self.assertIn("modelfcn=@model;", interactive_compact)
        self.assertIn("out=modelfcn(", interactive_compact)
        self.assertIn("altitudegrid_m=0:1000:20000", interactive_compact)
        self.assertIn("speedgrid_mps=0:25:350", interactive_compact)
        self.assertIn(
            "profilesample=modelfcn(altitudegrid_m(index),0,trueairspeed_mps)",
            interactive_compact,
        )
        self.assertIn("standard profile and selected local state", interactive_lower)
        for unit in ("m/s", "kg/m^3", "kpa", "km", "k"):
            self.assertIn(unit, interactive_lower)

        self.assertGreaterEqual(checks_lower.count("assert("), 20)
        for concept in (
            "determin",
            "sea-level",
            "zero true airspeed",
            "tropopause",
            "continuous",
            "hydrostatic exponent",
            "dynamic pressure",
            "mach",
            "altitude sweep",
            "temperature offset",
            "broken",
            "altituderange",
            "temperatureoffsetrange",
            "negativeairspeed",
            "airspeedrange",
            "rejected inputs",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept.replace("-", ""), checks_lower.replace("-", ""))
        self.assertIn("catch exception", checks_lower)
        self.assertIn("strcmp(exception.identifier,expectedIdentifier)", checks_compact)
        self.assertIn("P03 checks passed", checks_script)

    def test_no_opaque_toolbox_random_external_or_async_behavior(self) -> None:
        matlab = "\n".join(
            self.text[name]
            for name in ("model.m", "experiment.m", "interactive.m", "lesson.m", "run_checks.m")
        ).lower()
        forbidden_calls = (
            "atmosisa",
            "atmoscoesa",
            "atmosphere",
            "standardatmosphere",
            "isaatmosphere",
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
        )
        for call in forbidden_calls:
            with self.subTest(call=call):
                self.assertNotRegex(matlab, rf"\b{call}\s*\(")
        self.assertNotRegex(matlab, r"\brand(?:n|i)?\s*\(")
        self.assertNotRegex(matlab, r"\brng\s*\(")
        self.assertNotRegex(matlab, r"\b(?:load|save)\s*\(")
        self.assertNotRegex(matlab, re.compile(r"^\s*(?:while|parfor)\b", re.MULTILINE))
        self.assertNotRegex(matlab, r"\bclose\s+all\b")


class P03IndependentOracleTests(unittest.TestCase):
    def test_baseline_is_deterministic_and_physically_interpretable(self) -> None:
        first = _oracle()
        second = _oracle()
        self.assertEqual(first, second)
        self.assertAlmostEqual(first["temperature_k"], 255.65, places=12)
        self.assertAlmostEqual(first["pressure_pa"], 54019.8881881, places=6)
        self.assertAlmostEqual(first["density_kgpm3"], 0.7361155474, places=10)
        self.assertAlmostEqual(first["speed_of_sound_mps"], 320.5293944, places=7)
        self.assertAlmostEqual(first["mach"], 0.4679758007, places=10)
        self.assertAlmostEqual(first["dynamic_pressure_pa"], 8281.2999082, places=6)

    def test_sea_level_tropopause_and_twenty_km_limits(self) -> None:
        sea_level = _oracle(0.0, 0.0, 0.0)
        self.assertEqual(sea_level["temperature_k"], 288.15)
        self.assertEqual(sea_level["pressure_pa"], 101325.0)
        self.assertAlmostEqual(sea_level["density_kgpm3"], 1.2250000181, places=10)
        self.assertAlmostEqual(sea_level["speed_of_sound_mps"], 340.2939880, places=7)
        self.assertEqual(sea_level["mach"], 0.0)
        self.assertEqual(sea_level["dynamic_pressure_pa"], 0.0)
        self.assertEqual(sea_level["equivalent_airspeed_mps"], 0.0)

        tropopause = _oracle(11000.0)
        self.assertEqual(tropopause["layer"], "gradient troposphere")
        self.assertAlmostEqual(tropopause["temperature_k"], 216.65, places=12)
        self.assertAlmostEqual(tropopause["pressure_pa"], 22632.0400950, places=6)
        self.assertAlmostEqual(tropopause["density_kgpm3"], 0.3639176481, places=10)

        upper_bound = _oracle(20000.0)
        self.assertEqual(upper_bound["layer"], "isothermal lower stratosphere")
        self.assertAlmostEqual(upper_bound["temperature_k"], 216.65, places=12)
        self.assertAlmostEqual(upper_bound["pressure_pa"], 5474.8774243, places=6)
        self.assertAlmostEqual(upper_bound["density_kgpm3"], 0.0880346848, places=10)

    def test_layer_boundary_and_equation_invariants(self) -> None:
        below = _oracle(11000.0 - 1e-3)
        at_boundary = _oracle(11000.0)
        above = _oracle(11000.0 + 1e-3)
        self.assertLess(abs(below["pressure_pa"] - above["pressure_pa"]), 0.01)
        self.assertLess(abs(below["temperature_k"] - above["temperature_k"]), 1e-5)
        self.assertEqual(at_boundary["layer"], "gradient troposphere")
        self.assertEqual(above["layer"], "isothermal lower stratosphere")

        result = _oracle(7350.0, 12.0, 183.0)
        self.assertAlmostEqual(
            result["pressure_pa"],
            result["density_kgpm3"]
            * result["gas_constant_jpkgk"]
            * result["temperature_k"],
            places=9,
        )
        self.assertAlmostEqual(
            result["speed_of_sound_mps"] ** 2,
            result["heat_capacity_ratio"]
            * result["gas_constant_jpkgk"]
            * result["temperature_k"],
            places=9,
        )
        self.assertAlmostEqual(
            result["dynamic_pressure_pa"],
            0.5 * result["density_kgpm3"] * result["true_airspeed_mps"] ** 2,
            places=10,
        )

    def test_altitude_sweep_changes_only_the_intended_profile(self) -> None:
        altitudes = (0.0, 3000.0, 6000.0, 9000.0, 11000.0, 15000.0, 20000.0)
        results = [_oracle(altitude, 0.0, 150.0) for altitude in altitudes]
        pressures = [result["pressure_pa"] for result in results]
        densities = [result["density_kgpm3"] for result in results]
        dynamic_pressures = [result["dynamic_pressure_pa"] for result in results]
        temperatures = [result["temperature_k"] for result in results]
        mach = [result["mach"] for result in results]

        self.assertTrue(all(left > right for left, right in zip(pressures, pressures[1:])))
        self.assertTrue(all(left > right for left, right in zip(densities, densities[1:])))
        self.assertTrue(
            all(left > right for left, right in zip(dynamic_pressures, dynamic_pressures[1:]))
        )
        self.assertTrue(all(left > right for left, right in zip(temperatures[:4], temperatures[1:5])))
        self.assertTrue(all(abs(value - 216.65) < 1e-12 for value in temperatures[4:]))
        self.assertTrue(all(left < right for left, right in zip(mach[:4], mach[1:5])))
        self.assertEqual(mach[4], mach[5])
        self.assertEqual(mach[5], mach[6])

    def test_temperature_offset_sweep_has_fixed_pressure_and_expected_signs(self) -> None:
        offsets = (-30.0, -15.0, 0.0, 15.0, 30.0)
        results = [_oracle(5000.0, offset, 150.0) for offset in offsets]
        pressures = [result["pressure_pa"] for result in results]
        densities = [result["density_kgpm3"] for result in results]
        sounds = [result["speed_of_sound_mps"] for result in results]
        mach = [result["mach"] for result in results]
        dynamic_pressures = [result["dynamic_pressure_pa"] for result in results]

        self.assertEqual(pressures, [pressures[0]] * len(pressures))
        self.assertTrue(all(left > right for left, right in zip(densities, densities[1:])))
        self.assertTrue(all(left < right for left, right in zip(sounds, sounds[1:])))
        self.assertTrue(all(left > right for left, right in zip(mach, mach[1:])))
        self.assertTrue(
            all(left > right for left, right in zip(dynamic_pressures, dynamic_pressures[1:]))
        )

    def test_airspeed_scaling_and_broken_constant_density(self) -> None:
        slow = _oracle(5000.0, 0.0, 75.0)
        fast = _oracle(5000.0, 0.0, 150.0)
        self.assertAlmostEqual(fast["mach"] / slow["mach"], 2.0, places=14)
        self.assertAlmostEqual(
            fast["dynamic_pressure_pa"] / slow["dynamic_pressure_pa"], 4.0, places=14
        )

        sea_level = _oracle(0.0, 0.0, 150.0)
        at_eleven_km = _oracle(11000.0, 0.0, 150.0)
        broken_q = 0.5 * sea_level["density_kgpm3"] * 150.0**2
        self.assertGreater(broken_q / at_eleven_km["dynamic_pressure_pa"], 3.3)
        self.assertAlmostEqual(
            broken_q / at_eleven_km["dynamic_pressure_pa"], 3.3661462271, places=9
        )

    def test_equivalent_airspeed_preserves_dynamic_pressure_at_reference_density(self) -> None:
        sea_level = _oracle(0.0, 0.0, 150.0)
        altitude = _oracle(5000.0, 12.0, 183.0)

        self.assertEqual(
            sea_level["equivalent_airspeed_mps"], sea_level["true_airspeed_mps"]
        )
        equivalent_dynamic_pressure = (
            0.5
            * altitude["sea_level_density_kgpm3"]
            * altitude["equivalent_airspeed_mps"] ** 2
        )
        self.assertAlmostEqual(
            equivalent_dynamic_pressure, altitude["dynamic_pressure_pa"], places=9
        )
        self.assertLess(
            altitude["equivalent_airspeed_mps"], altitude["true_airspeed_mps"]
        )

    def test_malformed_inputs_fail_without_poisoning_recovery(self) -> None:
        baseline = _oracle()
        malformed_cases = (
            ("negative altitude", (-1.0, 0.0, 150.0)),
            ("above ceiling", (20000.1, 0.0, 150.0)),
            ("negative airspeed", (5000.0, 0.0, -1.0)),
            ("below offset range", (5000.0, -101.0, 150.0)),
            ("above offset range", (5000.0, 101.0, 150.0)),
            ("above speed range", (5000.0, 0.0, 1001.0)),
            ("nan altitude", (math.nan, 0.0, 150.0)),
            ("infinite offset", (5000.0, math.inf, 150.0)),
            ("vector speed", (5000.0, 0.0, [150.0])),
            ("complex altitude", (5000.0 + 1j, 0.0, 150.0)),
            ("text offset", (5000.0, "standard", 150.0)),
        )
        for name, values in malformed_cases:
            with self.subTest(case=name), self.assertRaises(ValueError):
                _oracle(*values)
        self.assertEqual(_oracle(), baseline)

    def test_all_accepted_input_boundaries_remain_finite(self) -> None:
        for altitude in (0.0, 20000.0):
            for offset in (-100.0, 100.0):
                for speed in (0.0, 1000.0):
                    result = _oracle(altitude, offset, speed)
                    for key in (
                        "temperature_k",
                        "pressure_pa",
                        "density_kgpm3",
                        "speed_of_sound_mps",
                        "mach",
                        "dynamic_pressure_pa",
                        "equivalent_airspeed_mps",
                    ):
                        self.assertTrue(math.isfinite(result[key]), (altitude, offset, speed, key))

    def test_representative_grid_is_finite_and_resource_bounded(self) -> None:
        altitudes = range(0, 20001, 1000)
        offsets = range(-40, 41, 10)
        speeds = (0.0, 50.0, 150.0, 350.0)
        case_count = 0
        for altitude in altitudes:
            for offset in offsets:
                for speed in speeds:
                    result = _oracle(float(altitude), float(offset), speed)
                    case_count += 1
                    for key in (
                        "temperature_k",
                        "pressure_pa",
                        "density_kgpm3",
                        "speed_of_sound_mps",
                        "mach",
                        "dynamic_pressure_pa",
                        "equivalent_airspeed_mps",
                    ):
                        self.assertTrue(math.isfinite(result[key]), (altitude, offset, speed, key))
                    self.assertGreater(result["temperature_k"], 0.0)
                    self.assertGreater(result["pressure_pa"], 0.0)
                    self.assertGreater(result["density_kgpm3"], 0.0)
        self.assertEqual(case_count, 756)
        self.assertLessEqual(case_count, 800)


if __name__ == "__main__":
    unittest.main()
