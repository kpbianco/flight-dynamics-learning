from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_ID = "P10"
MODULE_FOLDER = ROOT / "modules/10-model-actuator-dynamics-and-limits"
GUIDING_QUESTION = (
    "What inputs, observable effects, and failure modes matter when you model "
    "Actuator Dynamics and Limits?"
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


def _command_schedule(time_s: tuple[float, ...]) -> tuple[float, ...]:
    return tuple(
        0.0
        if time < 0.5
        else 25.0
        if time < 2.0
        else -25.0
        if time < 3.5
        else 5.0
        for time in time_s
    )


def _clip(value: float, lower: float, upper: float) -> float:
    return min(upper, max(lower, value))


def _simulate(
    command_deg: tuple[float, ...],
    time_s: tuple[float, ...],
    time_constant_s: float,
    rate_limit_deg_s: float,
    position_limit_deg: float,
    *,
    omit_position_limit: bool,
) -> dict[str, tuple[float, ...] | tuple[bool, ...]]:
    if omit_position_limit:
        limited_command = command_deg
    else:
        limited_command = tuple(
            _clip(value, -position_limit_deg, position_limit_deg)
            for value in command_deg
        )
    position_request_infeasible = tuple(
        abs(value) > position_limit_deg for value in command_deg
    )

    sample_count = len(time_s)
    deflection = [0.0] * sample_count
    lag_rate_demand = [0.0] * sample_count
    rate_limited_demand = [0.0] * sample_count
    actual_rate = [0.0] * sample_count
    rate_active = [False] * sample_count
    kinematic_closure_residual = [0.0] * sample_count

    for index in range(sample_count - 1):
        step_s = time_s[index + 1] - time_s[index]
        raw_rate = (
            limited_command[index] - deflection[index]
        ) / time_constant_s
        bounded_rate = _clip(
            raw_rate, -rate_limit_deg_s, rate_limit_deg_s
        )
        candidate = deflection[index] + step_s * bounded_rate
        next_deflection = (
            candidate
            if omit_position_limit
            else _clip(candidate, -position_limit_deg, position_limit_deg)
        )
        delivered_rate = (next_deflection - deflection[index]) / step_s

        lag_rate_demand[index] = raw_rate
        rate_limited_demand[index] = bounded_rate
        actual_rate[index] = delivered_rate
        rate_active[index] = abs(raw_rate) > rate_limit_deg_s
        kinematic_closure_residual[index] = (
            next_deflection
            - deflection[index]
            - step_s * delivered_rate
        )
        deflection[index + 1] = next_deflection

    final_raw_rate = (
        limited_command[-1] - deflection[-1]
    ) / time_constant_s
    lag_rate_demand[-1] = final_raw_rate
    rate_limited_demand[-1] = _clip(
        final_raw_rate, -rate_limit_deg_s, rate_limit_deg_s
    )
    actual_rate[-1] = rate_limited_demand[-1]
    rate_active[-1] = abs(final_raw_rate) > rate_limit_deg_s

    return {
        "limited_command_deg": limited_command,
        "deflection_deg": tuple(deflection),
        "lag_rate_demand_deg_s": tuple(lag_rate_demand),
        "rate_limited_demand_deg_s": tuple(rate_limited_demand),
        "actual_rate_deg_s": tuple(actual_rate),
        "position_request_infeasible": position_request_infeasible,
        "rate_limit_active": tuple(rate_active),
        "kinematic_closure_residual_deg": tuple(kinematic_closure_residual),
    }


def _rms(values: tuple[float, ...]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def _first_time(
    time_s: tuple[float, ...], predicate: tuple[bool, ...]
) -> float:
    return next(time for time, matched in zip(time_s, predicate) if matched)


def _oracle(
    time_constant_s: object = 0.18,
    rate_limit_deg_s: object = 45.0,
) -> dict[str, object]:
    """Pure-stdlib equation oracle independent of the MATLAB source."""
    time_constant = _bounded_scalar(
        "time constant", time_constant_s, 0.05, 0.50
    )
    rate_limit = _bounded_scalar(
        "rate limit", rate_limit_deg_s, 20.0, 120.0
    )
    sample_time_s = 0.01
    time_s = tuple(index * sample_time_s for index in range(501))
    command_deg = _command_schedule(time_s)
    position_limit_deg = 15.0
    moment_gain = 80.0
    normal = _simulate(
        command_deg,
        time_s,
        time_constant,
        rate_limit,
        position_limit_deg,
        omit_position_limit=False,
    )
    broken = _simulate(
        command_deg,
        time_s,
        time_constant,
        rate_limit,
        position_limit_deg,
        omit_position_limit=True,
    )

    deflection = normal["deflection_deg"]
    limited_command = normal["limited_command_deg"]
    actual_rate = normal["actual_rate_deg_s"]
    requested_error = tuple(
        command - delivered
        for command, delivered in zip(command_deg, deflection)
    )
    feasible_error = tuple(
        command - delivered
        for command, delivered in zip(limited_command, deflection)
    )
    requested_moment = tuple(moment_gain * value for value in command_deg)
    feasible_moment = tuple(moment_gain * value for value in limited_command)
    delivered_moment = tuple(moment_gain * value for value in deflection)
    broken_moment = tuple(
        moment_gain * value for value in broken["deflection_deg"]
    )
    maximum_feasible_moment = position_limit_deg * moment_gain
    broken_peak_moment = max(abs(value) for value in broken_moment)
    moment_shortfall = tuple(
        requested - delivered
        for requested, delivered in zip(requested_moment, delivered_moment)
    )
    position_excess = tuple(
        max(abs(value) - position_limit_deg, 0.0) for value in deflection
    )
    broken_position_excess = tuple(
        max(abs(value) - position_limit_deg, 0.0)
        for value in broken["deflection_deg"]
    )
    rate_excess = tuple(
        max(abs(value) - rate_limit, 0.0) for value in actual_rate
    )
    broken_rate_excess = tuple(
        max(abs(value) - rate_limit, 0.0)
        for value in broken["actual_rate_deg_s"]
    )
    positive_ninety_time = _first_time(
        time_s,
        tuple(
            time >= 0.5 and value >= 0.9 * position_limit_deg
            for time, value in zip(time_s, deflection)
        ),
    )
    reversal_zero_time = _first_time(
        time_s,
        tuple(
            time >= 2.0 and value <= 0.0
            for time, value in zip(time_s, deflection)
        ),
    )

    return {
        "time_constant_s": time_constant,
        "rate_limit_deg_s": rate_limit,
        "position_limit_deg": position_limit_deg,
        "sample_time_s": sample_time_s,
        "time_horizon_s": 5.0,
        "time_s": time_s,
        "sample_count": len(time_s),
        "update_count": len(time_s) - 1,
        "command_deg": command_deg,
        "normal": normal,
        "broken": broken,
        "requested_error_deg": requested_error,
        "feasible_error_deg": feasible_error,
        "rms_requested_error_deg": _rms(requested_error),
        "rms_feasible_error_deg": _rms(feasible_error),
        "total_absolute_feasible_error_deg_s": (
            sum(abs(value) for value in feasible_error[:-1]) * sample_time_s
        ),
        "infeasible_command_duration_s": (
            sum(normal["position_request_infeasible"][:-1]) * sample_time_s
        ),
        "rate_limited_duration_s": (
            sum(normal["rate_limit_active"][:-1]) * sample_time_s
        ),
        "positive_ninety_response_time_s": positive_ninety_time - 0.5,
        "reversal_zero_crossing_delay_s": reversal_zero_time - 2.0,
        "requested_moment_nm": requested_moment,
        "feasible_moment_nm": feasible_moment,
        "delivered_moment_nm": delivered_moment,
        "broken_moment_nm": broken_moment,
        "moment_shortfall_nm": moment_shortfall,
        "position_excess_deg": position_excess,
        "broken_position_excess_deg": broken_position_excess,
        "rate_excess_deg_s": rate_excess,
        "broken_rate_excess_deg_s": broken_rate_excess,
        "final_deflection_deg": deflection[-1],
        "peak_deflection_deg": max(abs(value) for value in deflection),
        "peak_rate_deg_s": max(abs(value) for value in actual_rate),
        "peak_delivered_moment_nm": max(abs(value) for value in delivered_moment),
        "maximum_feasible_moment_nm": maximum_feasible_moment,
        "maximum_moment_shortfall_nm": max(
            abs(value) for value in moment_shortfall
        ),
        "broken_maximum_position_excess_deg": max(broken_position_excess),
        "broken_peak_delivered_moment_nm": broken_peak_moment,
        "broken_peak_moment_excess_nm": max(
            broken_peak_moment - maximum_feasible_moment, 0.0
        ),
    }


class P10ArtifactTests(unittest.TestCase):
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
                "number": 10,
                "id": "P10",
                "title": "Model Actuator Dynamics and Limits",
                "guiding_question": GUIDING_QUESTION,
                "phase": 3,
                "phase_title": "Six-degree-of-freedom simulation",
                "slug": "model-actuator-dynamics-and-limits",
                "folder": "modules/10-model-actuator-dynamics-and-limits",
                "implementation_batch": "P10",
                "prerequisites": ["P09"],
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
            "p09",
            "command",
            "delivered deflection",
            "time constant",
            "first-order",
            "position",
            "hard stop",
            "rate limit",
            "deg/s",
            "pitch moment",
            "mechanism",
            "reset",
            "broken",
            "teach-back",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, combined)
        self.assertRegex(walkthrough.lower(), r"one plot|one .* at a time")
        self.assertIn("not a directly compatible package interface", combined)
        self.assertIn("no direct p09 adapter", self.text["model.m"].lower())
        self.assertIn("for the same remaining error", combined)
        self.assertIn("90% of the feasible", combined)
        experiment_lower = experiment.lower()
        self.assertEqual(experiment_lower.count("predict once:"), 1)
        self.assertLess(
            experiment_lower.index("predict once:"),
            experiment_lower.index("baseline=model("),
        )
        self.assertLess(lesson_script.index("experiment;"), lesson_script.index("interactive;"))
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
        self.assertIn("functionout=model(timeconstant_s,ratelimit_deg_s)", compact)
        self.assertIn("arguments", lower)
        self.assertIn(
            "timeconstant_s(1,1)double{mustbereal,mustbefinite}=0.18", compact
        )
        self.assertIn(
            "ratelimit_deg_s(1,1)double{mustbereal,mustbefinite}=45", compact
        )
        self.assertIn("timeconstant_s<0.05||timeconstant_s>0.50", compact)
        self.assertIn("ratelimit_deg_s<20||ratelimit_deg_s>120", compact)
        for identifier in (
            "P10:model:TimeConstantRange",
            "P10:model:RateLimitRange",
            "P10:model:ExpectedCrossingMissing",
        ):
            self.assertIn(identifier, model)

        for expression in (
            "sampletime_s=0.01;",
            "timehorizon_s=5;",
            "time_s=0:sampletime_s:timehorizon_s;",
            "positionlimit_deg=15;",
            "momentperdeflection_nm_per_deg=80;",
            "command_deg(time_s>=0.5&time_s<2.0)=25;",
            "command_deg(time_s>=2.0&time_s<3.5)=-25;",
            "command_deg(time_s>=3.5)=5;",
            "(limitedcommand_deg(k)-deflection_deg(k))/timeconstant_s;",
            "ratelimitactive(k)=abs(lagratedemand_deg_s(k))>ratelimit_deg_s;",
            "candidatedeflection_deg=deflection_deg(k)+step_s*ratelimiteddemand_deg_s(k);",
            "deflection_deg(k+1)=clampscalar(candidatedeflection_deg,-positionlimit_deg,positionlimit_deg);",
            "value=min(upperbound,max(lowerbound,value));",
        ):
            with self.subTest(expression=expression):
                self.assertIn(expression, compact)
        self.assertIn(
            "positionrequestinfeasible=abs(command_deg)>positionlimit_deg;",
            compact,
        )
        self.assertNotIn(
            "positionrequestinfeasible=false(size(command_deg));", compact
        )
        self.assertIn("position-limit ledger", lower)
        self.assertIn("whether or not the comparison enforces it", lower)
        self.assertIn("defensive post-update guard", lower)
        self.assertEqual(compact.count("simulateactuator("), 3)
        self.assertIn("positionlimit_deg,false);", compact)
        self.assertIn("positionlimit_deg,true);", compact)
        self.assertIn("'infeasibleCommandDuration_s'", model)

        for field in (
            "limitedCommand_deg",
            "deflection_deg",
            "lagRateDemand_deg_s",
            "actualRate_deg_s",
            "positionRequestInfeasible",
            "rateLimitActive",
            "kinematicClosureResidual_deg",
            "deliveredPitchMoment_Nm",
            "brokenDeflection_deg",
            "brokenPositionLimitExcess_deg",
            "brokenPeakMomentExcess_Nm",
            "positionRequestLedgerDefinition",
            "stateClipRole",
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

    def test_experiment_has_two_sweeps_metrics_and_broken_case(self) -> None:
        experiment = self.text["experiment.m"]
        lower = experiment.lower()
        compact = re.sub(r"\s+", "", experiment.replace("...", "")).lower()
        self.assertGreaterEqual(experiment.count("%%"), 14)
        self.assertGreaterEqual(lower.count("figure("), 5)
        self.assertIn("baseline", lower)
        self.assertGreaterEqual(lower.count("sweep"), 2)
        self.assertIn("time constant", lower)
        self.assertIn("rate limit", lower)
        self.assertIn("broken", lower)
        self.assertIn("hard stop", lower)
        for unit in ("deg/s", "n*m", "deg", "s"):
            self.assertIn(unit, lower)
        self.assertIn("fprintf", lower)
        self.assertGreaterEqual(lower.count("assert("), 6)
        self.assertIn("model(timeconstantsweep_s(k),45)", compact)
        self.assertIn("model(0.18,ratelimitsweep_deg_s(k))", compact)
        self.assertIn("lagonly=model(0.50,120)", compact)
        self.assertIn("subplot(1,3,2)", compact)
        self.assertIn("subplot(1,3,3)", compact)
        self.assertIn("subplot(2,2,4)", compact)
        self.assertNotIn(
            "90% response time (s) or feasible rms error (deg)", lower
        )
        self.assertNotRegex(lower, r"\bclose\s+all\b")
        self.assertIn("findall(groot", lower)
        self.assertIn("'^p10 '", lower)
        self.assertRegex(lower, r"clear run_checks;\s*run_checks;")
        self.assertNotIn("interactive;", lower)

        assignments = re.findall(
            r"(?:timeConstantSweep_s|rateLimitSweep_deg_s)\s*=\s*\[([^\]]+)\]",
            experiment,
        )
        self.assertEqual(len(assignments), 2)
        for values_text in assignments:
            values = [
                float(value)
                for value in re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)", values_text)
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
        self.assertIn("p10", interactive_lower)
        self.assertIn("existingui=findall(groot", interactive_compact)
        self.assertIn("close(existingui)", interactive_compact)
        self.assertEqual(interactive_lower.count("uislider("), 2)
        self.assertEqual(interactive_lower.count("uibutton("), 1)
        self.assertIn("'limits',[0.050.50]", interactive_compact)
        self.assertIn("'limits',[20120]", interactive_compact)
        self.assertIn("'value',0.18", interactive_compact)
        self.assertIn("'value',45", interactive_compact)
        self.assertIn("valuechangingfcn", interactive_lower)
        self.assertIn("valuechangedfcn", interactive_lower)
        self.assertIn("event.value", interactive_lower)
        self.assertIn("buttonpushedfcn", interactive_lower)
        self.assertIn("functionresetbaseline", interactive_compact)
        self.assertIn("timecontrol.value=0.18", interactive_compact)
        self.assertIn("ratecontrol.value=45", interactive_compact)
        self.assertIn("modelfcn=@model;", interactive_compact)
        self.assertEqual(interactive_compact.count("out=modelfcn("), 1)
        self.assertGreaterEqual(interactive_compact.count("cla("), 4)
        self.assertNotIn("yyaxis", interactive_lower)
        for unit in ("deg/s", "n*m", "deg", "s"):
            self.assertIn(unit, interactive_lower)

        self.assertGreaterEqual(checks_lower.count("assert("), 25)
        for concept in (
            "determinism",
            "fixed shape",
            "finite resources",
            "independent command reconstruction",
            "every recurrence update",
            "exact rest",
            "rate-to-lag regime transition",
            "inactive-rate",
            "isolated parameter sweeps",
            "broken position envelope",
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
        self.assertIn("samplecount==501", checks_compact)
        self.assertIn("updatecount==500", checks_compact)
        self.assertRegex(checks_compact, r"representativecasecount==9\b")
        self.assertIn("ratereversaldelay", checks_compact)
        self.assertIn("catch exception", checks_lower)
        self.assertIn("strcmp(exception.identifier,expectedidentifier)", checks_compact)
        self.assertIn("P10 checks passed", checks_script)

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


class P10IndependentOracleTests(unittest.TestCase):
    def test_baseline_is_deterministic_and_recognizable(self) -> None:
        first = _oracle()
        second = _oracle()
        self.assertEqual(first, second)
        self.assertEqual(first["sample_count"], 501)
        self.assertEqual(first["update_count"], 500)
        self.assertEqual(first["time_s"][0], 0.0)
        self.assertEqual(first["time_s"][-1], 5.0)
        self.assertEqual(first["sample_time_s"], 0.01)
        self.assertAlmostEqual(first["final_deflection_deg"], 4.993078514854386)
        self.assertAlmostEqual(
            first["peak_deflection_deg"], 14.996320961068609
        )
        self.assertAlmostEqual(first["peak_rate_deg_s"], 45.0)
        self.assertAlmostEqual(
            first["rms_requested_error_deg"], 13.245442291719344
        )
        self.assertAlmostEqual(
            first["rms_feasible_error_deg"], 7.700152257797825
        )
        self.assertAlmostEqual(
            first["total_absolute_feasible_error_deg_s"],
            19.316555655893882,
        )
        self.assertAlmostEqual(
            first["positive_ninety_response_time_s"], 0.45
        )
        self.assertAlmostEqual(
            first["reversal_zero_crossing_delay_s"], 0.34
        )
        self.assertAlmostEqual(first["infeasible_command_duration_s"], 3.0)
        self.assertAlmostEqual(first["rate_limited_duration_s"], 0.92)
        self.assertAlmostEqual(
            first["peak_delivered_moment_nm"], 1199.7056768854886
        )
        self.assertAlmostEqual(
            first["broken_maximum_position_excess_deg"],
            9.986896548243244,
        )
        self.assertAlmostEqual(
            first["broken_peak_delivered_moment_nm"],
            1998.9517238594594,
        )
        self.assertAlmostEqual(first["maximum_feasible_moment_nm"], 1200.0)
        self.assertAlmostEqual(
            first["broken_peak_moment_excess_nm"],
            798.9517238594594,
        )

    def test_fixed_shapes_finite_values_and_resource_boundary(self) -> None:
        result = _oracle()
        self.assertEqual(len(result["time_s"]), 501)
        self.assertEqual(len(result["command_deg"]), 501)
        for trajectory_name in ("normal", "broken"):
            trajectory = result[trajectory_name]
            for name, history in trajectory.items():
                with self.subTest(trajectory=trajectory_name, field=name):
                    self.assertEqual(len(history), 501)
                    if "active" in name or "infeasible" in name:
                        self.assertTrue(all(isinstance(value, bool) for value in history))
                    else:
                        self.assertTrue(all(math.isfinite(value) for value in history))
        for name in (
            "requested_error_deg",
            "feasible_error_deg",
            "requested_moment_nm",
            "feasible_moment_nm",
            "delivered_moment_nm",
            "broken_moment_nm",
            "moment_shortfall_nm",
            "position_excess_deg",
            "broken_position_excess_deg",
            "rate_excess_deg_s",
            "broken_rate_excess_deg_s",
        ):
            history = result[name]
            self.assertEqual(len(history), 501, name)
            self.assertTrue(all(math.isfinite(value) for value in history), name)

    def test_command_recurrence_limits_and_moment_mapping_close(self) -> None:
        result = _oracle()
        normal = result["normal"]
        command = result["command_deg"]
        limited = normal["limited_command_deg"]
        deflection = normal["deflection_deg"]
        rate = normal["actual_rate_deg_s"]
        self.assertEqual(command[:50], (0.0,) * 50)
        self.assertEqual(command[50], 25.0)
        self.assertEqual(command[200], -25.0)
        self.assertEqual(command[350], 5.0)
        self.assertEqual(limited[50], 15.0)
        self.assertEqual(limited[200], -15.0)
        self.assertEqual(limited[350], 5.0)

        for index in range(result["update_count"]):
            step_s = result["time_s"][index + 1] - result["time_s"][index]
            raw_rate = (limited[index] - deflection[index]) / result["time_constant_s"]
            expected_bounded_rate = _clip(
                raw_rate,
                -result["rate_limit_deg_s"],
                result["rate_limit_deg_s"],
            )
            candidate = deflection[index] + step_s * expected_bounded_rate
            expected_next = _clip(
                candidate,
                -result["position_limit_deg"],
                result["position_limit_deg"],
            )
            self.assertEqual(deflection[index + 1], expected_next)
            self.assertAlmostEqual(
                rate[index], (expected_next - deflection[index]) / step_s
            )
            self.assertLess(
                abs(normal["kinematic_closure_residual_deg"][index]), 1e-15
            )

        self.assertLessEqual(max(abs(value) for value in deflection), 15.0)
        self.assertLessEqual(max(abs(value) for value in rate), 45.0 + 1e-12)
        self.assertEqual(max(result["position_excess_deg"]), 0.0)
        self.assertLess(max(result["rate_excess_deg_s"]), 1e-12)
        for command_value, feasible_value, request, feasible, delivered, deflection_value in zip(
            command,
            limited,
            result["requested_moment_nm"],
            result["feasible_moment_nm"],
            result["delivered_moment_nm"],
            deflection,
        ):
            self.assertEqual(request, 80.0 * command_value)
            self.assertEqual(feasible, 80.0 * feasible_value)
            self.assertEqual(delivered, 80.0 * deflection_value)

    def test_exact_rest_and_inactive_rate_limiting_case(self) -> None:
        baseline = _oracle()
        normal = baseline["normal"]
        rest_indices = [
            index for index, time in enumerate(baseline["time_s"]) if time < 0.5
        ]
        for index in rest_indices:
            self.assertEqual(baseline["command_deg"][index], 0.0)
            self.assertEqual(normal["limited_command_deg"][index], 0.0)
            self.assertEqual(normal["deflection_deg"][index], 0.0)
            self.assertEqual(normal["actual_rate_deg_s"][index], 0.0)
            self.assertEqual(baseline["delivered_moment_nm"][index], 0.0)

        lag_only = _oracle(0.50, 120.0)
        lag_trajectory = lag_only["normal"]
        self.assertFalse(any(lag_trajectory["rate_limit_active"]))
        coefficient = lag_only["sample_time_s"] / lag_only["time_constant_s"]
        for index in range(lag_only["update_count"]):
            expected_next = (
                (1.0 - coefficient) * lag_trajectory["deflection_deg"][index]
                + coefficient * lag_trajectory["limited_command_deg"][index]
            )
            self.assertAlmostEqual(
                lag_trajectory["deflection_deg"][index + 1], expected_next
            )

    def test_large_reversal_is_rate_limited_then_returns_to_lag(self) -> None:
        baseline = _oracle()
        trajectory = baseline["normal"]
        reversal_index = baseline["time_s"].index(2.0)
        release_index = next(
            index
            for index in range(reversal_index, baseline["update_count"])
            if not trajectory["rate_limit_active"][index]
        )

        self.assertTrue(trajectory["rate_limit_active"][reversal_index])
        for index in range(reversal_index, release_index):
            with self.subTest(regime="rate-limited", index=index):
                self.assertTrue(trajectory["rate_limit_active"][index])
                self.assertAlmostEqual(
                    trajectory["actual_rate_deg_s"][index],
                    -baseline["rate_limit_deg_s"],
                )
        self.assertGreater(
            baseline["time_s"][release_index],
            2.0 + baseline["reversal_zero_crossing_delay_s"],
        )
        self.assertAlmostEqual(
            trajectory["actual_rate_deg_s"][release_index],
            trajectory["lag_rate_demand_deg_s"][release_index],
        )
        self.assertLess(
            abs(trajectory["actual_rate_deg_s"][release_index]),
            baseline["rate_limit_deg_s"],
        )

        rate_limits = (20.0, 30.0, 45.0, 60.0, 80.0)
        rate_results = [_oracle(0.18, value) for value in rate_limits]
        reversal_delays = []
        for rate_limit, result in zip(rate_limits, rate_results):
            index = result["time_s"].index(2.0)
            with self.subTest(regime="reversal", rate_limit=rate_limit):
                self.assertTrue(result["normal"]["rate_limit_active"][index])
                self.assertAlmostEqual(
                    result["normal"]["actual_rate_deg_s"][index],
                    -rate_limit,
                )
            reversal_delays.append(result["reversal_zero_crossing_delay_s"])
        self.assertTrue(
            all(
                left > right
                for left, right in zip(reversal_delays, reversal_delays[1:])
            )
        )

    def test_two_sweeps_change_independent_inputs_and_observables(self) -> None:
        time_constants = (0.08, 0.12, 0.18, 0.28, 0.40)
        time_results = [_oracle(value, 45.0) for value in time_constants]
        time_response = [
            result["positive_ninety_response_time_s"] for result in time_results
        ]
        time_error = [result["rms_feasible_error_deg"] for result in time_results]
        self.assertTrue(
            all(left < right for left, right in zip(time_response, time_response[1:]))
        )
        self.assertTrue(
            all(left < right for left, right in zip(time_error, time_error[1:]))
        )
        for result in time_results:
            self.assertEqual(result["command_deg"], time_results[0]["command_deg"])
            self.assertEqual(
                result["normal"]["limited_command_deg"],
                time_results[0]["normal"]["limited_command_deg"],
            )
            self.assertEqual(result["rate_limit_deg_s"], 45.0)
            self.assertEqual(result["position_limit_deg"], 15.0)

        rate_limits = (20.0, 30.0, 45.0, 60.0, 80.0)
        rate_results = [_oracle(0.18, value) for value in rate_limits]
        rate_response = [
            result["positive_ninety_response_time_s"] for result in rate_results
        ]
        rate_error = [result["rms_feasible_error_deg"] for result in rate_results]
        peak_rate = [result["peak_rate_deg_s"] for result in rate_results]
        self.assertTrue(
            all(left > right for left, right in zip(rate_response, rate_response[1:]))
        )
        self.assertTrue(
            all(left > right for left, right in zip(rate_error, rate_error[1:]))
        )
        self.assertTrue(
            all(left < right for left, right in zip(peak_rate, peak_rate[1:]))
        )
        for result in rate_results:
            self.assertEqual(result["command_deg"], rate_results[0]["command_deg"])
            self.assertEqual(
                result["normal"]["limited_command_deg"],
                rate_results[0]["normal"]["limited_command_deg"],
            )
            self.assertEqual(result["time_constant_s"], 0.18)
            self.assertEqual(result["position_limit_deg"], 15.0)

    def test_broken_case_omits_only_position_envelope(self) -> None:
        result = _oracle()
        normal = result["normal"]
        broken = result["broken"]
        self.assertEqual(broken["limited_command_deg"], result["command_deg"])
        self.assertEqual(
            broken["position_request_infeasible"],
            normal["position_request_infeasible"],
        )
        self.assertTrue(any(broken["position_request_infeasible"]))
        positive_step_index = result["time_s"].index(0.5)
        positive_release_index = next(
            index
            for index in range(positive_step_index, result["update_count"])
            if not normal["rate_limit_active"][index]
        )
        first_divergence_index = next(
            index
            for index, (complete, omitted) in enumerate(
                zip(normal["deflection_deg"], broken["deflection_deg"])
            )
            if complete != omitted
        )
        self.assertEqual(
            normal["deflection_deg"][: positive_release_index + 1],
            broken["deflection_deg"][: positive_release_index + 1],
        )
        self.assertEqual(first_divergence_index, positive_release_index + 1)
        self.assertAlmostEqual(result["time_s"][first_divergence_index], 0.67)
        self.assertGreater(result["broken_maximum_position_excess_deg"], 9.0)
        self.assertGreater(result["broken_peak_delivered_moment_nm"], 1900.0)
        self.assertAlmostEqual(
            result["broken_peak_moment_excess_nm"],
            798.9517238594594,
        )
        self.assertEqual(max(result["position_excess_deg"]), 0.0)
        self.assertLess(max(result["broken_rate_excess_deg_s"]), 1e-12)
        self.assertTrue(
            all(
                math.isfinite(value)
                for value in broken["deflection_deg"]
            )
        )

    def test_malformed_inputs_fail_without_poisoning_recovery(self) -> None:
        baseline = _oracle()
        malformed = (
            ("time below", (0.049, 45.0)),
            ("time above", (0.501, 45.0)),
            ("rate below", (0.18, 19.9)),
            ("rate above", (0.18, 120.1)),
            ("nan time", (math.nan, 45.0)),
            ("infinite rate", (0.18, math.inf)),
            ("list time", ([0.18], 45.0)),
            ("complex rate", (0.18, 45.0 + 1j)),
            ("text time", ("fast", 45.0)),
            ("boolean rate", (0.18, True)),
        )
        for name, values in malformed:
            with self.subTest(case=name), self.assertRaises(ValueError):
                _oracle(*values)
        self.assertEqual(_oracle(), baseline)

    def test_position_target_keeps_state_clip_inactive_at_domain_corners(self) -> None:
        for time_constant in (0.05, 0.50):
            for rate_limit in (20.0, 120.0):
                result = _oracle(time_constant, rate_limit)
                trajectory = result["normal"]
                candidates = tuple(
                    trajectory["deflection_deg"][index]
                    + (
                        result["time_s"][index + 1]
                        - result["time_s"][index]
                    )
                    * trajectory["rate_limited_demand_deg_s"][index]
                    for index in range(result["update_count"])
                )
                with self.subTest(
                    time_constant=time_constant, rate_limit=rate_limit
                ):
                    self.assertEqual(
                        candidates, trajectory["deflection_deg"][1:]
                    )
                    self.assertLessEqual(
                        max(abs(value) for value in candidates),
                        result["position_limit_deg"],
                    )

    def test_accepted_corners_and_representative_grid_are_bounded(self) -> None:
        corner_count = 0
        for time_constant in (0.05, 0.50):
            for rate_limit in (20.0, 120.0):
                result = _oracle(time_constant, rate_limit)
                corner_count += 1
                self.assertEqual(result["sample_count"], 501)
                self.assertEqual(result["update_count"], 500)
                self.assertLessEqual(result["peak_deflection_deg"], 15.0)
                self.assertLessEqual(result["peak_rate_deg_s"], rate_limit + 1e-12)
                self.assertLess(
                    max(abs(value) for value in result["broken"]["deflection_deg"]),
                    30.0,
                )
        self.assertEqual(corner_count, 4)

        representative_count = 0
        for time_constant in (0.05, 0.18, 0.50):
            for rate_limit in (20.0, 45.0, 120.0):
                result = _oracle(time_constant, rate_limit)
                representative_count += 1
                self.assertEqual(len(result["normal"]["deflection_deg"]), 501)
                self.assertEqual(len(result["broken"]["deflection_deg"]), 501)
        self.assertEqual(representative_count, 9)
        self.assertLessEqual(representative_count, 10)


if __name__ == "__main__":
    unittest.main()
