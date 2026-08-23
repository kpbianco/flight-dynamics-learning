from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULE_ID = "P08"
MODULE_FOLDER = ROOT / "modules/08-relate-stability-derivatives-to-motion"
GUIDING_QUESTION = (
    "What inputs, observable effects, and failure modes matter when you relate "
    "Stability Derivatives to Motion?"
)


def _finite_real_scalar(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite real scalar")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite real scalar")
    return result


def _matvec(
    matrix: tuple[tuple[float, ...], ...], state: tuple[float, ...]
) -> tuple[float, ...]:
    return tuple(
        sum(entry * value for entry, value in zip(row, state))
        for row in matrix
    )


def _state_add(
    state: tuple[float, ...],
    *increments: tuple[float, tuple[float, ...]],
) -> tuple[float, ...]:
    return tuple(
        value
        + sum(scale * increment[index] for scale, increment in increments)
        for index, value in enumerate(state)
    )


def _integrate_rk4(
    matrix: tuple[tuple[float, ...], ...],
    initial_state: tuple[float, ...],
    time_s: tuple[float, ...],
) -> tuple[tuple[float, ...], ...]:
    """Independent visible RK4 recurrence; it does not execute MATLAB."""
    history = [initial_state]
    state = initial_state
    for left, right in zip(time_s, time_s[1:]):
        step_s = right - left
        k1 = _matvec(matrix, state)
        k2 = _matvec(matrix, _state_add(state, (0.5 * step_s, k1)))
        k3 = _matvec(matrix, _state_add(state, (0.5 * step_s, k2)))
        k4 = _matvec(matrix, _state_add(state, (step_s, k3)))
        state = _state_add(
            state,
            (step_s / 6.0, k1),
            (step_s / 3.0, k2),
            (step_s / 3.0, k3),
            (step_s / 6.0, k4),
        )
        history.append(state)
    return tuple(tuple(row[index] for row in history) for index in range(4))


def _peak_and_time(
    values: tuple[float, ...], time_s: tuple[float, ...]
) -> tuple[float, float]:
    magnitudes = tuple(map(abs, values))
    index = max(range(len(magnitudes)), key=magnitudes.__getitem__)
    return magnitudes[index], time_s[index]


def _oracle(
    initial_sideslip_deg: object = 3.0,
    roll_damping_derivative_cl_p: object = -0.50,
    weathercock_derivative_cn_beta_per_rad: object = 0.18,
) -> dict[str, object]:
    """Pure-stdlib equation oracle independent of the MATLAB source."""
    beta0_deg = _finite_real_scalar("initial sideslip", initial_sideslip_deg)
    cl_p = _finite_real_scalar(
        "roll damping derivative", roll_damping_derivative_cl_p
    )
    cn_beta = _finite_real_scalar(
        "weathercock derivative", weathercock_derivative_cn_beta_per_rad
    )
    if abs(beta0_deg) > 4.0:
        raise ValueError("initial sideslip outside the learning range")
    if not -0.8 <= cl_p <= -0.3:
        raise ValueError("roll damping derivative outside the learning range")
    if not 0.0 <= cn_beta <= 0.24:
        raise ValueError("weathercock derivative outside the learning range")

    true_airspeed_mps = 60.0
    dynamic_pressure_pa = 1325.00798531847
    wing_area_m2 = 16.2
    wing_span_m = 10.9
    mass_kg = 1200.0
    roll_inertia_kgm2 = 2500.0
    yaw_inertia_kgm2 = 4000.0
    gravity_mps2 = 9.80665

    cy_beta = -0.65
    cy_p = -0.03
    cy_r = 0.25
    cl_beta = -0.12
    cl_r = 0.10
    cn_p = -0.06
    cn_r = -0.25

    rate_scale_s = wing_span_m / (2.0 * true_airspeed_mps)
    force_scale_n = dynamic_pressure_pa * wing_area_m2
    moment_scale_nm = force_scale_n * wing_span_m
    dimensional = {
        "y_beta": force_scale_n * cy_beta,
        "y_p": force_scale_n * cy_p * rate_scale_s,
        "y_r": force_scale_n * cy_r * rate_scale_s,
        "l_beta": moment_scale_nm * cl_beta,
        "l_p": moment_scale_nm * cl_p * rate_scale_s,
        "l_r": moment_scale_nm * cl_r * rate_scale_s,
        "n_beta": moment_scale_nm * cn_beta,
        "n_p": moment_scale_nm * cn_p * rate_scale_s,
        "n_r": moment_scale_nm * cn_r * rate_scale_s,
    }
    matrix = (
        (
            dimensional["y_beta"] / (mass_kg * true_airspeed_mps),
            dimensional["y_p"] / (mass_kg * true_airspeed_mps),
            dimensional["y_r"] / (mass_kg * true_airspeed_mps) - 1.0,
            gravity_mps2 / true_airspeed_mps,
        ),
        (
            dimensional["l_beta"] / roll_inertia_kgm2,
            dimensional["l_p"] / roll_inertia_kgm2,
            dimensional["l_r"] / roll_inertia_kgm2,
            0.0,
        ),
        (
            dimensional["n_beta"] / yaw_inertia_kgm2,
            dimensional["n_p"] / yaw_inertia_kgm2,
            dimensional["n_r"] / yaw_inertia_kgm2,
            0.0,
        ),
        (0.0, 1.0, 0.0, 0.0),
    )
    time_s = tuple(index * 0.02 for index in range(1251))
    initial_state = (math.radians(beta0_deg), 0.0, 0.0, 0.0)
    state = _integrate_rk4(matrix, initial_state, time_s)
    beta_rad, roll_rate_rad_s, yaw_rate_rad_s, bank_rad = state
    beta_deg = tuple(map(math.degrees, beta_rad))
    roll_rate_deg_s = tuple(map(math.degrees, roll_rate_rad_s))
    yaw_rate_deg_s = tuple(map(math.degrees, yaw_rate_rad_s))
    bank_deg = tuple(map(math.degrees, bank_rad))

    p_hat = tuple(value * rate_scale_s for value in roll_rate_rad_s)
    r_hat = tuple(value * rate_scale_s for value in yaw_rate_rad_s)
    cy_from_beta = tuple(cy_beta * value for value in beta_rad)
    cy_from_p = tuple(cy_p * value for value in p_hat)
    cy_from_r = tuple(cy_r * value for value in r_hat)
    cl_from_beta = tuple(cl_beta * value for value in beta_rad)
    cl_from_p = tuple(cl_p * value for value in p_hat)
    cl_from_r = tuple(cl_r * value for value in r_hat)
    cn_from_beta = tuple(cn_beta * value for value in beta_rad)
    cn_from_p = tuple(cn_p * value for value in p_hat)
    cn_from_r = tuple(cn_r * value for value in r_hat)
    cy_total = tuple(
        beta + p_term + r_term
        for beta, p_term, r_term in zip(cy_from_beta, cy_from_p, cy_from_r)
    )
    cl_total = tuple(
        beta + p_term + r_term
        for beta, p_term, r_term in zip(cl_from_beta, cl_from_p, cl_from_r)
    )
    cn_total = tuple(
        beta + p_term + r_term
        for beta, p_term, r_term in zip(cn_from_beta, cn_from_p, cn_from_r)
    )
    side_force_n = tuple(force_scale_n * value for value in cy_total)
    roll_moment_nm = tuple(moment_scale_nm * value for value in cl_total)
    yaw_moment_nm = tuple(moment_scale_nm * value for value in cn_total)
    ledger_derivative = tuple(
        (
            side_force_n[index] / (mass_kg * true_airspeed_mps)
            - yaw_rate_rad_s[index]
            + gravity_mps2 / true_airspeed_mps * bank_rad[index],
            roll_moment_nm[index] / roll_inertia_kgm2,
            yaw_moment_nm[index] / yaw_inertia_kgm2,
            roll_rate_rad_s[index],
        )
        for index in range(len(time_s))
    )
    matrix_derivative = tuple(
        _matvec(matrix, tuple(row[index] for row in state))
        for index in range(len(time_s))
    )

    if beta0_deg == 0.0:
        first_zero_s = 0.0
    else:
        first_zero_s = next(
            (
                time_s[index]
                for index, value in enumerate(beta_deg[1:], start=1)
                if value * beta0_deg <= 0.0
            ),
            math.inf,
        )
    peak_roll_deg_s, peak_roll_time_s = _peak_and_time(roll_rate_deg_s, time_s)
    peak_yaw_deg_s, peak_yaw_time_s = _peak_and_time(yaw_rate_deg_s, time_s)
    peak_bank_deg, peak_bank_time_s = _peak_and_time(bank_deg, time_s)

    broken_numeric_nm = moment_scale_nm * cl_p
    broken_matrix_rows = [list(row) for row in matrix]
    broken_matrix_rows[1][1] = broken_numeric_nm / roll_inertia_kgm2
    broken_matrix = tuple(tuple(row) for row in broken_matrix_rows)
    broken_state = _integrate_rk4(broken_matrix, initial_state, time_s)
    broken_roll_deg_s = tuple(map(math.degrees, broken_state[1]))
    broken_bank_deg = tuple(map(math.degrees, broken_state[3]))

    return {
        "initial_sideslip_deg": beta0_deg,
        "roll_damping_derivative_cl_p": cl_p,
        "weathercock_derivative_cn_beta_per_rad": cn_beta,
        "true_airspeed_mps": true_airspeed_mps,
        "dynamic_pressure_pa": dynamic_pressure_pa,
        "wing_area_m2": wing_area_m2,
        "wing_span_m": wing_span_m,
        "mass_kg": mass_kg,
        "roll_inertia_kgm2": roll_inertia_kgm2,
        "yaw_inertia_kgm2": yaw_inertia_kgm2,
        "gravity_mps2": gravity_mps2,
        "rate_scale_s": rate_scale_s,
        "force_scale_n": force_scale_n,
        "moment_scale_nm": moment_scale_nm,
        "dimensional": dimensional,
        "matrix": matrix,
        "time_s": time_s,
        "sample_count": len(time_s),
        "state": state,
        "beta_deg": beta_deg,
        "roll_rate_deg_s": roll_rate_deg_s,
        "yaw_rate_deg_s": yaw_rate_deg_s,
        "bank_deg": bank_deg,
        "p_hat": p_hat,
        "r_hat": r_hat,
        "cy_components": (cy_from_beta, cy_from_p, cy_from_r),
        "cl_components": (cl_from_beta, cl_from_p, cl_from_r),
        "cn_components": (cn_from_beta, cn_from_p, cn_from_r),
        "cy_total": cy_total,
        "cl_total": cl_total,
        "cn_total": cn_total,
        "side_force_n": side_force_n,
        "roll_moment_nm": roll_moment_nm,
        "yaw_moment_nm": yaw_moment_nm,
        "ledger_derivative": ledger_derivative,
        "matrix_derivative": matrix_derivative,
        "first_zero_s": first_zero_s,
        "peak_roll_deg_s": peak_roll_deg_s,
        "peak_roll_time_s": peak_roll_time_s,
        "peak_yaw_deg_s": peak_yaw_deg_s,
        "peak_yaw_time_s": peak_yaw_time_s,
        "peak_bank_deg": peak_bank_deg,
        "peak_bank_time_s": peak_bank_time_s,
        "peak_beta_deg": max(map(abs, beta_deg)),
        "broken_numeric_nm": broken_numeric_nm,
        "broken_matrix": broken_matrix,
        "broken_roll_deg_s": broken_roll_deg_s,
        "broken_bank_deg": broken_bank_deg,
        "broken_peak_roll_deg_s": max(map(abs, broken_roll_deg_s)),
        "broken_peak_bank_deg": max(map(abs, broken_bank_deg)),
        "broken_a22_numeric_ratio_per_s": abs(
            broken_matrix[1][1] / matrix[1][1]
        ),
    }


class P08ArtifactTests(unittest.TestCase):
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
                "number": 8,
                "id": "P08",
                "title": "Relate Stability Derivatives to Motion",
                "guiding_question": GUIDING_QUESTION,
                "phase": 2,
                "phase_title": "Stability and modes",
                "slug": "relate-stability-derivatives-to-motion",
                "folder": "modules/08-relate-stability-derivatives-to-motion",
                "implementation_batch": "P08",
                "prerequisites": ["P07"],
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

        for concept in (
            "p07",
            "sideslip",
            "roll rate",
            "yaw rate",
            "bank",
            "p-hat",
            "r-hat",
            "coefficient",
            "dimensional",
            "coupled",
            "c_l_p",
            "c_n_beta",
            "mechanism",
            "reset",
            "teach-back",
            "p09",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, combined)
        self.assertIn("read", walkthrough.lower())
        self.assertIn("baseline", walkthrough.lower())
        self.assertRegex(walkthrough.lower(), r"one visual transition|one .* at a time")
        experiment_lower = experiment.lower()
        self.assertEqual(experiment_lower.count("predict once:"), 1)
        self.assertLess(
            experiment_lower.index("predict once:"),
            experiment_lower.index("baseline=model("),
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
            "functionout=model(initialsideslip_deg,rolldampingderivative_cl_p,"
            "weathercockderivative_cn_beta_perrad)",
            compact,
        )
        self.assertIn("arguments", lower)
        self.assertEqual(
            compact.count("(1,1)double{mustbereal,mustbefinite}"), 3
        )
        for expression in (
            "abs(initialsideslip_deg)>4",
            "rolldampingderivative_cl_p<-0.8||rolldampingderivative_cl_p>-0.3",
            "weathercockderivative_cn_beta_perrad<0||"
            "weathercockderivative_cn_beta_perrad>0.24",
            "ratenormalizationtime_s=wingspan_m/(2*referencetrueairspeed_mps);",
            "time_s=0:0.02:25;",
            "nondimensionalrollrate=rollrate_rad_s*ratenormalizationtime_s;",
            "nondimensionalyawrate=yawrate_rad_s*ratenormalizationtime_s;",
            "stateMatrix(4,:)=[0100];".lower(),
            "k1=statematrix*state(:,k);",
            "k2=statematrix*(state(:,k)+0.5*step_s*k1);",
            "k3=statematrix*(state(:,k)+0.5*step_s*k2);",
            "k4=statematrix*(state(:,k)+step_s*k3);",
        ):
            self.assertIn(expression.lower(), compact)
        for identifier in (
            "P08:model:InitialSideslipRange",
            "P08:model:RollDampingDerivativeRange",
            "P08:model:WeathercockDerivativeRange",
        ):
            self.assertIn(identifier, model)
        for formula in (
            "sideforcederivative_cy_beta_perrad=-0.65;",
            "sideforcerollratederivative_cy_p=-0.03;",
            "sideforceyawratederivative_cy_r=0.25;",
            "dihedralderivative_cl_beta_perrad=-0.12;",
            "rollmomentyawratederivative_cl_r=0.10;",
            "yawmomentrollratederivative_cn_p=-0.06;",
            "yawdampingderivative_cn_r=-0.25;",
            "sideforcescale_n=referencedynamicpressure_pa*wingarea_m2;",
            "momentscale_nm=sideforcescale_n*wingspan_m;",
            "sideforcebetaderivative_n_per_rad=sideforcescale_n*"
            "sideforcederivative_cy_beta_perrad;",
            "sideforcerollratederivative_n_per_rad_s=sideforcescale_n*"
            "sideforcerollratederivative_cy_p*ratenormalizationtime_s;",
            "sideforceyawratederivative_n_per_rad_s=sideforcescale_n*"
            "sideforceyawratederivative_cy_r*ratenormalizationtime_s;",
            "rollmomentbetaderivative_nm_per_rad=momentscale_nm*"
            "dihedralderivative_cl_beta_perrad;",
            "rollmomentrollratederivative_nm_per_rad_s=momentscale_nm*"
            "rolldampingderivative_cl_p*ratenormalizationtime_s;",
            "rollmomentyawratederivative_nm_per_rad_s=momentscale_nm*"
            "rollmomentyawratederivative_cl_r*ratenormalizationtime_s;",
            "yawmomentbetaderivative_nm_per_rad=momentscale_nm*"
            "weathercockderivative_cn_beta_perrad;",
            "yawmomentrollratederivative_nm_per_rad_s=momentscale_nm*"
            "yawmomentrollratederivative_cn_p*ratenormalizationtime_s;",
            "yawmomentyawratederivative_nm_per_rad_s=momentscale_nm*"
            "yawdampingderivative_cn_r*ratenormalizationtime_s;",
            "statematrix(1,:)=[sideforcebetaderivative_n_per_rad/"
            "(mass_kg*referencetrueairspeed_mps),"
            "sideforcerollratederivative_n_per_rad_s/"
            "(mass_kg*referencetrueairspeed_mps),"
            "sideforceyawratederivative_n_per_rad_s/"
            "(mass_kg*referencetrueairspeed_mps)-1,"
            "gravity_mps2/referencetrueairspeed_mps];",
            "statematrix(2,:)=[rollmomentbetaderivative_nm_per_rad/"
            "rollinertia_kgm2,rollmomentrollratederivative_nm_per_rad_s/"
            "rollinertia_kgm2,rollmomentyawratederivative_nm_per_rad_s/"
            "rollinertia_kgm2,0];",
            "statematrix(3,:)=[yawmomentbetaderivative_nm_per_rad/"
            "yawinertia_kgm2,yawmomentrollratederivative_nm_per_rad_s/"
            "yawinertia_kgm2,yawmomentyawratederivative_nm_per_rad_s/"
            "yawinertia_kgm2,0];",
            "initialstate=[initialsideslip_deg*pi/180;0;0;0];",
            "state=integratefixedrk4(statematrix,initialstate,time_s);",
            "sideforce_n/(mass_kg*referencetrueairspeed_mps)-yawrate_rad_s+"
            "gravity_mps2/referencetrueairspeed_mps*bankangle_rad;",
            "brokenrolldampingnumeric_nm=momentscale_nm*"
            "rolldampingderivative_cl_p;",
            "brokenstatematrix(2,2)=brokenrolldampingnumeric_nm/"
            "rollinertia_kgm2;",
            "brokenstate=integratefixedrk4("
            "brokenstatematrix,initialstate,time_s);",
            "state(:,k+1)=state(:,k)+step_s/6*"
            "(k1+2*k2+2*k3+k4);",
        ):
            self.assertIn(formula, compact)
        self.assertIn("not an identified aircraft model", lower)
        self.assertIn("not identified data, nonlinear 6-DOF", model)

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
        self.assertGreaterEqual(experiment.count("%%"), 12)
        self.assertIn("baseline", lower)
        self.assertGreaterEqual(lower.count("sweep"), 2)
        self.assertIn("roll damping", lower)
        self.assertIn("weathercock", lower)
        self.assertIn("broken", lower)
        self.assertIn("omit b/(2*v0)", lower)
        self.assertGreaterEqual(lower.count("figure("), 5)
        self.assertIn("xlabel", lower)
        self.assertIn("ylabel", lower)
        for unit in (
            "1/rad",
            "deg/s^2",
            "deg/s",
            "m/s",
            "kg*m^2",
            "n*m",
            "pa",
            "deg",
            "s",
        ):
            with self.subTest(unit=unit):
                self.assertIn(unit, lower)
        self.assertIn("fprintf", lower)
        self.assertGreaterEqual(lower.count("assert("), 5)
        self.assertIn("model(3,rolldampingsweep_cl_p(k),0.18)", compact)
        self.assertIn(
            "model(3,-0.50,weathercocksweep_cn_beta_perrad(k))", compact
        )
        self.assertIn("matrixdifference(2,2)=0", compact)
        self.assertIn("matrixdifference(3,1)=0", compact)
        self.assertNotRegex(lower, r"\bclose\s+all\b")
        self.assertIn("findall(groot", lower)
        self.assertIn("'^p08 '", lower)
        self.assertRegex(lower, r"clear run_checks;\s*run_checks;")

        for variable in (
            "rollDampingSweep_Cl_p",
            "weathercockSweep_Cn_beta_perRad",
        ):
            match = re.search(rf"{variable}\s*=\s*\[([^\]]+)\]", experiment)
            self.assertIsNotNone(match, variable)
            values = [float(value) for value in match.group(1).split()]
            self.assertEqual(len(values), 5, variable)
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
        ).lower()

        self.assertIn("clear model;", "\n".join(experiment.splitlines()[:12]).lower())
        self.assertIn("clear model;", "\n".join(interactive_lower.splitlines()[:5]))
        self.assertIn("clear model;", "\n".join(checks_lower.splitlines()[:5]))
        self.assertIn("uifigure(", interactive_lower)
        self.assertIn(
            "existingui=findall(groot,'type','figure','name',"
            "'p08stabilityderivativestomotion')",
            interactive_compact,
        )
        self.assertIn("close(existingui)", interactive_compact)
        self.assertEqual(interactive_lower.count("uislider("), 3)
        for control_definition in (
            "betacontrol=uislider(gridlayout,'limits',[-44],'value',3,",
            "rolldampingcontrol=uislider(gridlayout,'limits',"
            "[-0.8-0.3],'value',-0.50,",
            "weathercockcontrol=uislider(gridlayout,'limits',"
            "[00.24],'value',0.18,",
        ):
            self.assertIn(control_definition, interactive_compact)
        for control in ("initial sideslip", "roll damping", "weathercock"):
            self.assertIn(control, interactive_lower)
        self.assertIn("valuechangingfcn", interactive_lower)
        self.assertIn("valuechangedfcn", interactive_lower)
        self.assertIn("event.value", interactive_lower)
        self.assertIn("modelfcn=@model;", interactive_compact)
        self.assertEqual(interactive_compact.count("out=modelfcn("), 1)
        for axis_name in (
            "axsideslip",
            "axrollrate",
            "axyawrate",
            "axbank",
            "axrollledger",
            "axyawledger",
        ):
            self.assertIn(f"cla({axis_name})", interactive_compact)
        self.assertNotIn("yyaxis", interactive_lower)
        for unit in ("1/rad", "deg/s^2", "deg/s", "m/s", "deg", "s"):
            self.assertIn(unit, interactive_lower)

        self.assertGreaterEqual(checks_lower.count("assert("), 35)
        for concept in (
            "determinism",
            "fixed shape",
            "finite resources",
            "contracting late motion",
            "nondimensional-to-dimensional derivative ledger",
            "rk4 propagation",
            "coefficient sums",
            "limiting inputs",
            "sign symmetry",
            "two isolated derivative sweeps",
            "broken roll-rate normalization",
            "malformed inputs",
            "rejected inputs",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, checks_lower)
        self.assertIn("edgecasecount==8", checks_compact)
        self.assertIn("representativecasecount==27", checks_compact)
        self.assertIn("samplecount==1251", checks_compact)
        self.assertIn(
            "all(laterlateenvelope<0.99*earlierlateenvelope)",
            checks_compact,
        )
        self.assertIn("catch exception", checks_lower)
        self.assertIn(
            "strcmp(exception.identifier,expectedidentifier)", checks_compact
        )
        self.assertIn("P08 checks passed", checks_script)

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
            "eig",
            "expm",
            "lsim",
            "initial",
            "impulse",
            "step",
            "ode45",
            "ode23",
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
        self.assertNotRegex(
            matlab, re.compile(r"^\s*(?:while|parfor)\b", re.MULTILINE)
        )
        self.assertNotRegex(
            matlab, re.compile(r"^\s*(?:global|persistent)\b", re.MULTILINE)
        )
        self.assertNotRegex(matlab, r"\bclose\s+all\b")


class P08IndependentOracleTests(unittest.TestCase):
    def test_baseline_is_deterministic_and_physically_interpretable(self) -> None:
        first = _oracle()
        second = _oracle()
        self.assertEqual(first, second)
        self.assertEqual(first["sample_count"], 1251)
        self.assertEqual(first["time_s"][0], 0.0)
        self.assertEqual(first["time_s"][-1], 25.0)
        self.assertAlmostEqual(first["rate_scale_s"], 0.09083333333333334)
        self.assertEqual(first["first_zero_s"], 0.54)
        self.assertAlmostEqual(first["peak_roll_deg_s"], 3.901553273234694)
        self.assertEqual(first["peak_roll_time_s"], 0.26)
        self.assertAlmostEqual(first["peak_yaw_deg_s"], 7.230284690111148)
        self.assertEqual(first["peak_yaw_time_s"], 0.42)
        self.assertAlmostEqual(first["peak_bank_deg"], 1.6434714052522248)
        self.assertEqual(first["peak_bank_time_s"], 0.66)
        self.assertLessEqual(first["peak_beta_deg"], 5.0)
        self.assertLessEqual(first["peak_bank_deg"], 15.0)
        self.assertLessEqual(first["peak_roll_deg_s"], 15.0)
        self.assertLessEqual(first["peak_yaw_deg_s"], 15.0)

    def test_accepted_derivative_corners_have_contracting_late_motion(self) -> None:
        for cl_p in (-0.8, -0.3):
            for cn_beta in (0.0, 0.24):
                result = _oracle(4.0, cl_p, cn_beta)
                earlier_indices = tuple(
                    index
                    for index, time_s in enumerate(result["time_s"])
                    if 15.0 <= time_s < 20.0
                )
                later_indices = tuple(
                    index
                    for index, time_s in enumerate(result["time_s"])
                    if 20.0 <= time_s <= 25.0
                )
                self.assertEqual(len(earlier_indices), 250)
                self.assertEqual(len(later_indices), 251)
                for key in (
                    "beta_deg",
                    "roll_rate_deg_s",
                    "yaw_rate_deg_s",
                    "bank_deg",
                ):
                    earlier_envelope = max(
                        abs(result[key][index]) for index in earlier_indices
                    )
                    later_envelope = max(
                        abs(result[key][index]) for index in later_indices
                    )
                    with self.subTest(cl_p=cl_p, cn_beta=cn_beta, state=key):
                        self.assertGreater(earlier_envelope, 0.0)
                        self.assertLess(later_envelope, 0.99 * earlier_envelope)

    def test_dimensional_derivatives_and_state_matrix(self) -> None:
        result = _oracle()
        expected_dimensional = {
            "y_beta": -13952.334085403489,
            "y_p": -58.492477511883855,
            "y_r": 487.4373125990321,
            "l_beta": -28076.389205704247,
            "l_p": -10626.1334146589,
            "l_r": 2125.22668293178,
            "n_beta": 42114.583808556374,
            "n_p": -1275.136009759068,
            "n_r": -5313.06670732945,
        }
        for name, expected in expected_dimensional.items():
            with self.subTest(derivative=name):
                self.assertAlmostEqual(result["dimensional"][name], expected)
        expected_matrix = (
            (-0.19378241785282624, -0.0008123955209983868, -0.9932300373250135, 0.16344416666666667),
            (-11.230555682281699, -4.25045336586356, 0.850090673172712, 0.0),
            (10.528645952139094, -0.318784002439767, -1.3282666768323625, 0.0),
            (0.0, 1.0, 0.0, 0.0),
        )
        for actual_row, expected_row in zip(result["matrix"], expected_matrix):
            for actual, expected in zip(actual_row, expected_row):
                self.assertAlmostEqual(actual, expected, places=14)
        self.assertAlmostEqual(result["matrix"][0][2] + 1.0, result["dimensional"]["y_r"] / (1200.0 * 60.0))
        self.assertAlmostEqual(result["matrix"][0][3], 9.80665 / 60.0)
        self.assertEqual(result["matrix"][3], (0.0, 1.0, 0.0, 0.0))

    def test_initial_loads_signs_and_degree_misuse(self) -> None:
        result = _oracle()
        self.assertAlmostEqual(result["side_force_n"][0], -730.5425, places=4)
        self.assertAlmostEqual(result["roll_moment_nm"][0], -1470.0763, places=4)
        self.assertAlmostEqual(result["yaw_moment_nm"][0], 2205.1145, places=4)
        derivative = tuple(math.degrees(value) for value in result["ledger_derivative"][0])
        self.assertAlmostEqual(derivative[0], -0.5813472535584787)
        self.assertAlmostEqual(derivative[1], -33.6916670468451)
        self.assertAlmostEqual(derivative[2], 31.58593785641728)
        self.assertEqual(derivative[3], 0.0)
        degree_misuse_p_dot = (
            result["dimensional"]["l_beta"]
            / result["roll_inertia_kgm2"]
            * 3.0
        )
        correct_p_dot_rad_s2 = result["ledger_derivative"][0][1]
        self.assertAlmostEqual(
            degree_misuse_p_dot / correct_p_dot_rad_s2,
            180.0 / math.pi,
        )

    def test_coefficient_load_and_state_equation_closure(self) -> None:
        result = _oracle()
        for components, total in (
            (result["cy_components"], result["cy_total"]),
            (result["cl_components"], result["cl_total"]),
            (result["cn_components"], result["cn_total"]),
        ):
            for index, expected in enumerate(total):
                self.assertAlmostEqual(
                    sum(component[index] for component in components),
                    expected,
                    places=15,
                )
        for index in range(result["sample_count"]):
            for ledger, matrix_value in zip(
                result["ledger_derivative"][index],
                result["matrix_derivative"][index],
            ):
                self.assertAlmostEqual(ledger, matrix_value, places=14)
            self.assertAlmostEqual(
                result["side_force_n"][index],
                result["force_scale_n"] * result["cy_total"][index],
                places=12,
            )
            self.assertAlmostEqual(
                result["roll_moment_nm"][index],
                result["moment_scale_nm"] * result["cl_total"][index],
                places=11,
            )
            self.assertAlmostEqual(
                result["yaw_moment_nm"][index],
                result["moment_scale_nm"] * result["cn_total"][index],
                places=11,
            )

    def test_every_state_obeys_the_explicit_rk4_recurrence(self) -> None:
        result = _oracle()
        reproduced = _integrate_rk4(
            result["matrix"],
            tuple(row[0] for row in result["state"]),
            result["time_s"],
        )
        self.assertEqual(reproduced, result["state"])
        first_state = tuple(row[0] for row in result["state"])
        step_s = result["time_s"][1] - result["time_s"][0]
        k1 = _matvec(result["matrix"], first_state)
        k2 = _matvec(
            result["matrix"], _state_add(first_state, (step_s / 2.0, k1))
        )
        k3 = _matvec(
            result["matrix"], _state_add(first_state, (step_s / 2.0, k2))
        )
        k4 = _matvec(
            result["matrix"], _state_add(first_state, (step_s, k3))
        )
        expected_second = _state_add(
            first_state,
            (step_s / 6.0, k1),
            (step_s / 3.0, k2),
            (step_s / 3.0, k3),
            (step_s / 6.0, k4),
        )
        self.assertEqual(expected_second, tuple(row[1] for row in result["state"]))

    def test_zero_sign_and_half_scale_limits(self) -> None:
        baseline = _oracle()
        zero = _oracle(0.0, -0.50, 0.18)
        opposite = _oracle(-3.0, -0.50, 0.18)
        half = _oracle(1.5, -0.50, 0.18)
        for key in (
            "beta_deg",
            "roll_rate_deg_s",
            "yaw_rate_deg_s",
            "bank_deg",
            "side_force_n",
            "roll_moment_nm",
            "yaw_moment_nm",
            "broken_roll_deg_s",
            "broken_bank_deg",
        ):
            self.assertEqual(zero[key], (0.0,) * 1251, key)
            for positive, negative, scaled in zip(
                baseline[key], opposite[key], half[key]
            ):
                self.assertAlmostEqual(positive, -negative, places=14)
                self.assertAlmostEqual(scaled, 0.5 * positive, places=14)
        self.assertEqual(zero["first_zero_s"], 0.0)
        self.assertEqual(zero["peak_roll_deg_s"], 0.0)
        self.assertAlmostEqual(
            opposite["peak_roll_deg_s"], baseline["peak_roll_deg_s"]
        )
        self.assertAlmostEqual(
            half["peak_yaw_deg_s"], 0.5 * baseline["peak_yaw_deg_s"]
        )

    def test_zero_weathercock_removes_direct_but_not_coupled_yaw(self) -> None:
        result = _oracle(3.0, -0.50, 0.0)
        self.assertEqual(result["dimensional"]["n_beta"], 0.0)
        self.assertEqual(result["cn_components"][0][0], 0.0)
        self.assertEqual(result["yaw_moment_nm"][0], 0.0)
        self.assertEqual(result["ledger_derivative"][0][2], 0.0)
        self.assertGreater(max(map(abs, result["yaw_rate_deg_s"][1:])), 0.5)
        self.assertEqual(result["first_zero_s"], 1.76)

    def test_two_derivative_sweeps_change_one_matrix_entry_each(self) -> None:
        baseline = _oracle()
        cl_p_values = (-0.30, -0.40, -0.50, -0.65, -0.80)
        roll_results = [_oracle(3.0, value, 0.18) for value in cl_p_values]
        roll_peaks = [result["peak_roll_deg_s"] for result in roll_results]
        bank_peaks = [result["peak_bank_deg"] for result in roll_results]
        initial_p_dot = [result["ledger_derivative"][0][1] for result in roll_results]
        expected_roll_peaks = (
            4.927271898192998,
            4.353351756176826,
            3.901553273234694,
            3.379513108837036,
            2.983204463014648,
        )
        for actual, expected in zip(roll_peaks, expected_roll_peaks):
            self.assertAlmostEqual(actual, expected, places=12)
        self.assertTrue(
            all(left > right for left, right in zip(roll_peaks, roll_peaks[1:]))
        )
        self.assertTrue(
            all(left > right for left, right in zip(bank_peaks, bank_peaks[1:]))
        )
        self.assertEqual(initial_p_dot, [initial_p_dot[0]] * len(initial_p_dot))
        for result in roll_results:
            changed = [
                (row, column)
                for row in range(4)
                for column in range(4)
                if result["matrix"][row][column]
                != baseline["matrix"][row][column]
            ]
            if result["roll_damping_derivative_cl_p"] == -0.50:
                self.assertEqual(changed, [])
            else:
                self.assertEqual(changed, [(1, 1)])

        cn_beta_values = (0.0, 0.06, 0.12, 0.18, 0.24)
        yaw_results = [_oracle(3.0, -0.50, value) for value in cn_beta_values]
        initial_r_dot = [result["ledger_derivative"][0][2] for result in yaw_results]
        zero_crossings = [result["first_zero_s"] for result in yaw_results]
        yaw_peaks = [result["peak_yaw_deg_s"] for result in yaw_results]
        self.assertEqual(zero_crossings, [1.76, 0.9, 0.66, 0.54, 0.46])
        self.assertEqual(initial_r_dot[0], 0.0)
        self.assertTrue(
            all(left < right for left, right in zip(initial_r_dot, initial_r_dot[1:]))
        )
        self.assertTrue(
            all(left > right for left, right in zip(zero_crossings, zero_crossings[1:]))
        )
        self.assertTrue(
            all(left < right for left, right in zip(yaw_peaks, yaw_peaks[1:]))
        )
        for result in yaw_results:
            changed = [
                (row, column)
                for row in range(4)
                for column in range(4)
                if result["matrix"][row][column]
                != baseline["matrix"][row][column]
            ]
            if result["weathercock_derivative_cn_beta_per_rad"] == 0.18:
                self.assertEqual(changed, [])
            else:
                self.assertEqual(changed, [(2, 0)])

    def test_broken_rate_normalization_is_smooth_but_wrong(self) -> None:
        result = _oracle()
        self.assertAlmostEqual(result["broken_numeric_nm"], -116984.9550237677)
        self.assertAlmostEqual(result["broken_matrix"][1][1], -46.79398200950708)
        self.assertAlmostEqual(
            result["broken_a22_numeric_ratio_per_s"], 120.0 / 10.9
        )
        for row in range(4):
            for column in range(4):
                if (row, column) == (1, 1):
                    continue
                self.assertEqual(
                    result["broken_matrix"][row][column],
                    result["matrix"][row][column],
                )
        self.assertAlmostEqual(
            result["broken_peak_roll_deg_s"], 0.6475833548151798
        )
        self.assertAlmostEqual(
            result["broken_peak_bank_deg"], 0.18417013306244692
        )
        self.assertGreater(
            result["peak_roll_deg_s"], 5.0 * result["broken_peak_roll_deg_s"]
        )
        self.assertGreater(
            result["peak_bank_deg"], 8.0 * result["broken_peak_bank_deg"]
        )
        self.assertTrue(
            all(math.isfinite(value) for value in result["broken_roll_deg_s"])
        )
        self.assertTrue(
            all(math.isfinite(value) for value in result["broken_bank_deg"])
        )

    def test_malformed_inputs_fail_without_poisoning_recovery(self) -> None:
        baseline = _oracle()
        malformed_cases = (
            ("sideslip below", (-4.01, -0.50, 0.18)),
            ("sideslip above", (4.01, -0.50, 0.18)),
            ("roll damping below", (3.0, -0.801, 0.18)),
            ("roll damping above", (3.0, -0.299, 0.18)),
            ("weathercock below", (3.0, -0.50, -0.001)),
            ("weathercock above", (3.0, -0.50, 0.241)),
            ("nan sideslip", (math.nan, -0.50, 0.18)),
            ("infinite damping", (3.0, math.inf, 0.18)),
            ("list weathercock", (3.0, -0.50, [0.18])),
            ("complex sideslip", (3.0 + 1j, -0.50, 0.18)),
            ("text damping", (3.0, "damped", 0.18)),
            ("boolean weathercock", (3.0, -0.50, True)),
        )
        for name, values in malformed_cases:
            with self.subTest(case=name), self.assertRaises(ValueError):
                _oracle(*values)
        self.assertEqual(_oracle(), baseline)

    def test_all_accepted_corners_are_finite_and_bounded(self) -> None:
        case_count = 0
        for beta0_deg in (-4.0, 4.0):
            for cl_p in (-0.8, -0.3):
                for cn_beta in (0.0, 0.24):
                    result = _oracle(beta0_deg, cl_p, cn_beta)
                    case_count += 1
                    self.assertEqual(result["sample_count"], 1251)
                    self.assertLessEqual(result["peak_beta_deg"], 5.0)
                    self.assertLessEqual(result["peak_bank_deg"], 15.0)
                    self.assertLessEqual(result["peak_roll_deg_s"], 15.0)
                    self.assertLessEqual(result["peak_yaw_deg_s"], 15.0)
                    for key in (
                        "beta_deg",
                        "roll_rate_deg_s",
                        "yaw_rate_deg_s",
                        "bank_deg",
                        "side_force_n",
                        "roll_moment_nm",
                        "yaw_moment_nm",
                        "broken_roll_deg_s",
                        "broken_bank_deg",
                    ):
                        self.assertTrue(
                            all(math.isfinite(value) for value in result[key]),
                            (beta0_deg, cl_p, cn_beta, key),
                        )
        self.assertEqual(case_count, 8)

    def test_representative_grid_is_resource_bounded(self) -> None:
        case_count = 0
        for beta0_deg in (-4.0, 0.0, 4.0):
            for cl_p in (-0.8, -0.50, -0.3):
                for cn_beta in (0.0, 0.18, 0.24):
                    result = _oracle(beta0_deg, cl_p, cn_beta)
                    case_count += 1
                    self.assertEqual(len(result["time_s"]), 1251)
                    self.assertEqual(len(result["state"]), 4)
                    self.assertTrue(
                        all(len(row) == 1251 for row in result["state"])
                    )
        self.assertEqual(case_count, 27)
        self.assertLessEqual(case_count, 30)


if __name__ == "__main__":
    unittest.main()
