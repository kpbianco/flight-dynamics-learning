from __future__ import annotations

import json
import math
import re
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MODULE_ID = "P09"
MODULE_FOLDER = ROOT / "modules/09-integrate-6-dof-equations"
GUIDING_QUESTION = (
    "What inputs, observable effects, and failure modes matter when you integrate "
    "6-DOF Equations?"
)


Vector = tuple[float, ...]
Matrix3 = tuple[tuple[float, float, float], ...]


def _finite_scale(name: str, value: object) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a finite real scalar")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{name} must be a finite real scalar")
    if not 0.0 <= result <= 1.5:
        raise ValueError(f"{name} must be between 0 and 1.5 inclusive")
    return result


def _add(*terms: tuple[float, Vector]) -> Vector:
    length = len(terms[0][1])
    return tuple(
        sum(scale * vector[index] for scale, vector in terms)
        for index in range(length)
    )


def _dot(left: Vector, right: Vector) -> float:
    return sum(a * b for a, b in zip(left, right))


def _norm(vector: Vector) -> float:
    return math.sqrt(_dot(vector, vector))


def _cross(left: Vector, right: Vector) -> Vector:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _matvec(matrix: Matrix3, vector: Vector) -> Vector:
    return tuple(_dot(row, vector) for row in matrix)


def _transpose(matrix: Matrix3) -> Matrix3:
    return tuple(tuple(matrix[row][column] for row in range(3)) for column in range(3))


def _matrix_product(left: Matrix3, right: Matrix3) -> Matrix3:
    right_transpose = _transpose(right)
    return tuple(
        tuple(_dot(left_row, right_column) for right_column in right_transpose)
        for left_row in left
    )


def _determinant3(matrix: Matrix3) -> float:
    return (
        matrix[0][0]
        * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
        - matrix[0][1]
        * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
        + matrix[0][2]
        * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0])
    )


def _normalize_quaternion(quaternion: Vector) -> Vector:
    magnitude = _norm(quaternion)
    if not math.isfinite(magnitude) or magnitude <= 0.0:
        raise ValueError("quaternion must have a positive finite norm")
    return tuple(component / magnitude for component in quaternion)


def _quaternion_product(left: Vector, right: Vector) -> Vector:
    left_scalar = left[0]
    right_scalar = right[0]
    left_vector = left[1:]
    right_vector = right[1:]
    vector_part = _add(
        (left_scalar, right_vector),
        (right_scalar, left_vector),
        (1.0, _cross(left_vector, right_vector)),
    )
    return (-_dot(left_vector, right_vector) + left_scalar * right_scalar, *vector_part)


def _body_to_ned_dcm(quaternion: Vector) -> Matrix3:
    q0, q1, q2, q3 = _normalize_quaternion(quaternion)
    return (
        (
            q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3,
            2.0 * (q1 * q2 - q0 * q3),
            2.0 * (q1 * q3 + q0 * q2),
        ),
        (
            2.0 * (q1 * q2 + q0 * q3),
            q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3,
            2.0 * (q2 * q3 - q0 * q1),
        ),
        (
            2.0 * (q1 * q3 - q0 * q2),
            2.0 * (q2 * q3 + q0 * q1),
            q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3,
        ),
    )


def _euler_321_deg(quaternion: Vector) -> Vector:
    dcm = _body_to_ned_dcm(quaternion)
    roll = math.atan2(dcm[2][1], dcm[2][2])
    pitch_argument = max(-1.0, min(1.0, -dcm[2][0]))
    pitch = math.asin(pitch_argument)
    yaw = math.atan2(dcm[1][0], dcm[0][0])
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


def _half_sine(time_s: float, duration_s: float) -> float:
    if 0.0 <= time_s < duration_s:
        return math.sin(math.pi * time_s / duration_s)
    return 0.0


def _loads(
    time_s: float,
    force_pulse_scale: float,
    moment_pulse_scale: float,
) -> tuple[Vector, Vector]:
    mass_kg = 1200.0
    gravity_mps2 = 9.80665
    force_shape = _half_sine(time_s, 1.5)
    moment_shape = _half_sine(time_s, 1.0)
    force_body_n = (
        2400.0 * force_pulse_scale * force_shape,
        0.0,
        -mass_kg * gravity_mps2,
    )
    moment_body_nm = tuple(
        moment_pulse_scale * component * moment_shape
        for component in (500.0, 700.0, 350.0)
    )
    return force_body_n, moment_body_nm


def _state_derivative(
    time_s: float,
    state: Vector,
    force_pulse_scale: float,
    moment_pulse_scale: float,
    *,
    omit_rotating_frame_term: bool = False,
) -> Vector:
    mass_kg = 1200.0
    gravity_ned_mps2 = (0.0, 0.0, 9.80665)
    inertia_diagonal_kgm2 = (2500.0, 3000.0, 4000.0)

    velocity_body_mps = state[3:6]
    quaternion_body_to_ned = _normalize_quaternion(state[6:10])
    body_rate_rad_s = state[10:13]
    body_to_ned = _body_to_ned_dcm(quaternion_body_to_ned)
    force_body_n, moment_body_nm = _loads(
        time_s, force_pulse_scale, moment_pulse_scale
    )

    position_rate_ned_mps = _matvec(body_to_ned, velocity_body_mps)
    gravity_body_mps2 = _matvec(_transpose(body_to_ned), gravity_ned_mps2)
    rotating_frame_acceleration_mps2 = _cross(
        body_rate_rad_s, velocity_body_mps
    )
    velocity_rate_body_mps2 = _add(
        (1.0 / mass_kg, force_body_n),
        (1.0, gravity_body_mps2),
        (
            0.0 if omit_rotating_frame_term else -1.0,
            rotating_frame_acceleration_mps2,
        ),
    )

    quaternion_rate = tuple(
        0.5 * value
        for value in _quaternion_product(
            quaternion_body_to_ned, (0.0, *body_rate_rad_s)
        )
    )
    angular_momentum_body = tuple(
        inertia * rate
        for inertia, rate in zip(inertia_diagonal_kgm2, body_rate_rad_s)
    )
    gyroscopic_moment_nm = _cross(body_rate_rad_s, angular_momentum_body)
    body_rate_derivative_rad_s2 = tuple(
        (moment - gyroscopic) / inertia
        for moment, gyroscopic, inertia in zip(
            moment_body_nm, gyroscopic_moment_nm, inertia_diagonal_kgm2
        )
    )
    return (
        *position_rate_ned_mps,
        *velocity_rate_body_mps2,
        *quaternion_rate,
        *body_rate_derivative_rad_s2,
    )


def _normalize_state_quaternion(state: Vector) -> Vector:
    return (*state[:6], *_normalize_quaternion(state[6:10]), *state[10:])


def _integrate_fixed_rk4(
    force_pulse_scale: float,
    moment_pulse_scale: float,
    *,
    omit_rotating_frame_term: bool = False,
) -> tuple[Vector, tuple[Vector, ...]]:
    time_s = tuple(index * 0.02 for index in range(301))
    state = (
        0.0,
        0.0,
        0.0,
        60.0,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    samples = [state]
    for index in range(len(time_s) - 1):
        time = time_s[index]
        step_s = time_s[index + 1] - time
        k1 = _state_derivative(
            time,
            state,
            force_pulse_scale,
            moment_pulse_scale,
            omit_rotating_frame_term=omit_rotating_frame_term,
        )
        k2 = _state_derivative(
            time + 0.5 * step_s,
            _add((1.0, state), (0.5 * step_s, k1)),
            force_pulse_scale,
            moment_pulse_scale,
            omit_rotating_frame_term=omit_rotating_frame_term,
        )
        k3 = _state_derivative(
            time + 0.5 * step_s,
            _add((1.0, state), (0.5 * step_s, k2)),
            force_pulse_scale,
            moment_pulse_scale,
            omit_rotating_frame_term=omit_rotating_frame_term,
        )
        k4 = _state_derivative(
            time + step_s,
            _add((1.0, state), (step_s, k3)),
            force_pulse_scale,
            moment_pulse_scale,
            omit_rotating_frame_term=omit_rotating_frame_term,
        )
        state = _normalize_state_quaternion(
            _add(
                (1.0, state),
                (step_s / 6.0, k1),
                (step_s / 3.0, k2),
                (step_s / 3.0, k3),
                (step_s / 6.0, k4),
            )
        )
        samples.append(state)
    return time_s, tuple(samples)


def _transpose_samples(samples: tuple[Vector, ...]) -> tuple[Vector, ...]:
    return tuple(tuple(sample[index] for sample in samples) for index in range(13))


def _oracle(
    force_pulse_scale: object = 1.0,
    moment_pulse_scale: object = 1.0,
) -> dict[str, object]:
    """Pure-stdlib equation oracle independent of the MATLAB source."""
    force_scale = _finite_scale("force pulse scale", force_pulse_scale)
    moment_scale = _finite_scale("moment pulse scale", moment_pulse_scale)
    time_s, samples = _integrate_fixed_rk4(force_scale, moment_scale)
    _, broken_samples = _integrate_fixed_rk4(
        force_scale, moment_scale, omit_rotating_frame_term=True
    )
    state = _transpose_samples(samples)
    broken_state = _transpose_samples(broken_samples)

    position_ned_m = state[0:3]
    velocity_body_mps = state[3:6]
    quaternion_body_to_ned = state[6:10]
    body_rate_rad_s = state[10:13]
    body_rate_deg_s = tuple(
        tuple(math.degrees(value) for value in row) for row in body_rate_rad_s
    )
    quaternions = tuple(sample[6:10] for sample in samples)
    rotation_matrices = tuple(_body_to_ned_dcm(q) for q in quaternions)
    euler_deg = tuple(_euler_321_deg(q) for q in quaternions)
    euler_rows_deg = tuple(
        tuple(sample[index] for sample in euler_deg) for index in range(3)
    )
    inertial_velocity_ned_mps = tuple(
        _matvec(matrix, sample[3:6])
        for matrix, sample in zip(rotation_matrices, samples)
    )
    speed_mps = tuple(_norm(sample[3:6]) for sample in samples)
    attitude_rotation_deg = tuple(
        math.degrees(2.0 * math.acos(min(1.0, abs(quaternion[0]))))
        for quaternion in quaternions
    )
    force_body_n = tuple(
        _loads(time, force_scale, moment_scale)[0] for time in time_s
    )
    moment_body_nm = tuple(
        _loads(time, force_scale, moment_scale)[1] for time in time_s
    )
    derivatives = tuple(
        _state_derivative(time, sample, force_scale, moment_scale)
        for time, sample in zip(time_s, samples)
    )

    quaternion_norm = tuple(_norm(quaternion) for quaternion in quaternions)
    dcm_orthogonality_residual = []
    dcm_determinant = []
    for matrix in rotation_matrices:
        product = _matrix_product(_transpose(matrix), matrix)
        residual = math.sqrt(
            sum(
                (
                    product[row][column]
                    - (1.0 if row == column else 0.0)
                )
                ** 2
                for row in range(3)
                for column in range(3)
            )
        )
        dcm_orthogonality_residual.append(residual)
        dcm_determinant.append(_determinant3(matrix))

    inertia_diagonal_kgm2 = (2500.0, 3000.0, 4000.0)
    angular_momentum_ned = tuple(
        _matvec(
            matrix,
            tuple(
                inertia * rate
                for inertia, rate in zip(
                    inertia_diagonal_kgm2, sample[10:13]
                )
            ),
        )
        for matrix, sample in zip(rotation_matrices, samples)
    )
    rotational_energy_j = tuple(
        0.5
        * sum(
            inertia * rate * rate
            for inertia, rate in zip(
                inertia_diagonal_kgm2, sample[10:13]
            )
        )
        for sample in samples
    )
    post_pulse_start = time_s.index(1.0)
    reference_angular_momentum = angular_momentum_ned[post_pulse_start]
    reference_angular_momentum_norm = _norm(reference_angular_momentum)
    if reference_angular_momentum_norm == 0.0:
        post_pulse_angular_momentum_relative_drift = 0.0
    else:
        post_pulse_angular_momentum_relative_drift = max(
            _norm(
                tuple(
                    value - reference
                    for value, reference in zip(
                        sample, reference_angular_momentum
                    )
                )
            )
            for sample in angular_momentum_ned[post_pulse_start:]
        ) / reference_angular_momentum_norm

    broken_residual_mps2 = tuple(
        _norm(_cross(sample[10:13], sample[3:6]))
        for sample in broken_samples
    )
    trajectory_separation_m = tuple(
        _norm(
            tuple(
                broken_sample[index] - normal_sample[index]
                for index in range(3)
            )
        )
        for normal_sample, broken_sample in zip(samples, broken_samples)
    )

    return {
        "force_pulse_scale": force_scale,
        "moment_pulse_scale": moment_scale,
        "time_s": time_s,
        "sample_count": len(time_s),
        "integration_step_s": 0.02,
        "derivative_evaluation_count": 4 * (len(time_s) - 1),
        "samples": samples,
        "state": state,
        "position_ned_m": position_ned_m,
        "velocity_body_mps": velocity_body_mps,
        "quaternion_body_to_ned": quaternion_body_to_ned,
        "body_rate_rad_s": body_rate_rad_s,
        "body_rate_deg_s": body_rate_deg_s,
        "euler_deg": euler_rows_deg,
        "rotation_matrices": rotation_matrices,
        "inertial_velocity_ned_mps": inertial_velocity_ned_mps,
        "speed_mps": speed_mps,
        "attitude_rotation_deg": attitude_rotation_deg,
        "force_body_n": force_body_n,
        "moment_body_nm": moment_body_nm,
        "derivatives": derivatives,
        "quaternion_norm": quaternion_norm,
        "dcm_orthogonality_residual": tuple(dcm_orthogonality_residual),
        "dcm_determinant": tuple(dcm_determinant),
        "angular_momentum_ned": angular_momentum_ned,
        "rotational_energy_j": rotational_energy_j,
        "post_pulse_angular_momentum_relative_drift": (
            post_pulse_angular_momentum_relative_drift
        ),
        "broken_samples": broken_samples,
        "broken_state": broken_state,
        "broken_residual_mps2": broken_residual_mps2,
        "trajectory_separation_m": trajectory_separation_m,
        "final_position_ned_m": tuple(row[-1] for row in position_ned_m),
        "final_velocity_body_mps": tuple(row[-1] for row in velocity_body_mps),
        "final_euler_deg": tuple(row[-1] for row in euler_rows_deg),
        "final_speed_mps": speed_mps[-1],
        "peak_speed_mps": max(speed_mps),
        "peak_body_rate_deg_s": max(
            _norm(tuple(row[index] for row in body_rate_deg_s))
            for index in range(len(time_s))
        ),
        "peak_attitude_rotation_deg": max(attitude_rotation_deg),
        "max_broken_residual_mps2": max(broken_residual_mps2),
        "final_trajectory_separation_m": trajectory_separation_m[-1],
    }


class P09ArtifactTests(unittest.TestCase):
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
                "number": 9,
                "id": "P09",
                "title": "Integrate 6-DOF Equations",
                "guiding_question": GUIDING_QUESTION,
                "phase": 3,
                "phase_title": "Six-degree-of-freedom simulation",
                "slug": "integrate-6-dof-equations",
                "folder": "modules/09-integrate-6-dof-equations",
                "implementation_batch": "P09",
                "prerequisites": ["P08"],
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
            "p08",
            "six-degree-of-freedom",
            "body",
            "ned",
            "force",
            "moment",
            "position",
            "velocity",
            "body rate",
            "quaternion",
            "gravity",
            "rotating-frame",
            "omega cross (i omega)",
            "rk4",
            "normalize",
            "mechanism",
            "reset",
            "teach-back",
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
            "functionout=model(forcepulsescale,momentpulsescale)", compact
        )
        self.assertIn("arguments", lower)
        self.assertEqual(
            compact.count("(1,1)double{mustbereal,mustbefinite}=1"), 2
        )
        self.assertIn("forcepulsescale<0||forcepulsescale>1.5", compact)
        self.assertIn("momentpulsescale<0||momentpulsescale>1.5", compact)
        for identifier in (
            "P09:model:ForcePulseScaleRange",
            "P09:model:MomentPulseScaleRange",
        ):
            self.assertIn(identifier, model)

        for expression in (
            "mass_kg=1200;",
            "gravity_mps2=9.80665;",
            "inertiatensor_kgm2=diag([250030004000]);",
            "initialforwardspeed_mps=60;",
            "forwardforceamplitude_n=2400;",
            "basemomentamplitude_nm=[500;700;350];",
            "forcepulseduration_s=1.5;",
            "momentpulseduration_s=1.0;",
            "integrationstep_s=0.02;",
            "timehorizon_s=6;",
            "time_s=0:integrationstep_s:timehorizon_s;",
            "initialstate=[zeros(3,1);initialforwardspeed_mps;0;0;1;0;0;0;0;0;0];",
            "positionderivativened_mps=bodytoned*bodyvelocity_mps;",
            "gravitybody_mps2=bodytoned'*parameters.gravityned_mps2;",
            "transportacceleration_mps2=cross(angularvelocity_rad_s,bodyvelocity_mps);",
            "velocityderivativebody_mps2=forcebody_n/parameters.mass_kg+gravitybody_mps2-transportacceleration_mps2;",
            "angularmomentumbody_nms=parameters.inertiatensor_kgm2*angularvelocity_rad_s;",
            "angularaccelerationbody_rad_s2=parameters.inertiatensor_kgm2\\(momentbody_nm-cross(angularvelocity_rad_s,angularmomentumbody_nms));",
            "forcepulse=halfsinepulse(time_s,parameters.forcepulseduration_s);",
            "momentpulse=halfsinepulse(time_s,parameters.momentpulseduration_s);",
            "parameters.forwardforceamplitude_n*parameters.forcepulsescale*forcepulse;",
            "parameters.basemomentamplitude_nm*parameters.momentpulsescale*momentpulse;",
            "iftime_s>=0&&time_s<duration_s",
            "value=sin(pi*time_s/duration_s);",
        ):
            with self.subTest(expression=expression):
                self.assertIn(expression, compact)

        for quaternion_term in (
            "-q1*p-q2*q-q3*r",
            "q0*p+q2*r-q3*q",
            "q0*q+q3*p-q1*r",
            "q0*r+q1*q-q2*p",
        ):
            self.assertIn(quaternion_term, compact)
        self.assertIn("quaternionderivative=0.5*[", compact)

        for stage in (
            "k1=rigidbodyderivative(time_s(k),state(:,k),parameters,omittransportterm);",
            "k2=rigidbodyderivative(time_s(k)+0.5*step_s,state(:,k)+0.5*step_s*k1,parameters,omittransportterm);",
            "k3=rigidbodyderivative(time_s(k)+0.5*step_s,state(:,k)+0.5*step_s*k2,parameters,omittransportterm);",
            "k4=rigidbodyderivative(time_s(k)+step_s,state(:,k)+step_s*k3,parameters,omittransportterm);",
            "nextstate=state(:,k)+step_s/6*(k1+2*k2+2*k3+k4);",
            "nextstate(7:10)=normalizequaternion(candidatequaternion);",
            "quaternionbodytoned=normalizequaternion(state(7:10));",
        ):
            with self.subTest(stage=stage):
                self.assertIn(stage, compact)

        self.assertIn(
            "[state,quaternionprojectioncorrection]=integratefixedrk4(initialstate,time_s,parameters,false);",
            compact,
        )
        self.assertIn(
            "[brokenstate,brokenquaternionprojectioncorrection]=integratefixedrk4(initialstate,time_s,parameters,true);",
            compact,
        )
        self.assertIn(
            "referenceangularmomentumnorm=norm(referenceangularmomentum);",
            compact,
        )
        self.assertIn("ifreferenceangularmomentumnorm==0", compact)
        self.assertNotIn("ifmomentpulsescale==0", compact)
        self.assertEqual(compact.count("ifomittransportterm"), 1)
        self.assertIn(
            "velocityderivativebody_mps2=forcebody_n/parameters.mass_kg+gravitybody_mps2;",
            compact,
        )

        for dcm_term in (
            "q0^2+q1^2-q2^2-q3^2",
            "2*(q1*q2-q0*q3)",
            "2*(q1*q3+q0*q2)",
            "2*(q1*q2+q0*q3)",
            "2*(q2*q3-q0*q1)",
            "2*(q1*q3-q0*q2)",
            "2*(q2*q3+q0*q1)",
        ):
            self.assertIn(dcm_term, compact)

        for field in (
            "positionNED_m",
            "velocityBody_mps",
            "quaternionBodyToNED",
            "bodyRates_rad_s",
            "bodyRates_deg_s",
            "eulerAngles_deg",
            "appliedForceBody_N",
            "appliedMomentBody_Nm",
            "angularMomentumNED_Nms",
            "translationalEquationResidual_mps2",
            "rotationalEquationResidual_Nm",
            "dcmOrthonormalityError",
            "dcmDeterminantError",
            "brokenPhysicalTransportResidual_mps2",
            "brokenPositionError_m",
        ):
            with self.subTest(field=field):
                self.assertIn(f"'{field}'", model)

        self.assertIn("body x forward, y right, z down", lower)
        self.assertIn("local navigation is ned", lower)
        self.assertIn("f_body excludes gravity", lower)
        self.assertIn("no aerodynamics", lower)
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
        self.assertIn("force pulse", lower)
        self.assertIn("moment pulse", lower)
        self.assertIn("broken", lower)
        self.assertIn("rotating-frame", lower)
        self.assertGreaterEqual(lower.count("figure("), 5)
        self.assertIn("xlabel", lower)
        self.assertIn("ylabel", lower)
        for unit in (
            "m/s^2",
            "deg/s",
            "m/s",
            "kg*m^2",
            "n*m",
            "deg",
            "m",
            "s",
        ):
            with self.subTest(unit=unit):
                self.assertIn(unit, lower)
        self.assertIn("fprintf", lower)
        self.assertGreaterEqual(lower.count("assert("), 5)
        self.assertRegex(
            compact,
            r"model\(force(?:pulse)?scalesweep\w*\(k\),1\)",
        )
        self.assertRegex(
            compact,
            r"model\(1,moment(?:pulse)?scalesweep\w*\(k\)\)",
        )
        self.assertNotRegex(lower, r"\bclose\s+all\b")
        self.assertIn("findall(groot", lower)
        self.assertIn("'^p09 '", lower)
        self.assertRegex(lower, r"clear run_checks;\s*run_checks;")

        sweep_assignments = re.findall(
            r"(?:force|moment)(?:Pulse)?ScaleSweep\w*\s*=\s*\[([^\]]+)\]",
            experiment,
            re.IGNORECASE,
        )
        self.assertEqual(len(sweep_assignments), 2)
        for values_text in sweep_assignments:
            values = [
                float(value)
                for value in re.findall(r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)", values_text)
            ]
            self.assertGreaterEqual(len(values), 5)
            self.assertTrue(all(math.isfinite(value) for value in values))
            self.assertTrue(all(0.0 <= value <= 1.5 for value in values))
            self.assertIn(1.0, values)

    def test_interaction_checks_recovery_and_resource_bounds(self) -> None:
        experiment = self.text["experiment.m"]
        interactive = self.text["interactive.m"]
        checks_script = self.text["run_checks.m"]
        checks_doc = self.text["checks.md"]
        interactive_lower = interactive.lower()
        checks_lower = checks_script.lower()
        combined_checks_lower = f"{checks_lower}\n{checks_doc.lower()}"
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
        self.assertIn("p09", interactive_lower)
        self.assertIn("existingui=findall(groot", interactive_compact)
        self.assertIn("close(existingui)", interactive_compact)
        self.assertEqual(interactive_lower.count("uislider("), 2)
        self.assertEqual(interactive_compact.count("'limits',[01.5]"), 2)
        self.assertEqual(interactive_compact.count("'value',1"), 2)
        for control in ("forward-force", "moment"):
            self.assertIn(control, interactive_lower)
        self.assertIn("valuechangingfcn", interactive_lower)
        self.assertIn("valuechangedfcn", interactive_lower)
        self.assertIn("event.value", interactive_lower)
        self.assertIn("modelfcn=@model;", interactive_compact)
        self.assertEqual(interactive_compact.count("out=modelfcn("), 1)
        self.assertGreaterEqual(interactive_compact.count("cla("), 4)
        self.assertNotIn("yyaxis", interactive_lower)
        for unit in ("m/s^2", "deg/s", "m/s", "deg", "m", "s"):
            self.assertIn(unit, interactive_lower)

        self.assertGreaterEqual(checks_lower.count("assert("), 25)
        for concept in (
            "determinism",
            "fixed shape",
            "finite resources",
            "all 13 state equations",
            "rk4 recurrence",
            "quaternion",
            "orthonormal",
            "straight",
            "force-only",
            "angular momentum",
            "isolated sweeps",
            "broken rotating-frame",
            "malformed inputs",
            "rejected inputs",
        ):
            with self.subTest(concept=concept):
                self.assertIn(concept, combined_checks_lower)
        self.assertIn("samplecount==301", checks_compact)
        self.assertRegex(checks_compact, r"representativecasecount==9\b")
        self.assertIn(
            "smallestpositivemoment=model(1,eps(0));", checks_compact
        )
        self.assertIn(
            "isfinite(smallestpositivemoment.postpulseangularmomentumrelativedrift)",
            checks_compact,
        )
        self.assertIn("rotationalenergy_j=0.5*sum(", checks_compact)
        self.assertIn("relativeenergydrift<1e-11", checks_compact)
        self.assertIn("catch exception", checks_lower)
        self.assertIn(
            "strcmp(exception.identifier,expectedidentifier)", checks_compact
        )
        self.assertIn("P09 checks passed", checks_script)
        self.assertIn("timeout", checks_doc.lower())
        self.assertIn("cancellation", checks_doc.lower())
        self.assertRegex(
            checks_doc.lower(), r"not applicable|no background|synchronous"
        )

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
            "ode15s",
            "quat2dcm",
            "quatmultiply",
            "quatnormalize",
            "quat2eul",
            "eul2quat",
            "angle2dcm",
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


class P09IndependentOracleTests(unittest.TestCase):
    def test_baseline_is_deterministic_and_recognizable(self) -> None:
        first = _oracle()
        second = _oracle()
        self.assertEqual(first, second)
        self.assertEqual(first["sample_count"], 301)
        self.assertEqual(first["time_s"][0], 0.0)
        self.assertEqual(first["time_s"][-1], 6.0)
        self.assertEqual(first["integration_step_s"], 0.02)
        self.assertEqual(first["derivative_evaluation_count"], 1200)

        expected_final_position = (
            328.49161043194323,
            28.829715914526187,
            13.362100336243557,
        )
        expected_final_velocity_body = (
            25.743967741234474,
            18.05862836267793,
            29.693106942973788,
        )
        expected_final_euler = (
            53.02105587097014,
            37.780743706893965,
            37.440862000307895,
        )
        for actual, expected in zip(
            first["final_position_ned_m"], expected_final_position
        ):
            self.assertAlmostEqual(actual, expected, places=11)
        for actual, expected in zip(
            first["final_velocity_body_mps"], expected_final_velocity_body
        ):
            self.assertAlmostEqual(actual, expected, places=11)
        for actual, expected in zip(first["final_euler_deg"], expected_final_euler):
            self.assertAlmostEqual(actual, expected, places=11)
        self.assertAlmostEqual(first["peak_speed_mps"], 61.331578859253995)
        self.assertAlmostEqual(first["final_speed_mps"], 43.2498154138247)
        self.assertAlmostEqual(
            first["peak_body_rate_deg_s"], 11.652319550297378
        )
        self.assertAlmostEqual(
            first["peak_attitude_rotation_deg"], 63.9572808763137
        )
        self.assertAlmostEqual(
            first["max_broken_residual_mps2"], 9.781530671020718
        )
        self.assertAlmostEqual(
            first["final_trajectory_separation_m"], 132.7917830719381
        )

    def test_fixed_shapes_finite_values_and_resource_boundary(self) -> None:
        result = _oracle()
        self.assertEqual(len(result["samples"]), 301)
        self.assertTrue(all(len(sample) == 13 for sample in result["samples"]))
        self.assertEqual(len(result["state"]), 13)
        self.assertTrue(all(len(row) == 301 for row in result["state"]))
        self.assertEqual(len(result["broken_state"]), 13)
        self.assertTrue(all(len(row) == 301 for row in result["broken_state"]))
        self.assertEqual(len(result["rotation_matrices"]), 301)
        self.assertEqual(len(result["force_body_n"]), 301)
        self.assertEqual(len(result["moment_body_nm"]), 301)
        self.assertEqual(len(result["derivatives"]), 301)
        self.assertEqual(len(result["trajectory_separation_m"]), 301)

        for key in ("samples", "broken_samples", "derivatives"):
            for sample in result[key]:
                self.assertTrue(all(math.isfinite(value) for value in sample), key)
        for key in (
            "position_ned_m",
            "velocity_body_mps",
            "quaternion_body_to_ned",
            "body_rate_rad_s",
            "body_rate_deg_s",
            "euler_deg",
        ):
            for row in result[key]:
                self.assertEqual(len(row), 301, key)
                self.assertTrue(all(math.isfinite(value) for value in row), key)
        for key in (
            "quaternion_norm",
            "dcm_orthogonality_residual",
            "dcm_determinant",
            "attitude_rotation_deg",
            "rotational_energy_j",
            "broken_residual_mps2",
            "trajectory_separation_m",
        ):
            self.assertEqual(len(result[key]), 301, key)
            self.assertTrue(all(math.isfinite(value) for value in result[key]), key)

    def test_scalar_first_quaternion_and_dcm_invariants(self) -> None:
        result = _oracle()
        self.assertEqual(tuple(row[0] for row in result["quaternion_body_to_ned"]), (1.0, 0.0, 0.0, 0.0))
        self.assertLess(
            max(abs(value - 1.0) for value in result["quaternion_norm"]),
            3e-16,
        )
        self.assertLess(max(result["dcm_orthogonality_residual"]), 2e-15)
        self.assertLess(
            max(abs(value - 1.0) for value in result["dcm_determinant"]),
            2e-15,
        )

        for matrix in result["rotation_matrices"]:
            transpose_product = _matrix_product(_transpose(matrix), matrix)
            for row in range(3):
                for column in range(3):
                    self.assertAlmostEqual(
                        transpose_product[row][column],
                        1.0 if row == column else 0.0,
                        places=14,
                    )
            self.assertAlmostEqual(_determinant3(matrix), 1.0, places=14)

        pitch_rad = math.radians(10.0)
        positive_pitch = (
            math.cos(0.5 * pitch_rad),
            0.0,
            math.sin(0.5 * pitch_rad),
            0.0,
        )
        inertial_forward = _matvec(
            _body_to_ned_dcm(positive_pitch), (1.0, 0.0, 0.0)
        )
        self.assertGreater(inertial_forward[0], 0.0)
        self.assertEqual(inertial_forward[1], 0.0)
        self.assertLess(inertial_forward[2], 0.0)

    def test_loads_and_all_state_equations_close_independently(self) -> None:
        result = _oracle()
        mass_kg = 1200.0
        gravity_ned_mps2 = (0.0, 0.0, 9.80665)
        inertia = (2500.0, 3000.0, 4000.0)
        for index, (time_s, sample, derivative, matrix) in enumerate(
            zip(
                result["time_s"],
                result["samples"],
                result["derivatives"],
                result["rotation_matrices"],
            )
        ):
            force_body_n, moment_body_nm = _loads(time_s, 1.0, 1.0)
            self.assertEqual(force_body_n, result["force_body_n"][index])
            self.assertEqual(moment_body_nm, result["moment_body_nm"][index])
            self.assertEqual(force_body_n[1], 0.0)
            self.assertAlmostEqual(force_body_n[2], -mass_kg * 9.80665)

            velocity_body = sample[3:6]
            quaternion = _normalize_quaternion(sample[6:10])
            body_rate = sample[10:13]
            expected_position_rate = _matvec(matrix, velocity_body)
            expected_gravity_body = _matvec(
                _transpose(matrix), gravity_ned_mps2
            )
            expected_velocity_rate = _add(
                (1.0 / mass_kg, force_body_n),
                (1.0, expected_gravity_body),
                (-1.0, _cross(body_rate, velocity_body)),
            )
            expected_quaternion_rate = tuple(
                0.5 * value
                for value in _quaternion_product(
                    quaternion, (0.0, *body_rate)
                )
            )
            angular_momentum_body = tuple(
                inertia_value * rate
                for inertia_value, rate in zip(inertia, body_rate)
            )
            expected_body_rate_derivative = tuple(
                (moment - gyroscopic) / inertia_value
                for moment, gyroscopic, inertia_value in zip(
                    moment_body_nm,
                    _cross(body_rate, angular_momentum_body),
                    inertia,
                )
            )
            expected = (
                *expected_position_rate,
                *expected_velocity_rate,
                *expected_quaternion_rate,
                *expected_body_rate_derivative,
            )
            for actual, expected_value in zip(derivative, expected):
                self.assertLess(abs(actual - expected_value), 5e-13)

        self.assertEqual(result["force_body_n"][0][0], 0.0)
        self.assertGreater(result["force_body_n"][1][0], 0.0)
        self.assertEqual(result["force_body_n"][75][0], 0.0)
        self.assertEqual(result["moment_body_nm"][0], (0.0, 0.0, 0.0))
        self.assertTrue(all(value > 0.0 for value in result["moment_body_nm"][1]))
        self.assertEqual(result["moment_body_nm"][50], (0.0, 0.0, 0.0))

    def test_every_state_obeys_rk4_with_stage_and_step_normalization(self) -> None:
        result = _oracle()
        reproduced = [result["samples"][0]]
        for index in range(result["sample_count"] - 1):
            time_s = result["time_s"][index]
            step_s = result["time_s"][index + 1] - time_s
            state = reproduced[-1]
            k1 = _state_derivative(time_s, state, 1.0, 1.0)
            stage_two = _add((1.0, state), (0.5 * step_s, k1))
            self.assertGreater(_norm(stage_two[6:10]), 0.0)
            k2 = _state_derivative(
                time_s + 0.5 * step_s, stage_two, 1.0, 1.0
            )
            stage_three = _add((1.0, state), (0.5 * step_s, k2))
            k3 = _state_derivative(
                time_s + 0.5 * step_s, stage_three, 1.0, 1.0
            )
            stage_four = _add((1.0, state), (step_s, k3))
            k4 = _state_derivative(time_s + step_s, stage_four, 1.0, 1.0)
            candidate = _add(
                (1.0, state),
                (step_s / 6.0, k1),
                (step_s / 3.0, k2),
                (step_s / 3.0, k3),
                (step_s / 6.0, k4),
            )
            next_state = _normalize_state_quaternion(candidate)
            self.assertEqual(next_state, result["samples"][index + 1])
            reproduced.append(next_state)
        self.assertEqual(tuple(reproduced), result["samples"])

        perturbed_stage = list(result["samples"][20])
        perturbed_stage[6] *= 1.2
        perturbed_stage[7] *= 1.2
        perturbed_stage[8] *= 1.2
        perturbed_stage[9] *= 1.2
        normalized_derivative = _state_derivative(
            result["time_s"][20], tuple(perturbed_stage), 1.0, 1.0
        )
        original_derivative = _state_derivative(
            result["time_s"][20], result["samples"][20], 1.0, 1.0
        )
        for actual, expected in zip(normalized_derivative, original_derivative):
            self.assertAlmostEqual(actual, expected, places=14)

    def test_zero_pulses_are_the_exact_straight_level_limit(self) -> None:
        result = _oracle(0.0, 0.0)
        for index, time_s in enumerate(result["time_s"]):
            self.assertAlmostEqual(result["position_ned_m"][0][index], 60.0 * time_s, places=11)
            self.assertEqual(result["position_ned_m"][1][index], 0.0)
            self.assertEqual(result["position_ned_m"][2][index], 0.0)
            self.assertEqual(
                tuple(row[index] for row in result["velocity_body_mps"]),
                (60.0, 0.0, 0.0),
            )
            self.assertEqual(
                tuple(row[index] for row in result["quaternion_body_to_ned"]),
                (1.0, 0.0, 0.0, 0.0),
            )
            self.assertEqual(
                tuple(row[index] for row in result["body_rate_rad_s"]),
                (0.0, 0.0, 0.0),
            )
        self.assertEqual(result["peak_speed_mps"], 60.0)
        self.assertEqual(result["peak_body_rate_deg_s"], 0.0)
        self.assertEqual(result["max_broken_residual_mps2"], 0.0)
        self.assertEqual(result["final_trajectory_separation_m"], 0.0)
        self.assertEqual(result["samples"], result["broken_samples"])

    def test_force_only_case_matches_the_analytic_impulse_limit(self) -> None:
        result = _oracle(1.0, 0.0)
        pulse_duration_s = 1.5
        acceleration_amplitude_mps2 = 2400.0 / 1200.0
        expected_delta_velocity_mps = (
            acceleration_amplitude_mps2 * 2.0 * pulse_duration_s / math.pi
        )
        expected_final_north_m = 60.0 * 6.0 + (
            6.0 * expected_delta_velocity_mps
            - acceleration_amplitude_mps2 * pulse_duration_s**2 / math.pi
        )
        self.assertAlmostEqual(
            result["final_velocity_body_mps"][0],
            60.0 + expected_delta_velocity_mps,
            places=8,
        )
        self.assertEqual(result["final_velocity_body_mps"][1:], (0.0, 0.0))
        self.assertAlmostEqual(
            result["final_position_ned_m"][0], expected_final_north_m, places=7
        )
        self.assertEqual(result["final_position_ned_m"][1:], (0.0, 0.0))
        self.assertEqual(result["final_euler_deg"], (0.0, -0.0, 0.0))
        self.assertEqual(result["peak_body_rate_deg_s"], 0.0)
        self.assertEqual(result["samples"], result["broken_samples"])

    def test_post_pulse_inertial_angular_momentum_is_conserved(self) -> None:
        result = _oracle()
        start_index = result["time_s"].index(1.0)
        reference = result["angular_momentum_ned"][start_index]
        reference_magnitude = _norm(reference)
        self.assertGreater(reference_magnitude, 0.0)
        relative_drift = max(
            _norm(
                tuple(value - baseline for value, baseline in zip(sample, reference))
            )
            for sample in result["angular_momentum_ned"][start_index:]
        ) / reference_magnitude
        self.assertLess(relative_drift, 1e-11)
        self.assertAlmostEqual(
            result["post_pulse_angular_momentum_relative_drift"],
            relative_drift,
            places=15,
        )

        zero_moment = _oracle(1.0, 0.0)
        self.assertTrue(
            all(
                angular_momentum == (0.0, 0.0, 0.0)
                for angular_momentum in zero_moment["angular_momentum_ned"]
            )
        )
        self.assertEqual(
            zero_moment["post_pulse_angular_momentum_relative_drift"], 0.0
        )

    def test_post_pulse_rotational_energy_is_conserved_during_coupled_motion(
        self,
    ) -> None:
        result = _oracle()
        start_index = result["time_s"].index(1.0)
        post_pulse_energy = result["rotational_energy_j"][start_index:]
        reference_energy = post_pulse_energy[0]
        self.assertGreater(reference_energy, 0.0)
        relative_drift = max(
            abs(value - reference_energy) for value in post_pulse_energy
        ) / reference_energy
        self.assertLess(relative_drift, 1e-11)

        post_pulse_rates = tuple(
            tuple(row[index] for row in result["body_rate_rad_s"])
            for index in range(start_index, result["sample_count"])
        )
        reference_rate = post_pulse_rates[0]
        maximum_rate_change = max(
            _norm(
                tuple(
                    value - reference
                    for value, reference in zip(sample, reference_rate)
                )
            )
            for sample in post_pulse_rates
        )
        self.assertGreater(maximum_rate_change, 0.01)

    def test_two_sweeps_change_independent_inputs_and_observables(self) -> None:
        force_scales = (0.0, 0.5, 1.0, 1.25, 1.5)
        force_results = [_oracle(scale, 1.0) for scale in force_scales]
        final_north = [result["final_position_ned_m"][0] for result in force_results]
        peak_speed = [result["peak_speed_mps"] for result in force_results]
        self.assertTrue(all(left < right for left, right in zip(final_north, final_north[1:])))
        self.assertTrue(all(left < right for left, right in zip(peak_speed, peak_speed[1:])))
        for result in force_results:
            self.assertEqual(
                result["quaternion_body_to_ned"],
                force_results[0]["quaternion_body_to_ned"],
            )
            self.assertEqual(
                result["body_rate_rad_s"], force_results[0]["body_rate_rad_s"]
            )
            self.assertEqual(
                result["moment_body_nm"], force_results[0]["moment_body_nm"]
            )
        for index in range(301):
            pulse = _half_sine(force_results[0]["time_s"][index], 1.5)
            for scale, result in zip(force_scales, force_results):
                self.assertAlmostEqual(
                    result["force_body_n"][index][0], 2400.0 * scale * pulse
                )

        moment_scales = (0.0, 0.5, 1.0, 1.25, 1.5)
        moment_results = [_oracle(1.0, scale) for scale in moment_scales]
        peak_body_rate = [
            result["peak_body_rate_deg_s"] for result in moment_results
        ]
        trajectory_separation = [
            result["final_trajectory_separation_m"] for result in moment_results
        ]
        self.assertEqual(peak_body_rate[0], 0.0)
        self.assertTrue(
            all(left < right for left, right in zip(peak_body_rate, peak_body_rate[1:]))
        )
        self.assertTrue(
            all(
                left < right
                for left, right in zip(
                    trajectory_separation, trajectory_separation[1:]
                )
            )
        )
        for result in moment_results:
            self.assertEqual(result["force_body_n"], moment_results[0]["force_body_n"])
        for index in range(301):
            pulse = _half_sine(moment_results[0]["time_s"][index], 1.0)
            for scale, result in zip(moment_scales, moment_results):
                expected = tuple(scale * component * pulse for component in (500.0, 700.0, 350.0))
                self.assertEqual(result["moment_body_nm"][index], expected)

    def test_broken_case_omits_only_rotating_frame_transport(self) -> None:
        result = _oracle()
        for normal, broken in zip(result["samples"], result["broken_samples"]):
            self.assertEqual(normal[6:13], broken[6:13])
        self.assertEqual(result["trajectory_separation_m"][0], 0.0)
        self.assertGreater(result["max_broken_residual_mps2"], 9.0)
        self.assertGreater(result["final_trajectory_separation_m"], 130.0)
        self.assertTrue(
            all(
                math.isfinite(value)
                for sample in result["broken_samples"]
                for value in sample
            )
        )

        for sample, residual_magnitude in zip(
            result["broken_samples"], result["broken_residual_mps2"]
        ):
            expected_residual = _norm(_cross(sample[10:13], sample[3:6]))
            self.assertAlmostEqual(residual_magnitude, expected_residual, places=14)

        no_rotation = _oracle(1.0, 0.0)
        self.assertEqual(no_rotation["samples"], no_rotation["broken_samples"])
        self.assertTrue(
            all(value == 0.0 for value in no_rotation["broken_residual_mps2"])
        )

    def test_malformed_inputs_fail_without_poisoning_recovery(self) -> None:
        baseline = _oracle()
        malformed_cases = (
            ("force below", (-0.001, 1.0)),
            ("force above", (1.501, 1.0)),
            ("moment below", (1.0, -0.001)),
            ("moment above", (1.0, 1.501)),
            ("nan force", (math.nan, 1.0)),
            ("infinite moment", (1.0, math.inf)),
            ("list force", ([1.0], 1.0)),
            ("complex moment", (1.0, 1.0 + 1j)),
            ("text force", ("full", 1.0)),
            ("boolean moment", (1.0, True)),
        )
        for name, values in malformed_cases:
            with self.subTest(case=name), self.assertRaises(ValueError):
                _oracle(*values)
        self.assertEqual(_oracle(), baseline)

    def test_accepted_corners_and_representative_grid_are_bounded(self) -> None:
        corner_count = 0
        for force_scale in (0.0, 1.5):
            for moment_scale in (0.0, 1.5):
                result = _oracle(force_scale, moment_scale)
                corner_count += 1
                self.assertEqual(result["sample_count"], 301)
                self.assertEqual(result["derivative_evaluation_count"], 1200)
                self.assertLess(
                    max(abs(value - 1.0) for value in result["quaternion_norm"]),
                    3e-16,
                )
                self.assertLess(max(result["dcm_orthogonality_residual"]), 2e-15)
                self.assertLessEqual(result["peak_speed_mps"], 63.0)
                self.assertLessEqual(result["peak_body_rate_deg_s"], 18.0)
                self.assertTrue(
                    all(
                        math.isfinite(value)
                        for sample in result["samples"]
                        for value in sample
                    )
                )
        self.assertEqual(corner_count, 4)

        representative_count = 0
        for force_scale in (0.0, 1.0, 1.5):
            for moment_scale in (0.0, 1.0, 1.5):
                result = _oracle(force_scale, moment_scale)
                representative_count += 1
                self.assertEqual(len(result["samples"]), 301)
                self.assertEqual(len(result["broken_samples"]), 301)
                self.assertTrue(all(len(sample) == 13 for sample in result["samples"]))
        self.assertEqual(representative_count, 9)
        self.assertLessEqual(representative_count, 12)

        smallest_positive = _oracle(1.0, math.ulp(0.0))
        self.assertTrue(
            all(
                rate == 0.0
                for row in smallest_positive["body_rate_rad_s"]
                for rate in row
            )
        )
        self.assertEqual(
            smallest_positive["post_pulse_angular_momentum_relative_drift"],
            0.0,
        )
        self.assertTrue(
            math.isfinite(
                smallest_positive[
                    "post_pulse_angular_momentum_relative_drift"
                ]
            )
        )
        self.assertEqual(_oracle(), _oracle())


if __name__ == "__main__":
    unittest.main()
