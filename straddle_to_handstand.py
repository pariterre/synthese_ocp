from time import time
import pickle

import numpy as np
import biorbd

from biorbd_optim import (
    OptimalControlProgram,
    DynamicsType,
    DynamicsTypeOption,
    BoundsOption,
    QAndQDotBounds,
    InitialConditionsOption,
    InterpolationType,
    ConstraintList,
    Constraint,
    Instant,
    ShowResult,
    Objective,
    ObjectiveList,
)


if __name__ == "__main__":
    # --- Options --- #
    biorbd_model = biorbd.Model("pyomecaman_handstand.bioMod")
    nb_shoot = 30
    final_time = 2
    nb_threads = 6
    use_SX = False
    tau_min, tau_max, tau_init = -200, 200, 0
    mu = 0.5
    is_flexible = True
    modified_straddle = True
    load_movement = True
    save_movement = False

    if modified_straddle:
        q0 = np.array([-0.144378, 0.5527, -2.29619, 0, 1.6, 0.696191, 1.6, 0.696191, 2.6, -0.45, 2.6, 0.45])
    else:
        q0 = np.array([-0.193044, 0.262388, -1.23482, 0, 0.75, 0.484817, 0.75, 0.484817, 2.6, -0.45, 2.6, 0.45])
    qf = np.array([0.00165561, 0.797517, -3.5573, 0, 3.25, 0.307299, 3.25, 0.307299, 0.1, -0.1, 0.1, 0.1])
    # --------------- #

    # --- Problem definion --- #
    n_root_trans = biorbd_model.segment(0).nbDofTrans()
    n_root = biorbd_model.nbRoot()
    n_q = biorbd_model.nbQ()
    if is_flexible:
        movement_name = "handstand_normal"
    else:
        movement_name = "handstand_not_flex"
    symmetry = [
        [n_root + 0, n_root + 2, 1],
        [n_root + 1, n_root + 3, 1],
        [n_root + 4, n_root + 6, 1],
        [n_root + 5, n_root + 7, -1],
    ]
    shoulder_idx = n_root
    hand_idx = shoulder_idx + 1
    hip_idx = hand_idx + 3

    objective_functions = ObjectiveList()
    # Main objective
    objective_functions.add(Objective.Lagrange.MINIMIZE_TORQUE, controls_idx=shoulder_idx, weight=10)
    # Regulation objectives
    objective_functions.add(Objective.Lagrange.MINIMIZE_TORQUE, weight=0.1)
    objective_functions.add(Objective.Lagrange.MINIMIZE_TORQUE_DERIVATIVE, weight=0.1)
    objective_functions.add(Objective.Lagrange.MINIMIZE_TORQUE_DERIVATIVE, controls_idx=hand_idx, weight=10)

    # Add constraints function
    constraints = ConstraintList()
    # Enforce symmetry of the model
    for s in symmetry:
        constraints.add(Constraint.PROPORTIONAL_STATE, instant=Instant.ALL, first_dof=s[0], second_dof=s[1], coef=s[2])
    # Model the contact on the ground
    constraints.add(
        Constraint.CONTACT_FORCE_INEQUALITY,
        direction="GREATER_THAN",
        instant=Instant.ALL,
        contact_force_idx=1,
        boundary=0,
    )
    constraints.add(
        Constraint.CONTACT_FORCE_INEQUALITY,
        direction="GREATER_THAN",
        instant=Instant.ALL,
        contact_force_idx=2,
        boundary=0,
    )
    constraints.add(
        Constraint.NON_SLIPPING,
        instant=Instant.ALL,
        normal_component_idx=(1, 2),
        tangential_component_idx=0,
        static_friction_coefficient=mu,
    )

    # Dynamics with contact at hands
    dynamics = DynamicsTypeOption(DynamicsType.TORQUE_DRIVEN_WITH_CONTACT)

    # Path constraint
    x_bounds = BoundsOption(QAndQDotBounds(biorbd_model))
    # Straddle at start (no velocity)
    x_bounds.min[:n_q, 0] = q0
    x_bounds.max[:n_q, 0] = q0
    x_bounds.min[n_q:, 0] = 0
    x_bounds.max[n_q:, 0] = 0
    # Handstand at end (with free root on translation, no velocity)
    x_bounds.min[:n_q, -1] = qf
    x_bounds.max[:n_q, -1] = qf
    x_bounds.min[n_q:, -1] = 0
    x_bounds.max[n_q:, -1] = 0
    if not is_flexible:
        x_bounds.min[hip_idx + 1, 1] = -0.6

    # Initial guess
    init_guess = np.zeros((n_q * 2, 2))
    init_guess[:n_q, 0] = q0
    init_guess[:n_q, 1] = qf
    x_init = InitialConditionsOption(init_guess, InterpolationType.LINEAR)

    # Define control path constraint
    u_bounds = BoundsOption([[tau_min] * n_q, [tau_max] * n_q])
    # No control on the root
    u_bounds.min[:n_root, :] = 0
    u_bounds.max[:n_root, :] = 0

    u_init = InitialConditionsOption([tau_init] * n_q)

    ocp = OptimalControlProgram(
        biorbd_model,
        dynamics,
        nb_shoot,
        final_time,
        x_init,
        u_init,
        x_bounds,
        u_bounds,
        objective_functions=objective_functions,
        constraints=constraints,
        nb_threads=nb_threads,
        use_SX=use_SX,
        nb_integration_steps=1,
    )
    # --------------------------- #

    # --- Solve the program --- #
    if load_movement:
        with open(f"resultats/{movement_name}.bob", "rb") as file:
            sol = pickle.load(file)

        from BiorbdViz import BiorbdViz

        b = BiorbdViz(
            loaded_model=biorbd_model,
            show_markers=False,
            show_global_center_of_mass=False,
            show_segments_center_of_mass=False,
            show_local_ref_frame=False,
        )
        b.load_movement(sol["data"][0]["q"])
        b.exec()

    else:
        tic = time()
        sol = ocp.solve(
            show_online_optim=True, solver_options={"max_iter": 1000, "hessian_approximation": "limited-memory"}
        )
        toc = time() - tic
        print(f"Time to solve : {toc} sec")
        if save_movement:
            ocp.save_get_data(sol, f"resultats/{movement_name}.bob")

        # --- Show results --- #
        result = ShowResult(ocp, sol)
        result.animate(nb_frames=nb_shoot)
