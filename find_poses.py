import casadi
import numpy as np
import biorbd
import BiorbdViz

from biorbd_optim import QAndQDotBounds, Bounds

# --- Options --- #
model = biorbd.Model("pyomecaman_handstand.bioMod")
straddle_position = True
modified_straddle = True
# --------------- #

# --- Parameters --- #
q_to_optim = (0, 1, 2, 5)
if straddle_position:
    if modified_straddle:
        q0 = np.array([-0.2, 0.6, -2, 0.35])

        def get_q_from_x(x):
            return casadi.vertcat(*[x[0], x[1], x[2], 0, 1.6, x[3], 1.60, x[3], 2.6, -0.45, 2.6, 0.45])

    else:
        q0 = np.array([-0.2, 0.2, -1, 0.35])

        def get_q_from_x(x):
            return casadi.vertcat(*[x[0], x[1], x[2], 0, 0.75, x[3], 0.75, x[3], 2.6, -0.45, 2.6, 0.45])


else:
    q0 = np.array([0.05, 0.75, -3.6, 0.5])

    def get_q_from_x(x):
        return casadi.vertcat(*[x[0], x[1], x[2], 0, 3.25, x[3], 3.25, x[3], 0.1, -0.1, 0.1, 0.1])


nq = model.nbQ()
q = casadi.MX.sym("q", len(q_to_optim))
q_bound = QAndQDotBounds(model)
qdot = np.zeros(nq,)
qddot = np.zeros(nq,)
# ------------------ #


# --- Preparing functions --- #
def forward_dyn(x):
    q = get_q_from_x(x)
    return model.ForwardDynamicsConstraintsDirect(q, qdot, qddot).to_mx()


def hand_position(x):
    q = get_q_from_x(x)
    return model.markers(q)[:, (39, 40)].T


def com_is_centered(x):
    q = get_q_from_x(x)
    com = model.CoM(q).to_mx()[1]
    hand = model.marker(q, 39).to_mx()[1]
    return com - (hand + 0.05)


def com_at_zero(x):
    q = get_q_from_x(x)
    com = model.CoM(q).to_mx()[1]
    return com


# Convert to casadi functions
forward_dyn_func = casadi.Function("handstand", [q], [forward_dyn(q)], ["q"], ["tau"]).expand()
com_is_centered_func = casadi.Function("com_is_centered", [q], [com_is_centered(q)], ["q"], ["com_xy"]).expand()
com_at_zero_func = casadi.Function("com_is_centered", [q], [com_at_zero(q)], ["q"], ["com_zero"]).expand()
hand_position_func = casadi.Function("hand_position", [q], [hand_position(q)], ["q"], ["hand_z"]).expand()
hands_on_floor_func = casadi.Function("hands_on_floor", [q], [hand_position(q)[:, 2]], ["q"], ["hand_z"]).expand()
# --------------------------- #


# --- Solving optimal problem --- #
# Objective function (no objective)
f = casadi.MX(0)

# Constraints
g = casadi.MX()
g_bound = Bounds()

# if crouched_position:
g = casadi.vertcat(g, com_is_centered_func(q))
g_bound.concatenate(Bounds(0, 0))

g = casadi.vertcat(g, com_at_zero_func(q))
g_bound.concatenate(Bounds(0, 0))

g = casadi.vertcat(g, hands_on_floor_func(q))
g_bound.concatenate(Bounds(0, 0))
g_bound.concatenate(Bounds(0, 0))

# Declare and solve the problem
ipopt_nlp = {"x": q, "f": f, "g": g}
ipopt_limits = {
    "lbx": q_bound.min[q_to_optim, 0],
    "ubx": q_bound.max[q_to_optim, 0],
    "lbg": g_bound.min[:, 0],
    "ubg": g_bound.max[:, 0],
    "x0": q0,
}
solver = casadi.nlpsol("nlpsol", "ipopt", ipopt_nlp)
out = solver.call(ipopt_limits)
# ------------------------------- #

# --- Plot answer --- #
q_optim = get_q_from_x(out["x"])
print(f"La position pour le {'straddle' if straddle_position else 'handstand'} est {q_optim}")
print(f"Le Tau de cette position est {forward_dyn_func(out['x'])}")
print(f"La main est située à {hand_position_func(out['x'])[0, :]}")
b = BiorbdViz.BiorbdViz(
    loaded_model=model,
    show_markers=False,
    show_global_center_of_mass=False,
    show_segments_center_of_mass=False,
    show_local_ref_frame=False,
)
b.set_q(np.array(q_optim)[:, 0])
b.exec()
# ------------------- #
