import math
import numpy as np
from .utils.steering_constraints import steering_constraints
from .utils.acceleration_constraints import acceleration_constraints
from .bolzern_common import A, B, C_1, C_2, D, J, Q_1, Q_2, w_a, Z_2

def vehicle_dynamics_bolzern(x, u_init, p):
    """
    vehicle_dynamics_bolzern - dynamic model acc. to Bolzern et al.
    
    UPDATED STATE VECTOR (Standardized):
    x[0] = x-position
    x[1] = y-position 
    x[2] = yaw angle (theta)
    x[3] = hitch angle (phi)
    x[4] = scalar velocity (v)      <-- Mapped to Index 4
    x[5] = steering angle (delta)   <-- Mapped to Index 5
    """

    # --- 1. Map Indices for Clarity & Safety ---
    # This prevents the index confusion between models
    theta = x[2]
    phi   = x[3]
    v     = x[4]  # VELOCITY
    delta = x[5]  # STEERING ANGLE

    # --- 2. Process Inputs ---
    u = list()
    u.append(u_init[0]) # Forces (6-element vector)
    
    # Constrain steering velocity (u_init[1] is steering_vel_input)
    # We pass 'delta' (current angle) to check limits
    u.append(steering_constraints(delta, u_init[1], p.control)) 

    # --- 3. Unpack Parameters ---
    a_1 = p.vehicle.a
    b_1 = p.vehicle.b
    c_1 = p.vehicle.c
    L_1 = p.vehicle.L
    m_1 = p.vehicle.m
    J_1 = p.vehicle.Izz

    a_2 = p.aircraft.a
    b_2 = p.aircraft.b
    L_2 = p.aircraft.L
    m_2 = p.aircraft.m
    J_2 = p.aircraft.Izz

    L = p.aircraft.L_landing

    # --- 4. Bolzern Helper Functions ---
    # Note: We pass named variables (phi, delta) to avoid index errors
    _A = A(phi, delta, c_1, L_1)
    _B = B(phi, delta, c_1, L_1)

    _J = J(phi, delta, b_1, c_1, L_1, b_2, L_2)
    _Z_2 = Z_2(delta, _A, _B, L_1, L_2)
    _Q_1 = Q_1(delta, L_1)
    _Q_2 = Q_2(theta, _A, c_1, L_1, L_2)
    
    _C_1 = C_1(delta, _Z_2, J_2, b_2, L_2, m_2)
    _C_2 = C_2(v, theta, _B, J_1, J_2, _Q_1, _Q_2, m_1, b_1, c_1, L_1, m_2, b_2)
    _D = D(phi, delta, J_1, J_2, m_1, b_1, c_1, L_1, m_2, b_2, L_2)
    _w_a = w_a(delta, u[0], L, a_1)

    # Calculate alpha_dot (Derivative of Velocity / Acceleration)
    # u[1] here is the actual steering velocity
    alpha_dot_val = (1 / _D) * (-(_C_1 + _C_2 * u[1]) * v + np.dot(_J, _w_a))
    
    # Constrain acceleration
    accel = acceleration_constraints(v, alpha_dot_val, p.control)

    # --- 5. Return Derivatives ---
    # Order must match: [x_dot, y_dot, theta_dot, phi_dot, v_dot, delta_dot]
    f = [
        v * math.cos(theta),             # x_dot
        v * math.sin(theta),             # y_dot
        v * math.tan(delta) / L_1,       # theta_dot
        -v * (math.tan(delta) / L_1 + _A / L_2), # phi_dot
        accel,                           # v_dot (Index 4)
        u[1]                             # delta_dot (Index 5)
    ]

    # CRITICAL FIX: Return numpy array, not list, so it can be multiplied by dt
    return np.array(f)