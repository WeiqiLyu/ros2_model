import math

def A(phi_1, delta, c_1, L_1):
    return math.sin(phi_1) + c_1 / L_1 * math.cos(phi_1) * math.tan(delta)

def B(phi_1, delta, c_1, L_1):
    return math.cos(phi_1) - c_1 / L_1 * math.sin(phi_1) * math.tan(delta)

def C_1(alpha, _Z_2, J_2, b_2, L_2, m_2):
    return (m_2 * _Z_2 * ((L_2 ** 2) - (b_2 ** 2)) - J_2 * _Z_2) * alpha

def C_2(delta, phi_1, _B, J_1, J_2, _Q_1, _Q_2, m_1, b_1, c_1, L_1, m_2, b_2):
    return (1 / (math.cos(delta) ** 2)) * (m_1 * (b_1 ** 2) * _Q_1 + J_1 * _Q_1
                                         + m_2 * ((b_2 ** 2) * _Q_2 - _B * c_1 * math.sin(phi_1) / L_1)
                                         + J_2 * _Q_2)

def D(phi_1, delta, J_1, J_2, m_1, b_1, c_1, L_1, m_2, b_2, L_2):
    return (m_1 * (1 + (b_1 ** 2) * (math.tan(delta) ** 2) / (L_1 ** 2))
            + J_1 * (math.tan(delta) ** 2) / (L_1 ** 2)
            + m_2 * ((A(phi_1, delta, c_1, L_1) ** 2) * (b_2 ** 2) / (L_2 ** 2) + (B(phi_1, delta, c_1, L_1) ** 2))
            + J_2 * (A(phi_1, delta, c_1, L_1) ** 2) / (L_2 ** 2))

def J(phi_1, delta, b_1, c_1, L_1, b_2, L_2):
    return [
        1,
        b_1 * math.tan(delta) / L_1,
        math.tan(delta) / L_1,
        B(phi_1, delta, c_1, L_1),
        -A(phi_1, delta, c_1, L_1) * b_2 / L_2,
        -A(phi_1, delta, c_1, L_1) / L_2
    ]

def Q_1(delta, L_1):
    return math.tan(delta) / (L_1 ** 2)

def Q_2(phi_1, _A, c_1, L_1, L_2):
    return _A * c_1 * math.cos(phi_1) / (L_1 * L_2 ** 2)

def w_a(delta, F_u, L, a_1):
    return [
        math.cos(delta) * F_u[0] + math.cos(delta) * F_u[1] + F_u[2] + F_u[3],
        math.sin(delta) * F_u[0] + math.sin(delta) * F_u[1],
        (-L / 2 * math.cos(delta) + a_1 * math.sin(delta)) * F_u[0]
            + (a_1 * math.sin(delta) + L / 2 * math.cos(delta)) * F_u[1]
            - L / 2 * (F_u[2] - F_u[3]),
        F_u[4] + F_u[5],
        0,
        -L / 2 * (F_u[4] - F_u[5])
    ]

def Z_2(delta, _A, _B, L_1, L_2):
    return _B * (_A ** 2) / (L_2 ** 3) + _A * _B * math.tan(delta) / (L_1 * (L_2 ** 2))