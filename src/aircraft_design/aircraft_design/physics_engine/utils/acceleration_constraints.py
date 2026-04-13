def acceleration_constraints(v, a, control):
    """
    Constrains the acceleration and prevents the vehicle velocity 
    from exceeding its maximum capabilities.
    
    Args:
        v (float): Current velocity (m/s)
        a (float): Requested acceleration (m/s^2)
        control (ControlParameters): Control limit parameters
        
    Returns:
        float: Constrained acceleration
    """
    # 1. Constrain the acceleration (engine/braking physical limits)
    constrained_a = max(control.a_min, min(control.a_max, a))
    
    # 2. Prevent the velocity from exceeding maximum forward/reverse speeds
    # If we hit max forward speed and are still trying to accelerate, stop accelerating.
    if v >= control.v_max and constrained_a > 0:
        constrained_a = 0.0
    # If we hit max reverse speed and are still trying to accelerate backwards, stop.
    elif v <= control.v_min and constrained_a < 0:
        constrained_a = 0.0
        
    return constrained_a