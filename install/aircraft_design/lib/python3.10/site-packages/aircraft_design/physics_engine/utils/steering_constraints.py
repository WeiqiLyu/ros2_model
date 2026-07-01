def steering_constraints(delta, u_delta, control):
    """
    Constrains the steering velocity and prevents the steering angle 
    from exceeding its physical mechanical limits.
    
    Args:
        delta (float): Current steering angle (radians)
        u_delta (float): Requested steering velocity (radians/sec)
        control (ControlParameters): Control limit parameters
        
    Returns:
        float: Constrained steering velocity
    """
    # 1. Constrain the steering speed (how fast the wheel can physically turn)
    constrained_u_delta = max(control.v_delta_min, min(control.v_delta_max, u_delta))
    
    # 2. Prevent the steering angle from exceeding its maximum limits
    # If we are at the max left angle and trying to turn further left, stop turning.
    if delta >= control.delta_max and constrained_u_delta > 0:
        constrained_u_delta = 0.0
    # If we are at the max right angle and trying to turn further right, stop turning.
    elif delta <= control.delta_min and constrained_u_delta < 0:
        constrained_u_delta = 0.0
        
    return constrained_u_delta