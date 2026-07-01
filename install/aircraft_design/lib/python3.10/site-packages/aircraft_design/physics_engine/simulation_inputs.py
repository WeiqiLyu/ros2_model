
import numpy as np

def get_maneuver_scenarios():
    """
    Defines the three maneuver scenarios: Straight, Left Turn, and Right Turn.
    Vehicle physical parameters are loaded separately from JSON files.
    """
    
    # --- Define the Three Maneuver Inputs ---
    maneuver_scenarios = {
        "Straight Line": {
            "mu": 0.8,                 # Coefficient of Friction (Grip)
            "propulsive_force": -35000.0, # Newtons
            "nose_gear_angle": 0,      # Angle for straight line
            "path": [
                {'duration': 30.0, 'velocity': -1.5, 'steering_angle': np.deg2rad(0)},
            ]
        },
        
        "Right Turn": {
            "mu": 0.5,                 # Lower grip to test handling
            "propulsive_force": -35000.0,
            "nose_gear_angle": 0,     # Angle for left turn
            "path": [
                {'duration': 40.0, 'velocity': -1.5, 'steering_angle': np.deg2rad(0)},
                {'duration': 35.0, 'velocity': -1.5, 'steering_angle': np.deg2rad(4)}, # Left steer
                {'duration': 30.0, 'velocity': -1.5, 'steering_angle': np.deg2rad(0)},
            ]
        },
        
        "Left Turn": {
            "mu": 0.5,                  # Lower grip
            "propulsive_force": -35000.0,
            "nose_gear_angle": 0,       # Angle for right turn
            "path": [
                {'duration': 35.0, 'velocity': -1.5, 'steering_angle': np.deg2rad(0)},
                {'duration': 35.0, 'velocity': -1.5, 'steering_angle': np.deg2rad(-3)},# Right steer
                {'duration': 60.0, 'velocity': -1.5, 'steering_angle': np.deg2rad(0)},
            ]
        }
    }
    
    return maneuver_scenarios