import json
import os

# --- Define placeholder classes ---
class Parameters:
    def __init__(self):
        pass

class VehicleParameters:
    def __init__(self):
        pass

class AircraftParameters:
    def __init__(self):
        pass

class ControlParameters:
    def __init__(self):
        pass

class VisualizationData:
    def __init__(self):
        pass

class InitialStateParameters:
    def __init__(self):
        # Initialize with None to ensure we populate from JSON
        self.x = None
        self.y = None
        self.theta = None
        self.phi = None
        self.v = None
        self.delta = None

def load_parameters(vehicle_json_file, aircraft_json_file):
    """
    Loads vehicle and aircraft JSON files.
    All 6 Initial states are strictly loaded from the VEHICLE file (TF6).
    """
    # FIXED: Use ROS 2's native package finder instead of relative Python paths
    from ament_index_python.packages import get_package_share_directory
    
    pkg_dir = get_package_share_directory('aircraft_design')
    vehicle_file_path = os.path.join(pkg_dir, 'parameters', vehicle_json_file)
    aircraft_file_path = os.path.join(pkg_dir, 'parameters', aircraft_json_file)
    
    try:
        with open(vehicle_file_path, 'r') as f:
            full_vehicle_data = json.load(f)
        vehicle_key = list(full_vehicle_data.keys())[0]
        vehicle_data = full_vehicle_data[vehicle_key]
    except FileNotFoundError:
        print(f"ERROR: Vehicle file not found: {vehicle_file_path}")
        raise
        
    try:
        with open(aircraft_file_path, 'r') as f:
            full_aircraft_data = json.load(f)
        aircraft_key = list(full_aircraft_data.keys())[0]
        aircraft_data = full_aircraft_data[aircraft_key]
    except FileNotFoundError:
        print(f"ERROR: Aircraft file not found: {aircraft_file_path}")
        raise

    p = Parameters()

    # --- Vehicle Parameters ---
    p.vehicle = VehicleParameters()
    vehicle_params = vehicle_data["parameters"]
    p.vehicle.m = vehicle_params["m"]["default"]
    p.vehicle.m_a = vehicle_params["m_a"]["default"]
    p.vehicle.m_b = vehicle_params["m_b"]["default"]
    p.vehicle.mu_a = vehicle_params["mu_a"]["default"]
    p.vehicle.mu_b = vehicle_params["mu_b"]["default"]
    p.vehicle.Izz = vehicle_params["Izz"]["default"]
    p.vehicle.L = vehicle_params["L"]["default"]
    p.vehicle.a = vehicle_params["a"]["default"]
    p.vehicle.b = vehicle_params["b"]["default"]
    p.vehicle.c = vehicle_params["c"]["default"]
    p.vehicle.L_a = vehicle_params["L_a"]["default"]
    p.vehicle.L_b = vehicle_params["L_b"]["default"]
   
    # --- Aircraft Parameters ---
    p.aircraft = AircraftParameters()
    ac_phys = aircraft_data["physic_parameters"]
    ac_geo = aircraft_data["geometric_parameters"]

    p.aircraft.m = ac_phys["m"]["default"]
    p.aircraft.m_a = ac_phys["m_a"]["default"]
    p.aircraft.m_b = ac_phys["m_b"]["default"]
    p.aircraft.mu = ac_phys["mu"]["default"]
    p.aircraft.Izz = ac_phys["Izz"]["default"]
    p.aircraft.Kzz = ac_phys["Kzz"]["default"]
    p.aircraft.m_max = ac_phys["MTOM"]["default"]
    p.aircraft.m_min = ac_phys["MFM"]["default"]
    
    p.aircraft.L = ac_geo["wheelbase"]["default"]
    p.aircraft.a = ac_geo["a"]["default"]
    p.aircraft.b = ac_geo["b"]["default"]
    p.aircraft.L_landing = ac_geo["L_landing"]["default"]
    p.aircraft.L_wing = ac_geo["L_wing"]["default"]
    p.aircraft.L_tail = ac_geo["L_tail"]["default"]
    p.aircraft.L_plane = ac_geo["L"]["default"]
    p.aircraft.L_nose = ac_geo["L_nose"]["default"]
    
    p.aircraft.mac = ac_geo["MAC"]["default"]
    p.aircraft.le_mac = ac_geo["LE_MAC"]["default"]
    p.aircraft.datum = ac_geo["Datum"]["default"]
    p.aircraft.min_mac_percent = ac_geo["min_MAC"]["default"]
    p.aircraft.max_mac_percent = ac_geo["max_MAC"]["default"]

    # --- Load Control Limits ---
    p.control = ControlParameters()
    control_limits = vehicle_data["control_limits"]
    p.control.delta_max = control_limits["delta_max"]["default"]
    p.control.delta_min = control_limits["delta_min"]["default"]
    p.control.v_delta_max = control_limits["v_delta_max"]["default"]
    p.control.v_delta_min = control_limits["v_delta_min"]["default"]
    p.control.v_max = control_limits["v_max"]["default"]
    p.control.v_min = control_limits["v_min"]["default"]
    p.control.a_max = control_limits["a_max"]["default"]
    p.control.a_min = control_limits["a_min"]["default"]
    p.control.phi_max = control_limits["phi_max"]["default"]
    p.control.phi_min = control_limits["phi_min"]["default"]
    p.control.F_max = control_limits["F_max"]["default"]
    p.control.F_min = control_limits["F_min"]["default"]
    p.control.v_switch = 0.1 

    # --- Load Initial States (Strictly from JSON) ---
    p.init = InitialStateParameters()
    
    # We expect 'states' to exist in the TF6 parameters file
    if "states" in vehicle_data:
        v_states = vehicle_data["states"]
        p.init.x = v_states["x"]["default"]
        p.init.y = v_states["y"]["default"]
        p.init.theta = v_states["theta"]["default"]
        p.init.phi = v_states["phi"]["default"]
        p.init.v = v_states["v"]["default"]
        p.init.delta = v_states["delta"]["default"]
    else:
        # Raise error if states are missing, ensuring we don't use hidden defaults
        raise KeyError(f"The 'states' block is missing from {vehicle_json_file}. Please ensure x, y, theta, phi, v, and delta are defined.")

    # --- Load Visualization Data ---
    p.visual = VisualizationData()
    visual_data = aircraft_data.get("visualization_data", {}) 
    p.visual.trace_init_data = visual_data.get("trace_init", [])
    p.visual.track_init_data = visual_data.get("track_init", [])
    p.visual.drag_init_data = visual_data.get("drag_init", [])

    return p

def save_final_states(json_file_name, x_final, y_final, theta_final):
    """
    (Deprecated) This function is kept for compatibility but should not be used 
    if we want to purely rely on initial parameters from JSON.
    """
    pass