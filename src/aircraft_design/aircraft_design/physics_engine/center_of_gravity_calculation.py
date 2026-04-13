import matplotlib.pyplot as plt

class ATR42CG:
    def __init__(self):
        """Initialize ATR 42-300 center of gravity calculator"""
        # Aircraft geometric parameters
        self.lemac_position = 11.425                                           # Distance from reference point to MAC leading edge (m)
        self.mac_length = 2.285                                                # Mean Aerodynamic Chord (m)
        self.fuselage_x = 2.362                                                # Distance from nose tip to reference point (m)
        self.fuselage_y = 3                                                    # Vertical distance from nose tip to reference point (m)
        self.fuselage_to_nosewheel = 1.683                                     # Horizontal distance from nose tip to nosewheel (m)
        self.wheelbase = 8.781                                                 # Distance between nose and main landing gear (m)
        self.distance_main_landing_gear = 4.1                                  # Distance between the two main landing gear wheels (m)
        self.nose_to_reference = self.fuselage_x + self.fuselage_to_nosewheel  # Distance from nose gear to ref point (m)
        self.mlg_to_reference = self.nose_to_reference + self.wheelbase        # Distance from main landing gear to ref point (m)

        self.length = 22.67                                                   
        self.wingspan = 24.572                                                

        # Basic aircraft and crew data (from AHM manual)
        self.basic_weight = 10291                                              
        self.basic_index = -50                                                 
        self.additional_crew_weight = 85
        self.additional_crew_index = -5

        # Maximum weight limits (kg)
        self.max_taxi_weight = 17070
        self.max_takeoff_weight = 16900
        self.max_landing_weight = 16400
        self.max_zero_fuel_weight = 15540
        self.min_flight_weight = 10452
        self.max_fuel_load = 4500
        self.max_payload = 4640

        # Maximum cargo load for each compartment (kg)
        self.max_loads = {
            'A': 1270,
            'B': 1220,
            'C': 1220,
            'D': 1220,
            'E': 1400,
            'F': 1500
        }
        
        # Index to %MAC conversion based on CG envelope chart
        # Linear relationship: %MAC = slope * index + intercept
        # Index 0 → 25% MAC, Index -37 → 7% MAC
        self.mac_slope = 37 / 18                                # slope (%MAC per index)
        self.mac_intercept = 25                                 # Index 0 corresponds to 25% MAC
        
        # CG limits in %MAC for take off and landing limit
        self.min_mac = 15.0                                     # Forward limit
        self.max_mac = 36.0                                     # Aft limit

    def cargo_index(self, section, weight):
        """Calculate cargo index contribution
        
        Args:
            section: Cargo compartment ('A' to 'F')
            weight: Cargo weight (kg)
        
        Returns:
            Index contribution
        """
        # Lookup table for cargo index by compartment
        index_table = {
            'A': {0: 0, 50: -2, 100: -4, 150: -6, 200: -8, 250: -11, 300: -13, 
                  350: -15, 400: -17, 450: -19, 500: -21, 550: -23, 600: -25,
                  650: -27, 700: -30, 750: -32, 800: -34, 850: -36, 900: -38,
                  950: -40, 1000: -42, 1050: -44, 1100: -46, 1150: -48, 1200: -50, 
                  1250: -52, 1300: -54},
            'B': {0: 0, 50: -1, 100: -2, 150: -3, 200: -4, 250: -5, 300: -6, 
                  350: -7, 400: -8, 450: -9, 500: -10, 550: -11, 600: -12,
                  650: -12, 700: -13, 750: -14, 800: -15, 850: -16, 900: -17,
                  950: -18, 1000: -19, 1050: -20, 1100: -21, 1150: -21, 1200: -21, 
                  1250: -21, 1300: -21},
            'C': {0: 0, 50: 0, 100: 0, 150: 0, 200: 0, 250: 1, 300: 1, 
                  350: 1, 400: 1, 450: 1, 500: 1, 550: 1, 600: 1,
                  650: 1, 700: 2, 750: 2, 800: 2, 850: 2, 900: 2,
                  950: 2, 1000: 2, 1050: 2, 1100: 3, 1150: 3, 1200: 3, 
                  1250: 3, 1300: 4},
            'D': {0: 0, 50: 1, 100: 2, 150: 4, 200: 5, 250: 6, 300: 7, 
                  350: 8, 400: 10, 450: 11, 500: 12, 550: 13, 600: 14,
                  650: 15, 700: 17, 750: 18, 800: 19, 850: 20, 900: 21,
                  950: 23, 1000: 24, 1050: 25, 1100: 26, 1150: 27, 1200: 28, 
                  1250: 29, 1300: 30},
            'E': {0: 0, 50: 2, 100: 4, 150: 7, 200: 8, 250: 11, 300: 13, 
                  350: 15, 400: 18, 450: 20, 500: 22, 550: 24, 600: 28,
                  650: 28, 700: 31, 750: 33, 800: 35, 850: 37, 900: 39,
                  950: 41, 1000: 43, 1050: 45, 1100: 47, 1150: 49, 1200: 51, 
                  1250: 53, 1300: 55, 1350: 57, 1400: 59},
            'F': {0: 0, 50: 3, 100: 6, 150: 9, 200: 12, 250: 15, 300: 17, 
                  350: 20, 400: 23, 450: 27, 500: 30, 550: 33, 600: 37,
                  650: 40, 700: 43, 750: 46, 800: 49, 850: 52, 900: 55,
                  950: 58, 1000: 61, 1050: 64, 1100: 67, 1150: 70, 1200: 73, 
                  1250: 76, 1300: 79, 1350: 81, 1400: 84, 1450: 87, 1500: 90}
        }

        # Ensure weight does not exceed maximum for the section
        if weight > self.max_loads[section]:
            raise ValueError(f"Cargo weight in section {section} exceeds max limit of {self.max_loads[section]} kg")
        
        if weight == 0:
            return 0

        # Find nearest index not less than the input weight
        valid_weights = [w for w in index_table[section].keys() if weight <= w]
        if valid_weights:
            closest_weight = min(valid_weights)
            return index_table[section][closest_weight]

        return 0

    def fuel_index(self, fuel_weight):
        """Calculate fuel index contribution
        
        Args:
            fuel_weight: Fuel weight (kg)
        
        Returns:
            Index contribution
        """
        fuel_indices = {
            0: 0, 600: 1, 1000: 2, 1400: 3, 1800: 4, 2200: 5,
            2600: 6, 3000: 7, 3400: 8, 3800: 9, 4200: 10, 4500: 11
        }

        if fuel_weight > self.max_fuel_load:
            raise ValueError(f"Fuel weight exceeds maximum limit of {self.max_fuel_load} kg")
        
        if fuel_weight == 0:
            return 0

        valid_weights = [w for w in fuel_indices.keys() if fuel_weight <= w]
        if valid_weights:
            closest_weight = min(valid_weights)
            return fuel_indices[closest_weight]

        return 0

    def crew_index(self, crew_count):
        """Calculate index contribution for additional crew members (observers)
        
        Args:
            crew_count: Number of additional observers
        
        Returns:
            Index contribution
        """
        return crew_count * (self.additional_crew_index)  # 85 kg per person, index -5

    def calculate_cg(self, crew_count=0, fuel_weight=0, cargo_weights=None):
        """Calculate center of gravity
        
        Args:
            crew_count: Extra crew members (2 standard pilots included in basic weight)
            fuel_weight: Fuel weight (kg)
            cargo_weights: Dict of section: weight pairs (kg)
        
        Returns:
            Tuple: (Total weight, total index, %MAC, CG from ref point, CG from MLG, Nosewheel %, Status info)
        """
        if cargo_weights is None:
            cargo_weights = {'A': 0, 'B': 0, 'C': 0, 'D': 0, 'E': 0, 'F': 0}
        
        total_weight = self.basic_weight
        crew_weight = crew_count * self.additional_crew_weight
        total_weight += crew_weight

        cargo_weight = sum(cargo_weights.values())
        if cargo_weight > self.max_payload:
            raise ValueError(f"Total cargo exceeds max payload of {self.max_payload} kg")
        
        total_weight += cargo_weight
        total_weight += fuel_weight

        total_index = self.basic_index
        total_index += self.crew_index(crew_count)

        for section, weight in cargo_weights.items():
            if weight > 0:
                total_index += self.cargo_index(section, weight)

        total_index += self.fuel_index(fuel_weight)

        mac_percent = self.index_to_mac(total_index)
        cg_position = self.lemac_position + (mac_percent / 100) * self.mac_length

        cg_to_mlg = cg_position - self.mlg_to_reference
        if cg_to_mlg <= 0:
            cg_to_mlg = abs(cg_to_mlg)
        else:                                             # If cg_to_mlg is a positive value, it means behind MLG (tail direction)
            raise ValueError("The center of gravity exceeds the range of the main landing gear!")
        
        nosewheel_load_percent = self.calculate_nosewheel_load(cg_to_mlg, total_weight)

        status = self.check_limits(total_weight, mac_percent, nosewheel_load_percent)

        return (total_weight, total_index, mac_percent, cg_position, cg_to_mlg, nosewheel_load_percent, status)

    def calculate_nosewheel_load(self, cg_to_mlg, total_weight):
        """Calculate nosewheel load percentage
        
        Args:
            cg_position: CG position from reference point (m)
            total_weight: Total aircraft weight (kg)
        
        Returns:
            Nosewheel load as a percentage of total weight
        """
        nosewheel_load = (total_weight * cg_to_mlg) / self.wheelbase
        return (nosewheel_load / total_weight) * 100

    def index_to_mac(self, index):
        """Convert index to %MAC
        
        Args:
            index: The parameter used to express the variation of location of CG which is the shortend moment of a certain weight.
        
        Returns:
            %MAC
        """
        return self.mac_slope * index + self.mac_intercept

    def check_limits(self, weight, mac_percent, nosewheel_load_percent):
        """Check whether CG and weight are within safe limits
        
        Args:
            weight: Total weight (kg)
            mac_percent: %MAC
            nosewheel_load_percent: The percentage of load on the nose wheel to the total load
        
        Returns:
            Dictionary with status booleans and messages
        """
        status = {
            'within_weight_limit': True,
            'within_cg_limit': True,
            'messages': []
        }

        if weight > self.max_takeoff_weight:
            status['within_weight_limit'] = False
            status['messages'].append(f"Exceeds MTOW! Current: {weight} kg, Limit: {self.max_takeoff_weight} kg")

        if weight > self.min_flight_weight and weight - self.min_flight_weight < 10:
            status['messages'].append(f"Close to Minimun Flight Weight! Current: {weight} kg, Limit: {self.min_flight_weight} kg")

        if mac_percent < self.min_mac:
            status['within_cg_limit'] = False
            status['messages'].append(f"CG too far forward! Current: {mac_percent:.2f}% MAC, Min: {self.min_mac}%")

        if mac_percent > self.max_mac:
            status['within_cg_limit'] = False
            status['messages'].append(f"CG too far aft! Current: {mac_percent:.2f}% MAC, Max: {self.max_mac}%")

        if not status['messages']:
            status['messages'].append("Weight and CG are within allowed limits")

        return status

    def visualization(self, weight, index, mac, cg_to_mlg, nosewheel_load):
        """Visualize CG envelope and CG position on aircraft
        
        Args:
            weight: Total weight (kg)
            index: Total index
            mac: %MAC
            cg_to_mlg: CG distance to MLG (m)
            nosewheel_load: Nosewheel load percentage
        """
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))

        # Plot CG envelope
        ax1.axhline(y=self.max_takeoff_weight/1000, color='r', linestyle='-', label=f'MTOW={self.max_takeoff_weight}')
        ax1.axhline(y=self.max_zero_fuel_weight/1000, color='g', linestyle='--', label=f'MZW={self.max_zero_fuel_weight}')
        ax1.axhline(y=self.min_flight_weight/1000, color='orange', linestyle='-.', label=f'MFW={self.min_flight_weight}')

        y_min, y_max = 9, 18
        ax1.fill_betweenx(y=[y_min, y_max], x1=self.min_mac, x2=self.max_mac, color='lightblue', alpha=0.5, label='CG Envelope')
        ax1.axvline(x=self.min_mac, color='blue', linestyle='-', label=f'{self.min_mac}% MAC Forward Limit')
        ax1.axvline(x=self.max_mac, color='blue', linestyle='-', label=f'{self.max_mac}% MAC Aft Limit')
        ax1.plot(mac, weight / 1000, 'ro', markersize=10, label=f'Current CG ({mac:.2f}% MAC)')

        ax1.set_title('ATR 42-300 Center of Gravity Envelope', fontsize=14)
        ax1.set_xlabel('% MAC', fontsize=12)
        ax1.set_ylabel('Weight (x1000 kg)', fontsize=12)
        ax1.set_xlim(7, 40)
        ax1.set_ylim(y_min, y_max)
        ax1.grid(True)
        ax1.legend(loc='upper right')

        # Plot CG position (top view)
        ax2.set_title('Position of CG', fontsize=14)
        ax2.set_xlim(0, self.nose_to_reference + self.wheelbase + 2)
        ax2.set_ylim(-self.distance_main_landing_gear / 2 - 1, self.distance_main_landing_gear / 2 + 1)
        ax2.set_aspect('equal')
        ax2.set_xlabel('Longitudinal distance (m)', fontsize=12)
        ax2.set_ylabel('Lateral position (m)', fontsize=12)
        ax2.grid(True)

        # Draw MLG wheels
        mlg_x = self.mlg_to_reference
        mlg_y1 = -self.distance_main_landing_gear / 2
        mlg_y2 = self.distance_main_landing_gear / 2
        ax2.plot(mlg_x, mlg_y1, 'ko', markersize=10)
        ax2.plot(mlg_x, mlg_y2, 'ko', markersize=10)
        ax2.text(mlg_x + 0.3, mlg_y1, 'MLG (L)', fontsize=10, ha='left')
        ax2.text(mlg_x + 0.3, mlg_y2, 'MLG (R)', fontsize=10, ha='left')
        ax2.plot([mlg_x, mlg_x], [mlg_y1, mlg_y2], 'k--', linewidth=1)

        # MLG center
        mlg_center_y = 0
        ax2.plot(mlg_x, mlg_center_y, 'ks', markersize=8)
        ax2.text(mlg_x + 0.3, mlg_center_y + 0.2, 'MLG Center', fontsize=9)

        # Draw nose wheel
        nose_x = self.nose_to_reference
        nose_y = 0
        ax2.plot(nose_x, nose_y, 'ko', markersize=8)
        ax2.text(nose_x - 0.3, nose_y - 0.5, 'Nose Wheel', fontsize=10, ha='right')

        ax2.plot([nose_x, mlg_x], [nose_y, mlg_center_y], 'gray', linestyle='-', linewidth=1)

        # Draw CG
        cg_x = mlg_x - cg_to_mlg  
        cg_y = 0
        ax2.plot(cg_x, cg_y, 'ro', markersize=8)
        ax2.text(cg_x, cg_y + 0.3, 'CG', color='red', ha='center')

        info_text = (f"Weight: {weight:.1f} kg\n"
                     f"Index: {index:.2f}\n"
                     f"CG: {mac:.2f}% MAC\n"
                     f"CG to MLG: {cg_to_mlg:.2f} m\n"
                     f"Nosewheel load: {nosewheel_load:.1f}%")
        ax2.text(0.5, 0.05, info_text, transform=ax2.transAxes,
                 bbox=dict(facecolor='white', alpha=0.6), fontsize=9)
        
        return fig

# Usage example
def main():
    # Create an instance of the calculator
    calculator = ATR42CG()
    
    # Input weights for different sections (as variables)
    extra_crew = 1         # Additional crew member count, max. 1
    fuel = 3900            # Fuel weight (kg)
    
    # Cargo weights for each compartment
    cargo = {
        'A': 210,          # Compartment A cargo (kg) (3*70)
        'B': 280,          # Compartment B cargo (kg) (4*70)
        'C': 420,          # Compartment C cargo (kg) (6*70)
        'D': 520,          # Compartment D cargo (kg) (6*70)
        'E': 480,          # Compartment E cargo (kg) (4*70)
        'F': 400           # Compartment F cargo (kg)
    }
    
    # Perform CG calculation
    weight, index, mac, cg_position, cg_to_mlg, nosewheel_load, status = calculator.calculate_cg(
        crew_count=extra_crew,
        fuel_weight=fuel,
        cargo_weights=cargo
    )
    
    # Print results
    print(f"ATR 42-300 Center of Gravity Calculation Result:")
    print("-" * 60)
    print(f"Total Weight: {weight:.1f} kg")
    print(f"Total Index: {index:.2f}")
    print(f"CG Position: {mac:.2f}% MAC")
    print(f"CG Distance from Reference Point: {cg_position:.2f} m")
    print(f"Distance from CG to MLG: {abs(cg_to_mlg):.2f} m")
    print(f"Nosewheel Load: {nosewheel_load:.1f}%")
    print("-" * 60)
    
    # Print status messages
    print("Status:")
    for msg in status['messages']:
        print(f"- {msg}")
    
    # Print detailed weight and index information
    print("-" * 60)
    print("Detailed Weight and Index Data:")
    print(f"Basic Aircraft: {calculator.basic_weight} kg, Index: {calculator.basic_index}")
    
    if extra_crew > 0:
        crew_weight = extra_crew * 85
        crew_index = calculator.crew_index(extra_crew)
        print(f"Extra Crew: {crew_weight} kg, Index: {crew_index}")
    
    for section, section_weight in cargo.items():
        if section_weight > 0:
            section_index = calculator.cargo_index(section, section_weight)
            print(f"Cargo Compartment {section}: {section_weight} kg, Index: {section_index}")
    
    if fuel > 0:
        fuel_index = calculator.fuel_index(fuel)
        print(f"Fuel: {fuel} kg, Index: {fuel_index}")
    
    # Plot CG envelope and top-view CG position
    fig = calculator.visualization(weight, index, mac, cg_to_mlg, nosewheel_load)
    
    # Show plots
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()