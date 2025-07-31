import numpy as np
from models.rotor import Rotor  # Adjust this import based on actual file structure
from config.params import params

def main():
    rotor = Rotor(params)

    # Define test state
    state = {
        "body_velocity": np.array([0.0, 0.0, 0.0]),  # u, v, w in ft/s
        "angular_rate": np.array([0.0, 0.0, 0.0]),  # p, q, r in rad/s
        "earth_velocity": np.array([0.0, 0.0, 0.0]),   # placeholder
        "vi_mr_prev": 33.44,
        "gv_7_prev": 0.0,
        "gv_8_prev": 0.0,
        "gr_7_prev": 0.0,
        "gr_8_prev": 0.0
    }

    # Define test controls: [collective, longitudinal swash, lateral swash]
    controls = np.deg2rad(np.array([9, 3.0, 0.0]))  # radian values

    # Define environment (sea level standard)
    environment = {
        "rho": 0.0023769  # slugs/ft³
    }

    result = rotor.get_force_and_moment(state, controls, environment)

    print("\n--- Main Rotor Output ---")
    for key, value in result.items():
        print(f"{key:10s}: {value:.4f}")

if __name__ == "__main__":
    main()
