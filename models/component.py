class Component:
    def __init__(self, name, config):
        self.name = name
        self.config = config

    def get_force_and_moment(self, state, controls, environment):
        """
        Base force method to be overridden.
        state: dict of flight state variables (airspeed, inputs, angles, etc.)
        """
        raise NotImplementedError(f"{self.name} must implement get_force().")

    def update_state(self, dt, state):
        """
        Optional override for time-dependent internal dynamics (e.g. flapping).
        """
        pass
