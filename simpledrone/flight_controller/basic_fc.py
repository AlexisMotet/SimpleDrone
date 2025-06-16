from simpledrone.data import RCInputs, PulseWidthModulation

class BasicFC:

    def get_rx_to_fc_delay() -> float:
        return None


    def update(self, cmds: RCInputs) -> PulseWidthModulation:
        return PulseWidthModulation(values=[1.0])
    
