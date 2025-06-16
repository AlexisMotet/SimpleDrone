import random
from simpledrone.data import RCInputs


class ExpressLRSReceiver:
    def __init__(self, packet_rate=500.0, packet_loss_prob=0.005, over_the_air_latency=0.001, rx_to_fc_delay=0.001):
        self.packet_interval = 1.0 / packet_rate
        self.packet_loss_prob = packet_loss_prob
        self.over_the_air_latency = over_the_air_latency
        self.rx_to_fc_delay = rx_to_fc_delay
        self.last_t = 0.0
        self.last_commands = RCInputs()

    def get_transmission_delay(self):
        return self.packet_interval + self.over_the_air_latency
    
    def get_rx_to_fc_delay(self):
        return self.rx_to_fc_delay

    def receive(self, t: float, rc_inputs: RCInputs) -> RCInputs:
        if t - self.last_t < self.packet_interval:
            return self.last_commands
        self.last_t = t
        if random.random() < self.packet_loss_prob:
            return self.last_commands
        self.last_commands = rc_inputs
        return rc_inputs
    

        