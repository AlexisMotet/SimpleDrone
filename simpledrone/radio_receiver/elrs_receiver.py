import random
from simpledrone.data import RCInputs


class ExpressLRSReceiver:
    def __init__(self, packet_rate: float = 500.0, packet_loss_prob: float = 0.005, transmission_delay_ms: float = 1.5):
        self.packet_rate = packet_rate
        self.packet_loss_prob = packet_loss_prob
        self.transmission_delay = transmission_delay_ms * 1e-3
        self.pending_inputs = []
        self.prev_request_t = 0.0

    def get_frequency(self) -> float:
        return self.packet_rate

    def receive(self, tx_time: float, rc_inputs: RCInputs):
        if random.random() < self.packet_loss_prob:
            return
        self.pending_inputs.append((tx_time + self.transmission_delay, rc_inputs))

        assert len(self.pending_inputs) == 1 or self.pending_inputs[-2][0] <= self.pending_inputs[-1][0] # always increasing

    def get_rc_inputs(self, rx_time: float) -> RCInputs: 
        assert rx_time >= self.prev_request_t # always increasing
        self.prev_request_t = rx_time

        rc_input = RCInputs()

        cutoff_indice = 0

        for i, (rx_t_candidate, _) in enumerate(self.pending_inputs):
            if rx_time >= rx_t_candidate: # rx_t_candidate is increasing
                cutoff_indice = min(i + 1, len(self.pending_inputs) - 1)
                _, rc_input = self.pending_inputs[i]

        self.pending_inputs = self.pending_inputs[cutoff_indice:]

        return rc_input

    

        