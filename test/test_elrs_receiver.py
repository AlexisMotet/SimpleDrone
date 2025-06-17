import unittest
from simpledrone.radio_receiver.elrs_receiver import ExpressLRSReceiver
from simpledrone.data import RCInputs

class TestExpressLRSReceiver(unittest.TestCase):
    def test_receive_and_get_rc_inputs(self):
        receiver = ExpressLRSReceiver(packet_rate=500.0, packet_loss_prob=0.0, transmission_delay_ms=2.0) # 2 ms delay

        rc_input_1 = RCInputs(throttle=0.1, roll=0.0, pitch=0.0, yaw_rate=0.0)
        rc_input_2 = RCInputs(throttle=0.5, roll=0.5, pitch=-0.1, yaw_rate=0.2)
        rc_input_3 = RCInputs(throttle=0.2, roll=0.5, pitch=-0.2, yaw_rate=0.3)
        rc_input_4 = RCInputs(throttle=0.4, roll=0.1, pitch=-0.5, yaw_rate=0.4)
        rc_input_5 = RCInputs(throttle=0.5, roll=0.2, pitch=-0.2, yaw_rate=0.5)
        rc_input_6 = RCInputs(throttle=0.1, roll=0.1, pitch=-0.1, yaw_rate=0.3)
        
        receiver.receive(tx_time=0.001, rc_inputs=rc_input_1) # rc inputs are stored in a queue
        receiver.receive(tx_time=0.002, rc_inputs=rc_input_2) # calls should be done with increasing tx_time !
        receiver.receive(tx_time=0.003, rc_inputs=rc_input_3)
        receiver.receive(tx_time=0.004, rc_inputs=rc_input_4)
        receiver.receive(tx_time=0.005, rc_inputs=rc_input_5)
        receiver.receive(tx_time=0.006, rc_inputs=rc_input_6)

        self.assertEqual(len(receiver.pending_inputs), 6)

        self.assertEqual(receiver.get_rc_inputs(rx_time=0.001), RCInputs()) # rc inputs are poped, only coherent rc inputs will be poped
                                                                            # calls should be done with increasing rx_time !
        self.assertEqual(receiver.get_rc_inputs(rx_time=0.002), RCInputs())

        self.assertEqual(receiver.get_rc_inputs(rx_time=0.003), rc_input_1)

        self.assertEqual(receiver.get_rc_inputs(rx_time=0.0035), RCInputs())

        self.assertEqual(receiver.get_rc_inputs(rx_time=0.004), rc_input_2)

        self.assertEqual(receiver.get_rc_inputs(rx_time=0.0048), RCInputs())

        self.assertEqual(receiver.get_rc_inputs(rx_time=0.005), rc_input_3)

        self.assertEqual(receiver.get_rc_inputs(rx_time=0.005), RCInputs())


if __name__ == "__main__":
    unittest.main()