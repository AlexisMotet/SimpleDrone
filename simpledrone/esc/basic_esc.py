from simpledrone.data import PulseWidthModulation, ESCOutput


class BasicESC:
    def update(self, pwm: PulseWidthModulation) -> ESCOutput:
        return ESCOutput(values=pwm.values)