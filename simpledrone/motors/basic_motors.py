from simpledrone.data import ESCOutput, RevolutionsPerMinute

class BasicMotors:
    def __init__(self, max_rpm=10000):
        self.max_rpm = max_rpm

    def update(self, esc_output: ESCOutput) -> RevolutionsPerMinute:
        rpm = [val * self.max_rpm for val in esc_output.values]
        return RevolutionsPerMinute(values=rpm)