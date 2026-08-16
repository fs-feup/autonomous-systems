from std_msgs.msg import Float64


class RunningStats:
    def __init__(self) -> None:
        self.count = 0
        self.sum = 0.0
        self.abs_sum = 0.0
        self.sum_sq = 0.0

    def add(self, value: float) -> None:
        self.count += 1
        self.sum += value
        self.abs_sum += abs(value)
        self.sum_sq += value * value

    def average(self) -> float:
        return self.sum / self.count if self.count else 0.0

    def average_abs(self) -> float:
        return self.abs_sum / self.count if self.count else 0.0

    def mse(self) -> float:
        return self.sum_sq / self.count if self.count else 0.0


def publish_float(publisher, value: float) -> None:
    msg = Float64()
    msg.data = float(value)
    publisher.publish(msg)
