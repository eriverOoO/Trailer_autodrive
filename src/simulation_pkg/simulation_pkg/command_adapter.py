"""Pure MotionCommand-to-Gazebo unit conversion."""


class SendSignal:
    def __init__(self, max_speed, max_steer=0.6458,
                 steering_direction=1, drive_direction=1):
        self.max_speed = float(max_speed)
        self.max_steer = float(max_steer)
        self.steering_direction = int(steering_direction)
        self.drive_direction = int(drive_direction)

    def map_to_steer(self, value):
        return max(-7, min(7, value)) / 7.0 * self.max_steer

    def map_to_speed(self, value):
        return max(-255, min(255, value)) / 255.0 * self.max_speed

    def process(self, motor):
        steer = self.map_to_steer(self.steering_direction * motor.steering)
        left = self.map_to_speed(self.drive_direction * motor.left_speed)
        right = self.map_to_speed(self.drive_direction * motor.right_speed)
        return steer, left, right
