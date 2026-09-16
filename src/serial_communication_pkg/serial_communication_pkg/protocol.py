"""Wire protocol used by the supplied stroller Arduino firmware."""


def encode_command(steering, left_speed, right_speed):
    values = (int(steering), int(left_speed), int(right_speed))
    if abs(values[0]) > 7 or abs(values[1]) > 255 or abs(values[2]) > 255:
        raise ValueError('Arduino command outside steering/PWM limits')
    return f's{values[0]}l{values[1]}r{values[2]}\n'.encode('ascii')
