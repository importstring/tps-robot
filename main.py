import motor
import color_sensor
import runloop
from hub import port, device
import motor_pair
import color

# Navigation states
LINE_FOLLOWING = 1
INTERSECTION_DETECTION = 2
ERROR_RECOVERY = 3

# PID controller parameters
Kp = 0.8
Ki = 0.1
Kd = 0.2
SETPOINT = 85  # Adjusted for white line detection [1]
BASE_SPEED = 360  # deg/sec for large motors (max 1050)
MAX_STEERING = 100  # PID output limit

class PIDController:
    def __init__(self):
        self.error_sum = 0
        self.last_error = 0

    def compute_steering(self, reflection):
        error = SETPOINT - reflection
        self.error_sum = max(-1000, min(self.error_sum + error, 1000))  # Anti-windup
        derivative = error - self.last_error
        output = (Kp * error) + (Ki * self.error_sum) + (Kd * derivative)
        self.last_error = error
        return int(max(-MAX_STEERING, min(output, MAX_STEERING)))

# Initialize PID controller
pid = PIDController()

async def navigate():
    # Validate hardware configuration [2]
    if device.id(port.C) != 61 or device.id(port.F) != 61:
        raise Exception("Color sensors required on ports C and F")
    if device.id(port.A) not in [48,49] or device.id(port.B) not in [48,49]:
        raise Exception("Large motors required on ports A/B")

    motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)
    state = LINE_FOLLOWING

    while True:
        bottom_reflection = color_sensor.reflection(port.C)
        front_reflection = color_sensor.reflection(port.F)

        if state == LINE_FOLLOWING:
            steering = pid.compute_steering(bottom_reflection)
            steering_scaled = steering * 3.6  # Scale to ±360 deg/s [3]
            
            # Calculate and clamp motor speeds [4]
            left_speed = max(-1050, min(BASE_SPEED - steering_scaled, 1050))
            right_speed = max(-1050, min(BASE_SPEED + steering_scaled, 1050))
            
            motor_pair.move_tank(
                motor_pair.PAIR_1,
                int(left_speed),
                int(right_speed),
                acceleration=800,
                deceleration=800
            )

            # State transitions [5]
            if front_reflection < 20:  # Black boundary
                state = ERROR_RECOVERY
            elif front_reflection > 80:  # White intersection
                state = INTERSECTION_DETECTION

        elif state == INTERSECTION_DETECTION:
            # 90° turn using degrees calculation [6]
            await motor_pair.move_for_degrees(
                motor_pair.PAIR_1,
                180,  # 180° rotation for differential drive
                steering=100,  # Full right turn
                velocity=540,
                acceleration=1000,
                deceleration=1000
            )
            state = LINE_FOLLOWING

        elif state == ERROR_RECOVERY:
            # Physics-based recovery [7]
            await motor_pair.move_tank_for_time(
                motor_pair.PAIR_1,
                -360, -360,  # Reverse both motors
                1000,  # 1 second reverse
                stop=motor.BRAKE
            )
            await motor_pair.move_for_degrees(
                motor_pair.PAIR_1,
                90,  # 45° search pattern
                steering=100,
                velocity=360,
                acceleration=1000
            )
            state = LINE_FOLLOWING

        await runloop.sleep_ms(10)

async def main():
    await navigate()

runloop.run(main())
