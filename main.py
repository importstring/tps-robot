import motor
import color_sensor
import runloop
from hub import port
import motor_pair

# Navigation states
LINE_FOLLOWING = 1
INTERSECTION_DETECTION = 2
TURN_EXECUTION = 3
ERROR_RECOVERY = 4

# PID controller parameters
Kp = 0.8
Ki = 0.1
Kd = 0.2
setpoint = 65
BASE_SPEED = 360  # deg/sec for large motors

class PIDController:
    def __init__(self):
        self.error_sum = 0
        self.last_error = 0

    def compute_steering(self, reflection):
        error = setpoint - reflection
        self.error_sum += error
        derivative = error - self.last_error
        output = (Kp * error) + (Ki * self.error_sum) + (Kd * derivative)
        self.last_error = error
        return int(max(-100, min(output, 100)))

# Initialize PID controller
pid = PIDController()

async def navigate():
    state = LINE_FOLLOWING
    motor_pair.pair(motor_pair.PAIR_1, port.A, port.B)

    while True:
        bottom_reflection = color_sensor.reflection(port.C)
        front_reflection = color_sensor.reflection(port.F)

        if state == LINE_FOLLOWING:
            steering = pid.compute_steering(bottom_reflection)
            steering_scaled = steering * 2.5
            
            left_speed = max(-1050, min(int(BASE_SPEED - steering_scaled), 1050))
            right_speed = max(-1050, min(int(BASE_SPEED + steering_scaled), 1050))
            
            motor_pair.move_tank(
                motor_pair.PAIR_1,
                left_speed,
                right_speed,
                acceleration=800,
                deceleration=800
            )

            if front_reflection < 20:
                state = ERROR_RECOVERY
            elif front_reflection > 70:
                state = INTERSECTION_DETECTION

        elif state == INTERSECTION_DETECTION:
            await motor_pair.move_tank_for_time(
                motor_pair.PAIR_1,
                -540,  
                540,   
                1000,  
                velocity=540
            )
            state = LINE_FOLLOWING

        elif state == ERROR_RECOVERY:
            await motor_pair.move_tank_for_time(
                motor_pair.PAIR_1,
                -360,  
                -360,  
                500,   
                velocity=-360
            )
            await motor_pair.move_tank_for_time(
                motor_pair.PAIR_1,
                540,
                -540,
                1000,
                velocity=540
            )
            state = LINE_FOLLOWING

        await runloop.sleep_ms(10)

async def main():
    await navigate()

runloop.run(main())

