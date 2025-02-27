import motor
import color_sensor
import color
import runloop
from hub import port, motion_sensor
import motor_pair

# Configuration (SPIKE Prime Docs v2.3.2 verified)
MOTOR_PAIR = motor_pair.PAIR_1
LEFT_PORT = port.A# Confirmed left motor
RIGHT_PORT = port.B# Confirmed right motor
FRONT_SENSOR = port.F# Front path sensor (under front)
CENTER_SENSOR = port.C# Center path sensor (under middle)

# Competition-tuned parameters (Search Result 1 optimized)
BASE_SPEED = 300# deg/sec (reduced for control)
TURN_SPEED = 200# deg/sec (Search Result 7)
BLACK_THRESHOLD = 5 # Documentation-valid reflection value
DEBOUNCE_MS = 75    # Search Result 1 recommendation

class MazeSolver:
    def __init__(self):
        self.last_steering = 0

    async def calibrate_sensors(self):
        """Triple-sample calibration"""
        front_samples = [color_sensor.reflection(FRONT_SENSOR) for _ in range(3)]
        center_samples = [color_sensor.reflection(CENTER_SENSOR) for _ in range(3)]
        self.front_black = sum(front_samples) / len(front_samples)# Use float division
        self.center_black = sum(center_samples) / len(center_samples)# Use float division


    async def path_correction(self):
        """Dual-sensor centering algorithm (Search Result 5)"""
        front = color_sensor.reflection(FRONT_SENSOR)
        center = color_sensor.reflection(CENTER_SENSOR)

        # Calculate steering error
        error = (front - self.front_black) - (center - self.center_black)
        steering = max(-100, min(error * 2, 100))# Tuned multiplier

        motor_pair.move(
            MOTOR_PAIR,
            int(steering),
            velocity=BASE_SPEED,
            acceleration=800# Docs-compliant value
        )
        self.last_steering = steering

    async def emergency_turn(self):
        # Back up slowly to avoid hitting the wall
        await motor_pair.move_tank_for_time(MOTOR_PAIR, 500, -100, -100)

        # Perform a wider turn to avoid the wall
        await motor_pair.move_tank_for_time(MOTOR_PAIR, 1000, 200, -200)

    async def main_loop(self):
        while True:
            try:
                front_val = color_sensor.reflection(FRONT_SENSOR)
                center_val = color_sensor.reflection(CENTER_SENSOR)

                if color_sensor.color(port.C) is not color.BLACK:

                    for i in range(1000):

                        if color_sensor.color(port.C) is not color.BLACK:

                            motor_pair.pair(motor_pair.PAIR_3, port.A, port.B)

                            motor_pair.move_for_time(motor_pair.PAIR_3, 1, 0)
                            motor.run(port.B, 50000)

                            break
                        
                        motor.run(port.A, 10000)

                elif color_sensor.color(port.F) is not color.BLACK:

                    for i in range(50):

                        if color_sensor.color(port.C) is not color.BLACK:

                            motor_pair.pair(motor_pair.PAIR_3, port.A, port.B)
                            
                            motor_pair.move_for_time(motor_pair.PAIR_3, 100, 0)
                            motor.run(port.A, 1000000)

                            break

                        motor_pair.pair(motor_pair.PAIR_3, port.A, port.B)

                        motor_pair.move_for_time(motor_pair.PAIR_3, 20, 0)
                        motor.run(port.B, 10000)
                else:
                    if front_val == 0 and center_val == 0:
                        motor_pair.move_for_time(motor_pair.PAIR_3, 2000, 0)
                    else:
                        await self.path_correction()



                await runloop.sleep_ms(DEBOUNCE_MS)

            except Exception as e:

                print("ERROR:", e)
                break

async def main():
    bot = MazeSolver()
    await bot.calibrate_sensors()

    try:
        motor_pair.pair(MOTOR_PAIR, LEFT_PORT, RIGHT_PORT)
        motor.reset_relative_position(LEFT_PORT, 0)
        motor.reset_relative_position(RIGHT_PORT, 0)

        await bot.main_loop()

    finally:
        motor.stop(LEFT_PORT, stop=motor.BRAKE)
        motor.stop(RIGHT_PORT, stop=motor.BRAKE)
        await runloop.sleep_ms(500)

runloop.run(main())
