from STservo_sdk import *
import math

BAUDRATE = 1000000
DEVICENAME = '/dev/ttyACM0'

TARGET_POSITION_THRESHOLD  = 20          # SCServo moving status threshold
MOVING_SPEED               = 50000         # SCServo moving speed
MOVING_ACCELERATION        = 255           # SCServo moving acc


class Robodog:
    def __init__(self):
        self.portHandler = PortHandler(DEVICENAME)
        self.servo_functions = sts(self.portHandler)

        # Open port and set baudrate
        if not self.portHandler.openPort() or not self.portHandler.setBaudRate(BAUDRATE):
            print("Failed to open port or set baudrate")
            quit()

        self.center_pos = 2048
        self.home_pos_angles = {
            1: 0,
            2: 45,
            3: -90,
            4: 0,
            5: 45,
            6: -90,
            7: 0,
            8: 45,
            9: -90,
            10: 0,
            11: 45,
            12: -90
        }
        self.home_pos_coordinates = {
            "front_left": {"x": 0, "y": -120},
            "front_right": {"x": 0, "y": -120},
            "back_left": {"x": 0, "y": -120},
            "back_right": {"x": 0, "y": -120}
        }

        self.lie_flat_coordinates = {
            "front_left": {"x": 180, "y": -60},
            "front_right": {"x": 180, "y": -60},
            "back_left": {"x": 0, "y": -50},
            "back_right": {"x": 0, "y": -50}
        }

        self.sit_coordinates = {
            "front_left": {"x": -10, "y": -180},
            "front_right": {"x": -10, "y": -180},
            "back_left": {"x": 0, "y": -50},
            "back_right": {"x": 0, "y": -50}
        }

        self.servo_directions = {
            1: "CW",
            2: "CW",
            3: "CW",
            4: "CCW",
            5: "CCW",
            6: "CCW",
            7: "CW",
            8: "CW",
            9: "CW",
            10: "CCW",
            11: "CCW",
            12: "CCW"
        }
        self.leg_configuration = {
            "front_right": {"hip": 7, "upper": 8, "lower": 9},
            "front_left": {"hip": 4, "upper": 5, "lower": 6},
            "back_left": {"hip": 10, "upper": 11, "lower": 12},
            "back_right": {"hip": 1, "upper": 2, "lower": 3}
        }

    def reset_position(self):
        self.move_legs_to_coordinates(self.home_pos_coordinates)

    def send_goal_position(self, servo_id, goal_position, moving_speed=MOVING_SPEED, moving_acceleration=MOVING_ACCELERATION):
        self.servo_functions.WritePosEx(servo_id, goal_position, moving_speed, moving_acceleration)

    def read_current_position(self, servo_id):
        sts_present_position, sts_present_speed, sts_comm_result, sts_error = self.servo_functions.ReadPosSpeed(servo_id)

        if sts_present_position == 0 and sts_present_speed == 0:
            return None, None

        return sts_present_position, sts_present_speed

    def move_servos_to_positions_at_same_time(self, target_angles):
        current_positions = {}
        distances = {}
        max_distance = 0

        # Calculate target positions from target angles
        target_positions = {servo_id: self._angle_to_servo_pos(servo_id, angle) for servo_id, angle in target_angles.items()}

        for servo_id, target_position in target_positions.items():
            current_position, _ = self.read_current_position(servo_id)
            if current_position is None:
                continue

            current_positions[servo_id] = current_position
            distance = abs(target_position - current_position)
            distances[servo_id] = distance
            max_distance = max(max_distance, distance)

        if max_distance == 0:
            return

        for servo_id, distance in distances.items():
            speed = int((distance / max_distance) * MOVING_SPEED)
            speed = max(1, min(speed, MOVING_SPEED)) 

            self.servo_functions.RegWritePosEx(servo_id, target_positions[servo_id], speed, MOVING_ACCELERATION)

        self.servo_functions.RegAction()

    def get_servo_angles(self, num_servos):
        current_angles = {}

        for servo_id in range(1, num_servos + 1):
            position, _ = self.read_current_position(servo_id)
            if position is not None:
                angle = self._servo_pos_to_angle(servo_id, position)
                current_angles[servo_id] = angle

        return current_angles
    
    def check_movement(self, num_servos):
        for servo_id in range(1, num_servos + 1):
            moving, _, _ = self.servo_functions.ReadMoving(servo_id)
            if moving:
                return True

        return False

    def calibrate_all_connected_servos_to_mid(self, num_of_connected_servos):
        for servo_id in range(1, num_of_connected_servos + 1):
            self.servo_functions.calibrationOfs(servo_id)

        print(self.get_servo_positions(num_of_connected_servos))
        return

    def set_leg_positions(self, home_pos_angles):
        leg_positions = {}

        for leg_name, servos in self.leg_configuration.items():
            leg_positions[leg_name] = {}
            for part_name, servo_id in servos.items():
                angle = home_pos_angles[part_name]
                servo_position = self.angle_to_servo_pos(servo_id, angle)
                leg_positions[leg_name][part_name] = servo_position

        return leg_positions
    

    def _angle_to_servo_pos(self, servo_id, angle):
        offset_per_degree = 500 / 45
        total_offset = angle * offset_per_degree

        if self.servo_directions[servo_id] == "CW":
            return int(self.center_pos + total_offset)
        elif self.servo_directions[servo_id] == "CCW":
            return int(self.center_pos - total_offset)
        else:
            raise ValueError("Invalid servo direction")
        
    def _servo_pos_to_angle(self, servo_id, servo_position):
        offset_per_degree = 500 / 45
        if self.servo_directions[servo_id] == "CW":
            total_offset = servo_position - self.center_pos
        elif self.servo_directions[servo_id] == "CCW":
            total_offset = self.center_pos - servo_position
        else:
            raise ValueError("Invalid servo direction")
        
        angle = total_offset / offset_per_degree
        return angle
    
    def _inverse_kinematics(self, x, y):
        L = 100
        y = -y
        x = -x

        D = math.sqrt(x**2 + y**2)

        if D > 2 * L:
            raise ValueError("Position is unreachable, too far away")

        angle_lower = math.acos((L**2 + L**2 - D**2) / (2 * L * L))

        theta1 = math.atan2(y, x)
        theta2 = math.acos((D**2 + L**2 - L**2) / (2 * D * L))
        angle_upper = theta1 - theta2

        angle_upper = math.degrees(angle_upper)
        angle_lower = math.degrees(angle_lower)

        angle_upper = 90 - angle_upper
        angle_lower = 180 - angle_lower
        
        return angle_upper, -angle_lower
    
    def move_legs_to_coordinates(self, coordinates):
        target_angles = {}
        for leg_name, coordinate in coordinates.items():
            upper_angle, lower_angle = self._inverse_kinematics(coordinate["x"], coordinate["y"])
            target_angles[self.leg_configuration[leg_name]["upper"]] = upper_angle
            target_angles[self.leg_configuration[leg_name]["lower"]] = lower_angle

        self.move_servos_to_positions_at_same_time(target_angles)

    def execute_movement_sequence(self, sequence):
        for movement in sequence:
            self.move_legs_to_coordinates(movement)
            # Check if the last servo has completed its movement
            while self.check_movement(12):  # Assuming this checks if servo 12 has completed its movement
                time.sleep(0.1)

robodog = Robodog()
robodog.move_servos_to_positions_at_same_time(robodog.home_pos_angles)

time.sleep(2)
base_height = -120
step_height = -80
step_length = 60
home_pos_coordinates = {
    "front_left": {"x": -60, "y": base_height},
    "front_right": {"x": -60, "y": base_height},
    "back_left": {"x": -60, "y": base_height},
    "back_right": {"x": -60, "y": base_height}
}
robodog.move_legs_to_coordinates(home_pos_coordinates)
time.sleep(2)

movement_sequence = [
    {
        "back_left": {"x": -step_length, "y": step_height},
    },       
    {
        "back_left": {"x": 0, "y": step_height},
    },       
    {
        "back_left": {"x": 0, "y": base_height},
    },
    {
        "front_left": {"x": 0, "y": base_height},
        "front_right": {"x": 0, "y": base_height},
        "back_left": {"x": step_length, "y": base_height},
        "back_right": {"x": 0, "y": base_height}
    },
        {
        "front_right": {"x": 0, "y": step_height},
    },       
    {
        "front_right": {"x": step_length, "y": step_height},
    },       
    {
        "front_right": {"x": step_length, "y": base_height},
    },
    {
        "front_left": {"x": -step_length, "y": base_height},
        "front_right": {"x": 0, "y": base_height},
        "back_left": {"x": 0, "y": base_height},
        "back_right": {"x": -step_length, "y": base_height}
    },
        {
        "back_right": {"x": -step_length, "y": step_height},
    },       
    {
        "back_right": {"x": 0, "y": step_height},
    },       
    {
        "back_right": {"x": 0, "y": base_height},
    },
        {
        "front_left": {"x": -step_length, "y": step_height},
    },       
    {
        "front_left": {"x": 0, "y": step_height}, #here
    },       
    {
        "front_left": {"x": 0, "y": base_height}, #here
    },
    {    "front_left": {"x": -60, "y": base_height},
        "front_right": {"x": -60, "y": base_height},
        "back_left": {"x": -60, "y": base_height},
        "back_right": {"x": -60, "y": base_height}
    }
]  



for i in range(2):
    robodog.execute_movement_sequence(movement_sequence)