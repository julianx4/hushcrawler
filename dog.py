from STservo_sdk import *
import math
import numpy as np
import threading

BAUDRATE = 1000000
DEVICENAME = '/dev/ttyACM0'

TARGET_POSITION_THRESHOLD  = 200         # SCServo moving status threshold
MOVING_SPEED               = 50000         # SCServo moving speed
MOVING_ACCELERATION        = 0       # SCServo moving acc

class Robodog:
    def __init__(self):
        self.portHandler = PortHandler(DEVICENAME)
        self.servo_functions = sts(self.portHandler)
        self.controller_lock = threading.Lock()  # Lock for synchronizing access to the servo controller

        if not self.portHandler.openPort() or not self.portHandler.setBaudRate(BAUDRATE):
            print("Failed to open port or set baudrate")
            quit()

        self.center_pos = 2048

        self.servo_directions = {
            1: "CCW",
            2: "CW",
            3: "CW",
            4: "CCW",
            5: "CCW",
            6: "CCW",
            7: "CW",
            8: "CW",
            9: "CW",
            10: "CW",
            11: "CCW",
            12: "CCW"
        }
        self.leg_configuration = {
            "front_right": {"hip": 4, "upper": 5, "lower": 6},
            "front_left": {"hip": 7, "upper": 8, "lower": 9},
            "back_left": {"hip": 1, "upper": 2, "lower": 3},
            "back_right": {"hip": 10, "upper": 11, "lower": 12}
        }

        self.home_pos_coordinates = {
            "front_left": (0, -55, -120),
            "front_right": (0, -55, -120),
            "back_left": (0, -55, -120),
            "back_right": (0, -55, -120)
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

    def reset_position(self):
        self.move_legs_to_coordinates(self.home_pos_coordinates)

    def send_goal_servo_position(self, servo_id, goal_position, moving_speed=MOVING_SPEED, moving_acceleration=MOVING_ACCELERATION):
        self.servo_functions.WritePosEx(servo_id, goal_position, moving_speed, moving_acceleration)

    def read_current_servo_position(self, servo_id):
        sts_present_position, sts_present_speed, sts_comm_result, sts_error = self.servo_functions.ReadPosSpeed(servo_id)

        if sts_present_position == 0 and sts_present_speed == 0:
            return None, None

        return sts_present_position, sts_present_speed

    def move_servos_to_angles_at_same_time(self, target_angles):
        current_positions = {}
        distances = {}
        max_distance = 0

        # Calculate target positions from target angles
        target_positions = {servo_id: self._angle_to_servo_pos(servo_id, angle) for servo_id, angle in target_angles.items()}

        for servo_id, target_position in target_positions.items():
            current_position, _ = self.read_current_servo_position(servo_id)
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

    def is_leg_close_to_target(self, target_angles):
        for servo_id, target_angle in target_angles.items():
            current_position, _ = self.read_current_servo_position(servo_id)
            if current_position is None:
                return False

            target_position = self._angle_to_servo_pos(servo_id, target_angle)
            distance = abs(target_position - current_position)

            if distance > TARGET_POSITION_THRESHOLD:
                return False
        return True

    def get_servo_angles(self, num_servos):
        current_angles = {}

        for servo_id in range(1, num_servos + 1):
            position, _ = self.read_current_servo_position(servo_id)
            if position is not None:
                angle = self._servo_pos_to_angle(servo_id, position)
                current_angles[servo_id] = angle

        return current_angles

    def get_servo_movement(self, num_servos):
        for servo_id in range(1, num_servos + 1):
            moving, _, _ = self.servo_functions.ReadMoving(servo_id)
            if moving:
                return True

        return False
    
    def get_servo_load(self, servo_id):
        load, _, _ = self.servo_functions.ReadLoad(servo_id)
        return load

    def calibrate_all_connected_servos_to_mid(self, num_of_connected_servos):
        for servo_id in range(1, num_of_connected_servos + 1):
            self.servo_functions.calibrationOfs(servo_id)

        print(self.get_servo_angles(num_of_connected_servos))
        return
    
    def _angle_to_servo_pos(self, servo_id, angle):
        offset_per_degree = 4096 / 360
        total_offset = angle * offset_per_degree

        servo_pos = int(self.center_pos + total_offset) if self.servo_directions[servo_id] == "CW" else int(self.center_pos - total_offset)

        return servo_pos
    
    def _servo_pos_to_angle(self, servo_id, servo_position):
        offset_per_degree = 4096 / 360
        if self.servo_directions[servo_id] == "CW":
            total_offset = servo_position - self.center_pos
        elif self.servo_directions[servo_id] == "CCW":
            total_offset = self.center_pos - servo_position
        else:
            raise ValueError("Invalid servo direction")
        
        angle = total_offset / offset_per_degree
        return angle
    
    def checkdomain(self, D):
        if D > 1 or D < -1:
            print("____OUT OF DOMAIN____")
            if D > 1: 
                D = 0.99
                return D
            elif D < -1:
                D = -0.99
                return D
        else:
            return D

    def _inverse_kinematics(self, coord):
        coxa = 55
        femur = 100
        tibia = 125

        D = (coord[1]**2+(-coord[2])**2-coxa**2+(-coord[0])**2-femur**2-tibia**2)/(2*tibia*femur) 
        D = self.checkdomain(D)
        gamma = np.arctan2(-np.sqrt(1-D**2),D)
        tetta = -np.arctan2(coord[2],coord[1])-np.arctan2(np.sqrt(coord[1]**2+(-coord[2])**2-coxa**2),-coxa)
        alpha = np.arctan2(-coord[0],np.sqrt(coord[1]**2+(-coord[2])**2-coxa**2))-np.arctan2(tibia*np.sin(gamma),femur+tibia*np.cos(gamma))
        angles = np.array([-tetta, alpha, gamma])
        return angles
        
    def move_legs_to_coordinates(self, coordinates_dict):
        for leg_name, coordinate in coordinates_dict.items():
            x, y, z = coordinate

            target_angles = {}
            leg_config = self.leg_configuration[leg_name]
            angles = self._inverse_kinematics((x, y, z))
            target_angles[leg_config["hip"]] = math.degrees(angles[0])
            target_angles[leg_config["upper"]] = math.degrees(angles[1])
            target_angles[leg_config["lower"]] = math.degrees(angles[2])

            self.move_servos_to_angles_at_same_time(target_angles)
            print(leg_name, target_angles)

    def execute_movement_sequence(self, sequence):
        for movement in sequence:
            self.move_legs_to_coordinates(movement)
            # Check if the last servo has completed its movement
            while self.get_servo_movement(12):
                time.sleep(0.01)

    def move_leg_to_coordinate(self, leg_name, coordinate):
        """
        Move a single leg to the specified coordinate.

        :param leg_name: Name of the leg to move.
        :param coordinate: Tuple of (x, y, z) coordinates.
        """
        target_angles = {}
        leg_config = self.leg_configuration[leg_name]
        angles = self._inverse_kinematics(coordinate)
        target_angles[leg_config["hip"]] = math.degrees(angles[0])
        target_angles[leg_config["upper"]] = math.degrees(angles[1])
        target_angles[leg_config["lower"]] = math.degrees(angles[2])

        self.move_servos_to_angles_at_same_time(target_angles)

    def circular_motion(self, midpoint, radius, offset, steps, reverse=False):
        """
        Make each foot perform a circular motion in the X-Z plane with a defined midpoint in 3D space.

        :param midpoint: A dictionary with the midpoints for each leg's circle in the format {"front_left": (x, y, z), ...}
        :param radius: The radius of the circle.
        :param offset: A dictionary with the offset for each leg in degrees {"front_left": offset_angle, ...}
        :param steps: The number of steps to complete one circular motion.
        :param reverse: Boolean to indicate if the direction of the circle should be reversed.
        """
        for step in range(steps):
            for leg_name, mid in midpoint.items():
                if not reverse:
                    angle = np.radians((360 - (step * 360 / steps) + offset[leg_name]) % 360)
                else:
                    angle = np.radians((step * 360 / steps + offset[leg_name]) % 360)
                x = mid[0] + radius * np.cos(angle)
                z = mid[2] + radius * np.sin(angle)
                y = mid[1]  # Y-coordinate is taken from the midpoint

                self.move_leg_to_coordinate(leg_name, (x, y, z))
            time.sleep(0.1)  # Adjust the sleep time for the desired speed of motion

    def elliptical_motion(self, midpoint, major_radius, minor_radius, offset, steps, reverse=False):

        for step in range(steps):
            target_angles = {}
            for leg_name, mid in midpoint.items():
                if not reverse:
                    angle = np.radians((360 - (step * 360 / steps) + offset[leg_name]) % 360)
                else:
                    angle = np.radians((step * 360 / steps + offset[leg_name]) % 360)
                #x = mid[0] + major_radius * np.sign(np.cos(angle))  # X-coordinate follows major 'radius'
                #z = mid[2] + minor_radius * np.sign(np.sin(angle))  # Z-coordinate follows minor 'radius'
           
                x = mid[0] + major_radius * np.cos(angle)  # X-coordinate follows major radius
                z = mid[2] + minor_radius * np.sin(angle)  # Z-coordinate follows minor radius
                y = mid[1]  # Y-coordinate is constant

                
                leg_config = self.leg_configuration[leg_name]
                angles = self._inverse_kinematics((x,y,z))
                target_angles[leg_config["hip"]] = math.degrees(angles[0])
                target_angles[leg_config["upper"]] = math.degrees(angles[1])
                target_angles[leg_config["lower"]] = math.degrees(angles[2])

            self.move_servos_to_angles_at_same_time(target_angles)
            while not self.is_leg_close_to_target(target_angles):
                pass



robodog = Robodog()
robodog.move_legs_to_coordinates(robodog.home_pos_coordinates)
# while True:
#     robodog.send_goal_servo_position(7,1548)
#     time.sleep(0.31)
#     robodog.send_goal_servo_position(7,2000)
#     time.sleep(0.31)