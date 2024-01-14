from STservo_sdk import *
import math
import numpy as np

BAUDRATE = 1000000
DEVICENAME = '/dev/ttyACM0'

TARGET_POSITION_THRESHOLD  = 20          # SCServo moving status threshold
MOVING_SPEED               = 500         # SCServo moving speed
MOVING_ACCELERATION        = 255       # SCServo moving acc


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

        D = (coord[1]**2+(-coord[2])**2-coxa**2+(-coord[0])**2-femur**2-tibia**2)/(2*tibia*femur)  #siempre <1
        D = self.checkdomain(D)
        gamma = np.arctan2(-np.sqrt(1-D**2),D)
        tetta = -np.arctan2(coord[2],coord[1])-np.arctan2(np.sqrt(coord[1]**2+(-coord[2])**2-coxa**2),-coxa)
        alpha = np.arctan2(-coord[0],np.sqrt(coord[1]**2+(-coord[2])**2-coxa**2))-np.arctan2(tibia*np.sin(gamma),femur+tibia*np.cos(gamma))
        angles = np.array([-tetta, alpha, gamma])
        return angles
    
    def _inverse_kinematics_old(_oldself, x, y):
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
        
    def move_legs_to_coordinates(self, coordinates_dict):
        for leg_name, coordinate in coordinates_dict.items():
            x, y, z = coordinate

            target_angles = {}
            leg_config = self.leg_configuration[leg_name]
            angles = self._inverse_kinematics((x, y, z))
            target_angles[leg_config["hip"]] = math.degrees(angles[0])
            target_angles[leg_config["upper"]] = math.degrees(angles[1])
            target_angles[leg_config["lower"]] = math.degrees(angles[2])

            self.move_servos_to_positions_at_same_time(target_angles)
            print(leg_name, target_angles)


    def execute_movement_sequence(self, sequence):
        for movement in sequence:
            self.move_legs_to_coordinates(movement)
            # Check if the last servo has completed its movement
            while self.check_movement(12):
                time.sleep(0.01)

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
        """
        Make each foot perform an elliptical motion in the X-Z plane with a defined midpoint in 3D space.

        :param midpoint: A dictionary with the midpoints for each leg's ellipse in the format {"front_left": (x, y, z), ...}
        :param major_radius: The major radius of the ellipse.
        :param minor_radius: The minor radius of the ellipse.
        :param offset: A dictionary with the offset for each leg in degrees {"front_left": offset_angle, ...}
        :param steps: The number of steps to complete one elliptical motion.
        :param reverse: Boolean to indicate if the direction of the ellipse should be reversed.
        """
        for step in range(steps):
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

                self.move_leg_to_coordinate(leg_name, (x, y, z))
            time.sleep(0.04)  # Adjust the sleep time for the desired speed of motion


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

        self.move_servos_to_positions_at_same_time(target_angles)

robodog = Robodog()
robodog.move_servos_to_positions_at_same_time(robodog.home_pos_angles)
time.sleep(0.3)

# Usage example:
base_height = 150
midpoints = {
    "front_left": (-10, -65, -base_height),
    "front_right": (-10, -65, -base_height),
    "back_left": (-20, -65, -base_height),
    "back_right": (-20, -65, -base_height)
}
radius = 15  # example radius
major_radius = 60  # example major radius
minor_radius = 20  # example minor radius
offsets = {
    "front_left": 0,
    "front_right": 180,
    "back_left": 180,
    "back_right": 0
}
steps = 20

while True:
    
    #robodog.circular_motion(midpoints, radius, offsets, steps, reverse=False)
    robodog.elliptical_motion(midpoints, major_radius, minor_radius, offsets, steps, reverse=False)

"""
time.sleep(0.5)
base_height = 130
step_height = 30
step_length = 60
home_pos_coordinates = {
    "front_left": (0, -55, -base_height),
    "front_right": (0, -55, -base_height),
    "back_left": (0, -55, -base_height),
    "back_right":(0, -55, -base_height)
}
robodog.move_legs_to_coordinates(home_pos_coordinates)
time.sleep(2)

movement_sequence = [
    {"front_left": (50, -55, -50),
     "front_right": (50, -55, -50)
    },
    {"front_left": (50, -55, -180),
     "front_right": (50, -55, -180)
    },
    {"front_left": (50, -55, -50),
     "front_right": (50, -55, -50)
    },
]


robodog.execute_movement_sequence(movement_sequence)

movement_sequence = [
    {"front_left": (0, -55, -base_height - 30),
     "back_left": (0, -55, -base_height - 30)
    },
    {"front_left": (0, -55, -base_height + step_height),
     "back_left": (0, -55, -base_height - 30)
    },
    {"front_left": (step_length, -55, -base_height + step_height),
     "back_left": (0, -55, -base_height - 30)
    },
    {"front_left": (step_length, -55, -base_height - 30),
     "back_left": (0, -55, -base_height - 30)
    },
    {"front_left": (step_length, -55, -base_height),
     "back_left": (0, -55, -base_height)
    },
    {"front_right": (0, -55, -base_height - 30),
     "back_right": (0, -55, -base_height -30)
    },
    {"front_right": (0, -55, -base_height - 30),
     "back_right": (0, -55, -base_height + step_height)
    },
    {"front_right": (0, -55, -base_height - 30),
     "back_right": (step_length, -55, -base_height + step_height)
    },
    {"front_right": (0, -55, -base_height - 30),
     "back_right": (step_length, -55, -base_height - 30)
    },
    {"front_right": (0, -55, -base_height),
     "back_right": (step_length, -55, -base_height)
    },
    {"front_left": (-step_length, -55, -base_height),
     "back_right": (-step_length, -55, -base_height)
    },
    {"front_right": (0, -55, -base_height - 30),
     "back_right": (-step_length, -55, -base_height - 30)
    }

]  



# robodog.execute_movement_sequence(movement_sequence)

"""