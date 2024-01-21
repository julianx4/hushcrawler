from STservo_sdk import *
import math
import numpy as np
import threading, queue
import board
import busio
import adafruit_bno055
import time


BAUDRATE = 1000000
DEVICENAME = '/dev/ttyACM0'

TARGET_POSITION_THRESHOLD  = 40
MOVING_SPEED               = 5000
MOVING_ACCELERATION        = 0

class BNO055_Sensor:
    def __init__(self):
        self.i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno055.BNO055_I2C(self.i2c)

    def read_euler(self):
        return self.sensor.euler
    

class ThreadSafeServoController:
    def __init__(self, portHandler):
        self.servo_functions = sts(portHandler)
        self.lock = threading.Lock()
    
    def send_speed_acceleration(self, servo_id, goal_position, moving_speed=MOVING_SPEED, moving_acceleration=MOVING_ACCELERATION):
        with self.lock:
            self.servo_functions.RegWritePosEx(servo_id, goal_position, moving_speed, moving_acceleration)
    
    def execute_servo_action(self):
        with self.lock:
            self.servo_functions.RegAction()

    def read_current_servo_position(self, servo_id):
        with self.lock:
            position, _, _ = self.servo_functions.ReadPos(servo_id)
            return position, None
    
    def read_servo_load(self, servo_id):
        with self.lock:
            load, _, _ = self.servo_functions.ReadLoad(servo_id)
            return load, None
        

class LegThread(threading.Thread):
    def __init__(self, leg_name, controller, command_queue, status_dict, status_dict_lock):
        super().__init__()
        self.leg_name = leg_name
        self.controller = controller
        self.command_queue = command_queue
        self.status_dict_lock = status_dict_lock
        self.status_dict = status_dict
        self.leg_configuration = {
            "front_right": {"hip": 4, "upper": 5, "lower": 6},
            "front_left": {"hip": 7, "upper": 8, "lower": 9},
            "back_left": {"hip": 1, "upper": 2, "lower": 3},
            "back_right": {"hip": 10, "upper": 11, "lower": 12}
        }

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
        self.should_stop = threading.Event()
        print("LegThread initialized: ", leg_name)
    
    def run(self):
        while not self.should_stop.is_set():
            try:
                command = self.command_queue.get(timeout = 0.1)
                if command['action'] == 'move':
                    self.move_to_coordinate(command['coordinate'])
                elif command['action'] == 'elliptical_motion':
                    self.elliptical_motion(command['midpoints'],
                                           command['major_radius'],
                                           command['minor_radius'],
                                           command['offsets'],
                                           command['steps'])
                self.command_queue.task_done()
            except queue.Empty:
                continue
    
    def move_to_coordinate(self, coordinate):
        target_angles = {}
        leg_config = self.leg_configuration[self.leg_name]
        angles = self._inverse_kinematics(coordinate)
        target_angles[leg_config["hip"]] = math.degrees(angles[0])
        target_angles[leg_config["upper"]] = math.degrees(angles[1])
        target_angles[leg_config["lower"]] = math.degrees(angles[2])

        self.move_servos_to_angles_at_same_time(target_angles)

    def move_servos_to_angles_at_same_time(self, target_angles):
        current_positions = {}
        distances = {}
        max_distance = 0

        # Calculate target positions from target angles
        target_positions = {servo_id: self._angle_to_servo_pos(servo_id, angle) for servo_id, angle in target_angles.items()}

        for servo_id, target_position in target_positions.items():
            current_position, _ = self.controller.read_current_servo_position(servo_id)
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

            self.controller.send_speed_acceleration(servo_id, target_positions[servo_id], speed, MOVING_ACCELERATION)
        self.controller.execute_servo_action()

    def is_leg_close_to_target(self, target_angles):
        for servo_id, target_angle in target_angles.items():
            current_position, _ = self.controller.read_current_servo_position(servo_id)
            if current_position is None:
                return False

            target_position = self._angle_to_servo_pos(servo_id, target_angle)
            distance = abs(target_position - current_position)

            if distance > TARGET_POSITION_THRESHOLD:
                return False
        return True
    
    def elliptical_motion(self, midpoint, major_radius, minor_radius, offset, steps, reverse=False):
        for step in range(steps):
            target_angles = {}
            leg_name = self.leg_name
            mid = midpoint[leg_name]
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

    def update_status(self):
        leg_config = self.leg_configuration[self.leg_name]
        new_status = {
            "servos": {}
        }
        for part_name, servo_id in leg_config.items():
            position, _ = self.controller.read_current_servo_position(servo_id)
            load, _ = self.controller.read_servo_load(servo_id)
            new_status["servos"][servo_id] = {"link": part_name,"position": position, "load": load}
        
        with self.status_dict_lock:
            self.status_dict[self.leg_name] = new_status

            
    def stop(self):
        self.should_stop.set()

    def _checkdomain(self, D):
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
        D = self._checkdomain(D)
        gamma = np.arctan2(-np.sqrt(1-D**2),D)
        tetta = -np.arctan2(coord[2],coord[1])-np.arctan2(np.sqrt(coord[1]**2+(-coord[2])**2-coxa**2),-coxa)
        alpha = np.arctan2(-coord[0],np.sqrt(coord[1]**2+(-coord[2])**2-coxa**2))-np.arctan2(tibia*np.sin(gamma),femur+tibia*np.cos(gamma))
        angles = np.array([-tetta, alpha, gamma])
        return angles

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


class Robodog:
    def __init__(self):
        self.portHandler = PortHandler(DEVICENAME)
        if not self.portHandler.openPort() or not self.portHandler.setBaudRate(BAUDRATE):
            print("Failed to open port or set baudrate")
            quit()

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

        self.controller = ThreadSafeServoController(self.portHandler)
        self.command_queues = {leg: queue.Queue() for leg in ['front_left', 'front_right', 'back_left', 'back_right']}
        self.status_dict_lock = threading.Lock()
        self.status_dict = {}  # Shared, thread-safe structure to track leg status
        self.legs = {
            leg_name: LegThread(
                leg_name, 
                self.controller, 
                self.command_queues[leg_name], 
                self.status_dict, 
                self.status_dict_lock  # Pass the lock to the LegThread
            ) for leg_name in self.command_queues
        }        

        for leg in self.legs.values():
            leg.start()

    def move_to_home_coords(self):
        for leg_name, coordinates in self.home_pos_coordinates.items():
            self.command_queues[leg_name].put({
                'action': 'move',
                'coordinate': coordinates
            })

    def elliptical_motion_all_legs(self, midpoints, major_radius, minor_radius, offsets, steps):
        for leg_name in self.legs:
            self.command_queues[leg_name].put({
                'action': 'elliptical_motion',
                'midpoints': midpoints,
                'major_radius': major_radius,
                'minor_radius': minor_radius,
                'offsets': offsets,
                'steps': steps
            })
    
    def stop_all_legs(self):
        for leg in self.legs.values():
            leg.stop()
            leg.join()
    
robodog = Robodog()

robodog.move_to_home_coords()
base_height = 150
major_radius = 50
minor_radius = 20
offsets = {
    "front_left": 0,
    "front_right": 180,
    "back_left": 180,
    "back_right": 0
}
steps = 15
midpoints = {
    "front_left": (-10, -65, -base_height),
    "front_right": (-10, -65, -base_height),
    "back_left": (-20, -65, -base_height),
    "back_right": (-20, -65, -base_height)
}

#move rear right leg to the side
robodog.command_queues["back_right"].put({
    'action': 'move',
    'coordinate': (0, -55, -90)
})

time.sleep(1)

robodog.command_queues["back_right"].put({
    'action': 'move',
    'coordinate': (0, -55, -130)
})

try:
    while True:
        # Get and update the status for each leg
        for leg_name, leg_thread in robodog.legs.items():
            leg_status = leg_thread.update_status()



        time.sleep(0.02)
except KeyboardInterrupt:
    robodog.stop_all_legs()
    print("KeyboardInterrupt")
    quit()
