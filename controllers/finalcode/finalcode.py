""" controller code for the wall follower robot """

from controller import Robot
import math

# implementing Bezier curve smoothing to smooth motor speed changes
class BezierSmoother:
    def __init__(self, smoothing_factor=0.3):
        self.prev_left_speed = 0
        self.prev_right_speed = 0
        self.smoothing_factor = smoothing_factor
    
    # defined a function to gradually adjust wheel speed using linear interpolation
    def smooth_speeds(self, target_left, target_right):
        t = self.smoothing_factor
        smooth_left = self.prev_left_speed + t * (target_left - self.prev_left_speed)
        smooth_right = self.prev_right_speed + t * (target_right - self.prev_right_speed)
        
        self.prev_left_speed = smooth_left
        self.prev_right_speed = smooth_right
        return smooth_left, smooth_right

# implemenation of adjusting the robo speed during cornering and straight paths
class AdaptiveSpeedController:
    def __init__(self):
        self.corner_history = []
        self.straight_history = []
        self.performance_score = 1.0
        self.base_speed = -6.28
        self.max_speed = -6.28  # motor limit that can be used in webots
        self.shortcut_mode = False
        self.shortcut_counter = 0

    # function to update corner and straight path difficulty based on recent sensor readings      
    def update_performance(self, behavior_type, sensors):
        sensor_changes = sum(abs(s - 450) for s in sensors)  
        
        # defined a function to adjust the robot's speed based on recent sensor feedback and movement quality
        if behavior_type == "corner":
            self.corner_history.append(sensor_changes)
            if len(self.corner_history) > 10:
                self.corner_history.pop(0)
        elif behavior_type == "straight":
            self.straight_history.append(sensor_changes)
            if len(self.straight_history) > 20:
                self.straight_history.pop(0)
    
    # defined a function to adjust the robot's speed based on recent sensor feedback and movement quality
    def get_adaptive_speed(self, behavior_type):
        if behavior_type == "corner":
            # increase speed on corners if previous turns were handled smoothly
            if len(self.corner_history) > 5:
                avg_performance = sum(self.corner_history) / len(self.corner_history)
                if avg_performance < 200: 
                    return self.max_speed * 0.85  # increasing speed in smooth turns
                else:
                    return self.max_speed * 0.6   # reducing speed in rough turns
            return self.max_speed * 0.65
            
        elif behavior_type == "straight":
            return self.max_speed   # use maximum speed when there is a clear path ahead
        
        return self.max_speed * 0.8

def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())
    
    # initialize motors and set them to velocity control mode
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    # proximity sensors for obstacle detection (ps1 to ps6)
    prox_sensors = []
    for i in range(1,7):
        sensor_name = 'ps' + str(i)
        prox_sensors.append(robot.getDevice(sensor_name))
        prox_sensors[i-1].enable(timestep)
    
    # create instances for speed smoothing and adaptive control logic
    smoother = BezierSmoother(smoothing_factor=0.4)
    speed_controller = AdaptiveSpeedController()
    
    # declared variables to track robot's current state and sensor history
    stuck_counter = 0
    last_sensor_sum = 0
    behavior_counter = 0  # track how long we've been doing same behavior
    wall_found = False    # track if we've found a wall to follow
    initial_search_time = 0
    
    
    while robot.step(timestep) != -1:
        # read sensors (ps1 to ps6)
        sensors = [prox_sensors[i-1].getValue() for i in range(1,7)]
        
        # display sensor readings and performance every 2 seconds for debugging
        if robot.getTime() % 2.0 < 0.032:
            wall_status = "Found" if wall_found else "SEARCHING"
            print(f"Sensors: {[round(s, 1) for s in sensors]} | Wall: {wall_status} | Speed: {speed_controller.performance_score:.2f}")
        
        CLEAR_THRESHOLD = 450  # high values = clear, low values = obstacle
        
        # detect if the robot has encountered any wall using threshold logic
        any_wall_detected = any(sensor < CLEAR_THRESHOLD for sensor in sensors)
        if any_wall_detected and not wall_found:
            wall_found = True
            print(" Wall Detected! Switching to wall-following mode...")
        
        initial_search_time += 1
        
        # interpret sensor data - values below threshold means an obstacle is present
        front_left = sensors[2] < CLEAR_THRESHOLD      # ps3 - obstacle if < 450
        front_center = sensors[3] < CLEAR_THRESHOLD    # ps4 - obstacle if < 450
        front_right = sensors[4] < CLEAR_THRESHOLD     # ps5 - obstacle if < 450
        left_side = sensors[0] < CLEAR_THRESHOLD       # ps1 - wall if < 450
        left_front = sensors[1] < CLEAR_THRESHOLD      # ps2 - wall if < 450
        right_side = sensors[5] < CLEAR_THRESHOLD      # ps6 - wall if < 450
        
        
        # detect if robot is stuck
        current_sensor_sum = sum(sensors)
        if abs(current_sensor_sum - last_sensor_sum) < 100:  # increased threshold for new range
            stuck_counter += 1
        else:
            stuck_counter = 0
        last_sensor_sum = current_sensor_sum
        
        """ determine the behaviour of the robot based on sensor input and set the appropriate speed """
        
        # look for a wall by slowly curving left
        if not wall_found:
            print(" searching for wall - gentle left angle")
            behavior_type = "search"
            base_speed = speed_controller.get_adaptive_speed("straight") * 0.75
            # slight left curve by making the right wheel a bit faster
            target_left = base_speed * 0.85   # slow down left wheel a bit
            target_right = base_speed         # keep right wheel at search speed
        
        # follow the wall using reactive behavior
        elif stuck_counter > 15:  
            print("Stuck! Fast escape...")
            behavior_type = "escape"
            base_speed = speed_controller.get_adaptive_speed("corner")
            target_left = -base_speed
            target_right = base_speed * 0.6
            stuck_counter = 0
            
        elif front_center or front_left or front_right:
            print("Wall ahead - optimized turn")
            behavior_type = "corner"
            base_speed = speed_controller.get_adaptive_speed("corner")
            target_left = base_speed     
            target_right = -base_speed * 0.8  # slow down one wheel for a tight turn
            
        elif left_side and not left_front:
            print("Following wall - FULL SPEED!")
            behavior_type = "straight" 
            base_speed = speed_controller.get_adaptive_speed("straight")  # use max speed on clear paths
            target_left = base_speed
            target_right = base_speed
            
        elif left_front and left_side:
            print("Too close - quick adjust")
            behavior_type = "corner"
            base_speed = speed_controller.get_adaptive_speed("corner")
            target_left = base_speed
            target_right = base_speed * 0.4
            
        elif not left_side and not left_front:
            print("Searching for wall - fast sweep")
            behavior_type = "corner"
            base_speed = speed_controller.get_adaptive_speed("corner")
            target_left = base_speed * 0.5
            target_right = base_speed
            
        else:
            print("Fast forward")
            behavior_type = "straight"
            base_speed = speed_controller.get_adaptive_speed("straight")
            target_left = base_speed
            target_right = base_speed
        
        # record performance data to improve future speed decisions
        if wall_found:
            speed_controller.update_performance(behavior_type, sensors)
        
        # applying the Bezier curve smoothing
        left_speed, right_speed = smoother.smooth_speeds(target_left, target_right)
        
        # set final smoothed speeds to the wheel motors
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)