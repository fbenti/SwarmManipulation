"""
Generate spiral trajectory along x-axis.
"""

from crazyflie_py import Crazyswarm
import numpy as np
# from tf import TransformListener
import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import geometry_msgs.msg
import threading
from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy


from .stateMachine import StateMachine
from .orchestrator import orchestratorStates


TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0
INIT_HEIGHT = 2.0


class SpiralTraj(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.swarm.ids = list(self.swarm.allcfs.crazyfliesById.keys())

        
        # Add position and waypoint to dict of single cfs
        for id in self.swarm.ids:
            cf = self.swarm.allcfs.crazyfliesById[id]
            cf.curr_position = None
            cf.curr_waypoint = None

        # Control variables
        self.initMissionFlag = True # if True, the mission start automatically
        self.distance_threshold = 0.2

        # Initialise params
        self.initQosProfile()
        self.initPubs()

        # TF - for position feedback
        self.tf_buffer = tf2_ros.Buffer()
    
        # Define parameters

        self.radius = 0.5  # Radius of the spiral
        self.pitch = np.deg2rad(15)    # Pitch of the spiral
        self.height = INIT_HEIGHT   # Starting height
        self.speed = 1    # Speed of the drone along the spiral
        self.num_waypoints = 100  # Number of points to generate
        self.idx_w = 0
        x,y,z,time = self.generate_spiral_trajectory(
            self.radius, 
            self.pitch, 
            self.height, 
            self.speed, 
            self.num_waypoints)
        self.waypoints = np.vstack((x,y,z)).T

        self.commanded_takeoff = False
        self.takeoff_completed = False
        
        self.pos_feedback_timer = self.create_timer(1, self.get_position)
        # self.control_loop_timer = self.create_timer(1, self.control_loop)

        # threading.Thread(target=lambda: self.control_loop(), daemon=True).start()
        # state machine
        self.stateMachineInit()
        self.stateMachineStart()
    
    def initPubs(self):
        self.statePub = self.create_publisher(
            String, 'state', self.qos_profile)
        
    def initQosProfile(self):
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

    # state machine functionalities
    def stateMachineInit(self) -> None:
        self.stateMachine = StateMachine()
        self.stateMachine.loadStates([
            orchestratorStates.IDLE,
            orchestratorStates.TAKING_OFF,
            orchestratorStates.HOVERING,
            orchestratorStates.GOING_TO_WAYPOINT,
            orchestratorStates.LANDING
        ], self)

        self.stateMachine.stateTransit(orchestratorStates.IDLE)

    def stateMachineStart(self) -> None:
        freq = 10  # [Hz]
        self.stateMachineTimer = self.create_timer(1/freq, self.stateMachineCallback)

    def stateMachineCallback(self) -> None:
        self.stateMachine.stateIterate()

        
    def get_position(self):
        target_frame = 'cf231'
        for id in self.swarm.ids:
            target_frame = id
            try:
                transform = self.tf_buffer.lookup_transform(
                    'world', target_frame,  # source and target frame
                    rclpy.time.Time(),            # get the latest available transform
                    rclpy.time.Duration(seconds=1)  # wait for up to 1 second
                )
                position = transform.transform.translation
                self.swarm.allcfs.crazyfliesById[id].curr_position = np.array([position.x,position.y,position.z])
                # self.curr_position = np.array([position.x,position.y,position.z])
                self.get_logger().info(f"pos-{target_frame} {self.swarm.allcfs.crazyfliesById[id].curr_position}")
            
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().error(f"Failed to lookup transform: {e}")



    def takeoff(self):
        """
        Command takeoff 
        """
        self.commanded_takeoff = True
        self.curr_waypoint = np.array([0.0, .0, INIT_HEIGHT])
        self.cf.takeoff(targetHeight=INIT_HEIGHT, duration=1) # TODO: for swarm has to be changed to broadcast
        # self.timeHelper.sleep(TAKEOFF_DURATION*2)

    def land(self):
        self.cf.land(targetHeight=0.04, duration=2.5)
        # self.timeHelper.sleep(TAKEOFF_DURATION)
    
    
    def has_takeoff(self):
        curr_pos = self.curr_position
        takeoff_goal = np.array([0,0,INIT_HEIGHT])
        self.get_logger().info(f"err: {np.linalg.norm(curr_pos-takeoff_goal)}")
        # self.get_logger().info(f"{curr_pos.shape} {takeoff_goal.shape}")
        # self.get_logger().info(f"{self.euclidian_dist(curr_pos, takeoff_goal)}")

        return self.euclidian_dist(curr_pos,takeoff_goal) < 0.3  


    def has_reached_waypoint(self):
        """
        Check if drone has reached current waypoint.
        """
        return self.euclidian_dist(self.curr_waypoint, self.curr_position) < self.distance_threshold
    
    def move_to_next_waypoint(self):
        """
        Move to next waypoiint.
        """
        self.curr_waypoint = self.waypoints[self.idx_w,:]
        self.idx_w += 1
        self.get_logger().info(f"{self.idx_w} : moving to {self.curr_waypoint}")
        self.cf.goTo(goal=self.curr_waypoint, yaw=0.0, duration=1)

    def euclidian_dist(self, pos1, pos2):
        """
        Compute euclidean dist given two numpy array shape (3,)
        """
        return np.linalg.norm(pos1 - pos2)

    def any_waypoint_left(self):
        """
        Any waypoint left
        """
        return self.idx_w < self.num_waypoints

    def tf_callback(self, msg):
        self.get_logger().info(f"{msg}")
        # pass


    def generate_spiral_trajectory(self, radius, pitch, height, speed, num_points):
        theta = np.linspace(0, 5*np.pi, num_points)
        z = radius * np.cos(theta) + height
        y = radius * np.sin(theta)
        x = pitch * theta 
        time = np.linspace(0, 2*np.pi, num_points) / speed
        return x, y, z, time




def main():
    rclpy.init()
    node = SpiralTraj()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
