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
from .states import singleUAVStates



class SingleUAV(Node):

    def __init__(self,
                 parent,
                 cf, 
                 distance_threshold,
                 takeoff_height,
                 takeoff_duration,
                 waypoints):
        super().__init__(cf.prefix[1:]+'_single_uav')
        self.parent = parent
        self.cf = cf

        # Control variables
        self.initMissionFlag = False # if True, the mission start automatically
        self.distance_threshold = distance_threshold
        self.takeoff_height = takeoff_height
        self.takeoff_duration = takeoff_duration
        self.waypoints = waypoints
        self.idx_w = 0
        self.num_waypoints = self.waypoints.shape[0]
        self.precheck_completed = False
        self.commanded_takeoff = False
        self.commanded_landing = False
        self.takeoff_completed = False
        self.start_trajectory = False

        # TF - for position feedback
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.parent)

        # Initialise ROS methods                
        self.initQosProfile()
        self.initPubs()
        self.initTimers()
        self.stateMachineInit()
    
    def initPubs(self):
        self.get_logger().info(f"{self.cf.prefix}")
        self.statePub = self.create_publisher(
            String, self.cf.prefix + '/state', self.qos_profile)
        
    def initQosProfile(self):
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

    def initTimers(self):
        self.pos_feedback_timer = self.parent.create_timer(0.2, self.get_position)
        self.stateMachineStart()

    # state machine functionalities
    def stateMachineInit(self) -> None:
        self.stateMachine = StateMachine()
        self.stateMachine.loadStates([
            singleUAVStates.PRE_CHECKS,
            singleUAVStates.IDLE,
            singleUAVStates.TAKING_OFF,
            singleUAVStates.HOVERING,
            singleUAVStates.GOING_TO_WAYPOINT,
            singleUAVStates.LANDING
        ], self)

        self.stateMachine.stateTransit(singleUAVStates.PRE_CHECKS)

    def stateMachineStart(self) -> None:
        freq = 10  # [Hz]
        self.stateMachineTimer = self.parent.create_timer(1/freq, self.stateMachineCallback)

    def stateMachineCallback(self) -> None:
        self.stateMachine.stateIterate()

    def get_position(self):
        target_frame = self.cf.prefix[1:]
        # self.get_logger().info(f"FRAME {target_frame}")
        try:
            transform = self.tf_buffer.lookup_transform(
                'world', target_frame,  # source and target frame
                rclpy.time.Time(),            # get the latest available transform
                rclpy.time.Duration(seconds=1)  # wait for up to 1 second
            )
            position = transform.transform.translation
            self.cf.curr_position = np.array([position.x,position.y,position.z])
            # self.curr_position = np.array([position.x,position.y,position.z])
            self.get_logger().info(f"{self.cf.prefix[1:]} {self.cf.curr_position}")
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Failed to lookup transform: {e} {target_frame}")

    def takeoff(self):
        """
        Command takeoff 
        """
        self.commanded_takeoff = True
        self.cf.setParam('kalman.resetEstimation', 1)
        self.parent.timeHelper.sleep(1.0)
        # self.cf.setParam('kalman.resetEstimation', 0)
        # self.parent.timeHelper.sleep(1.0)
        # self.cf.curr_waypoint = np.array([self.cf.curr_position[0], 
        #                                   self.cf.curr_position[1], 
        #                                   self.takeoff_height])
        self.cf.curr_waypoint = np.array([self.cf.initialPosition[0], 
                                          self.cf.initialPosition[1], 
                                          self.takeoff_height])
        self.cf.takeoff(targetHeight=self.takeoff_height, 
                        duration=self.takeoff_duration) 

    def land(self):
        self.cf.land(targetHeight=0.04, duration=self.takeoff_duration)
    

    def has_tookoff(self):
        dist = self.euclidian_dist(self.cf.curr_position[2], self.cf.curr_waypoint[2])
        self.get_logger().info(f"CURRENT POSITION : {self.cf.curr_position}")
        self.get_logger().info(f"CURRENT WAYPOINT : {self.cf.curr_waypoint}")
        self.get_logger().info(f"err: {dist}")
        return  dist < self.distance_threshold


    def has_reached_waypoint(self):
        """
        Check if drone has reached current waypoint.
        """
        self.get_logger().info(f"CURRENT POSITION : {self.cf.curr_position}")
        self.get_logger().info(f"CURRENT WAYPOINT : {self.cf.curr_waypoint}")
        dist = self.euclidian_dist(self.cf.curr_waypoint, self.cf.curr_position)
        self.get_logger().info(f"err: {dist}")
        return  dist < self.distance_threshold
    
    def move_to_next_waypoint(self):
        """
        Move to next waypoiint.
        """
        self.cf.curr_waypoint = self.waypoints[self.idx_w,:] + self.cf.initialPosition.reshape(1,3)
        self.cf.curr_waypoint = self.cf.curr_waypoint.ravel().astype(float)
        self.idx_w += 1
        self.get_logger().info(f"{self.idx_w} : moving to {self.cf.curr_waypoint}")
        self.cf.goTo(goal=self.cf.curr_waypoint, yaw=0.0, duration=1, relative=False)

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






def main():
    rclpy.init()
    node = SingleUAV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
