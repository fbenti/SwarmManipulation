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
from .states import orchestratorStates, singleUAVStates
from crazyflie_traj.singleUAVNode import SingleUAV

TAKEOFF_DURATION = 2.5
HOVER_DURATION = 5.0
TAKEOFF_HEIGHT = 2.0


class OrchestratorNode(Node):

    def __init__(self):
        super().__init__('orchestrator')

        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.swarm.ids = list(self.swarm.allcfs.crazyfliesById.keys())

        
        # Define trajectory (waypoints)
        self.radius = 0.5  # Radius of the spiral
        self.pitch = np.deg2rad(15)    # Pitch of the spiral
        self.height = TAKEOFF_HEIGHT   # Starting height
        self.speed = 1    # Speed of the drone along the spiral
        self.num_waypoints = 100  # Number of points to generate
        self.idx_w = 0
        self.waypoints = self.generate_spiral_trajectory() 

        # Control variables
        self.initMissionFlag = True # if True, the mission start automatically
        self.distance_threshold = 0.2

        # Add position and waypoint to dict of single cfs
        self.UAVS = {}
        for id in self.swarm.ids:
            cf = self.swarm.allcfs.crazyfliesById[id]
            cf.curr_position = None
            cf.curr_waypoint = None
            self.UAVS[id] = SingleUAV(self,
                                      cf,
                                      self.distance_threshold,
                                      TAKEOFF_HEIGHT,
                                      TAKEOFF_DURATION,
                                      self.waypoints)

        # Initialise params
        self.initQosProfile()
        self.initPubs()

        self.commanded_takeoff = False
        self.takeoff_completed = False
        
        # State machine
        self.stateMachineInit()
        self.stateMachineStart()




    
    def initPubs(self):
        self.statePub = self.create_publisher(
            String, 'orchestrator/state', self.qos_profile)
        
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
            orchestratorStates.TRAJECTORY,
            orchestratorStates.TERMINATED,
        ], self)

        self.stateMachine.stateTransit(orchestratorStates.IDLE)

    def stateMachineStart(self) -> None:
        freq = 5  # [Hz]
        self.stateMachineTimer = self.create_timer(1/freq, self.stateMachineCallback)

    def stateMachineCallback(self) -> None:
        self.stateMachine.stateIterate()

        
    def get_position(self):
        target_frame = "cf231"
        try:
            transform = self.tf_buffer.lookup_transform(
                'world',
                target_frame,  # source and target frame
                rclpy.time.Time(),            # get the latest available transform
                rclpy.time.Duration(seconds=1)  # wait for up to 1 second
            )
            # position = transform.transform.translation
            # self.swarm.allcfs.crazyfliesById[id].curr_position = np.array([position.x,position.y,position.z])
            # self.curr_position = np.array([position.x,position.y,position.z])
            # self.get_logger().info(f"pos-{target_frame} {self.swarm.allcfs.crazyfliesById[id].curr_position}")
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Failed to lookup transform: {e}")

    def command_start_trajectory(self):
        for id in self.UAVS:
            uav = self.UAVS[id]
            uav.start_trajectory = True


    def takeoff(self):
        """
        Command takeoff - It's the only command that is sent synchronously
        """
        # self.commanded_takeoff = True
        # self.curr_waypoint = np.array([0.0, .0, TAKEOFF_HEIGHT])
        # self.swarm.allcfs.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=1) # TODO: for swarm has to be changed to broadcast
        for id in self.UAVS:
            uav = self.UAVS[id]
            uav.initMissionFlag = True

    def land(self):
        self.cf.land(targetHeight=0.04, duration=2.5)
        # self.timeHelper.sleep(TAKEOFF_DURATION)
    
    
    def have_all_tookoff(self):
        for id in self.UAVS:
            uav = self.UAVS[id]
            if not uav.takeoff_completed:
                return False
        return True
    
    def have_all_landed(self):
        for id in self.UAVS:
            uav = self.UAVS[id]
            if not uav.commanded_landing:
                return False
        return True


    def has_reached_waypoint(self):
        """
        Check if drone has reached current waypoint.
        """
        return self.euclidian_dist(self.curr_waypoint, self.curr_position) < self.distance_threshold
    
    def are_all_ready(self):
        for id in self.UAVS:
            uav = self.UAVS[id]
            if not uav.precheck_completed:
                return False
        return True
    
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

    def generate_spiral_trajectory(self):
        theta = np.linspace(-np.pi, 4*np.pi, self.num_waypoints)
        z = self.radius * np.cos(theta) + self.height
        y = self.radius * np.sin(theta)
        x = self.pitch * theta 
        time = np.linspace(0, 2*np.pi, self.num_waypoints) / self.speed
        return np.vstack((x,y,z)).T




def main():
    rclpy.init()
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    node = OrchestratorNode()
    # executor.add_node(node)
    # node.UAVS = {}
    # for id in node.swarm.ids:
    #     cf = node.swarm.allcfs.crazyfliesById[id]
    #     cf.curr_position = None
    #     cf.curr_waypoint = None
    #     node.UAVS[id] = SingleUAV(cf,
    #                                 node.distance_threshold,
    #                                 TAKEOFF_HEIGHT,
    #                                 TAKEOFF_DURATION,
    #                                 node.waypoints)
    #     executor.add_node(node.UAVS[id])
    #     break
    try:
        rclpy.spin(node)
        # executor.spin()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    


if __name__ == '__main__':
    main()
