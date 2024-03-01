#!/usr/bin/env python
from __future__ import annotations
from std_msgs.msg import Bool, String, Int16
import time
from ..stateMachine import State
from ..stateMachine import StateMachine
from typing import TYPE_CHECKING


if TYPE_CHECKING:
    from ..singleUAVNode import SingleUAV


class PRE_CHECKS(State):
    def __init__(self, parentnObj) -> None:
        super().__init__("PRE_CHECKS", parentnObj)
        self.parentObj: SingleUAV

    def iterate(self) -> None:
        self.parentObj.statePub.publish(String(data=self.name))
        self.parentObj.get_logger().info(f"{self.parentObj.cf.prefix[1:]} : PRE_CHECKS")

        if self.parentObj.cf.curr_position is not None:
            self.parentObj.get_logger().info(f"{self.parentObj.cf.prefix[1:]} : Mission ready, transitioning to IDLE")
            self.parentObj.precheck_completed = True
            self.transit(IDLE)

class IDLE(State):
    def __init__(self, parentnObj) -> None:
        super().__init__("IDLE", parentnObj)
        self.parentObj: SingleUAV

    def iterate(self) -> None:
        self.parentObj.statePub.publish(String(data=self.name))
        self.parentObj.get_logger().info(f"{self.parentObj.cf.prefix[1:]} : IDLE")

        if self.parentObj.initMissionFlag:
            self.parentObj.get_logger().info(f"{self.parentObj.cf.prefix[1:]} : Mission initiated, transitioning to TAKING_OFF")
            self.parentObj.takeoff()
            self.transit(TAKING_OFF)

class TAKING_OFF(State):
    def __init__(self, parentnObj) -> None:
        super().__init__("TAKING_OFF", parentnObj)
        self.parentObj: SingleUAV  # set type hinting for parentObj

    def iterate(self) -> None:
        self.parentObj.get_logger().info(f"{self.parentObj.cf.prefix[1:]} : TAKING_OFF")
        self.parentObj.statePub.publish(String(data=self.name))  # TODO:find a way to send just integers
        
        if self.parentObj.has_tookoff():
            self.parentObj.takeoff_completed = True
            self.transit(HOVERING)

class HOVERING(State):
    def __init__(self, parentnObj) -> None:
        super().__init__("HOVERING", parentnObj)
        self.parentObj: SingleUAV  # set type hinting for parentObj

    def iterate(self) -> None:
        """UAV is hovering stationry and waiting for the next command"""
        self.parentObj.get_logger().info(f"{self.parentObj.cf.prefix[1:]} : HOVERING")
        self.parentObj.statePub.publish(String(data=self.name))
        if self.parentObj.start_trajectory:
            self.transit(GOING_TO_WAYPOINT)


class GOING_TO_WAYPOINT(State):
    def __init__(self, parentObj: SingleUAV) -> None:
        super().__init__("GOING_TO_WAYPOINT", parentObj)
        self.parentObj: SingleUAV  # set type hinting for parentObj

    def iterate(self) -> None:
        self.parentObj.statePub.publish(String(data=self.name))

        if self.parentObj.has_reached_waypoint():

            # If waypoints left in the queue -> goTo
            if self.parentObj.any_waypoint_left():
                self.parentObj.get_logger().info(f"{self.parentObj.cf.prefix[1:]} : Reached waypoint ... MOVING to next")
                self.parentObj.move_to_next_waypoint()
            else:
                self.parentObj.get_logger().info(f"{self.parentObj.cf.prefix[1:]} : Trajectory terminated -> LANDING")
                self.parentObj.land()
                self.parentObj.commanded_landing = True
                self.transit(LANDING)

class LANDING(State):
    def __init__(self, parentObj: SingleUAV) -> None:
        super().__init__("LANDING", parentObj)
        self.parentObj: SingleUAV  # set type hinting for parentObj

    def iterate(self) -> None:
        self.parentObj.statePub.publish(String(data=self.name))

        self.parentObj.get_logger().info(f"{self.parentObj.cf.prefix[1:]} : LANDING")
        # if self.parentObj.parent.px4Handler.isLanded():
        # if self.parentObj.parent.isLanded():
        #     self.parentObj.parent.landingFlag = False
        #     self.parentObj.parent.homingFlag = False

        #     self.parentObj.parent.px4Handler.disarm()
        #     self.transit(IDLE)