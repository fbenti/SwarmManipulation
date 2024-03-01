#!/usr/bin/env python
from __future__ import annotations
from std_msgs.msg import Bool, String, Int16
import time
from ..stateMachine import State
from ..stateMachine import StateMachine
from typing import TYPE_CHECKING


if TYPE_CHECKING:
    from ..orchestratorNode import OrchestratorNode


class IDLE(State):
    def __init__(self, parentnObj) -> None:
        super().__init__("IDLE", parentnObj)
        self.parentObj: OrchestratorNode

    def iterate(self) -> None:
        self.parentObj.statePub.publish(String(data=self.name))
        self.parentObj.get_logger().info(f"{self.parentObj.get_name()} : IDLE")
        if self.parentObj.are_all_ready():
            self.parentObj.get_logger().info(f"{self.parentObj.get_name()} : Mission ready, transitioning to TAKING_OFF")
            self.parentObj.takeoff()
            self.transit(TAKING_OFF)

class TAKING_OFF(State):
    def __init__(self, parentnObj) -> None:
        super().__init__("TAKING_OFF", parentnObj)
        self.parentObj: OrchestratorNode  # set type hinting for parentObj

    def iterate(self) -> None:
        self.parentObj.get_logger().info(f"{self.parentObj.get_name()} : TAKING_OFF")
        self.parentObj.statePub.publish(String(data=self.name))  # TODO:find a way to send just integers
        
        if self.parentObj.have_all_tookoff():
            self.parentObj.command_start_trajectory()
            self.transit(TRAJECTORY)

class TRAJECTORY(State):
    def __init__(self, parentnObj) -> None:
        super().__init__("TRAJECTORY", parentnObj)
        self.parentObj: OrchestratorNode  # set type hinting for parentObj

    def iterate(self) -> None:
        """UAV is hovering stationry and waiting for the next command"""
        self.parentObj.get_logger().info(f"{self.parentObj.get_name()} : TRAJECTORY")
        self.parentObj.statePub.publish(String(data=self.name))
        if self.parentObj.have_all_landed():
            self.parentObj.get_logger().info(f"{self.parentObj.get_name()} : trajecotroy terminated -> transition to TERMINATED")
            self.transit(TERMINATED)


class TERMINATED(State):
    def __init__(self, parentObj: OrchestratorNode) -> None:
        super().__init__("TERMINATED", parentObj)
        self.parentObj: OrchestratorNode  # set type hinting for parentObj

    def iterate(self) -> None:
        self.parentObj.statePub.publish(String(data=self.name))
        self.parentObj.get_logger().info(f"{self.parentObj.get_name()} : TERMINATED")
