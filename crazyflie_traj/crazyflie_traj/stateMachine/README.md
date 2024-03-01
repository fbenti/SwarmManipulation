## State Machine Library in Pure Python

This library is a simple state machine library written in pure python in order to keep it as simple as possible. 

### Installation
No need to install anything. Just clone the repository.

### Usage
In this library is state needs to be defined by the user and is a class that inherits from the `State` class. The `State` class has two methods that need to be implemented by the user. These are the `__init__` and `iterate` methods. The `__init__` method is the constructor of the state and the `iterate` method is the method that is called every time the state machine iterates the current state. The `iterate` method is the place where the user can implement the logic of the state.

In oorder to setup the machine the user needs to create an instance of the `StateMachine` class and load the states into it. The `StateMachine` class has two methods that need to be called by the user. These are the `loadStates` and `stateTransit`. The `loadStates` method takes a list of states and the parent object as arguments. The `stateTransit` method takes a state as an argument and sets it as the initial state of the machine.

To iterate the machine, the user needs to call the `stateIterate` method of the `StateMachine` class in an interval of their preference or asynchronously. This method iterates the current state of the machine.
### Example
Simple example of a state machine with two states and one transition between them.

`offboardControl.py`

```python
class OffboardControl:
    def __init__(self) -> None:
        
        ...
        
        self.stateMachineInit()

    def stateMachineInit(self) -> None:
        self.stateMachine = StateMachine()
        self.stateMachine.loadStates([
            offboard_control_states.IDLE,
            offboard_control_states.TAKING_OFF,
            ...
        ], self)

        self.stateMachine.stateTransit(offboard_control_states.IDLE) # set initial state

    def stateMachineStart(self) -> None:
        freq = 10  # [Hz] state machine frequency
        self.stateMachineTimer = self.create_timer(1/freq, self.stateMachineCallback)

    def stateMachineCallback(self) -> None:
        self.stateMachine.stateIterate() # iterate current state

    ...
```

`states.py`
```python
from state_machine import StateMachine, State

class IDLE(State):
    def __init__(self, parentObj: OffboardControl) -> None:
        super().__init__("IDLE", parentObj)

        self.parentObj: OffboardControl  # set type hinting for parentObj
    
     def iterate(self) -> None:
        if parentObj.takeoffCommanded():
            print("take off commanded...")

            self.parentObj.px4Handler.takeoff()

            self.transit(TAKING_OFF)

class TAKING_OFF(State):
    def __init__(self, parentnObj) -> None:
        super().__init__("TAKING_OFF", parentnObj)
        self.parentObj: OffboardControl  # set type hinting for parentObj

    def iterate(self) -> None:
        if self.parentObj.isTakenOff():
            self.parentObj.px4Handler.offboard()
            self.parentObj.px4Handler.position_mode()

            print("UAV took off ... Transiting to HOVERING")
            self.transit(HOVERING)
    
    ...
```