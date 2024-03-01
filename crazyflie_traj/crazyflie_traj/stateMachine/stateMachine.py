import time
from typing import List, Type
from .stateBase import State


class StateMachine:
    def __init__(self) -> None:
        self.states: List[State] = []

        self.current_state: State = None
        self.current_state_index = 0

    def loadStates(self, states: List[Type[State]], parentObj) -> None:
        """Load the given states into the state machine."""
        self.states = [state(parentObj) for state in states]

    def stateTransit(self, stateType: Type[State]) -> None:
        """Transit to the given state based on its State class"""

        for state in self.states:
            if isinstance(state, stateType):
                self.current_state = state
                self.current_state.resetTransition()
                return

        raise Exception("State not found")

    def stateIterate(self) -> None:
        self.current_state.iterate()

        if self.current_state.transitionCommanded():
            self.stateTransit(self.current_state.getTransitionCommanded())
