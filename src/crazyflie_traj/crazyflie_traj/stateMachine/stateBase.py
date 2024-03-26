from __future__ import annotations
from abc import ABC, abstractmethod
from typing import Type

class State(ABC):
    def __init__(self,name:str, parentnObj) -> None:
        self.name = name
        self.parentObj = parentnObj

        self._transitionCommanded:Type[State] = None #TODO: find way to fix the None type warning

    @abstractmethod
    def iterate(self) -> None:
        """Update the state. This method needs to be overridden by the subclass."""
        pass
    
    def transit(self, state: Type[State]) -> None:
        """Transit to the given state."""
        self._transitionCommanded = state

    def transitionCommanded(self) -> bool:
        """Returns the state to which the transition is commanded."""
        return self._transitionCommanded is not None
    
    def getTransitionCommanded(self) -> Type[State]:
        """Returns the state to which the transition is commanded."""
        return self._transitionCommanded
    
    def resetTransition(self) -> None:
        """Reset the transition command."""
        self._transitionCommanded = None

    def __str__(self) -> str:
        return self.name 
