
from attr import dataclass
from ender_fdm import Direction

@dataclass   
class custom_test_msg:
    z_inc: float
    direction: Direction

    def to_json_encodable(self):
        return self.__dict__
	
@dataclass
class panic_msg:
    reason: str
    undo_steps: custom_test_msg