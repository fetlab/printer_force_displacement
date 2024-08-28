from enum import Enum

#Movement directions
class Direction(Enum):
	DOWN  = 'DOWN'
	UP    = 'UP'
	STILL = 'STILL'

	def flip(self):
		match self.value:
			case 'DOWN':  return self.__class__('UP')
			case 'UP':    return self.__class__('DOWN')
			case 'STILL': return self.__class__('STILL')

	def __str__(self):
		return self.value

	def to_json_encodable(self):
		return str(self)

UP    = Direction.UP
DOWN  = Direction.DOWN
STILL = Direction.STILL

def inc2dir(inc:float, direction:Direction) -> float:
	"""Set sign of `inc` based on `dir`"""
	assert direction in (UP, DOWN)
	return abs(inc) * (-1 if direction == DOWN else 1)


def force2dir(force:float) -> Direction:
	"""Return a Direction based on the sign of `force`."""
	return STILL if force == 0 else UP if force > 0 else DOWN
