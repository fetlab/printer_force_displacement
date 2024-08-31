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

	@property
	def sign(self):
		if self.value == self.__class__.DOWN.value: return -1
		if self.value == self.__class__.UP.value:   return  1
		return 0

UP    = Direction.UP
DOWN  = Direction.DOWN
STILL = Direction.STILL

def inc2dir(inc:float, direction:Direction) -> float:
	"""Set sign of `inc` based on `dir`"""
	assert direction in (UP, DOWN)
	return abs(inc) * direction.sign


def force2dir(force:float) -> Direction:
	"""Return a Direction based on the sign of `force`. If the meter is moving
	down, it's pushing, and so the force will be negative."""
	if force == 0: return STILL
	if force <  0: return DOWN
	return UP
