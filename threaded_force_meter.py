"""
Force meter for reading forces from serial port force meters of ZP-50 etc
models.

This code currently assumes the meter is mounted pointing down, so when
readings are positive, the meter is pushing, and when readings are negative,
the meter is pulling.
"""
from time import time
from serial import Serial
from serial.threaded import Protocol, ReaderThread
from .direction import Direction, UP, DOWN, STILL, force2dir

MAX_FORCE = 3.5

class ThreadedForceMeter(Protocol):
	def __init__(self):
		self.buffer    = bytearray()
		self.transport = None
		self._value    = None
		self.timestamp = 0
		self.new_value = False
		self.ready     = False


	def connection_made(self, transport):
		self.transport = transport
		self.transport.serial.reset_input_buffer()
		print('Please make force meter read negative')


	def data_received(self, data):
		self.buffer.extend(data)

		if not self.ready:
			#Throw away data until the buffer starts with '-', then we know we're
			# aligned and can read 6 bytes at a time
			if b'-' in self.buffer:
				self.buffer = self.buffer[self.buffer.index(b'-'):]
				self.ready = True
				print('---ALIGNED---')
			elif len(self.buffer) > 6:
				self.buffer = self.buffer[6:]

		#Only keep the last value
		if self.ready and len(self.buffer) >= 6:
			rest_pos = 6 * (len(self.buffer)//6)
			self.value = float(self.buffer[rest_pos-6:rest_pos])
			self.buffer = self.buffer[rest_pos:]


	def connection_lost(self, exc):
		self.__init__()  #Reset everything
		if isinstance(exc, Exception):
			print(f'Connection lost: {exc}')
			raise exc


	@property
	def value(self):
		self.new_value = False
		return self._value


	@value.setter
	def value(self, new_value):
		if abs(new_value) >= MAX_FORCE:
			raise ValueError(f"Force of {new_value} exceeded max force threshold {MAX_FORCE}")
		self.timestamp = time()
		self._value = new_value
		self.new_value = True


	@property
	def pushing(self) -> bool:
		return self._value > 0

	@property
	def pulling(self) -> bool:
		return self._value < 0

	@property
	def direction(self) -> Direction:
		return force2dir(self._value)

if __name__ == "__main__":
	import sys

	s = Serial(sys.argv[1], 2400)
	with ReaderThread(s, ThreadedForceMeter) as m:
		while True:
			if m.new_value:
				print(m.value)
