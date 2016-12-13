class ARENA:
	def __init__(self, x,y):
		self.x = x
		self.y = y
		self.obstaclesx= []
		self.obstaclesy = []
		self.obstaclesr = []

	def add_obstacle(self,center_x, center_y, radius):
		self.obstaclesx.append[center_x]
		self.obstaclesy.append[center_y]
		self.obstaclesr.append[radius]


arena = ARENA(5,5)
arena.add_obstacle(2,2,2)
print arena.obstaclex
print arena.x, arena.y

