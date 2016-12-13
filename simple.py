import random, pygame, math, numpy, pyevolve
from pyevolve import G1DBinaryString
from pyevolve import GSimpleGA
from pyevolve import Selectors
from pyevolve import Mutators

#all angles to be taken in degrees
pygame.init()

white = (255,255,255)
red = (255,0,0)
black = (0,0,0)
v_max = 10
t_act = 1

pos_obs = [[315.0 ,325.0] ,[125.0 ,380.0] ,[125.0 ,115.0] ,[320.0 ,105.0] ,[385.0, 195.0], [100,250]]
#say 500 represents 500cm
quit_flag = 0
arena_x = 500
arena_y = 500
epsilon = arena_x/100		#threshold, one generation ends after coming this close
flag=0
dt = 0.1			#seconds after update
obstacle_radius = float(arena_x)/20;
robot_radius = float(arena_x)/20;
sensor_radius = float(robot_radius)/10;
sensor_range = 3*obstacle_radius		#actual range + obstacle_radius, as we'll later on check with obstacle center only
iterations = 1

gameDisplay = pygame.display.set_mode((arena_x,arena_y))

#0,0 is top left
pygame.display.set_caption('Arena')

#putting in the obstacles, assumed to be cylindrical, x,y, radius assinged
class OBSTACLE:
	def __init__(self, pos):

		self.radius = obstacle_radius;		#robot radius taken same as obstacle radius, arena_x/20
		self.x = pos[0]
		self.y = pos[1]
		#self.x = float(arena_x)*(float(random.randrange(20,40))/100 +0.4*random.randrange(0,2))
		#self.y = float(arena_y)*(float(random.randrange(20,40))/100 +0.4*random.randrange(0,2))

class ROBOT:
	def __init__(self):
		self.radius = robot_radius;
		self.x = float(arena_x)/2 #+ 0.9*arena_x*(random.random() -0.5)
		self.y = float(arena_y)/2 #+ 0.9*arena_y*(random.random() -0.5)
		self.theta = 0 #360*(random.random())			#initial direction of velocity
		self.ref = self.theta			#the reference marker to estimate the robot orientation
		self.wheel_radius = 20
		self.width = robot_radius
		self.sensor = []

	def new_sense(self, orient): 

		r = 0.9*self.radius			#sensors placed at 0.9r times the robot radius from the center
		orient = (orient)%360
		x = self.x + r*math.cos(math.radians(orient))
		y = self.y + r*math.sin(math.radians(orient))
		val = 0			#sensed value, will depend on robot orientation, sensor orientation, x,y coordinates and obstacle positions
		temp = [x,y,orient,val]
		self.sensor.append(temp)
		return self.sensor

	def sense_bot(self, obstacles):
		global quit_flag					
		for i in range(0,len(self.sensor)):	
			temp1 = self.sensor[i][2]
			#print self.ref
			self.sensor[i][3] = 0		#initialize as zero at every update step

			if self.sensor[i][0]<=epsilon or self.sensor[i][0]>=arena_x-epsilon or self.sensor[i][1]<=epsilon or self.sensor[i][1]>=arena_y-epsilon:
				quit_flag = 1
			
			if self.sensor[i][0]<=sensor_range and self.sensor[i][2]>=90 and self.sensor[i][2]<=270:
				self.sensor[i][3] = self.sensor[i][3] + epsilon*abs(math.cos(math.radians(self.sensor[i][2])))**2/self.sensor[i][0]
				#print self.sensor[i][0], sensor_range, obstacle_radius
			if self.sensor[i][0]>=arena_x -sensor_range  and (self.sensor[i][2]>=270 or self.sensor[i][2]<=90):
				self.sensor[i][3] = self.sensor[i][3] + epsilon*abs(math.cos(math.radians(self.sensor[i][2])))**2/(arena_x -self.sensor[i][0])	
				#print self.sensor[i][0], arena_x - (sensor_range - obstacle_radius)

			if self.sensor[i][1]<=sensor_range and self.sensor[i][2]>=180 and self.sensor[i][2]<=360:
				self.sensor[i][3] = self.sensor[i][3] + epsilon*abs(math.sin(math.radians(self.sensor[i][2])))**2/self.sensor[i][1]

			if self.sensor[i][1]>=arena_y - sensor_range and self.sensor[i][2]>=0 and self.sensor[i][2]<=180:
				self.sensor[i][3] = self.sensor[i][3] + epsilon*abs(math.sin(math.radians(self.sensor[i][2])))**2/(arena_y -self.sensor[i][1])	
				
			for j in range(len(obstacles)):			
				if (self.x-obstacles[j].x)**2 + (self.y-obstacles[j].y)**2 <= (self.radius+obstacles[j].radius+epsilon)**2:
					quit_flag = 1
				#print (self.x-obstacles[j].x)**2 + (self.y-obstacles[j].y)**2, (self.radius+obstacles[j].radius+epsilon)**2, quit_flag	
				temp2 = (math.degrees(math.atan2(obs[j].y-self.sensor[i][1],obs[j].x-self.sensor[i][0]))+2*360)%360
				temp3 = math.sqrt((self.sensor[i][0]-obstacles[j].x)**2 + (self.sensor[i][1]-obstacles[j].y)**2)  
				
				#print i,temp1,temp2, temp3, sensor_range**2
				#if (temp3 <= sensor_range**2 and (abs(temp1-temp2)<=90 or ((temp1>=270 or temp2>270) and (temp2>(temp1-90)%360 or temp2<=(temp1+90)%360)))):
				if temp3 <= sensor_range+obstacle_radius:
					if (temp1>=270 and ((temp2>=temp1-90 and temp2<=360) or temp2<= (temp1+90)%360)) or (temp2>=270 and ((temp1>=temp2-90 and temp1<=360) or temp1<= (temp2+90)%360)) or (abs(temp1-temp2) <=90):
						self.sensor[i][3] = self.sensor[i][3]+ epsilon*abs(math.cos(math.radians(temp1-temp2))/(abs(temp3-obstacles[j].radius)))
								
						#quit one generation when sensor is (or the 'epsilon') 5 cm away from the obstacle i.e. temp3-obstacle_radius
						#above fact is to be used in normalization of sensor readings, all reading to be multiplied with 5 as max reading is 1/5.
						#print  math.sqrt(temp3)-obstacles[j].radius, epsilon
    	
    		return [self.sensor[p][3] for p in range(8)]

   	def move_bot(self, sensor_values,params):

   		input1 = sensor_values+[1, w_old[0]]
		input2 = sensor_values+[1, w_old[1]]
		out1 = sigmoid(numpy.dot(numpy.array((input1)), numpy.array((params[0])))) - 0.5
		out2 = sigmoid(numpy.dot(numpy.array((input2)), numpy.array((params[1])))) - 0.5

		omega = (out1-out2)*self.wheel_radius/self.width	#rotation of the robot body
		vel = (out1+out2)*self.wheel_radius/2 			#velocity of robot com
		dtheta = omega*dt
		#print out1,out2,vel,dtheta
   		r = 0.9*self.radius			#sensors placed at 0.9r times the robot radius from the center
		self.theta = (self.theta + dtheta)%360
		self.x = self.x + vel*dt*math.cos(math.radians(self.theta))
		self.y = self.y + vel*dt*math.sin(math.radians(self.theta))
		self.ref = (self.ref + dtheta)%360

		for i in range(0,len(self.sensor)):
			self.sensor[i][2] = (self.sensor[i][2]+dtheta)%360
			self.sensor[i][0] = self.x + r*math.cos(math.radians(self.sensor[i][2]))
			self.sensor[i][1] = self.y + r*math.sin(math.radians(self.sensor[i][2]))	

		w_old[0] = out1
		w_old[1] = out2	
		
		return [out1,out2]


def sigmoid(sig_in):
	if sig_in < -50:
		return 0
	else:
		return 1/(1+math.exp(-sig_in))

w_old = [0, 0]

def create_bot():
	robo = ROBOT()
	#sensors embedded
	for i in range(0,8):
		theta = i*360/8	
		sensor = robo.new_sense(theta)
	return robo
				
obs = []
for n in range(len(pos_obs)):
		a = OBSTACLE(pos_obs[n])
		obs.append(a)

fin_para = 0

def dec_params(chromosome, len_sub):
	par = []
	for i in range(len(chromosome)/len_sub):
		temp = chromosome[i*len_sub:(i+1)*len_sub]
		num = int(''.join(map(str,temp)))
		num = float(int(str(num),2))
		par.append(num/100-5)
	par1 = par[0:len(par)/2]
	par2 = par[len(par)/2:]	
	par = [par1,par2]	
	return par
		

#sort this eval function out!!!!!!!!!!!!!!!!! math range error in sigmoid, while lopp are the major concerns

def eval_func(chromosome):
	global quit_flag, fin_para
	quit_flag=0
	score = float(0.0)
	count = 0
	i_max = 0
	V = 0
	v = 0
	robo = create_bot()
	params = dec_params(chromosome,10)
	fin_para = params
	while count<=1000:
		count = count+1
		#if count>1:
		#	print count.174590032922
		vals = robo.sense_bot(obs)
		#print quit_flag, vals
		if quit_flag==1:
			return 0
			break

		i_max = i_max + max(vals)
		w = robo.move_bot(vals,params)
		#print w, vals
		V = V + abs(w[0]+w[1])
		v = v + abs(w[0]-w[1])
	i_max = i_max/count
	v = v/count
	V = V/count
	score = float(V*(1-math.sqrt(v))*i_max)*count 
	return score		


def ga_bot():
# Genome instance
	genome = G1DBinaryString.G1DBinaryString(200)

	# The evaluator function (objective function)
	genome.evaluator.set(eval_func)
	genome.mutator.set(Mutators.G1DBinaryStringMutatorFlip)

	# Genetic Algorithm Instance
	ga = GSimpleGA.GSimpleGA(genome)
	ga.selector.set(Selectors.GTournamentSelector)
	ga.setGenerations(50)
	# Do the evolution, with stats dump
	# frequency of 10 generations
	ga.evolve(freq_stats=1)

	# Best individual
	best = ga.bestIndividual()
	print best

ga_bot()

#chrome = [random.randrange(1,2) for _ in range(200)]
#eval_func(chrome)
#eval_func(chrome)
#	while flag<1:
#		b = OBSTACLE()
#		if (b.x-robo.x)**2 + (b.y-robo.y)**2 >= (robot_radius + obstacle_radius)**2:
#			obs.append(b)
#			flag = flag+1
#			print b.x,b.y
#	count =1

	#4 obstacles being setup pseudo-randomly in the arena
#	while count<5:
#		flag=0
#		a = OBSTACLE()
		#following code ensures the obstacles do not overlap
#		for j in range(len(obs)):
#			if (a.x-obs[j].x)**2 + (a.y-obs[j].y)**2 >= (4*obstacle_radius)**2 and (a.x-robo.x)**2 + (a.y-robo.y)**2 >= (2*robot_radius+2*obstacle_radius)**2:
#				flag = flag+1
#		if flag==len(obs):
#			obs.append(a)
#			print a.x,a.y
#			count = count + 1

count1 = 0	
quit_flag=0
robo = create_bot()

for n in range(iterations):		
	gameEXIT = False

	while not gameEXIT and (quit_flag==0 and count1<=500):	
		count1 = count1 + 1
		sensor_values = [robo.sensor[p][3] for p in range(8)]
			
		vals = robo.sense_bot(obs)
		robo.move_bot(vals,fin_para)

		for event in pygame.event.get():
			if event.type == pygame.QUIT:
				gameEXIT = True
			if event.type == pygame.KEYDOWN:
				if event.key == pygame.K_LEFT:
					robo.x = robo.x -5;
				if event.key == pygame.K_RIGHT:
					robo.x = robo.x +5;
				if event.key == pygame.K_UP:
					robo.y = robo.y -5;
				if event.key == pygame.K_DOWN:
					robo.y = robo.y +5;				
		
		gameDisplay.fill(black)
		
		pygame.draw.circle(gameDisplay, white, (int(robo.x), int(robo.y)), int(robo.radius), 0)
		pygame.draw.line(gameDisplay, (0,0,0), (int(robo.x), int(robo.y)), (int(robo.x+robot_radius*math.cos(math.radians(robo.ref))),int(robo.y+robot_radius*math.sin(math.radians(robo.ref)))), 2) 
	 	
	 	for i in range(0,8):
	 		if robo.sensor[i][3] > 0:
				pygame.draw.circle(gameDisplay, (255,0,0), (int(robo.sensor[i][0]), int(robo.sensor[i][1])), int(sensor_radius), 0) 	
			else:
				pygame.draw.circle(gameDisplay, (0,0,0), (int(robo.sensor[i][0]), int(robo.sensor[i][1])), int(sensor_radius), 0) 	
			
		for i in range(0,len(obs)):
			pygame.draw.circle(gameDisplay, (250,0,0), (int(obs[i].x), int(obs[i].y)), int(obs[i].radius), 0) 
		pygame.draw.circle(gameDisplay, (0,255,0), (int(obs[0].x), int(obs[0].y)), int(obs[0].radius), 0) 
		pygame.time.delay(100)			#updated every milisecond
		pygame.display.update()
	
pygame.quit()
quit()