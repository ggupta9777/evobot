"""
Copyright -  2015-2017
Gaurav Gupta (Now at Univ. of Heidelberg)
Department of Mechanical Enginering (ME769A)
Indian Institute of Technology, Kanpur (India)
"""

import random, pygame, math, numpy, pyevolve
from pyevolve import G1DBinaryString
from pyevolve import GSimpleGA
from pyevolve import Selectors
from pyevolve import Mutators
from pyevolve import DBAdapters
import matplotlib.pyplot as plt
from scipy.optimize import minimize
import time

flag_test = 0

#a flag to do optimization with NM, 0 means GA, 1 means NM
nm=0
#all angles to be taken in degrees
pygame.init()

white = (255,255,255)
red = (255,0,0)
black = (0,0,0)
v_max = 10
t_act = 1

pos_obs = [[315.0 ,325.0] ,[125.0 ,380.0] ,[125.0 ,115.0] ,[320.0 ,105.0] ,[385.0, 195.0],[100,290]]
quit_flag = 0
arena_x = 500
arena_y = 500
epsilon = arena_x/100				#threshold, one generation ends after coming this close
flag=0						#quit flag 
dt = 0.3					#update every 0.3 seconds
obstacle_radius = float(arena_x)/20;
robot_radius = float(arena_x)/40;
sensor_radius = float(robot_radius)/10;
sensor_range = 3*obstacle_radius		#i.e. actual range + obstacle_radius, collision will be checked with obstacle center for ease
iterations = 1

#Initialize pygame for visualization
gameDisplay = pygame.display.set_mode((arena_x,arena_y))
pygame.display.set_caption('Arena')

#putting in the obstacles, assumed to be cylindrical, x,y, radius assinged
class OBSTACLE:
	def __init__(self, pos):

		self.radius = obstacle_radius;		
		self.x = pos[0]
		self.y = pos[1]

class ROBOT:
	#Robot is initialized in space, properties etc defined
	def __init__(self):
		self.radius = robot_radius;
		self.x = float(arena_x)/2 #+ 0.9*arena_x*(random.random() -0.5) 
		self.y = float(arena_y)/2 #+ 0.9*arena_y*(random.random() -0.5) 
		self.theta = 0 		 #360*(random.random())			#initial direction of velocity
		self.ref = self.theta			#the reference marker to estimate the robot orientation
		self.wheel_radius = 20 			#Used to relate rpms to robot motion in x-y-theta form
		self.width = robot_radius		#distance between the two wheels
		self.sensor = []

	#Sensor Information at every update
	def new_sense(self, orient): 
		r = 0.9*self.radius			#sensors placed at 0.9r times the robot radius from the center
		orient = (orient)%360			#angular position of sensor 
		x = self.x + r*math.cos(math.radians(orient))
		y = self.y + r*math.sin(math.radians(orient))
		val = 0					#sensed value, will depend on robot orientation, sensor orientation, x,y coordinates and obstacle positions
		temp = [x,y,orient,val]
		self.sensor.append(temp)
		return self.sensor

	def sense_bot(self, obstacles):
		global quit_flag					
		for i in range(0,len(self.sensor)):	

			#orientation of ith sensor
			temp1 = self.sensor[i][2]

			#sensor value initialized as zero at every update step
			self.sensor[i][3] = 0		

			#check if any sensor goes out of arena, quit in that case!
			if self.sensor[i][0]<=epsilon or self.sensor[i][0]>=arena_x-epsilon or self.sensor[i][1]<=epsilon or self.sensor[i][1]>=arena_y-epsilon:
				quit_flag = 1
			
			#The following 4 cases are for computing sensor values due to the boundaries of the arena
			if self.sensor[i][0]<=sensor_range and self.sensor[i][2]>=90 and self.sensor[i][2]<=270:
				self.sensor[i][3] = self.sensor[i][3] + epsilon*abs(math.cos(math.radians(self.sensor[i][2])))**2/self.sensor[i][0]

			if self.sensor[i][0]>=arena_x -sensor_range  and (self.sensor[i][2]>=270 or self.sensor[i][2]<=90):
				self.sensor[i][3] = self.sensor[i][3] + epsilon*abs(math.cos(math.radians(self.sensor[i][2])))**2/(arena_x -self.sensor[i][0])	

			if self.sensor[i][1]<=sensor_range and self.sensor[i][2]>=180 and self.sensor[i][2]<=360:
				self.sensor[i][3] = self.sensor[i][3] + epsilon*abs(math.sin(math.radians(self.sensor[i][2])))**2/self.sensor[i][1]

			if self.sensor[i][1]>=arena_y - sensor_range and self.sensor[i][2]>=0 and self.sensor[i][2]<=180:
				self.sensor[i][3] = self.sensor[i][3] + epsilon*abs(math.sin(math.radians(self.sensor[i][2])))**2/(arena_y -self.sensor[i][1])	
				

			#To check for sensor readings due to obstacles within the arena confinement
			for j in range(len(obstacles)):			
				if (self.x-obstacles[j].x)**2 + (self.y-obstacles[j].y)**2 <= (self.radius+obstacles[j].radius+epsilon)**2:
					quit_flag = 1
				temp2 = (math.degrees(math.atan2(obs[j].y-self.sensor[i][1],obs[j].x-self.sensor[i][0]))+2*360)%360
				temp3 = math.sqrt((self.sensor[i][0]-obstacles[j].x)**2 + (self.sensor[i][1]-obstacles[j].y)**2)  
				
				if temp3 <= sensor_range+obstacle_radius:
					if (temp1>=270 and ((temp2>=temp1-90 and temp2<=360) or temp2<= (temp1+90)%360)) or (temp2>=270 and ((temp1>=temp2-90 and temp1<=360) or temp1<= (temp2+90)%360)) or (abs(temp1-temp2) <=90):
						if epsilon*abs(math.cos(math.radians(temp1-temp2))/(abs(temp3-obstacles[j].radius))) > self.sensor[i][3]:
							self.sensor[i][3] = epsilon*abs(math.cos(math.radians(temp1-temp2))/(abs(temp3-obstacles[j].radius)))
								
						#quit one generation when sensor is (or the 'epsilon') 5 cm away from the obstacle i.e. temp3-obstacle_radius
						#above fact is to be used in normalization of sensor readings, all reading to be multiplied with 5 as max reading is 1/5.
						#print  math.sqrt(temp3)-obstacles[j].radius, epsilon
    	
    		#sensor values returned
    		return [self.sensor[p][3] for p in range(8)]

   	def move_bot(self, sensor_values,params):

   		input1 = sensor_values+[1, w_old[0]]
		input2 = sensor_values+[1, w_old[1]]
		out1 = sigmoid(numpy.dot(numpy.array((input1)), numpy.array((params[0])))) - 0.5
		out2 = sigmoid(numpy.dot(numpy.array((input2)), numpy.array((params[1])))) - 0.5

		#rotation of the robot body, angular velocity
		omega = (out1-out2)*self.wheel_radius/self.width	

		#velocity of robot com
		vel = (out1+out2)*self.wheel_radius/2 			
		dtheta = omega*dt
		#print out1,out2,vel,dtheta

		#sensors placed at 0.9r times the robot radius from the center
   		r = 0.9*self.radius			

   		#update of sensor positions in global frame
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


#sigmoidal function
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

#converts binary results of GA to decimal format
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
		
#returns the fitness score for 1500 updates/collision, whichever happens first
def eval_func(chromosome):
	botx = []
	boty = []
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
	while count<=1500:
		count = count+1
		vals = robo.sense_bot(obs)
		#print quit_flag, vals
		if quit_flag==1:
			return 0
			break

		i_max = i_max + max(vals)
		if max(vals)>=1:
			print max(vals)
		w = robo.move_bot(vals,params)
		botx.append(robo.x)
		boty.append(robo.y)
		#print w, vals
		V = V + abs(w[0])+abs(w[1])
		w0t1 = w[0] + 0.5
		w0t2 = w[1] + 0.5
		v = v + abs(w0t1-w0t2)
	i_max = i_max/count
	v = v/count
	V = V/count
	score = float(V*(1-(math.sqrt(v))**2)*(1-i_max**2)*math.sqrt((max(botx)-250)**2 + (max(boty)-250)**2)/math.sqrt(250**2+250**2))
	return score		

def neldermead(chromosome):
	botx = []
	boty = []
	global quit_flag, fin_para
	quit_flag=0
	score = float(0.0)
	count = 0
	i_max = 0
	V = 0
	v = 0
	robo = create_bot()
	params = [chromosome[0:len(chromosome)/2], chromosome[len(chromosome)/2:]]
	fin_para = params
	while count<=1500:
		vals = robo.sense_bot(obs)
		if quit_flag==1:
			return 0
			break

		i_max = i_max + max(vals)
		if max(vals)>=1:
			print max(vals)
		w = robo.move_bot(vals,params)
		botx.append(robo.x)
		boty.append(robo.y)
		#print w, vals
		V = V + abs(w[0])+abs(w[1])
		w0t1 = w[0] + 0.5
		w0t2 = w[1] + 0.5
		v = v + abs(w0t1-w0t2)
	i_max = i_max/count
	v = v/count
	V = V/count
	score = 5.0 - float(V*(1-(math.sqrt(v))**2)*(1-i_max**2)*math.sqrt((max(botx)-250)**2 + (max(boty)-250)**2)/math.sqrt(250**2+250**2))
	return score		


#parameters of NN that decide the behavior of bot
writable_params = 0

#GA based optimization
def ga_bot():
	global writable_params
# Genome instance
	genome = G1DBinaryString.G1DBinaryString(200)

	# The evaluator function (objective function)
	genome.evaluator.set(eval_func)
	genome.mutator.set(Mutators.G1DBinaryStringMutatorFlip)

	# Genetic Algorithm Instance
	ga = GSimpleGA.GSimpleGA(genome)
	ga.selector.set(Selectors.GTournamentSelector)
	ga.setGenerations(25)
	# Do the evolution, with stats dump
	# frequency of 10 generations
	xyz = ga.evolve(freq_stats=1)
	print xyz
	# Best individual
	best = ga.bestIndividual()
	writable_params = best
	print best
	#sqlite_adapter = DBAdapters.DBSQLite(identify="ex1")
	#ga.setDBAdapter(sqlite_adapter)
	#pyevolve_graph.py -i ex1 -1


def nm_bot(guess):
	res = minimize(neldermead, guess, method='nelder-mead',
	                options={'xtol': 1e-1, 'disp': True})
	print res
	writable_params = res

ig = numpy.array([0]*20)
if flag_test==0:	
	if nm==0:
		ga_bot()
		writable_params =dec_params(writable_params,10)
	else:
		tic = time.time()
		nm_bot(ig)
		toc = time.time()
		print toc,tic

count1 = 0	
quit_flag=0
robo = create_bot()
plot_data = []

for n in range(iterations):		
	gameEXIT = False

	while not gameEXIT and (quit_flag==0 and count1<=1500):	
		sensor_values = [robo.sensor[p][3] for p in range(8)]
			
		vals = robo.sense_bot(obs)
		if flag_test==0:
			speeds = robo.move_bot(vals, writable_params)#[[0 for _ in range(10)],[1 for _ in range(10)]])#fin_para)
		else:	
			speeds = robo.move_bot(vals, [[0 for _ in range(10)],[1 for _ in range(10)]])#)#fin_para)
		
		plot_data.append([robo.x, robo.y]) 

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
		#pygame.draw.circle(gameDisplay, (0,255,0), (int(obs[0].x), int(obs[0].y)), int(obs[0].radius), 0) 
		pygame.time.delay(100)			#updated every milisecond
		pygame.display.update()
		count1 = count1 + 1

print writable_params
	

#THIS IS WHERE PLOTS ARE MADE

circles = []
point_circles = []
fig = plt.gcf()
ax = plt.gca()
ax.cla() # clear things for fresh plot
# change default range so that new circles will work
ax.set_xlim((0,arena_x))
ax.set_ylim((0,arena_y))
for i in range(len(pos_obs)):
	circles.append(plt.Circle((pos_obs[i][0], arena_y - pos_obs[i][1]),obstacle_radius,color='r'))
	fig.gca().add_artist(circles[i])
fig.gca().add_artist(plt.Circle((arena_x/2, arena_y/2),5,color='g'))
for i in range(len(plot_data)):
	point_circles.append(plt.Circle((plot_data[i][0], arena_y - plot_data[i][1]),1,color='b'))
	fig.gca().add_artist(point_circles[i])
fig.savefig('plotcircles2.jpg')

pygame.quit()
quit()