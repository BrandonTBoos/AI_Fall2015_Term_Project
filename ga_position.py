import sys
import rospy
import gazebo_ros
from std_srvs.srv import Empty
from std_msgs.msg import Time
from gazebo_msgs.srv import *
import random
import time
import atexit
import math

class PID:
	def __init__(self):
		self.kp = 100
		self.ki = 0.02
		self.kd = -.5
		self.past = 0
		self.sum = 0
		self.sumMax = 100
		self.sumMin = -100
		self.set_point = 0
		self.error = 0

	def update(self,current_value):
		self.error = self.set_point - current_value
		p_value = self.kp*self.error
		d_value = self.kd*(self.error-self.past)
		self.past = self.error
		self.sum += self.error
		if self.sum > self.sumMax:
			self.sum = self.sumMax
		elif self.sum < self.sumMin:
			self.sum = self.sumMin
		i_value = self.sum*self.ki
		pid_val = p_value + d_value + i_value
		if pid_val < -.47:
			return -.47
		if pid_val > .47:
			return .47
		return pid_val

	def setPoint(self,set_point):
		self.set_point = set_point
		self.sum = 0
		self.past = 0


class Gene:
	def __init__(self, minp,maxp): #gene info is array of six torque values
		p = []
		for i in range(6):
			p.append(random.random()*(maxp[i]-minp[i])+minp[i])
		self.p = p

class Genome:
	def __init__(self, minv, maxv):
		self.maxv = maxv
		self.minv = minv
		self.geneList = []
		self.value = -1

	def add_Genes(self,length):
		self.geneList.append(Gene([0,0,0,0,0,0],[0,0,0,0,0,0]))
		while(len(self.geneList) < length):
			maxp = []
			minp = []
			for i in range(6):
				maxp.append(self.geneList[-1].p[i] + self.maxv*1)
				minp.append(self.geneList[-1].p[i] + self.minv*1)
			self.geneList.append(Gene(minp,maxp))
		max_to_norm_p = []
		for i in range(6):
			p = self.geneList[-1].p[i]
			if abs(p) < 2*math.pi:
				self.stretch(i)
				max_to_norm_p.append(1)
			else:
				max_to_norm_p.append(p/float(2*math.pi))
		for g in self.geneList:
			for i in range(6):
				g.p[i] = g.p[i]/float(max_to_norm_p[i]) #normalize

	def stretch(self, leg):
		max_p = self.geneList[-1].p[leg]
		if max_p > 0:
			sign_p = 1
		else:
			sign_p = -1
		stretch_amount = (2*math.pi - abs(max_p))*sign_p
		stretch_per = 0
		stretch_count = len(self.geneList)
		while(abs(stretch_amount) > 0.001):
			print stretch_amount
			stretch_per = stretch_amount/float(stretch_count)
			stretch_count = 0
			for i in range(len(self.geneList)-1):
				change = (self.geneList[i+1].p[leg] + stretch_per) - self.geneList[i].p[leg]
				if change < self.maxv*1 and change > self.minv*1:
					stretch_count += 1
					self.geneList[i+1].p[leg] = self.geneList[i+1].p[leg] + stretch_per
					stretch_amount += -1*stretch_per
				elif sign_p > 0:
					stretch_amount = stretch_amount - (self.maxv*1 - (self.geneList[i+1].p[leg] - self.geneList[i].p[leg]))
					self.geneList[i+1].p[leg] = self.maxv*1 + self.geneList[i].p[leg]
				elif sign_p < 0:
					stretch_amount = stretch_amount - (self.minv*1 - (self.geneList[i+1].p[leg] - self.geneList[i].p[leg]))
					self.geneList[i+1].p[leg] = self.minv*1 + self.geneList[i].p[leg]



	def mutate(self):
		option = random.randing(0,1)
		if option == 0:
			mutate1(self)
		else:
			mutate2(self)



	def mutate1(self):
		gene_num = random.randint(0,len(self.geneList)-1)
		leg = random.randint(0,5)
		if self.geneList[gene_num].p[leg] > self.geneList[gene_num-1].p[leg]:
			bound1 = self.geneList[gene_num-1].p[leg] + self.maxv*1
		else:
			bound1 = self.geneList[gene_num-1].p[leg] + self.minv*1
		if self.geneList[gene_num+1].p[leg] > self.geneList[gene_num].p[leg]:
			bound2 = self.geneList[gene_num+1].p[leg] - self.maxv*1
		else:
			bound2 = self.geneList[gene_num+1].p[leg] - self.minv*1
		self.geneList[gene_num].p[leg] = random.random()*(bound1 - bound2) + bound2

	def mutate2(self):
		leg1 = random.randint(0,5)
		leg2 = leg1
		while(leg1 == leg2):
			leg2 = random.randint(0,5)
		for g in geneList:
			g.t[leg1] = g.t[leg2]

	def testEvaluate(self): #evaluate is y distance
		if self.value == -1:
			joints = ['leg_1_joint', 'leg_2_joint','leg_3_joint','leg_4_joint','leg_5_joint','leg_6_joint']
			pid_cont = []
			sign = []
			for i in range(6):
				pid_cont.append(PID())
				if self.geneList[-1].p[i] < 0:
					sign.append(-1)
				else:
					sign.append(1)
			reset_model()
			time_g = update_time()
			for g in self.geneList:
				for loop in range(3):
					start = time_g
					velocity_set = []
					for i in range(6):
						pid_cont[i].setPoint(calculate_velocity(g.p[i]+loop*2*math.pi*sign[i],joints[i]))
					while(time_g < start+1):
						time_pid = time_g
						for i in range(6):
							apply_torque(pid_cont[i].update(get_velocity(joints[i])),joints[i])
						while(time_g < (time_pid+.01)):
							time_g = update_time()
			self.value = get_y()
		return self.value

	def smooth_about(self,g):
		cur_g = g
		smoothed_count = 0
		while(smoothed_count < 6):
			smoothed_count = 0
			for i in range(6):
				if self.geneList[cur_g].p[i]> self.geneList[cur_g-1].p[i]:
					if self.geneList[cur_g].p[i] > self.geneList[cur_g-1].p[i]+self.maxv*1:
						self.geneList[cur_g-1].p[i] = self.geneList[cur_g].p[i] - self.maxv*1
					else:
						smoothed_count += 1
				else:
					if self.geneList[cur_g].p[i] < self.geneList[cur_g-1].p[i]+self.minv*1:
						self.geneList[cur_g-1].p[i] = self.geneList[cur_g].p[i] - self.minv*1
					else: 
						smoothed_count += 1
			cur_g += -1
		cur_g = g
		smoothed_count = 0
		while(smoothed_count < 6):
			smoothed_count = 0
			for i in range(6):
				if self.geneList[cur_g] < self.geneList[cur_g+1]:
					if self.geneList[cur_g]+self.maxv*1 < self.geneList[cur_g+1]:
						self.geneList[cur_g+1] = self.geneList[cur_g] + self.maxv*1
					else:
						smoothed_count += 1
				else:
					if self.geneList[cur_g]+self.minv*1 > self.geneList[cur_g+1]:
						self.geneList[cur_g+1] = self.geneList[cur_g] + self.minv*1
					else: 
						smoothed_count += 1
			cur_g += -1


	def copy(self):
		new_Gene = Genome(self.minv, self.maxv)
		new_Gene.geneList.extend(self.geneList)
		return new_Gene

def update_time():
	rospy.wait_for_service('/gazebo/get_world_properties')
	try:
		reset_model = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
		world = reset_model()
		time_g = world.sim_time
		return time_g
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def calculate_velocity(position,joint):
	cur_pos = get_position(joint)
	des_vel = (position - cur_pos)/1
	return des_vel

def get_position(joint):
	rospy.wait_for_service('/gazebo/get_joint_properties')
	try:
		joint_call = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
		joint_data = joint_call(joint)
		position = joint_data.position[0]
		return position
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def get_velocity(joint):
	rospy.wait_for_service('/gazebo/get_joint_properties')
	try:
		joint_call = rospy.ServiceProxy('/gazebo/get_joint_properties', GetJointProperties)
		joint_data = joint_call(joint)
		velocity = joint_data.rate
		return velocity[0]
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def crossover(genome1,genome2):
	g_len = len(genome1.geneList)
	split_point = random.randint(1,g_len-2)
	child1 = Genome(genome1.minv, genome1.maxv)
	child2 = Genome(genome1.minv, genome1.maxv)
	for i in range(g_len):
		if i < split_point:
			child1.geneList.append(genome1.geneList[i])
			child2.geneList.append(genome2.geneList[i])
		else:
			child1.geneList.append(genome2.geneList[i])
			child2.geneList.append(genome1.geneList[i])
	child1.smooth_about(i)
	child2.smooth_about(i)
	return [child1, child2]

def display_pop(pop):
	val_list = []
	for i in pop:
		val_list.append(i.testEvaluate())
	print val_list

def save(generation, genome, best_r, r_avg):
	f = open('best_result.txt', 'w')
	gen = str(generation)
	f.write(gen)
	for g in genome.geneList:
		f.write('\n')
		no_comma = True
		for tor in g.p:
			if no_comma:
				no_comma = False
			else:
				f.write(',')
			f.write(str(tor))
	f.write('\n \n')
	no_comma = True
	for b in best_r:
		if no_comma:
			no_comma = False
		else:
			f.write(',')
		f.write(str(b))

	f.write('\n \n')
	no_comma = True
	for r in r_avg:
		if no_comma:
			no_comma = False
		else:
			f.write(',')
		f.write(str(r))
	f.close()

def average_E(population):
	val = []
	for g in population:
		val.append(g.testEvaluate())
	return sum(val)/len(val)

def test_pid():
	reset_model()
	joints = ['leg_1_joint', 'leg_2_joint','leg_3_joint','leg_4_joint','leg_5_joint','leg_6_joint']
	pid_cont = []
	for i in range(6):
		pid_cont.append(PID())
		pid_cont[-1].setPoint(-1)
	time_g = 0
	time_pid = 0
	start = update_time()
	while(time_g < start+30):
		for i in range(6):
			apply_torque(pid_cont[i].update(get_velocity(joints[i])),joints[i])
		while(time_g < time_pid+.01):
			time_g = update_time()
		time_pid = time_g

def ga_main():
	pop_num = 20
	gene_num = 10
	vmin = -math.pi/2
	vmax = math.pi/2
	generation_limit = 1000
	loop_count = 0
	population = []
	for i in range(pop_num):
		temp_genome = Genome(vmin,vmax)
		temp_genome.add_Genes(gene_num)
		population.append(temp_genome)

	best_records = [];
	records = [];
	atexit.register(save, loop_count, population[-1], best_records,records)
	population.sort(key=Genome.testEvaluate)
	best = population[pop_num-1].testEvaluate()	
	best_records.append(best)
	records.append(average_E(population))
	
	while(loop_count < generation_limit):
		for i in range(int(pop_num/4)):
			parent1 = random.randint(0,pop_num-1)
			parent2 = random.randint(0,pop_num-1)
			while(parent1 == parent2):
				parent2 = random.randint(0,pop_num-1)
			population.extend(crossover(population[parent1], population[parent2]))
			population[-2].mutate()
			population[-1].mutate()
		for i in range(int(pop_num/2)):
			randMutate = random.randint(0,pop_num-1)
			temp_Gene = population[randMutate].copy()
			temp_Gene.mutate()
			population.append(temp_Gene)
		population.sort(key=Genome.testEvaluate)
		del population[0:(len(population)-pop_num)]
		best = population[-1].testEvaluate()
		loop_count += 1
		print "generation: "
		print loop_count
	print "list_final"
	display_pop(population)


def reset_model():
	rospy.wait_for_service('/gazebo/reset_simulation')
	try:
		reset_model = rospy.ServiceProxy('/gazebo/reset_simulation', Empty )
		reset_model()
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e

def apply_torque(torque,joint):
	clear_torque(joint)
	add_torque(torque,joint)

def clear_torque(joint):
	rospy.wait_for_service('/gazebo/clear_joint_forces')
	try:
		reset_torque = rospy.ServiceProxy('/gazebo/clear_joint_forces', JointRequest)
		reset_torque(joint)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e	

def add_torque(torque,joint):
	rospy.wait_for_service('/gazebo/apply_joint_effort')
	try:
		apply_torque = rospy.ServiceProxy('/gazebo/apply_joint_effort', ApplyJointEffort )
		apply_torque(joint,torque,rospy.Time(),rospy.Duration(10))
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e	

def get_y():
	rospy.wait_for_service('/gazebo/get_model_state')
	try:
		model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		model = model_state('my_model','')
		return abs(model.pose.position.y)
	except rospy.ServiceException, e:
		print "Service call failed: %s"%e	
		return 0

if __name__ == "__main__":
	ga_main()