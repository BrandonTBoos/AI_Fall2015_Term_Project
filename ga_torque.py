import sys
import rospy
import gazebo_ros
from std_srvs.srv import Empty
from std_msgs.msg import Time
from gazebo_msgs.srv import *
import random
import time


class Gene:
	def __init__(self, mint,maxt): #gene info is array of six torque values
		t = []
		for i in range(6):
			t.append(random.random()*(maxt-mint)+mint)
		self.t = t

class Genome:
	def __init__(self, mint, maxt):
		self.maxt = maxt
		self.mint = mint
		self.geneList = []
		self.value = -1

	def add_Genes(self,length):
		while(len(self.geneList) < length):
			self.geneList.append(Gene(self.mint,self.maxt))

	def mutate1(self):
		gene_num = random.randint(0,len(self.geneList)-1)
		self.geneList[gene_num].t[random.randint(0,6)] = random.random()*(self.maxt - self.mint) + self.mint

	def testEvaluate(self): #evaluate is y distance
		if self.value == -1:
			reset_model()
			time_g = update_time()
			joints = ['leg_1_joint', 'leg_2_joint','leg_3_joint','leg_4_joint','leg_5_joint','leg_6_joint']
			for g in self.geneList:
				for loop in range(5):
					start = time_g
					for i in range(6):
						apply_torque(g.t[i],joints[i])
					while(time_g < start+1):
						time_g = update_time()
					print time_g
			self.value = get_y()
		return self.value

	def copy(self):
		new_Gene = Genome(self.mint, self.maxt)
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

def crossover(genome1,genome2):
	g_len = len(genome1.geneList)
	split_point = random.randint(1,g_len-2)
	child1 = Genome(genome1.mint, genome1.maxt)
	child2 = Genome(genome1.mint, genome1.maxt)
	for i in range(g_len):
		if i < split_point:
			child1.geneList.append(genome1.geneList[i])
			child2.geneList.append(genome2.geneList[i])
		else:
			child1.geneList.append(genome2.geneList[i])
			child2.geneList.append(genome1.geneList[i])
	return [child1, child2]

def display_pop(pop):
	val_list = []
	for i in pop:
		val_list.append(i.testEvaluate())
	print val_list

def ga_main():
	pop_num = 100
	gene_num = 10
	vmin = -.47
	vmax = .47
	generation_limit = 1000
	loop_count = 0
	population = []
	for i in range(pop_num):
		temp_genome = Genome(vmin,vmax)
		temp_genome.add_Genes(gene_num)
		population.append(temp_genome)
	population.sort(key=Genome.testEvaluate)
	best = 0;
	while(loop_count < generation_limit):
		for i in range(int(pop_num/2)):
			parent1 = random.randint(0,pop_num-1)
			parent2 = random.randint(0,pop_num-1)
			while(parent1 == parent2):
				parent2 = random.randint(0,pop_num-1)
			population.extend(crossover(population[parent1], population[parent2]))
			population[-2].mutate1()
			population[-1].mutate1()
		for i in range(int(pop_num/2)):
			randMutate = random.randint(0,pop_num-1)
			temp_Gene = population[randMutate].copy()
			temp_Gene.mutate1()
			population.append(temp_Gene)
		population.sort(key=Genome.testEvaluate)
		del population[0:(len(population)-pop_num)]
		best = population[-1].testEvaluate()
		loop_count += 1
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
