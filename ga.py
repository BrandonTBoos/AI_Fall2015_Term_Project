import random


class Gene:
	def __init__(self, v):
		self.v = v

class Genome:
	def __init__(self, minv, maxv):
		self.maxv = maxv
		self.minv = minv
		self.geneList = []
		self.value = -1

	def add_Genes(self,length):
		while(len(self.geneList) < length):
			self.geneList.append(Gene(random.random()*(self.maxv - self.minv) + self.minv))

	def mutate(self):
		gene_num = random.randint(0,len(self.geneList)-1)
		self.geneList[gene_num].v = random.random()*(self.maxv - self.minv) + self.minv

	def testEvaluate(self):
		if self.value == -1:
			vel = 0
			for i in range(len(self.geneList)):
				vel += self.geneList[i].v
			self.value = vel
		return self.value

	def copy(self):
		new_Gene = Genome(self.minv, self.maxv)
		new_Gene.geneList.extend(self.geneList)
		return new_Gene

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
	return [child1, child2]

def display_pop(pop):
	val_list = []
	for i in pop:
		val_list.append(i.testEvaluate())
	print val_list

def main():
	pop_num = 1000
	gene_num = 10
	vmin = 0
	vmax = 10
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
	print "list_final"
	display_pop(population)
		


main()