from pyevolve import G1DBinaryString
from pyevolve import GSimpleGA
from pyevolve import Selectors
from pyevolve import Mutators

# This function is the evaluation function, we want
# to give high score to more zero'ed chromosomes

def test(chromosome, len_main, len_frac):
	par = []
	len_sub = len_main+len_frac
	for i in range(len(chromosome)/len_sub):
		temp = chromosome[i*len_sub:(i+1)*len_sub]
		temp_1 = temp[0:len_main]
		temp_2 = temp[len_main:]
		num = int(''.join(map(str,temp_1)))
		num = int(str(num),2)
		num = float(num)
		frac = int(''.join(map(str,temp_2)))
		frac = int(str(frac),2)
		b = float(frac)/(10**(len(str(frac))))
		par.append(num+b)
	return float(1000 - (sum(par[1:10]) - sum(par[10:])))
	
def eval_func(chromosome):
   score = 0.0
   score = test(chromosome, 2,8)
      
   return score

# Genome instance
genome = G1DBinaryString.G1DBinaryString(60)

# The evaluator function (objective function)
genome.evaluator.set(eval_func)
genome.mutator.set(Mutators.G1DBinaryStringMutatorFlip)

# Genetic Algorithm Instance
ga = GSimpleGA.GSimpleGA(genome)
ga.selector.set(Selectors.GTournamentSelector)

ga.setGenerations(100)

# Do the evolution, with stats dump
# frequency of 10 generations
ga.evolve(freq_stats=10)

# Best individual
best = ga.bestIndividual()
print best, test(best, 2, 2)
