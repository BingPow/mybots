from solution import SOLUTION
import copy
import constants as c

import os

class PARALLEL_HILL_CLIMBER:

    def __init__(self):
        os.system("rm brain*.nndf")

        os.system("rm fitness*.txt")
        
        #self.parent = SOLUTION()
        self.parents = {}

        self.nextAvailableID = 0
        
        for x in range(c.populationSize):
            # create solution object
            sol = SOLUTION(self.nextAvailableID)

            #incremement ID by one for next round
            self.nextAvailableID += 1
        
            # put the solution object into an array
            self.parents[x] = sol
        

    def Show_Best(self):
        self.Select()
        low = 10
        number = 0
        for i in range(c.populationSize):
            #Switched < to >
            if self.parents[i].fitness > low:
                low = self.parents[i].fitness
                number = i

        #fitness = open("bestFitness.txt","r")
        #bestFitness = fitness.readline()

        self.G = "GUI"

        self.weights = self.parents[number].Get_Weights()
        f = open("weights.txt", "w")
        for i in range(c.numSensorNeurons):
            for j in range(c.numMotorNeurons):
                f.write(str(self.weights[i][j]) + "\n")
        f.close()
        
        self.parents[number].Start_Simulation(self.G)

    

    def Evolve(self):
        #G = "GUI"
        self.G = "DIRECT"
        
        self.Evaluate(self.parents)

        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation(self.G)
            currentGeneration += 1

    def Evolve_For_One_Generation(self,G):
        self.Spawn()
        self.Mutate()
        self.Evaluate(self.children)
        self.Print()
        self.Select()
    
    def Spawn(self):
        self.children = {}
        for i in range(c.populationSize):
            self.children[i] = copy.deepcopy(self.parents[i])
        

    def Mutate(self):
        for i in range(c.populationSize):
            self.children[i].Mutate()

    def Select(self):
        for i in range(c.populationSize):
            #Switched < to >
            if self.children[i].fitness > self.parents[i].fitness:
                self.parents[i] = self.children[i]

    def Print(self):
        print('\n')
        for n in range(c.populationSize):
            print("\nParent Fitness: "+str(self.parents[n].fitness)+" Child Fitness: "+str(self.children[n].fitness))
        print('\n')

    def Evaluate(self, solutions):
        # self.G = "GUI"
        self.G = "DIRECT"

        for x in range(c.populationSize):
            solutions[x].Start_Simulation(self.G)

        for y in range(c.populationSize):
            solutions[y].Wait_For_Simulation_To_End()
        

