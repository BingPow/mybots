from solution import SOLUTION
import copy
import constants as c

class HILL_CLIMBER:

    def __init__(self):

        self.parent = SOLUTION()

    def Show_Best(self):
        self.Select()
        self.parent.Evaluate("GUI")

    def Evolve(self):
        G = "GUI"
        #G = "DIRECT"
        self.parent.Evaluate(G)
        G = "DIRECT"
        for currentGeneration in range(c.numberOfGenerations):
            self.Evolve_For_One_Generation(G)

    def Evolve_For_One_Generation(self,G):
        self.Spawn()

        self.Mutate()
        self.child.Evaluate(G)
        self.Print()
        self.Select()
    
    def Spawn(self):
        self.child = copy.deepcopy(self.parent)

    def Mutate(self):
        self.child.Mutate()

    def Select(self):
        if self.child.fitness<self.parent.fitness:
            self.parent = self.child

    def Print(self):
        print("\nParent Fitness: "+str(self.parent.fitness)+" Child Fitness: "+str(self.child.fitness))

