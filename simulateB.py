from simulationB import SIMULATIONB
import sys

directOrGUI = sys.argv[1]
solutionID = sys.argv[2]

simulation = SIMULATIONB(directOrGUI, solutionID)
simulation.Run()
simulation.Get_Fitness()
