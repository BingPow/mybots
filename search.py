from cmath import sin
from math import pi
import matplotlib.pylab as plt
import constants as c

import time

import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER
from parallelHillClimberB import PARALLEL_HILL_CLIMBERB

start = time.time()
for i in range(1):
    startSing = time.time()
    phc = PARALLEL_HILL_CLIMBER()

    phc.Evolve()

    phc.Show_Best()
    endSing = time.time()
    print("A-Time took for run " + str(i) + ": " + str(endSing-startSing) + "\n")
end = time.time()
print("Time took A: " + str(end-start) + "\n")
    

# start = time.time()
# for j in range(1):
#     startSing = time.time()
#     phcB = PARALLEL_HILL_CLIMBERB()

#     phcB.Evolve()

#     phcB.Show_Best()
#     endSing = time.time()
#     print("B-Time took for run " + str(j) + ": " + str(endSing-startSing) + "\n")
# end = time.time()
# print("Time took B: " + str(end-start) + "\n")
    

