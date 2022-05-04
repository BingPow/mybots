from cmath import sin
from math import pi
import matplotlib.pylab as plt
import constants as c

import time

import os
from parallelHillClimber import PARALLEL_HILL_CLIMBER
from parallelHillClimberB import PARALLEL_HILL_CLIMBERB

for i in range(10):
    phc = PARALLEL_HILL_CLIMBER()

    start = time.time()

    phc.Evolve()

    end = time.time()

    print("Time took: " + str(end-start) + "\n")

    phc.Show_Best()

    
for j in range(10):
    phcB = PARALLEL_HILL_CLIMBERB()

    start = time.time()

    phcB.Evolve()

    end = time.time()

    print("Time took: " + str(end-start) + "\n")

    phcB.Show_Best()
    

