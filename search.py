from cmath import sin
from math import pi
import matplotlib.pylab as plt
import constants as c

import time

import os
from hillclimber import HILL_CLIMBER
from parallelHillClimber import PARALLEL_HILL_CLIMBER

for i in range(1):
    phc = PARALLEL_HILL_CLIMBER()

    start = time.time()

    phc.Evolve()

    end = time.time()

    print("Time took: " + str(end-start))

    phc.Show_Best()

    print("printing")

    

