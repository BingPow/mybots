from cmath import sin
from math import pi
import matplotlib.pylab as plt
import constants as c

import os
from hillclimber import HILL_CLIMBER
from parallelHillClimber import PARALLEL_HILL_CLIMBER

for i in range(1):
    phc = PARALLEL_HILL_CLIMBER()

    phc.Evolve()
    phc.Show_Best()
    

