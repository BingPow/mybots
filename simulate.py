import time
import pybullet as p
physicsClient = p.connect(p.GUI)
for i in range (1,1001):
    print(i)
    p.stepSimulation()
    time.sleep(1/60)
p.disconnect()
