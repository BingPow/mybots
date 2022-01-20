from time import sleep
import pybullet as p
physicsClient = p.connect(p.GUI)
for i in range (1,1001):
    print(i)
    p.stepSimulation()
    time.sleep(5)
p.disconnect()
