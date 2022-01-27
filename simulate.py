import time
import pybullet as p
physicsClient = p.connect(p.GUI)
p.setGravity(0,0,-9.8)
p.loadSDF("box.sdf")
for i in range (1,1001):
    print(i)
    p.stepSimulation()
    time.sleep(1/60)
p.disconnect()
