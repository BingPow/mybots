import pyrosim.pyrosim as pyrosim
pyrosim.Start_SDF("boxes.sdf")
perc = 0.9
for n in range(5):
    for m in range(5):
        length = 1
        width = 1
        height = 1
        x = 0
        y = 0
        z = 0.5
        for i in range(10):
            pyrosim.Send_Cube(name="Box", pos=[x+n,y+m,z] , size=[length,width,height])
            length *= perc
            width *= perc
            height *= perc
            z += 1
pyrosim.End()

