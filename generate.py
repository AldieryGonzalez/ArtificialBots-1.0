import pyrosim.pyrosim as pyrosim

length, width, height = 1, 1, 1
x, y, z = 0, 0, height/2

pyrosim.Start_SDF("world.sdf")
pyrosim.Send_Cube(name=f'Box', pos=[x, y, z], size=[length, width, height])

pyrosim.End()
