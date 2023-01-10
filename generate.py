import pyrosim.pyrosim as pyrosim

length, width, height = 1, 1, 1
x, y, z = 0, 0, height/2

pyrosim.Start_SDF("boxes.sdf")
for k in range(4):
    for j in range(10):
        for i in range(10):
            scale = .90**i
            pyrosim.Send_Cube(name=f'Box{i}', pos=[
                x + (j * length), y + (k * width), z + (i * height)], size=[length * scale, width * scale, height * scale])


pyrosim.End()
