import sys
from simulation import SIMULATION

directOrGUI = sys.argv[1]
solutionID = sys.argv[2]
eval = sys.argv[3]

simulation = SIMULATION(directOrGUI, solutionID)
simulation.Run()
if eval == "True":
    simulation.Get_Fitness()
