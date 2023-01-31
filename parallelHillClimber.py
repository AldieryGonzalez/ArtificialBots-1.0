from solution import SOLUTION
import constants as c
import copy


class PARALLEL_HILL_CLIMBER:
    def __init__(self):
        # self.parent = SOLUTION()
        self.nextAvailableID = 0
        self.parents = {}
        for i in range(c.populationSize):
            self.parents[i] = SOLUTION(self.nextAvailableID)
            self.nextAvailableID += 1

    def Evolve(self):
        # self.parent.Evaluate("GUI")
        # for currentGeneration in range(c.numberOfGenerations):
        #     self.Evolve_For_One_Generation()
        for key in self.parents:
            self.parents[key].Start_Simulation("DIRECT")
        for key in self.parents:
            self.parents[key].Wait_For_Simulation_To_End()

    def Evolve_For_One_Generation(self):
        self.Spawn()
        self.Mutate()
        self.child.Evaluate("DIRECT")
        self.Select()
        print(
            f"\n\nParent: {self.parent.fitness}, Child: {self.child.fitness}\n")

    def Spawn(self):
        self.child = copy.deepcopy(self.parent)
        self.child.Set_ID(self.nextAvailableID)
        self.nextAvailableID += 1

    def Mutate(self):
        self.child.Mutate()

    def Select(self):
        if (self.parent.fitness > self.child.fitness):
            self.parent = self.child

    def Show_Best(self):
        # self.parent.Evaluate("GUI")
        pass
