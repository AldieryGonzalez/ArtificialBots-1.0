# ArtificialBots-1.0
Repository to hold assignments for LudoBots in my Artificial Life Course at Northwestern University
**To run code, use the command `python search.py` in terminal**
You may need to use `python3` instead

[Video Demonstration](https://youtu.be/UfuODCzTVGQ)

[Example Graph](https://i.imgur.com/rBnmZal.png)


## Fitness Function
The main requirement is for the bot to use whatever means possible to make the ball into the goal
Since the bot currently has no arms, no harm, no foul for not using their feet. As you will probably notice,
the bot tends to favor headers when trained

TLDR: Closer the ball gets to goal, the better.

## How to play around
The `constants.py` file has some features that can be played with such as the goalSize and goalPosition.
There are also generations and populationSize that can be messed with to maximize the efficiency of the bot training
though the default settings have hit a happy medium in my computer. 

If the code stops running due to permission errors, just rerun the code, this is an issue with OS not updating
permissions on time in the middle of runtime, so not much can be done.

## Required Pip Installs

 1. Numpy
 2. pybullet (Ludobots Reddit below details installation instructions)

## Credits and other repositories

 1. [LudoBots Reddit by DoctorJosh](https://www.reddit.com/r/ludobots/wiki/installation/)
 2. [Pyrosim Repo (Github)](https://github.com/jbongard/pyrosim)
    
