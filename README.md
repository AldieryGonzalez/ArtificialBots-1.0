# ArtificialBots-1.0
Repository to hold assignments for LudoBots in my Artificial Life Course at Northwestern University
**To run code, use the command `python search.py` in terminal**
You may need to use `python3` instead

[Video Demonstration](https://youtu.be/F5WWDQPztYo)

How it works


![alt text](https://github.com/AldieryGonzalez/ArtificialBots-1.0/blob/Bodies-2.0/Bodies2.PNG?raw=true)

Example Bot


![alt text](https://github.com/AldieryGonzalez/ArtificialBots-1.0/blob/Bodies-2.0/Bot5.PNG?raw=true)

## Fitness Function
Distance from starting Point as fast as possible
Sensing body parts are labeled by being green, while standard parts are a familiar cyan.

## How to play around
The `constants.py` file has some features that can be played with. Since the snake is procedurally generated, you can minmax the features of the body.
For now the snakes use a random 3d section of a cube with size that can be set in the constants. Functionality will be added later to support seeds and 
recursive body part creation for 3D generations of creatures.

If the code stops running due to permission errors, just rerun the code, this is an issue with OS not updating
permissions on time in the middle of runtime, so not much can be done.

## Required Pip Installs

 1. Numpy
 2. pybullet (Ludobots Reddit below details installation instructions)

## Credits and other repositories

 1. [LudoBots Reddit by DoctorJosh](https://www.reddit.com/r/ludobots/wiki/installation/)
 2. [Pyrosim Repo (Github)](https://github.com/jbongard/pyrosim)
    

    
