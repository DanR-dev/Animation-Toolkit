# Sentinel-Script
This script is a universal tool for animating robots in the game Space Engineers. The game has a built in C# API that allows scripts to control in-game vehicles and vehicle components. The script has an implementation for the Sentinel walker but can be implemented for any robot that performs repeating tasks at variable speeds.

### How it Works
- Each rotor, hinge or hydraulic is modelled as a Joint that can be set to move towards a given position in a given amount of time.
- Any number of Joints can be grouped into a Leg (or more generically, a limb).
- A Pose is a set of Joint positions that can be applied to a Leg and a base time that indicates how long the Joints of that Leg should take to reach those positions.
- Each Leg has a sequence of Poses that define its walking animation. The positions in each Pose define the exact positions of each Joint at key points in the animation, and the base times define the relative speed at which the leg should move between Poses.
- By incrementing through these Poses and multiplying the produced Joint velocities by a target speed, the Leg will perform its walking animation at the chosen speed.
- By decrementing through poses instead, the Leg can perform its walking animation in reverse.
- When applied to a robot like the Sentinel walker, it can produce a very complex walking animation very precisely and at any speed.

### Links
Sentinel walker: https://steamcommunity.com/sharedfiles/filedetails/?id=2254052110  
API Index (fan-made): https://github.com/malware-dev/MDK-SE/wiki/Api-Index
