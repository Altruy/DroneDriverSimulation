# Drone Driver Simulation

This project consists of path planning, obstacle detection and avoidance in a 3D Occupancy map for a fixed wing UAV

## Summary

For intial path planning it uses on of:

- RRT
- RRT\*
- BiRRT
- PRM

then an obstacle is added on one of the waypoints at random to simmulate a real time obstacle

The drone follows waypoints while there is no obstacle in the way, then it avoides the obstacle from above, below, left or right which ever is shortest.

It updates the next local waypoint with the help of a sliding window which uses EUclidean distance aswell as shortest change in orientation angle from current orientation.

## Demo

Run the command in matlab for all 4 path planning algorithms

```
 main('RRT'); main('RRT*'); main('BiRRT'); main('PRM'); legend({'World','Start','Goal','RRT','RRT*','BiRRT','PRM'}, 'Location', 'southwest')
```

## Configurations

The main file contains editable global variables which govern various behaviors of different sub modules
