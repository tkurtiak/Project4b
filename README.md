Team 5 QDMC Submission for Project 4B in ENAE788M

This project consists of two file groups:
1) Bridge Detection and Crossing;
2) Wall Detection and Avoidance

The files for each are described:
1) Bridge Detect:
rivercross.launch - launches the bridge detection script and coresponding bebop and camera nodes
bridge_detect_realtime_v5.py - detects the bridge using the down facing camera and texture convolutions.  Plots a trajectory line to cross the bridge normal to the river.
cross_bridge.py - This script translated detected bridge information and controls the vehicle to cross the bridge

2) Wall Avoidance:
wallavoid.launch - launches the wall detection script and coresponding bebop and camera nodes
Wallcontroller_V3_WORKS WELL! - Wall avoidance controller using center points of the wall as identified by the wall detection script.
detectWall_v1.py - wall detection script using feature matching, RANSAC, clustering, masking, and moving average.  Pshew, that's a lot.