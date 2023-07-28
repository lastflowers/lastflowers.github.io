---
layout: page
title: Indoor and outdoor integrated navigation
description: Visual-inertial odometry for a quadruped robot (2022)
img: assets/img/quad.png
importance: 1
category: research
related_publications:
---

We showcase ensemble visual-inertial odometry (EnVIO) working on a quadruped robot. We saw a lot of challenges in this platform due to a sudden change in motion as indicated by the accelerometer measurements in the bottom right. We manually controlled the robot in a campus environment using off-the-shelf sensors. 

The ground-truth position was marked by a laser tracker (Leica TS-16). We post-processed EnVIO in NUC i7 having CPU usages ~200% and ~40ms processing time per frame. In a total of 96.5m trajectory, EnVIO had 0.70m time RMSE.


🚀 **Outdoor test** 🚀

<a href="https://www.youtube.com/watch?v=IsweJAlBv5s&t=0s" target="_blank">
 <img src="http://img.youtube.com/vi/IsweJAlBv5s/0.jpg" alt="Watch the video" width="400" />
</a>