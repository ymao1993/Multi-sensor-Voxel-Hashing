# Multi-Sensor Voxel Hashing

## Background

Voxel Hashing [1] is a system for scalable real-time 3D reconstruction. It is based on Truncated Signed Distance Function (TSDF) and Voxels. It achieves scalability of sensor motion with a sparse representation of the world, i.e., Voxel hash table, and bidirectional streaming of voxel data between GPU and CPU.


## Problem Statement

Though efficient, the Voxel Hashing system [1] is built around a single sensor. In this project, we extended the system to support real-time 3D reconstruction with multiple sensors contributing to the same model. Specifically, we make the following contributions:

1. We demonstrate how a single-sensor system, exemplified by Voxel Hashing, can be adapted to support 3D reconstruction from multiple sensors.

2. We identify the critical performance issues that arise in the presence of multiple sensors.

3. We propose several techniques to tackle those issues to allow for scalability of multiple sensors.


## System Architecture



## Optimizations

### Baseline

Our baseline solution is to poll all registered sensors in round robin and fetch one frame at a time. The frame is integrated into the voxel model immediately after fetching. This is the most straightforward solution when one wants to integrate all input sensors to a single 3D model at real-time.


### Buffering and Mini-Batch Processing

With this optimization, we maintain a buffer for incoming frames from different sensors
and schedule the whole batch for the integration process.

We find such buffering significantly improves data locality during the integration process,
because the voxel hashing data structures remain in cache across consecutive integration operations.

Buffering also opens opportunities of *reordering* frames for integration. In this project, we consider the following reordering strategies:

1. Random order.

2. Order by sensor ID.

3. Iteratively selecting the frame whose camera position is closest to the last integrated one.


Note that it is not scalable to integrate one frame at a time as the number of sensors increases. Buffering reduces the average integration time per frame and thus allows for better scalability. We can maintain user interactiveness by selecting a proper batch size. Consider a batch size of 30 and three input sensors, where each sensor captures at 30 fps. Then each sensor is buffered for 10 frames, resulting in 1/3 second update latency, which should still be acceptable for user interactiveness.


### Adaptive Streaming

In Voxel Hashing, streaming means to migrate voxel data back and forth between GPU and CPU. It is necessary to achieve scalability and avoid overflowing the hash table that needs to fit in GPU.

However, we observe that blind streaming as in the original system can become a bottleneck and hurt performance. We improve it with *adaptive* streaming. That is, we dynamically disable streaming when the hash table occupancy is low and enable it when the occupancy is high.

During reconstruction with multiple sensors, the occupancy of the hash table can vary drastically. The occupancy may be low when several sensors are pointing at roughly the same object. However, occupancy can become *N*x higher if all those sensors are focusing on different scenes.



### Heatmap Based Skipping


## Results

### Demo

### Experiments



Reference:

[[1] Nießner, M., Zollhöfer, M., Izadi, S., & Stamminger, M. (2013). Real-time 3D reconstruction at scale using voxel hashing. ACM Transactions on Graphics (TOG), 32(6), 169.](http://www.graphics.stanford.edu/~niessner/niessner2013hashing.html)
