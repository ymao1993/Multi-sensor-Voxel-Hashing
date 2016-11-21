# Checkpoint Report: Supporting Multi-User Concurrent Model Update in Voxel Hashing 

(21 Nov 2016)

## Summary

We have:

1. Compiled and run the starter code from [1] on our own computers. We have successfully run the code with video and Kinect input.

2. Studied and understood the code structure

3. Annotated potential code segments we will work on.

4. Profiled the time breakdown of two major components: pose estimation (ICP) and model integration (voxel hashing).

5. Written the test harness to display performance result.


## Objective

Our objective is to enhance the system with capability to handle multiple input streams concunrrently. That is, we imagine a couple of users scanning the environment with multiple devices. 


## Details

### Running the starter code 

(screenshot)

### Code Structure

The Voxel Hashing system uses the DXUT application framework. Functions are registered to callbacks to handle input frames, perform 3D reconstruction and render the model on screen.

Each frame undergoes two major steps: (1) Pose estimation via the iterative cloest point (ICP) algorithm and (2) world model update via Voxel Hashing.

### Preliminary Profiles

(profile numbers)


## Challenges

+ Modifying the DXUT-based system to support multiple input streams.

+ Accelerate the ICP algorithm as it is shown to be a bottleneck.

+ Design a mechanism to allow for concurrent update to the voxel hash table (i.e., scene representation) with minimal synchronization overhead.


## Schedule

+ Week ending Dec 16: Tuning and improving performance based on GPU.
+ Week ending Dec 9: Implement concurrent update mechanism to the scene representation.
+ Week ending Dec 2: Optimize ICP; Design and implement better concurrent update mechanism to the scene representation.
+ Week ending Nov 25: Set up the framework to receive multiple input streams and allow them to update the scene representation (with a global lock as start)


References:

[[1] Nießner, M., Zollhöfer, M., Izadi, S., & Stamminger, M. (2013). Real-time 3D reconstruction at scale using voxel hashing. ACM Transactions on Graphics (TOG), 32(6), 169.](http://www.graphics.stanford.edu/~niessner/niessner2013hashing.html)
