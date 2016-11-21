
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

### Running and Modifying the starter code

After setting up the project. We made three captures with Kinect and modified the sensor interface so we can directly load the Kinect's binary dump to run our project even without a Kinect at hand.

Below are some screenshots:

![nssh_1](screenshot0.jpg)

![nssh_2](screenshot1.jpg)


### System Structure

The Voxel Hashing system uses the DXUT application framework. Functions are registered to callbacks to handle input frames, perform 3D reconstruction and render the model on screen.

Each frame undergoes two major steps: (1) Pose estimation via the iterative cloest point (ICP) algorithm and (2) world model update via Voxel Hashing.

### Preliminary Profiling

We profiled the original code on the following three sequences. In **nsh4224_static.sensor** we are simply holding the sensor statically. In **nsh4224_dynamic.sensor** we are rotating and moving the Kinect within a small area. In the last sequence **nsh4224_dynamic_quick.sensor** we are rotating the Kinect by large angles in a random manner. The results are shown below:

+ nsh4224_static.sensor

```
=================Time Stats=================
id      bucket name     total time      average time
[1]     Integration     3451.00ms       11.5418ms
[2]     ICP Tracking    1550.00ms       5.2013ms
[3]     Streaming       141.00ms        0.4716ms
===================END======================
```

+ nsh4224_dynamic.sensor

```
=================Time Stats=================
id      bucket name     total time      average time
[1]     Integration     5333.00ms       13.6744ms
[2]     ICP Tracking    2186.00ms       5.6195ms
[3]     Streaming       923.00ms        2.3667ms
===================END======================
```

+ nsh4224_dynamic_quick.sensor

```
=================Time Stats=================
id      bucket name     total time      average time
[1]     Integration     4353.00ms       13.9968ms
[2]     Streaming       2475.00ms       7.9582ms
[3]     ICP Tracking    1955.00ms       6.3065ms
===================END======================
```

Notice that volumetric integration is still the most time-consuming part despite the fact that it is already parallelized on GPU. It is also noteworthy that When we are dealing with an RGBD camera which moves fast, the overhead of streaming voxels back and forth between CPU and GPU increases heavily. Compared to integration and data streaming, the time spent in ICP tracking is actually pretty stable.

### Supporting Multiple Input Streams

With multiple input streams, the system will become highly stressed in both compute and memory. For example, when there are *N* users, each user needs to run her own ICP for pose estimation, and the scene model may need to be updated at *N*x frame rate. We anticipate the GPU will become fully loaded as both ICP and Voxel Hashing are running on it.

We propose to tackle this problem by smartly **scheduling** the operations of each input streams. It includes:

1. Re-ordering the ICP and Voxel Hashing steps among all inputs. Specifically, rearranging the order in which the inputs update the scene model may have significantly impact on memory reuse on GPU, because the voxel hash table is too large to reside entirely in GPU memory so there is swapping between CPU and GPU as in [1].

2. Potentially skipping some input frames. When the system becomes highly loaded, we may want to skipping model update with some frames with minimal quality loss. One intuition is to skip a frame if the scene in its frustum is alreadly well reconstructed (as measured by a *confidence* score).

Currently we are considering the following metrics that can be used to schedule the inputs:

+ The number of unprocessed frames in each input's buffer

+ The estimated camera pose, and its locality with respect to the portion of hash table being in GPU memory

+ The confidence of the frame's frustum measuring how well the scene is reconstructed.


## Challenges

+ Modifying the system architecture to support multiple input streams.

+ Designing a good scheduler to handle different input streams.


## Schedule

+ Week ending Dec 16: Tuning and improving performance based on GPU.
+ Week ending Dec 9: Implement concurrent update mechanism to the scene representation.
+ Week ending Dec 2: Optimize ICP; Design and implement better concurrent update mechanism to the scene representation.
+ Week ending Nov 25: Set up the framework to receive multiple input streams and allow them to update the scene representation (with a global lock as start)


References:

[[1] Nie??ner, M., Zollh??fer, M., Izadi, S., & Stamminger, M. (2013). Real-time 3D reconstruction at scale using voxel hashing. ACM Transactions on Graphics (TOG), 32(6), 169.](http://www.graphics.stanford.edu/~niessner/niessner2013hashing.html)
