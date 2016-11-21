# Supporting Multi-User Concurrent Model Update in Voxel Hashing


## Team

Yu Mao (yumao@cmu.edu)

Ziqiang Feng (zf@andrew.cmu.edu)


## Summary

We will enhance Voxel Hashing based 3D reconstruction [4] with capability to handle multiple user input concurrently. That is, we can have multiple users scanning the environment with different devices and contribute to a shared 3D model.


## Background & Challenges

### Voxel Hashing [4]

Online 3D reconstruction is gaining newfound interest due to the availability of real-time consumer depth cameras. The basic problem takes live overlapping depth maps as input and incrementally fuses these into a single 3D model. This is challenging particularly when real-time performance is desired without trading quality or scale. We contribute an online system for large and fine scale volumetric reconstruction based on a memory and speed efficient data structure. Our system uses a simple spatial hashing scheme that compresses space, and allows for real-time access and updates of implicit surface data, without the need for a regular or hierarchical grid data structure. Surface data is only stored densely where measurements are observed. Additionally, data can be streamed efficiently in or out of the hash table, allowing for further scalability during sensor motion. We show interactive reconstructions of a variety of scenes, reconstructing both fine-grained details and large scale environments. We illustrate how all parts of our pipeline from depth map pre-processing, camera pose estimation, depth map fusion, and surface rendering are performed at real-time rates on commodity graphics hardware. We conclude with a comparison to current state-of-the-art online systems, illustrating improved performance and reconstruction quality.


### Challenges

1. Handling concurrent updates to a shared 3D model poses challenges to avoid synchronization overhead and race conditions.

2. It is unclear how one should fuse input images from two different input streams.

3. Carrying out computation on multiple input streams poses scalability issue to the algorithm.


## Resources 

+ **Data set**: RGB-D Scanning data from [4]
+ **Machine**: Windows system with NVidia video cards (because the starter code requires DirectX).
+ **Code**: We will be working based on Voxel Hashing's source code [4]. 


## Goals & Deliverable

The goal of our project is to deliver an improved Voxel Hashing system that can handle multiple users 3D reconstruction in real time.



## References

[[1] Muja, Marius, and David G. Lowe. "Scalable nearest neighbor algorithms for high dimensional data." IEEE Transactions on Pattern Analysis and Machine Intelligence 36.11 (2014): 2227-2240.](http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6809191)


[[2] Muja, Marius, and David G. Lowe. "Fast Approximate Nearest Neighbors with Automatic Algorithm Configuration." VISAPP (1) 2.331-340 (2009): 2.](https://lear.inrialpes.fr/~douze/enseignement/2014-2015/presentation_papers/muja_flann.pdf)


[[3] Dai, Angela, Matthias Nießner, Michael Zollhöfer, Shahram Izadi, and Christian Theobalt. "BundleFusion: Real-time Globally Consistent 3D Reconstruction using On-the-fly Surface Re-integration." arXiv preprint arXiv:1604.01093 (2016).](http://graphics.stanford.edu/projects/bundlefusion/)

[[4] Nießner, M., Zollhöfer, M., Izadi, S., & Stamminger, M. (2013). Real-time 3D reconstruction at scale using voxel hashing. ACM Transactions on Graphics (TOG), 32(6), 169.](http://www.graphics.stanford.edu/~niessner/niessner2013hashing.html)
