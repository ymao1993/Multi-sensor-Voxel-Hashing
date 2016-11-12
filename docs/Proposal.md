# Supporting Multi-User Concurrent Model Update in BundleFusion


## Team

Yu Mao (yumao@cmu.edu)

Ziqiang Feng (zf@andrew.cmu.edu)


## Summary

We will improve the BundleFusion system [3] in the following two aspects:

1. Accelerating SIFT feature correspondence search with fast adaptive approximate nearest neighbor (ANN) search based on [2].

2. Implementing functionality to support multiple users contributing to the 3D reconstruction.


If time permits, we will further investigate global optimizations in the presence of multiple user input streams.

For purpose of learning, we will rewrite the two modules from scratch.


## Background & Challenges

Real-time, high-quality, 3D scanning of large-scale scenes is key to mixed reality and robotic applications. However, scalability brings challenges of drift in pose estimation, introducing significant errors in the accumulated model. Approaches often require hours of offline processing to globally correct model errors. Recent online methods demonstrate compelling results, but suffer from: (1) needing minutes to perform online correction preventing true real-time use; (2) brittle frame-to-frame (or frame-to-model) pose estimation resulting in many tracking failures; or (3) supporting only unstructured point-based representations, which limit scan quality and applicability. We systematically address these issues with a novel, real-time, end-to-end reconstruction framework. At its core is a robust pose estimation strategy, optimizing per frame for a global set of camera poses by considering the complete history of RGB-D input with an efficient hierarchical approach. We remove the heavy reliance on temporal tracking, and continually localize to the globally optimized frames instead. We contribute a parallelizable optimization framework, which employs correspondences based on sparse features and dense geometric and photometric matching. Our approach estimates globally optimized (i.e., bundle adjusted poses) in real-time, supports robust tracking with recovery from gross tracking failures (i.e., relocalization), and re-estimates the 3D model in real-time to ensure global consistency; all within a single framework. We outperform state-of-the-art online systems with quality on par to offline methods, but with unprecedented speed and scan completeness. Our framework leads to as-simple-as-possible scanning, enabling ease of use and high-quality results.


Challenges:

1. SIFT features of a new frame need to be matched with all previously seen frames, even in the presence of multiple user input streams.

2. Efficiently handling concurrent update to a global 3D model while reducing synchronization overhead.

3. Accelerating relocalization if there are multiple users.


## Resources 

+ **Data set**: RGB-D Scanning data from [3]
+ **Machine**: a cluster of GPU nodes
+ **Code**: We will be working based on BundleFusion's source code [3]. 


## Goals & Deliverable

The goal of our project is to deliver an improved BundleFusion system that can handle multiple users 3D reconstruction in real time.



## References

[[1] Muja, Marius, and David G. Lowe. "Scalable nearest neighbor algorithms for high dimensional data." IEEE Transactions on Pattern Analysis and Machine Intelligence 36.11 (2014): 2227-2240.](http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6809191)


[[2] Muja, Marius, and David G. Lowe. "Fast Approximate Nearest Neighbors with Automatic Algorithm Configuration." VISAPP (1) 2.331-340 (2009): 2.](https://lear.inrialpes.fr/~douze/enseignement/2014-2015/presentation_papers/muja_flann.pdf)


[[3] Dai, Angela, Matthias Nießner, Michael Zollhöfer, Shahram Izadi, and Christian Theobalt. "BundleFusion: Real-time Globally Consistent 3D Reconstruction using On-the-fly Surface Re-integration." arXiv preprint arXiv:1604.01093 (2016).](http://graphics.stanford.edu/projects/bundlefusion/)
