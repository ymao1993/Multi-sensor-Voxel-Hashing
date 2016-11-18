# Notes on Voxel Hashing Source Code

Yu Mao (yumao@cmu.edu)

### Background

The real-time 3D reconstruction in this paper is implemented by first estimating the ego-motion of the sensor, then integrate(or, fuse) the transformed depth image into a mesh model.

Estimating the ego-motion is implemented with ICP(Iterated Closest Point). A brief introduction can be found [here](http://www.onerussian.com/classes/cis780/icp-slides.pdf). It is a simple but effective algorithm.

The second stage is for performing volumetrix integration is implemented with Truncated Signed Distance Field(TSDF). A quick introduction can be found [here](http://www.cs.unc.edu/~marc/tutorial/node129.html).

The contribution of the paper is mainly about implementing a efficient hashing strategy to accelerate the second stage.


### Build the Project

To build the project, I am using the following SDKs and softwares. It is important to note that the version of Visual Studio matters if you don't want to change any configurations of the VS project.

+ DirectX SDK June 2010
+ Kinext SDK 1.8
+ NVIDIA CUDA 7.5
+ Visual Studio 2013 Professional

The project **DepthSensing** is accelerated with DirectX, while **DepthSensingCUDA** is accelerated with CUDA. For the purpose of this course, We will dig into **DepthSensingCUDA**.

### CUDA Documentation

[CUDA C Programming Guide](https://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html)

[CUDA C Best Practice Guide](https://docs.nvidia.com/cuda/cuda-c-best-practices-guide/index.html)

### Project Structure

