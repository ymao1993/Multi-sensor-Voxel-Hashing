# Notes

Yu Mao (yumao@cmu.edu)

### Background

The real-time 3D reconstruction in this paper is implemented by first estimating the ego-motion of the sensor, then integrate(or, fuse) the transformed depth image into a mesh model.

Estimating the ego-motion is implemented with ICP(Iterated Closest Point). A brief introduction can be found [here](http://www.onerussian.com/classes/cis780/icp-slides.pdf). It is a simple but effective algorithm.

The second stage is for performing volumetrix integration is implemented with Truncated Signed Distance Field(TSDF). A quick introduction can be found [here](http://www.cs.unc.edu/~marc/tutorial/node129.html).

The contribution of the paper is mainly about implementing a efficient hashing strategy to accelerate the second stage.


### Build

To build the project, I am using the following SDKs and softwares. It is important to note that the version of Visual Studio matters if you don't want to change any configurations of the VS project.

+ DirectX SDK June 2010
+ Kinext SDK 1.8
+ NVIDIA CUDA 7.5
+ Visual Studio 2013 Professional

The project **DepthSensing** is accelerated with DirectX, while **DepthSensingCUDA** is accelerated with CUDA. For the purpose of this course, We will work based on **DepthSensingCUDA**.

### Configuration

The executable takes two configuration file as its input argument. If it is not specified, it will be using _zParametersDefault.txt_ and _zParametersTrackingDefault.txt_ by default. As the names suggest, the first file specifies the global applicatoin variable values and the second file is specificlly for the tracking part.

The most important configuration is the sensor. The project supports a range of RGBD sensors including: Kinect(Kinect v1), Kinect One(Kinect for Xbox One, or Kinext v2), Structure Sensor, Realsense, etc. To build the project, we need to :

+ Enable the corresponding macros in GlobalAppState.h;

+ Give the correct value to `s_sensorIdx` in the first input configuration file.

A good practice when working on the project is to creat a bunch of scripts like runxxx.bat to specify what configuration files to use. And of course, to correctly run the program, the corrresponding macros should be enabled or disabled.

### Profiling & Analysis

### Convention

+ **Cooridnate System**: the coordinate system in the starter code follows DirectX's left-hand convention.

### Thoughts

+ There is no need to write the ICP output to a binary file and read it in runtime. ICP process can be skipped if the frames are read from binary dump.

+ We need to write a multi-user sensor adaptor (definitely).

+ Maybe we can keep several active regions within one GPU for several users.



### Resources

+ **CUDA**: The main funtionalities including ICP estimation and Voxel Hashing are implemented with CUDA. Some useful CUDA tutorials can be found at [CUDA C Programming Guide](https://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html), [CUDA C/C++ Basics](http://www.nvidia.com/docs/io/116711/sc11-cuda-c-basics.pdf) and [CUDA C Best Practice Guide](https://docs.nvidia.com/cuda/cuda-c-best-practices-guide/index.html).

+ **DXUT**: DXUT is a "GLUT"-like framework for Direct3D 11.x Windows desktop applications, primarily for samples, demos, and prototypes. It is used in _DepthSensing.cpp_ to set up some application callbacks. A brief Introduction and sample code with DXUT can be found [here](https://code.msdn.microsoft.com/DXUT-Tutorial-Win32-Sample-fe15e440).
