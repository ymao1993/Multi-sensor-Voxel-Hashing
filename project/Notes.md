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

+ StreamingOutParts contols the number of frames required to sweep through the entrire hash to operate streaming out, which might be something interesting to look into.

+ We already have a helper thread used in streaming responsible for
  consuming the intermediate buffer written by the GPU and constructing the intermediate buffer that is to be sent to GPU. This is to utlize CPU-GPU concurrency. If the thread's processing can be fully hidden by the GPU processing, maybe we can hide more CPU computation by creating threads to do other work.

+ We should definitely use CUDA stream to fully utilize the engines on the GPU.


Err: ChunkToGlobalHashPass2Kernel


### Resources

+ **CUDA**: The main funtionalities including ICP estimation and Voxel Hashing are implemented with CUDA. Some useful CUDA tutorials can be found at [CUDA C Programming Guide](https://docs.nvidia.com/cuda/cuda-c-programming-guide/index.html), [CUDA C/C++ Basics](http://www.nvidia.com/docs/io/116711/sc11-cuda-c-basics.pdf) and [CUDA C Best Practice Guide](https://docs.nvidia.com/cuda/cuda-c-best-practices-guide/index.html).

+ **Nsight Visual Studio Edition** [User Guide](http://docs.nvidia.com/nsight-visual-studio-edition/5.2/Nsight_Visual_Studio_Edition_User_Guide.htm)

+ **DXUT**: DXUT is a "GLUT"-like framework for Direct3D 11.x Windows desktop applications, primarily for samples, demos, and prototypes. It is used in _DepthSensing.cpp_ to set up some application callbacks. A brief Introduction and sample code with DXUT can be found [here](https://code.msdn.microsoft.com/DXUT-Tutorial-Win32-Sample-fe15e440).

+ Timing on Windows: A example usage of [Query Performance Counter](https://msdn.microsoft.com/en-us/library/windows/desktop/dn553408(v=vs.85).aspx) can be found [here](http://stackoverflow.com/questions/1739259/how-to-use-queryperformancecounter).

+ **Synchronization Primitives in Win API** including Event, Mutex, etc is introduced [here](https://msdn.microsoft.com/en-us/library/windows/desktop/ms681924(v=vs.85).aspx). Multi-threading is introduced [here](https://msdn.microsoft.com/en-us/library/windows/desktop/ms684841(v=vs.85).aspx).

	+ **Event** An event object is a synchronization object whose state can be explicitly set to signaled by use of the SetEvent function. Depending on whether the event is reset(unsignal) manually or not, we have Manual-reset event and Auto-reset event. In some sense, a signaled event indicates something has happened. For example, in overlapped input and output, the system sets a specified event object to the signaled state when the overlapped operation has been completed. A single thread can specify different event objects in several simultaneous overlapped operations, then use one of the multiple-object wait functions to wait for the state of any one of the event objects to be signaled. Below is an example of using event to synchronize access of producer and consumer to an intermediate buffer.
	
		```
		
		HANDLE event_producer = CreateEvent(NULL, FALSE, TRUE, NULL);
		HANDLE event_consumer = CreateEvent(NULL, FALSE, FALSE, NULL);
		
		void thread_producer()
		{
			WaitForSingleObject(event_producer);
			
			// write to intermediate buffer
			
			SetEvent(event_consumer);
		}
		
		void thread_consumer()
		{
			WiatForSingleObject(event_consumer);
			
			// read from intermediate buffer
			
			SetEvent(event_producer);
		}
		
		CloseHandle(event_producer);
		CloseHandle(event_consumer);
		
		```
	
	+ **Mutex** A mutex object is a synchronization object whose state is set to signaled when it is not owned by any thread, and nonsignaled when it is owned. Only one thread at a time can own a mutex object, whose name comes from the fact that it is useful in coordinating mutually exclusive access to a shared resource.
	
		```
		// create a mutex not own by any threads
		HANDLE mutex = CreateMutex(NULL, FALSE, NULL);
		
		void thread_work()
		{
			// wait for the mutex to be released
			WaitForSingleObject(mutex, INFINITE);
			
			// critical_section...
			
			// set event to signaled state
			ReleaseMutex(mutex);	
		}
		
		CloseHandle(mutex);
		
		```
	
	

