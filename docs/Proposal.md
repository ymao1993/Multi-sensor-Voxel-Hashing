## Team

Yu Mao (yumao@cmu.edu) | Ziqiang Feng (zf@andrew.cmu.edu)

## Summary

We will develop a high performance framework for Image Retrieval via Adaptive Approximate Nearest Neighbor(ANN) search. The framework features optimized ANN search in Hamming space and algorithm auto-selection. The framework will be GPU-accelerated and also capable of scaling out to a cluster. 

If we have additional time, we will build an example CBIR system based on our framework by deploying a imaging hashing stage with the deep learning approach as proposed by [4].

## Background & Challenges

For many computer vision problems, the most computationally expensive part is finding nearest neighbor matches in a high-dimensional space. A disappointing fact is that for high-dimensional space, there are no algorithms for nearest neighbor search more efficient than simple linear scan, which is too costly for applications dealing with a large amount of data. Therefore, many algorithms are developed to perform approximate nearest neighbor search where non-optimal neighbors are sometimes returned. There have been many algorithms proposed and often we want to ask the question: "What is the fastest ANN algorithm for my data and what are the optimal parameters to use". [3] proposed an algorithm for automatic ANN algorithm selection and configuration, along with an open library [FLANN](http://www.cs.ubc.ca/research/flann/).

Recent years, there has been growing interests in mapping image data onto compact binary codes for fast near neighbor search. Because in this way, fast image search can be carried out via binary pattern matching or Hamming distance measurement. However, it can still be too slow to use linear search in the case of large datasets. In the FLANN library, a sequential version of **LSH** and  **Hierarchical Clustering** is implemented. These are the state-of-art algorithms for fast binary feature matching. 

In FLANN, a simple scalable solution is provided for general ANN search algorithms by simply dividing the data onto nodes, perform ANN search independently and sum the results from each node in the end. 

Because we are going to implement a library specifically for binary features matching, there are several aspects we will be looking at:

+ How to effectively **scale up** and **scale out** the candidate binary feature matching algorithms? In this project, we will optimize and parallelize **LSH** and **Hierarchical Clustering** on both CPU and GPU. We will also design specific strategies to scale them out on a cluster of nodes. 

+ Our library will only be focusing on ANN search within Hamming space. With this knowledge, how can we improve the algorithm auto-selection part in [FLANN](http://www.cs.ubc.ca/research/flann/).

+ The algorithm auto-selection strategy proposed in [3] requires evaluating at least on one tenth of the dataset. How to effectively parallelize and scaling out the auto-selection algorithm on a cluster of nodes?

## Resources 

+ **Data set**: For optimization and parallelization on single machines we will be working on small dataset like CIFAR-10 and Caltech256. For scaling out the framework, we will be working on large dataset like ImageNet.
+ **Machine**: a cluster of GPU nodes
+ **Code**: We will be referencing [FLANN](http://www.cs.ubc.ca/research/flann/) to gain a better understanding of [3]. We will probably reuse/integrate some FLANN code into our framework.

## Goals & Deliverables

The goal of our project is to build a framework which beats the FLANN library in terms of performance with binary features. We will report the performance of our framework compared to FLANN library on the same dataset.

If we have additional time, we will build an example CBIR system based on our framework by deploying an imaging hashing stage with the deep learning approach proposed by [4]. Then we can demonstrate the our CBIR system interactively.

## Schedule

A tentative schedule is provided below.

+ week 1: Read [1][2][3].

+ week 2: Set up FLANN and get familiar with the code.

+ week 3~4: Implement and optimize **LSH** and **Hierarchical Clustering** on both CPUs and GPUs.

+ week 5: Design and implement a better strategy for automatic algorithm selection specifically for binary features.

+ week 6: Scale out LSH, Hierarchical Clustering and automatic algorithm selection onto a cluster.

+ Optional: Build an example CBIR system based on our framework by deploying a imaging hashing stage with the deep learning approach proposed by [4]


## Reference

[[1] Muja, Marius, and David G. Lowe. "Scalable nearest neighbor algorithms for high dimensional data." IEEE Transactions on Pattern Analysis and Machine Intelligence 36.11 (2014): 2227-2240.](http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6809191)

[[2] Muja, Marius, and David G. Lowe. "Fast matching of binary features." Computer and Robot Vision (CRV), 2012 Ninth Conference on. IEEE, 2012.](http://www.cs.ubc.ca/research/flann/uploads/FLANN/binary_matching_crv2012.pdf)

[[3] Muja, Marius, and David G. Lowe. "Fast Approximate Nearest Neighbors with Automatic Algorithm Configuration." VISAPP (1) 2.331-340 (2009): 2.](https://lear.inrialpes.fr/~douze/enseignement/2014-2015/presentation_papers/muja_flann.pdf)

[[4] Lin, Kevin, et al. "Deep learning of binary hash codes for fast image retrieval." Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition Workshops. 2015.](http://www.cv-foundation.org/openaccess/content_cvpr_workshops_2015/W03/html/Lin_Deep_Learning_of_2015_CVPR_paper.html)
