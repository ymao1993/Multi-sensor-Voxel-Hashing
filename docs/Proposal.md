# High Performance Adaptive Approximate Nearest Neighbor Search with Image Binary Hash Codes

## Team

Yu Mao (yumao@cmu.edu)

Ziqiang Feng (zf@andrew.cmu.edu)

## Summary

We will develop a high performance framework for Adaptive Approximate Nearest Neighbor(ANN) search based on [3]. Our project features: 

(1) GPU-accelerated and distributed ANN search in Hamming space; and

(2) data-adaptive automatic algorithm selection and parameter configuration.

If time permits, we will also build an example context-based image retrieval system based on our framework and image binary codes extracted with deep-learned hash functions [4].


## Background & Challenges

For many computer vision problems, the most computationally expensive part is finding nearest neighbor matches in a high-dimensional space. A disappointing fact is that for high-dimensional space, there are no algorithms for nearest neighbor search more efficient than simple linear scan, which is too costly for applications dealing with a large amount of data. Therefore, many algorithms are developed to perform *approximate* nearest neighbor search where non-optimal neighbors are sometimes returned. Despite many proposed algorithms, one often asked question is *"What is the fastest ANN algorithm for my data and what are the optimal parameters to use"*. The [FLANN](http://www.cs.ubc.ca/research/flann/) library [3] is an attempt to answer this question by proposing a method for automatic ANN algorithm selection and parameter configuration based on the given dataset.

In recent years, there has been growing interests in mapping image data onto compact binary hash codes for fast nearest neighbor search. The state-of-the-art of image hash code is to *learn* "good" hash functions from the dataset [4]. With such hash codes, fast image search can be carried out by matching binary codes or calculating Hamming distance. Nonetheless, it can still be too slow to use linear search in the case of large datasets. So it is desirable to combine deep-learned binary hash codes with efficient approximate nearest neighbor search.
 
In the FLANN library, a sequential version of **LSH** and  **Hierarchical Clustering** is implemented as they are found to be the state-of-art algorithms for fast binary feature matching. A naive parallel implementation is also provided for the algorithms by simply dividing the data onto nodes, performing ANN search independently and merging the results from each node in the end. 

As we are targeting ANN with binary codes running efficiently on modern hardware, we will be specifically looking into the following aspects:

+ How to effectively **scale up** and **scale out** the candidate binary feature matching algorithms? In this project, we will optimize and parallelize **LSH** and **Hierarchical Clustering** on both CPU and GPU. We will also design specific strategies to scale them out on a cluster of nodes. 

+ Our library will only be focusing on ANN search within Hamming space of binary codes. With this knowledge, how can we improve the automatic algorithm selection and parameter configuration in [FLANN](http://www.cs.ubc.ca/research/flann/)?

+ The auto-selection strategy proposed in [3] requires evaluating at least on one tenth of the dataset. How to effectively parallelize and scaling out the auto-selection algorithm on a cluster of nodes?

## Resources 

+ **Data set**: For optimization and parallelization on a single machine we will be working on small datasets like CIFAR-10 and Caltech256. For scaling out the framework, we will be working on large datasets like ImageNet.
+ **Machine**: a cluster of GPU nodes
+ **Code**: We will be referencing [FLANN](http://www.cs.ubc.ca/research/flann/) to gain a better understanding of [3]. We will probably reuse/integrate some FLANN code into our framework.

## Goals & Deliverable

The goal of our project is to build a framework which outperforms the FLANN library with binary image features. We will report the performance of our framework compared to FLANN library on the same dataset.

If we have additional time, we will build an example CBIR system based on our framework by deploying an imaging hashing stage with deep-learned hash functions as proposed by [4]. Then we can demonstrate the our CBIR system interactively.

## Schedule

A tentative schedule is provided below.

+ week 1: Read [1][2][3].

+ week 2: Set up FLANN and get familiar with the code.

+ week 3~4: Implement and optimize **LSH** and **Hierarchical Clustering** on both CPUs and GPUs.

+ week 5: Design and implement a better strategy for automatic algorithm selection specifically for binary features.

+ week 6: Scale out LSH, Hierarchical Clustering and automatic algorithm selection onto a cluster.

+ Optional: Build an example CBIR system based on our framework by deploying an image hashing stage with the deep learned hash functions as proposed by [4]


## Reference

[[1] Muja, Marius, and David G. Lowe. "Scalable nearest neighbor algorithms for high dimensional data." IEEE Transactions on Pattern Analysis and Machine Intelligence 36.11 (2014): 2227-2240.](http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6809191)

[[2] Muja, Marius, and David G. Lowe. "Fast matching of binary features." Computer and Robot Vision (CRV), 2012 Ninth Conference on. IEEE, 2012.](http://www.cs.ubc.ca/research/flann/uploads/FLANN/binary_matching_crv2012.pdf)

[[3] Muja, Marius, and David G. Lowe. "Fast Approximate Nearest Neighbors with Automatic Algorithm Configuration." VISAPP (1) 2.331-340 (2009): 2.](https://lear.inrialpes.fr/~douze/enseignement/2014-2015/presentation_papers/muja_flann.pdf)

[[4] Lin, Kevin, et al. "Deep learning of binary hash codes for fast image retrieval." Proceedings of the IEEE Conference on Computer Vision and Pattern Recognition Workshops. 2015.](http://www.cv-foundation.org/openaccess/content_cvpr_workshops_2015/W03/html/Lin_Deep_Learning_of_2015_CVPR_paper.html)
