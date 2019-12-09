Cluster-based Penalty Scaling for Robust Pose Graph Optimization
=======

### To Use:

```
git clone https://github.com/amberwood31/clustering.git clustering
cd clustering
mkdir build
cd build
cmake ..
make -j4
cd ../bin
./slam_incre_clustering -i <clustering_input_pose_graph_file>
```

Please note the clustering_input_pose_graph_file need to be sorted so that vertices are accessed in incremental manner. The sorting script **sort.py** is provided.

The executable will generate a file named **clustering_results.txt** with the following template
>CLUSTER \<from\> \<to\> \<quality\> \
>CLUSTER \<from\> \<to\> \<quality\> \
> ... \
>CLUSTER \<from\> \<to\> \<quality\> \
>\<an empty line indicates the start of a new cluster\> \
>CLUSTER \<from\> \<to\> \<quality\> \
>CLUSTER \<from\> \<to\> \<quality\> \
> ... \
>CLUSTER \<from\> \<to\> \<quality\> \
>\<an empty line indicates the start of a new cluster\> \
>CLUSTER_R \<from\> \<to\>  \
> ...

Label "CLUSTER" refers to intra-consistent clusters and "CLUSTER_R" indicates all the edges that have been rejected after the clustering process.

 

