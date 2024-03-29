# Transferring facade labels between point clouds with semantic octrees while considering change detection
The repository contains implementation for automatic label transfer as presented in the paper: "Transferring facade labels between point clouds with semantic octrees while considering change detection" by Sophia Schwarz, Tanja Pilz, Olaf Wysocki, Ludwig Hoegner and Uwe Stilla

## Structure of the repository
- Paper
- Method description
- User Guide


## Paper: presented at the 3D GeoInfo Conference in Munich, 12.-14.09.2023
Point clouds and high-resolution 3D data have become increasingly important in various fields, including surveying, construction, and virtual reality. 
However, simply having this data is not enough; to extract useful information, semantic labeling is crucial. 
In this context, we propose a method to transfer annotations from a labeled to an unlabeled point cloud using an octree structure. 
The structure also analyses changes between the point clouds. 
Our experiments confirm that our method effectively transfers annotations while addressing changes. 
The primary contribution of this project is the development of the method for automatic label transfer between two different point clouds that represent the same real-world object. 
The proposed method can be of great importance for data-driven deep learning algorithms as it can also allow circumventing stochastic transfer learning by deterministic label transfer between datasets depicting the same objects.

## Method description
The developed method for the semantic label transfer between two urban point clouds consists of using an octree-based data structure, that considers semantic information as a leaf-criterium. Preceding this is a plane-based coregistration. The implementation contains an outlier removal as an additional pre-processing step. The changes between the two point clouds are identified by utilizing the octree to describe the occupancy of the octree leaves. The method has been developed with the used case of building point clouds in mind. The following figure shows the workfow. The method of the GICP and semantic octree are briefly described below.For the in-depth understanding of the method, do not hesitate to check out the paper: _add link_

![Workflow](Figures/Workflow.png)




### Generalized ICP
Generalized Iterative Closest Point (GICP) is a variant of the Iterative Closest Point (ICP) algorithm used for point cloud coregistration. The ICP algorithm iteratively aligns two point clouds by minimizing the distance between corresponding point pairs. The algorithm iterates until the transformation converges to a sufficiently accurate solution or a maximum number of iterations is reached [Besl & McKay, 1992]. The GICP works like the ICP but expands it by taking into account the local geometry of the point clouds. Instead of relying solely on the distance between points, the GICP minimizes the distance between planes, that are calculated in advance. This results in more accurate alignment and is particularly useful when dealing with point clouds that have significant noise or partial overlap [Segal et al., 2009]. 

### Semantic Octree
Octrees are a hierarchical data structure used in computer graphics and computer geometry to efficiently represent and process complex spatial data [Hornung et al., 2013]. An octree is subdividing a 3D space into smaller and smaller cubes, with each cube representing a particular region of space. This subdivision continues until the cubes reach a desired size or until a certain level of subdivision is reached. The criterium that is introduced to decide between node and leaf is called leaf criterium [Gehrung et al. 2019].

In case of our semantic octree, we define three leaf-criteria: _empty leaf_, _one label_ and _max depth_.
A leaf is considered as _empty leaf_ if no points are contained in its volume.
The criterion _one label_ is fulfilled if all points in a node have the same semantic class.
_Max depth_ is controlled by the largest permitted side length of the smallest leaf container, _maxLat_. The formula below specifies the relationship between the side length of the point cloud,i.e building, the side length of the leaf and the depth of the octree.

![Formula maxDepth](https://github.com/SchwarzSophia/Transferring_urban_labels_between_pointclouds/blob/main/Figures/Formula%20maxdepth.PNG)

In the figure below the  utilized semantic octree is illustrated.

![Illustration_Octree](https://github.com/SchwarzSophia/Transferring_urban_labels_between_pointclouds/blob/main/Figures/Octree_Illustration.PNG)

If the points get sorted into an empty leaf, a new label that represents the class “addition” is assigned. This allows for a change detection in the case of added building parts. In order to detect a change the other way around, i.e. removal of building parts, it is necessary to record the queried leaves.


## User Guide

### Getting started

This Project is implemented in Python 3.8.16 

Required libaries:
- [numpy](https://pypi.org/project/numpy/)
- [open3d](http://www.open3d.org/)
- [laspy](https://laspy.readthedocs.io/en/latest/)
- [sklearn](https://scikit-learn.org/stable/)

  

### Data requirements

The implementation requires two point clouds in the .las format.
The input data must already be pre-processed to a certain degree. The point clouds should contain approximately the same building sections. It is also necessary to define the appropriate coordinate system and, if nessecary, apply a coarse coregistration.

### Run code
To run the code, the paths to source point cloud(line 253), target point cloud(line 254) and output point cloud(line 316) have to be defined. 




## References
- Besl PJ, McKay ND (1992). A method for registration of 3-D Shapes. In: IEEE Transactions on pattern analysis and machine intelligence, 14 (2): 239 – 256.
- Fernandez-Fernandez JA, Westhofen L, Löschner F, Jeske SR, Longva A, Bender J (2022). Fast octree neighborhood search for SPH simulations. ACM Trans. Graph., 41(6): 242.
- Gehrung J, Hebel M, Arens M, Stilla U (2019). A fast voxel-based indicator for change detection using low resolution octrees. In: ISPRS Annals of the Photogrammetry, Remote Sensing and Spatial Information Sciences.  - Enschede, VI-2/WS: 357 – 364.
- Gehrung J, Hebel M, Arens M, Stilla U (2018). A voxel-based metadata structure for change detection in point clouds of large-scale urban areas. In: ISPRS Annals of the Photogrammetry, Remote Sensing and Spatial Information Sciences. Riva del Garda, IV-2: 97 – 104.
- Hornung A, Wurm KM, Bennewitz M, Stachniss C, Burgard W (2013). OctoMap: an efficient probabilistic 3D mapping framework based on octrees. Auton. Robot. 34: 189 – 206.
- Open3D (2021). ICP registration from Open3D Documentation, Internetblog. http://www.open3d.org/docs/release/tutorial/pipelines/icp_registration.html (accessed 24.03.2023).
- Segal A, Haehnel D, Thrun S (2009). Generalized-ICP. In: Robotics: Science and Systems, V: 021.
- Shmueli B (2019). Multi-Class metrics made simple, part 3: Kappa score. Towardsdatascience.com, Internetblog. https://towardsdatascience.com/multi-class-metrics-made-simple-the-kappa-score-aka-cohens-kappa-coefficient-bdea137af09c (accessed 25.03.2023).
- Vo AV, Truong-Hong L, Laefer DF, Bertolotto M (2015). Octree-based region growing for point cloud segmentation. In: ISPRS Journal of Photogrammetry and Remote Sensing, 104: 88 – 100.


