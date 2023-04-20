import open3d as o3d
import copy
import numpy as np
import laspy
import time
from sklearn.metrics import confusion_matrix
import csv
import math

#global variables: counter for different types of octree leaves 
empty_leaves = 0
max_depth_leaves = 0
one_class_leaves = 0

class Octree:
    def __init__(self, points, labels, depth=0, max_depth=1):
        self.points = points
        self.labels = labels
        self.depth = depth
        self.max_depth = max_depth
        self.children = []
        self.removed = True
        self.leaf = True
        self.label = None
        self.bounds = self.bounding_box()
        self.split()

    def bounding_box(self):
        if len(self.points) == 0:
            return None
        else:
            # Compute the bounding box of the points in the node
            bbox = np.zeros((2, 3))
            bbox[0] = np.min(self.points, axis=0)
            bbox[1] = np.max(self.points, axis=0)
            return bbox

    def one_label(self):
        #leave criterium one label 
        bol = False
        label=self.labels
        num = np.unique(label)
        if len(num) == 1:
            bol = True
        return bol

    def split(self):
        #check if node is empty
        if len(self.points) == 0:
            self.leaf = True
            self.label = 18
            global empty_leaves 
            empty_leaves = empty_leaves + 1
            self.removed = False
            return

        #check if node has only one label
        if self.one_label() == True:
            self.label = self.labels[0] if len(self.points) > 0 else None
            self.leaf = True
            global one_class_leaves 
            one_class_leaves = one_class_leaves + 1
            return

        #check if depth is maxdepth (if so assing most frequent label)
        if self.depth >= self.max_depth:
            l = np.bincount(self.labels).argmax()
            self.label = l
            self.leaf = True
            global max_depth_leaves 
            max_depth_leaves =max_depth_leaves + 1
            return
        
        self.leaf = False

        # determine bounding box of the points
        xmin, ymin, zmin = self.bounds[0]
        xmax, ymax, zmax = self.bounds[1]
        xmid, ymid, zmid = (xmin+xmax)/2, (ymin+ymax)/2, (zmin+zmax)/2

        # split the points into 8 children based on their positions
        for i in range(8):

            if i in [1, 3, 5, 7]:
                x_condition = (self.points[:,0] < xmid)
            else:
                x_condition = (self.points[:,0] >= xmid)

            if i in [2, 3, 6, 7]:
                y_condition = (self.points[:,1] < ymid)
            else:
                y_condition = (self.points[:,1] >= ymid)

            if i in [4, 5, 6, 7]:
                z_condition = (self.points[:,2] < zmid)
            else:
                z_condition = (self.points[:,2] >= zmid)
            
            condition = np.logical_and(np.logical_and(x_condition, y_condition), z_condition)
            child_points = self.points[condition]
            child_labels = self.labels[condition]
            self.children.append(Octree(child_points, child_labels, self.depth+1, self.max_depth))
        
            
    def get_label(self, point):
    # inserts unlabled point in the octree (recrusive function)
        if self.leaf:
            self.removed = False
            return self.label
        
        else:
            # determine bounding box of the node
            xmin, ymin, zmin = self.bounds[0]
            xmax, ymax, zmax = self.bounds[1]
            xmid, ymid, zmid = (xmin+xmax)/2, (ymin+ymax)/2, (zmin+zmax)/2

            x_condition = (point[0] < xmid) 
            y_condition = (point[1] < ymid) 
            z_condition = (point[2] < zmid)
            j = x_condition + 2 * y_condition + 4 * z_condition
            
            return self.children[j].get_label(point)


    def insert(self, points):
    # iterate through point array and call get lable to insert in octree
        labels = np.zeros(len(points))
        for i in range(0,len(points)):
            labels[i] = self.get_label(points[i])
        return labels


def write_las(points,labels,name):
    # color values for the different classes according to TumFacade
    class_values = { 
        1: [255,242,202],
        2: [142,169,219],
        3: [198,89,17],
        4: [0,128,128],
        5: [180,130,218],
        6: [0,176,240],
        7: [255,0,0],
        8: [48,84,150],
        9: [255,165,0],
        10:[188,143,143],
        11:[124,252,0],
        12:[198,224,180],
        13:[168,0,0],
        14:[84,130,53],
        15:[191,143,0],
        16:[255,255,0],
        17:[105,105,105],
        18:[253,20,238]
    }
    
    colors = np.zeros_like(points)
    for i in range(0,len(points)):
        colors[i]=class_values[labels[i]]
    
    # create a new header
    header = laspy.header.LasHeader(point_format=2) # LAS point format 2 supports color    
    header.offsets = np.min(points, axis=0)
    header.scales = np.array([0.001, 0.001, 0.001])

    # add point values, classification and colors to the output file
    point_record = laspy.LasData(header)
    point_record.x = points[:, 0]
    point_record.y = points[:, 1]
    point_record.z = points[:, 2]
    point_record.classification = labels
    point_record.red = colors[:,0]
    point_record.green = colors[:,1]
    point_record.blue = colors[:,2]
    point_record.write(name)
    return


def generalized_icp(source, target,max_corr ):
    print("Apply generalized ICP")

    tr = np.eye(4)#inital transformation matrix

    #calculate transformation matrix
    result_icp = o3d.pipelines.registration.registration_generalized_icp(source, target, max_corr,tr,o3d.pipelines.registration.TransformationEstimationForGeneralizedICP())
    
    return result_icp


def preprocess_point_cloud(pcd, voxel_size):
    #voxelizing
    pcd_down = pcd.voxel_down_sample(voxel_size)

    #compute normals 
    radius_normal = voxel_size * 2
    pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    
    return pcd_down

def prepare_dataset(souce_path,target_path,val):
    #load las point clouds
    source_las = laspy.read(souce_path)
    target_las = laspy.read(target_path)

    #extract point values from las point clouds
    source_point_data = source_las.xyz
    target_point_data = target_las.xyz

    #create open3d point clouds and assign point values to open3d point clouds
    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(source_point_data)
    target = o3d.geometry.PointCloud()
    target.points = o3d.utility.Vector3dVector(target_point_data)

    #extract labels from las-point cloud
    classes = np.asarray(source_las.classification)
    labels = np.zeros((len(classes),3))
    labels [:,0] = classes

    #assign labes to red value in open3d point cloud color vector
    source.colors = o3d.utility.Vector3dVector(labels)

    if val:
        #extract labels from las-point cloud
        classes_t = np.asarray(target_las.classification)
        labels_t = np.zeros((len(classes_t),3))
        labels_t [:,0] = classes_t
        #assign labes to red value in open3d point cloud color vector
        target.colors = o3d.utility.Vector3dVector(labels_t)
   
    #outlier removal
    temp, inlier_source = source.remove_statistical_outlier(nb_neighbors=20,std_ratio=1.7)
    source = source.select_by_index(inlier_source)

    temp, inlier_target = target.remove_statistical_outlier(nb_neighbors=20,std_ratio=1.7)
    target = target.select_by_index(inlier_target) 

    source_point_data = np.take(source_point_data, inlier_source, 0)
    target_point_data = np.take(target_point_data, inlier_target, 0)


    #compute voxels with normals for gicp
    voxel_size = 0.1 # 10 cm
    print("Downsample with a voxel size %.3f." % voxel_size)
    source_down = preprocess_point_cloud(source, voxel_size)
    target_down = preprocess_point_cloud(target, voxel_size)

    return source, target, source_down, target_down, source_point_data,target_point_data


if __name__ == "__main__":
    #data loading and downsampling
    tic = time.perf_counter()
    souce_path = "Path_to_labled_pointcloud"
    target_path = "Path_to_unlabled_pointcloud"
    validation_dataset = False #Detemines if the confusion martix should be calculated

    source, target, source_down, target_down, source_las, target_las = prepare_dataset(souce_path,target_path,validation_dataset)
    tock = time.perf_counter()
    t = str(tock - tic)
    print("Preprocessing the data took: " + t +" seconds" )
    
    #compute GICP
    tic = time.perf_counter()
    max_corr = 10 #maximum correspondense distance [m]
    result_icp = generalized_icp(source_down, target_down,max_corr)
    tri=result_icp.transformation
    print(tri)
    print(result_icp)
    
    #apply transformation to full point cloud
    source.transform(tri)
    tock = time.perf_counter()
    t = str(tock - tic)
    print("Transformation done. It took: " + t +" seconds" )

    #calculate the octree depth
    tic = time.perf_counter()
    labeled_points = np.asarray(source.points)
    min_p = np.min(labeled_points, axis=0)
    max_p = np.max(labeled_points, axis=0)
    diff = max_p-min_p
    lat_length = 0.1 #lateral length of octree leaf at maximal depth [m]
    max_dep = math.ceil(math.log(np.max(diff)/lat_length,2))
    colors = np.asarray(source.colors,dtype = 'int')
    labels = colors[:,0]

    #build up octree
    octree = Octree(labeled_points, labels,max_depth=max_dep)
    print ("Number of octree leaves that have one label: " + str(one_class_leaves))
    print ("Number of octree leaves that are empty: " + str(empty_leaves))
    print ("Number of octree leaves that reached maximal depth: " + str(max_depth_leaves))
    print ("Maximal octree depth: " + str( max_dep))
    print ("Lateral length of octree leaf at maximal depth: " + str(np.max(diff)/(2**max_dep)) + " meters")
    tock = time.perf_counter()
    t = str(tock - tic)
    print("Building up the octree took: " + t +" seconds" )

    #insert unlabled points in the octree
    tic = time.perf_counter()
    unlabelled_points = np.array(target.points)
    new_labels = octree.insert(unlabelled_points)
    tock = time.perf_counter()
    t = str(tock - tic)
    print("Inserting the points in the octree took:" + t +" seconds" ) 

    #calculate confusion matrix
    if validation_dataset:
        color = np.asarray(target.colors,dtype = 'int')
        labels_test = color[:,0]
        confusion = confusion_matrix(labels_test, new_labels,labels=[1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18])
        with open("Confusionmatrix.csv", mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerows(confusion)

    #write the newly labled point cloud (with global coordinates)
    write_las(target_las,new_labels,"Labeled_Pointcloud.las")
