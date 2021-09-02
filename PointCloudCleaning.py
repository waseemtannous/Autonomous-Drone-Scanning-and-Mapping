"""
this module is for point cloud cleaning using multiple algorithms
source: http://www.open3d.org/docs/0.9.0/tutorial/Advanced/pointcloud_outlier_removal.html
"""


def getInlierOutlier(pcd, ind):
    inlier_cloud = pcd.select_by_index(ind)
    outlier_cloud = pcd.select_by_index(ind, invert=True)
    return inlier_cloud, outlier_cloud


def voxelDown(pcd, voxel_size=0.02):
    return pcd.voxel_down_sample(voxel_size=voxel_size)


# Every Kth points are selected
def selectEveryKPoints(pcd, K=5):
    return pcd.uniform_down_sample(every_k_points=K)


def removeStatisticalOutlier(pcd, voxel_size=0.02, nb_neighbors=20, std_ratio=2.0):
    voxel_down_pcd = voxelDown(pcd, voxel_size)
    cl, ind = voxel_down_pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
    return getInlierOutlier(voxel_down_pcd, ind)


def removeRadiusOutlier(pcd, voxel_size=0.02, nb_points=16, radius=0.05):
    voxel_down_pcd = voxelDown(pcd, voxel_size)
    cl, ind = voxel_down_pcd.remove_radius_outlier(nb_points=nb_points, radius=radius)
    return getInlierOutlier(voxel_down_pcd, ind)
