#ifndef _GETDIRECTIONFROMCLOUD_V2_H
#define _GETDIRECTIONFROMCLOUD_V2_H

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <math.h>

class GetDirectionFromCloud_v2
{

public:
	GetDirectionFromCloud_v2(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud);
	~GetDirectionFromCloud_v2();

	float getMoveDirection();
	float getMoveDistance();

private:
	//judge public part of two set
	bool haveNoPublicPart(float min_1, float max_1, float min_2, float max_2);
	//compute two min point from pointSet no public part
	pcl::PointCloud<pcl::PointXYZ>::Ptr computeNearObj_2( pcl::PointCloud<pcl::PointXYZ>::Ptr pointSet );
	// Create the filtering object: downsample the dataset using a leaf size of 1cm
	pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud );
	//remove 70% planar
	pcl::PointCloud<pcl::PointXYZ>::Ptr removePlanar( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered );

	pcl::PointCloud<pcl::PointXYZ>::Ptr clusterExtraction( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered );

private:
	float _diretion;
	float _distance;
};

#endif