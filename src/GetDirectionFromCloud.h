#ifndef GETDIRECTIONFROMCOUD_H
#define GETDIRECTIONFROMCOUD_H

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/common.h>
#include <math.h>
#include <iostream>

class GetDirectionFromCloud
{
private:
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;

	pcl::PointXYZ minPt;
	pcl::PointXYZ maxPt;

public:
	GetDirectionFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud);
	~GetDirectionFromCloud();

	float getMoveDirection();
	float getMoveDistance();
};

#endif