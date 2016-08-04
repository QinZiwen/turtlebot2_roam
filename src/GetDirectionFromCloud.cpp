#include "GetDirectionFromCloud.h"

std::atomic<bool> threadExit(false);

GetDirectionFromCloud::GetDirectionFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud)
{
	std::cout << ">>> inputCloud size = " << inputCloud->points.size() << std::endl;
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_x = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PassThrough<pcl::PointXYZ> pass_z;
	pass_z.setInputCloud (inputCloud);
	pass_z.setFilterFieldName ("z");
	pass_z.setFilterLimits (-0.4, 0.3);
	pass_z.filter (*cloud_filtered_z);

	//std::cout << ">>> cloud_filtered_z size = " << cloud_filtered_z->points.size() << std::endl;

	pcl::PassThrough<pcl::PointXYZ> pass_x;
	pass_x.setInputCloud (cloud_filtered_z);
	pass_x.setFilterFieldName ("x");
	pass_x.setFilterLimits (0.5, 8);
	pass_x.filter (*cloud_filtered_x);

	pcl::PassThrough<pcl::PointXYZ> pass_y;
	pass_y.setInputCloud (cloud_filtered_x);
	pass_y.setFilterFieldName ("y");
	pass_y.setFilterLimits (-0.3, 0.3);
	pass_y.filter (*cloud_filtered);

	std::cout << ">>> cloud_filtered size = " << cloud_filtered->points.size() << std::endl;

	if(cloud_filtered->points.size() > 0)
	{
		//std::cout << ">>> Compute value of minPt and maxPt" << std::endl;
		pcl::getMinMax3D(*cloud_filtered, minPt, maxPt);       //<<< get min and max

	    std::cout << ">>> minPt[x,y,z] = " << minPt.x << ", " << minPt.y << ", " << minPt.z << ";" << std::endl;
	    std::cout << ">>> maxPt[x,y,z] = " << maxPt.x << ", " << maxPt.y << ", " << maxPt.z << ";" << std::endl;
	}
	else
	{
		std::cout << ">>> minPt and maxPt will be set to zero !" << std::endl;

		minPt.x = 0.0;
		minPt.y = 0.0;
		minPt.z = 0.0;

		maxPt.x = 0.0;
		maxPt.y = 0.0;
		maxPt.z = 0.0;
	}
}

GetDirectionFromCloud::~GetDirectionFromCloud()
{}

float GetDirectionFromCloud::getMoveDirection()
{
	float direction = 0.0f;
	//direction = atan(3/2.0);
	if(maxPt.x != 0 && maxPt.y != 0)
	{
		direction = atan(maxPt.y / maxPt.x);
		float dir = direction * 3.14159 / 180;
		direction = dir > 0.3 ? dir : 0.3;
	}
	else if(maxPt.x != 0 && maxPt.y == 0)
	{
		direction = 0.3;
	}
	else if(maxPt.x == 0 && maxPt.y != 0)
	{
		int orientation = maxPt.y > 0 ? 1 : -1;
		direction = orientation * 1.57;
	}
	else
	{
		direction = 0;
	}

	return direction;
}

float GetDirectionFromCloud::getMoveDistance()
{
	float distance = 0.0f;
	double sum = 0.0;
	if(minPt.x > 0)
	{
		distance = minPt.x - 0.5;

		if(distance < 0)
			distance = 0;
	}
	else
	{
		distance = 0.3;
	}

	return (-1 * distance);
}