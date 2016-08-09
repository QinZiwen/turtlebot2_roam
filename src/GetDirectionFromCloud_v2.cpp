#include "GetDirectionFromCloud_v2.h"

GetDirectionFromCloud_v2::GetDirectionFromCloud_v2(pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = downsampleCloud( inputCloud );
	pcl::PointCloud<pcl::PointXYZ>::Ptr remove_planar = removePlanar( cloud_filtered );

	pcl::PointCloud<pcl::PointXYZ>::Ptr two_object;
	if(remove_planar)
	{
		two_object = clusterExtraction( remove_planar );
	}
	else
	{
		_diretion = 1.57;
		_distance = 0;
		return;
	}

	// std::cout << "two_object->points.size() = " << two_object->points.size() << std::endl;

	if( two_object->points.size() == 0 )
	{
		//_diretion = 0.78;
        _diretion = 1.57;
		_distance = 0;
	}
	else if(two_object->points.size() == 2)
	{
		float minX = two_object->points[1].x;
		float minY = two_object->points[1].y;
		float maxY = two_object->points[1].z;
		float centerY = ( minY + maxY ) / 2;

		if( minX - 0.5 > 0)
		{
			_distance = minX - 0.5;
		}
		else
		{
			_distance = 0;
		}

		if( centerY >= 0 )
		{
			float angle = atan(minY / minX);
            if(angle >= 15)
			    _diretion = angle * 3.14159 / 180;
             else
                _diretion = 0;
		}
		else if( centerY < 0 )
		{
			float angle = atan(maxY / minX);
			_diretion = angle * 3.14159 / 180;
		}
	}
	else
	{
		float ObjectDistance = 0.0f;
		float minX_1 = two_object->points[1].x;
		float minY_1 = two_object->points[1].y;
		float maxY_1 = two_object->points[1].z;
		float minX_2 = two_object->points[3].x;
		float minY_2 = two_object->points[3].y;
		float maxY_2 = two_object->points[3].z;

		// std::cout << " minX_1 = " << minX_1 << ", minY_1 = " << minY_1 << ", maxY_1 = " << maxY_1 << std::endl;
		// std::cout << " minX_2 = " << minX_2 << ", minY_2 = " << minY_2 << ", maxY_2 = " << maxY_2 << std::endl;

		if( maxY_1 < minY_2 )
		{
			if( minY_2 - maxY_1 >= 0.5 )
			{
				float angle = atan( ((minY_2 + maxY_1)/2) / minX_1 );
                if(angle >= 15)
                    _diretion = angle * 3.14159 / 180;
                else
                    _diretion = 0;

				float diff = (minY_2 - maxY_1) / 2;
				_distance = fabs( diff * diff + minX_1 * minX_1 );
			}
			else
			{
				int dir = ( (minY_2 + maxY_1) / 2 ) >= 0 ? -1 : 1; 
				_diretion = dir * 45 * 3.14159 / 180;
				_distance = 0;
			}
		}
		else if( minY_1 > maxY_2)
		{
			if( minY_1 - maxY_2 >= 0.5 )
			{
				float angle = atan( ((minY_1 + maxY_2) / 2 ) / minX_2 );
				if(angle >= 15)
                    _diretion = angle * 3.14159 / 180;
                else
                    _diretion = 0;

				float diff = (minY_1 - maxY_2) / 2;
				_distance = fabs( diff * diff + minX_2 * minX_2 );
			}
			else
			{
				int dir = (minY_1 + maxY_2) / 2 >= 0 ? -1 : 1; 
				_diretion = dir * 45 * 3.14159 / 180;
				_distance = 0;
			}
		}
		else
		{
			_diretion = 1.57;
			_distance = 0;
		}
	}
}

GetDirectionFromCloud_v2::~GetDirectionFromCloud_v2()
{}

float GetDirectionFromCloud_v2::getMoveDirection()
{
	return _diretion;
}

float GetDirectionFromCloud_v2::getMoveDistance()
{
	return _distance;
}

//judge public part of two set
bool GetDirectionFromCloud_v2::haveNoPublicPart(float min_1, float max_1, float min_2, float max_2)
{
    if( max_1 < min_2 || min_1 > max_2 )
        return true;
    else
        return false;
}

/**
 * get two object from cloud to nearest camera and no public part
 * param: pointSet: every part include two section: PointXYZ and PointXYZ( min x, min y, max y)
 * return : point, two setion nearest to camera
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr GetDirectionFromCloud_v2::computeNearObj_2( pcl::PointCloud<pcl::PointXYZ>::Ptr pointSet )
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>);

    if(pointSet->points.size() <= 4)
    {
        return pointSet;
    }

    //init min points
    pcl::PointXYZ min_1, min_2;
    min_1.x = pointSet->points[0].x;
    min_1.y = pointSet->points[0].y;
    min_1.z = pointSet->points[0].z;
    min_2.x = pointSet->points[2].x;
    min_2.y = pointSet->points[2].y;
    min_2.z = pointSet->points[2].z;
    int minIndex_1 = 0, minIndex_2 = 2;

    //find minimum and second minimum
    for (size_t i = 4; i < pointSet->points.size (); i += 2)
    {
        if( pointSet->points[i].x < min_1.x )
        {            
            min_2.x = min_1.x;
            min_2.y = min_1.y;
            min_2.z = min_1.z;
            minIndex_2 = minIndex_1;

            min_1.x = pointSet->points[i].x;
            min_1.y = pointSet->points[i].y;
            min_1.z = pointSet->points[i].z;
            minIndex_1 = i;

            continue;
        }

        if( pointSet->points[i].x < min_2.x )
        {
            min_2.x = pointSet->points[i].x;
            min_2.y = pointSet->points[i].y;
            min_2.z = pointSet->points[i].z;
            minIndex_2 = i;
        }
    }

    //check up second minimum
    if( !haveNoPublicPart(pointSet->points[minIndex_1 + 1].y, pointSet->points[minIndex_1 + 1].z,
            pointSet->points[minIndex_2 + 1].y, pointSet->points[minIndex_2 + 1].z) )
    {
        int minIndex_2_temp = minIndex_2;

        min_2.x = 10000;  //init max value for find three minmum
        min_2.y = 0;
        min_2.z = 0; 
        for(size_t i = 0; i < pointSet->points.size(); i += 2)
        {
            if( i == minIndex_1 || i == minIndex_2_temp ) continue;
            if( haveNoPublicPart(pointSet->points[minIndex_1 + 1].y, pointSet->points[minIndex_1 + 1].z,
                pointSet->points[i + 1].y, pointSet->points[i + 1].z) && pointSet->points[i].x < min_2.x)
            {
                min_2.x = pointSet->points[i].x;
                min_2.y = pointSet->points[i].y;
                min_2.z = pointSet->points[i].z;
                minIndex_2 = i;
            }
        }
    }

    result->points.push_back( min_1 );
    result->points.push_back( pointSet->points[minIndex_1 + 1] );
    result->points.push_back( min_2 );
    result->points.push_back( pointSet->points[minIndex_2 + 1] );

    return result;
}

/**
 * Create the filtering object: downsample the dataset using a leaf size of 1cm
 * param : 1. cloud : you want downsample cloud
 * result: downsample cloud point
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr GetDirectionFromCloud_v2::downsampleCloud( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud )
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_z = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PassThrough<pcl::PointXYZ> pass_z;
	pass_z.setInputCloud (cloud);
	pass_z.setFilterFieldName ("z");
	pass_z.setFilterLimits (-0.4, 8.0);
	pass_z.filter (*cloud_filtered_z);

	// Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (cloud_filtered_z);
    vg.setLeafSize (0.02f, 0.02f, 0.02f);
    vg.filter (*cloud_filtered);
    // std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;

    return cloud_filtered;
}

/**
 * remove planar with 70% area from cloud
 * param: cloud point that you should downsample for compute
 * result: 1.when someone iteration, planar area is 80% with input cloud, return 0;
 *         2.else: return no planar cloud
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr GetDirectionFromCloud_v2::removePlanar( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered)
{
	// Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0;
    int nr_points = (int) cloud_filtered->points.size ();
    //remove planar
    while (cloud_filtered->points.size () > 0.3 * nr_points)
    {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
        {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Write the planar inliers to disk
        extract.filter (*cloud_plane);
        //std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
        if(cloud_plane->points.size () > 0.8 * nr_points)
        {
            //planar at the head;
            std::cout << "planar at the head" << std::endl;
            cloud_filtered->points.resize (0);
            return cloud_filtered;
        }

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
        extract.filter (*cloud_f);
        cloud_filtered = cloud_f;
    }

    // std::cout << "PointCloud after remove planar size = " << cloud_filtered->points.size() << std::endl;
    return cloud_filtered;
}

/**
 * get object from cloud remove planar
 * param: remove planar cloud
 * result: two object: PointXYZ and PointXYZ(min x, min y, max y)
 *         the cloud point may is zero, you should judge cloud point size;
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr GetDirectionFromCloud_v2::clusterExtraction( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered )
{
	// Creating the KdTree object for the search method of the extraction
    //std::cout << ">>> cloud_filtered size = " << cloud_filtered->size() << std::endl;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.15); 
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    int j = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointSet (new pcl::PointCloud<pcl::PointXYZ>);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        float cloud_xsum = 0;
        float cloud_ysum = 0;
        float cloud_zsum = 0;

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        {
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
        }

        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

        pcl::PointXYZ minPt, maxPt;
        pcl::getMinMax3D(*cloud_cluster, minPt, maxPt); 
        pcl::PointXYZ centerPoint;
        centerPoint.x = (minPt.x + maxPt.x) / 2;
        centerPoint.y = (minPt.y + maxPt.y) / 2;
        centerPoint.z = (minPt.z + maxPt.z) / 2;
        pointSet->points.push_back(centerPoint);
        // std::cout << j++ <<" , centerPoint x = " << centerPoint.x << ", y = " << centerPoint.y << ", z = " << centerPoint.z << std::endl;

        pcl::PointXYZ MinXMaxYPoint;
        MinXMaxYPoint.x = minPt.x;
        MinXMaxYPoint.y = minPt.y;
        MinXMaxYPoint.z = maxPt.y;
        pointSet->points.push_back(MinXMaxYPoint);

        // std::cout << "     minPt x = " << minPt.x << ", y = " << minPt.y << ", z = " << minPt.z << std::endl;
        // std::cout << "     maxPt x = " << maxPt.x << ", y = " << maxPt.y << ", z = " << maxPt.z << std::endl;
        // std::cout << "     MinXMaxYPoint x = " << MinXMaxYPoint.x << ", y = " << MinXMaxYPoint.y << ", z = " << MinXMaxYPoint.z << std::endl;
    }

    if(pointSet->points.size() >= 4 )
    	return computeNearObj_2( pointSet );
    else
    	return pointSet;
}