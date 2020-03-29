#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <std_msgs/Float64.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2::Ptr cloud_filtered_blob (new pcl::PCLPointCloud2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
  
  std::vector<std::vector<double> > coeff_matrix;

  // Convert the input message to the PCL data type
  pcl_conversions::toPCL(*input, *cloud);

  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered_blob);
  
  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);

  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  
  int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.2 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    // Write the model coefficients to a vector? or mutlidimensional array?
    std::vector<double>inter_mat;
    for(int j=0; j<4; j++){
      inter_mat.push_back(coefficients->values[j]);
    }
    coeff_matrix.push_back(inter_mat);

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
    i++;
  }

  //Debug
  int numPlanes = coeff_matrix.size();//No. of planes detected or no. of vectors in a vector of vectors
	std::cerr << "No. of planes " << numPlanes << std::endl;
	int numCoeffs = coeff_matrix[0].size();//We always have 4 elements to a vector coeff_matrix[i]
	//std::cout << "No. of coeffs " << numCoeffs << std::endl;

	double epsilon = 0.05;//Closeness factor to distinguish planes
	double phi = 0.1;//Factor to distinguish close planes
	double height =-1;
	int j,k;
	std::vector< std::vector<double> > heightVec;//Optional in case there are multiple parallel planes

	for (i =0; i<numPlanes-1; i++){//Select every plane vector but last
		for (j=i+1; j<numPlanes; j++){//Select every ahead from ith
			//std::cout << "Planes " << i << " and " << j << " selected for this pass" << std::endl;//Debug message
			for (k=0; k<numCoeffs; k++){//Select every coeff in a plane
				//std::cout << "Coefficent k= " << k << std::endl;
				if (k< 3){//For the first three coeffs check parallel
					if (std::abs(coeff_matrix[i][k] - coeff_matrix[j][k]) > epsilon){
						
						//std::cout << "Planes " << i << " and " << j << " are not parallel" << std::endl;
						break; //Because they are not parallel we don't care about other coefficients of these two planes now
					}
				}
				else {
					if (std::abs(coeff_matrix[i][k] - coeff_matrix[j][k]) > phi){//Planes are parallel and distant enough to be actual separate planes
						height = std::abs(coeff_matrix[i][k] - coeff_matrix[j][k]);
						//Optional
						std::vector <double> temp;
						temp.push_back(i);temp.push_back(j);temp.push_back(height);
						heightVec.push_back(temp);
						std::cerr << "Planes " << i << " and " << j << " are separated by a distant of "  << height << std::endl;
					}
				}
			}
      if (height!=-1) break;
		}	
	}

  std_msgs::Float64 heightValue;

  heightValue.data = height;

  // Publish the model coefficients
  //pcl_msgs::ModelCoefficients ros_coefficients;
  //pcl_conversions::fromPCL(*coefficients, ros_coefficients);
  pub.publish (heightValue);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<std_msgs::Float64>("bridgeheight", 1000);

  // n.advertise<std_msgs::Float64>("/auc", 1000, true)
  // Spin
  ros::spin ();
}