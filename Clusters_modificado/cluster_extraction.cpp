#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>


int 
main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
  std::string filename0 = "";
  if (!pcd_filename_indices.empty ())
  {
    std::string filename = argv[pcd_filename_indices[0]];
    filename0 =filename;
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud_filtered) == -1)
    {
    PCL_ERROR ("Couldn't read file the .pcd file \n");
    return (-1);
    }
  }
  else
  {
      PCL_ERROR ("Couldn't read file the .pcd file \n");
    return (-1);
  }
 
  pcl::PCDWriter writer;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance (0.5); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (500000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);

  std::cout << "SIZE:  " << cluster_indices.size () << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clustered_demean (new pcl::PointCloud<pcl::PointXYZ>);
  int j = 0;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    cloud_cluster->clear();
    cloud_clustered_demean->clear();
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

    Eigen::Vector4f centroide; 
    pcl::compute3DCentroid (*cloud_cluster, centroide);
    pcl::demeanPointCloud<pcl::PointXYZ> (*cloud_cluster, centroide, *cloud_clustered_demean);

    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*

    std::stringstream kk;
    kk << "cloud_cluster_" << j << "demean.pcd";
    writer.write<pcl::PointXYZ> (kk.str (), *cloud_clustered_demean, false); //*
    j++;
  }

  return (0);
}