#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_x (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  std::map<std::string,pcl::PointCloud<pcl::PointXYZ> > my_map;

  std::vector<int> pcd_filename_indices = pcl::console::parse_file_extension_argument (argc, argv, "pcd");
  if (!pcd_filename_indices.empty ())
  {
    std::string filename = argv[pcd_filename_indices[0]];
    if (pcl::io::loadPCDFile (filename, *cloud) == -1)
    {
      std::cout << "Was not able to open file \""<<filename<<"\".\n";
      return 0;
    }                       
  }

  // Create the filtering object
  std::string limit = "x";
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName (limit);
  pass.setFilterLimits (1.2, 6.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("nube_original.pcd",*cloud, false); 
  pcl::PCDWriter writer2;
  writer2.write<pcl::PointXYZ> ("nube_filtrada.pcd",*cloud_filtered, false); 

  my_map["x"] =  *cloud;
  my_map["y"] =  *cloud_filtered;
  *cloud_x = my_map["y"];

  pcl::PCDWriter writer3;
  writer3.write<pcl::PointXYZ> ("nube_original2.pcd",my_map["x"], false); 
  pcl::PCDWriter writer4;
  writer4.write<pcl::PointXYZ> ("nube_filtrada2.pcd",my_map["y"], false);
  pcl::PCDWriter writer5;
  writer5.write<pcl::PointXYZ> ("nube_filtrada3.pcd",*cloud_x, false);

  return (0);
}