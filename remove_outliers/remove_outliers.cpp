#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

int main (int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  pcl::PCDReader reader;
  reader.read ("test_pcd.pcd", *cloud);
  std::cerr << "hola " << std::endl;

  if (strcmp(argv[1], "-r") == 0){
    std::cerr << "please  " << std::endl;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    std::cerr << "please a " << std::endl;
    // build the filter
    outrem.setInputCloud(cloud);
    outrem.setRadiusSearch(0.01);
    outrem.setMinNeighborsInRadius (2);
    // apply filter
    std::cerr << "please b " << std::endl;
    outrem.filter (*cloud_filtered);
    std::cerr << "please c " << std::endl;
  }
  else if (strcmp(argv[1], "-c") == 0){
    // build the condition
    std::cerr << "NO " << std::endl;
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_cond (new
      pcl::ConditionAnd<pcl::PointXYZ> ());
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::GT, -0.1)));
    range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new
      pcl::FieldComparison<pcl::PointXYZ> ("z", pcl::ComparisonOps::LT, 0.8)));
    // build the filter
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setCondition (range_cond);
    condrem.setInputCloud (cloud);
    condrem.setKeepOrganized(true);
    // apply filter
    condrem.filter (*cloud_filtered);
  }
  else{
    std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
    exit(0);
  }

  std::cerr << "please specify " << std::endl;
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("table_scene_lms400_removed.pcd", *cloud_filtered, false);
  std::cerr << "please specify command line arg '-r' or '-c'" << std::endl;
  return (0);
}