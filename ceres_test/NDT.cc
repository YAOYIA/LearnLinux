#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>
#include <iostream>

int main ()
{
  // 1. 加载点云数据（源点云和目标点云）
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
  
  if (pcl::io::loadPCDFile("source.pcd", *cloud_source) == -1)
  {
    PCL_ERROR("无法读取文件 source.pcd\n");
    return -1;
  }
  
  if (pcl::io::loadPCDFile("target.pcd", *cloud_target) == -1)
  {
    PCL_ERROR("无法读取文件 target.pcd\n");
    return -1;
  }
  
  // 2. 初始化 NDT 配准对象
  pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
  
  // 设置 NDT 参数（这些参数可根据实际情况调整）
  ndt.setTransformationEpsilon(0.01);
  ndt.setStepSize(0.1);
  ndt.setResolution(1.0);
  
  // 设置输入源和目标点云
  ndt.setInputSource(cloud_source);
  ndt.setInputTarget(cloud_target);
  
  // 3. 初始变换矩阵（此处设为单位矩阵，可根据需要提供更合理的初始估计）
  Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
  
  // 4. 执行配准
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_aligned(new pcl::PointCloud<pcl::PointXYZ>);
  ndt.align(*cloud_aligned, init_guess);
  
  // 输出配准结果
  std::cout << "NDT 是否收敛: " << ndt.hasConverged() 
            << " 适配得分: " << ndt.getFitnessScore() << std::endl;
  std::cout << "最终变换矩阵: " << std::endl << ndt.getFinalTransformation() << std::endl;
  
  // 5. 可视化结果
  pcl::visualization::PCLVisualizer viewer("NDT 点云配准结果");
  
  // 显示目标点云（绿色）
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_target, 0, 255, 0);
  viewer.addPointCloud(cloud_target, target_color, "target_cloud");
  
  // 显示配准后的源点云（红色）
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligned_color(cloud_aligned, 255, 0, 0);
  viewer.addPointCloud(cloud_aligned, aligned_color, "aligned_cloud");
  
  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
  }
  
  return 0;
}
