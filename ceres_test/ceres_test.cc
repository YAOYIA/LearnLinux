#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <fstream>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/parallel_for.h>

#include "cnpy.h"
#include <Eigen/Geometry>
#include <filesystem>
#include <flann/flann.hpp>
#include <iostream>
#include <mutex> // 引入 std::mutex
#include <opencv2/core/eigen.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

struct GaussianParam {
  Eigen::Vector3f means3D;                   // XYZ坐标
  Eigen::Vector3f rgb_colors;                // RGB颜色 [0,1]
  Eigen::Quaternion<float> unnorm_rotations; // 四元数 (qx, qy, qz, qw)
  float logit_opacities;                     // 不透明度logit值
  // Vector3f log_scale;   // 对数尺度参数
  float log_scales; // 对数尺度参数

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen内存对齐
};

struct FrameData {
  Eigen::Quaternion<float> cam_unnorm_rots; // 相机旋转
  Eigen::Vector3f cam_trans;                // 相机平移
  std::vector<GaussianParam> gaussians;     // 该帧的高斯点

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class SLAMDataset {
public:
  explicit SLAMDataset(const std::string &data_path) {
    load_data(data_path);
    std::cout << "frame_map_ size = " << frame_map_.size() << std::endl;
  }

  // 获取指定时间戳的帧数据
  const FrameData *get_frame(uint32_t timestep) const {
    auto it = frame_map_.find(timestep);
    return (it != frame_map_.end()) ? &it->second : nullptr;
  }

private:
  std::unordered_map<uint32_t, FrameData> frame_map_;
  Eigen::Matrix3f intrinsics_;
  Eigen::Matrix4f init_w2c_;

  std::vector<std::vector<int>> groupIndicesByValue(const int *data,
                                                    size_t size) {
    std::map<int, std::vector<int>> indexMap;

    // 遍历数组，将下标按数字值分组
    for (size_t i = 0; i < size; ++i) {
      indexMap[data[i]].push_back(i);
    }

    // 将结果存储到一个vector中
    std::vector<std::vector<int>> result;
    for (const auto &pair : indexMap) {
      result.push_back(pair.second);
    }

    return result;
  }
  void load_data(const std::string &path) {
    cnpy::npz_t npz = cnpy::npz_load(path);

    cnpy::NpyArray Array_Means3D = npz["means3D"];
    cnpy::NpyArray Array_Rgb_Colors = npz["rgb_colors"];
    cnpy::NpyArray Array_Unnorm_Rotations = npz["unnorm_rotations"];
    cnpy::NpyArray Array_Logit_Opacities = npz["logit_opacities"];
    cnpy::NpyArray Array_Log_Scales = npz["log_scales"];

    cnpy::NpyArray Array_Cam_Unnorm_Rots = npz["cam_unnorm_rots"];
    cnpy::NpyArray Array_Cam_Trans = npz["cam_trans"];
    int num_frames = Array_Cam_Trans.shape[2];

    cnpy::NpyArray Array_Timestep = npz["timestep"];
    int *Timestep_data = Array_Timestep.data<int>();
    std::vector<std::vector<int>> changePositions =
        groupIndicesByValue(Timestep_data, Array_Timestep.shape[0]);

    std::cout << "Number of frames: " << num_frames << std::endl;
    std::cout << "Change positions size: " << changePositions.size()
              << std::endl;

    for (int i = 0; i < num_frames; ++i) {
      FrameData fd;
      fd.gaussians = load_and_distribute_gaussians(
          Array_Means3D, Array_Rgb_Colors, Array_Unnorm_Rotations,
          Array_Logit_Opacities, Array_Log_Scales, i, changePositions, 1);
      fd.cam_trans = Eigen::Vector3f(Array_Cam_Trans.data<float>()[i * 3 + 0],
                                     Array_Cam_Trans.data<float>()[i * 3 + 1],
                                     Array_Cam_Trans.data<float>()[i * 3 + 2]);
      fd.cam_unnorm_rots = Eigen::Quaternion<float>(
          Array_Cam_Unnorm_Rots.data<float>()[i * 4 + 3],  // qw
          Array_Cam_Unnorm_Rots.data<float>()[i * 4 + 0],  // qx
          Array_Cam_Unnorm_Rots.data<float>()[i * 4 + 1],  // qy
          Array_Cam_Unnorm_Rots.data<float>()[i * 4 + 2]); // qz
      frame_map_[i] = fd;
    }
  }
  std::vector<GaussianParam> load_and_distribute_gaussians(
      const cnpy::NpyArray &Array_Means3D,
      const cnpy::NpyArray &Array_Rgb_Colors,
      const cnpy::NpyArray &Array_Unnorm_Rotations,
      const cnpy::NpyArray &Array_Logit_Opacities,
      const cnpy::NpyArray &Array_Log_Scales, int now_frame_id,
      std::vector<std::vector<int>> &changePositions, int scale_type) {
    std::vector<GaussianParam> gaussians;

    // changepositions
    // 每一维表示每一帧，第二个维度里边是每一站里边的高斯参数的index
    for (int a = 0; a < changePositions[now_frame_id].size(); a++) {
      GaussianParam g;
      int i = changePositions[now_frame_id][a];
      g.means3D = Eigen::Vector3f(Array_Means3D.data<float>()[i * 3 + 0],
                                  Array_Means3D.data<float>()[i * 3 + 1],
                                  Array_Means3D.data<float>()[i * 3 + 2]);

      g.rgb_colors = Eigen::Vector3f(Array_Rgb_Colors.data<float>()[i * 3 + 0],
                                     Array_Rgb_Colors.data<float>()[i * 3 + 1],
                                     Array_Rgb_Colors.data<float>()[i * 3 + 2]);

      g.unnorm_rotations = Eigen::Quaternion<float>(
          Array_Unnorm_Rotations.data<float>()[i * 4 + 0],
          Array_Unnorm_Rotations.data<float>()[i * 4 + 1],
          Array_Unnorm_Rotations.data<float>()[i * 4 + 2],
          Array_Unnorm_Rotations.data<float>()[i * 4 + 3]);

      g.logit_opacities = Array_Logit_Opacities.data<float>()[i];
      g.log_scales = Array_Log_Scales.data<float>()[i];
      gaussians.push_back(g);
    }

    return gaussians;
  }
};

using GSPoint3D = std::vector<GaussianParam>;

struct PointCloudAdaptor {
  const GSPoint3D &points;
  PointCloudAdaptor(const GSPoint3D &c) : points(c) {}

  size_t kdtree_get_point_count() const { return points.size(); }

  double kdtree_get_pt(const size_t idx, const size_t dim) const {
    if (dim == 0)
      return points[idx].means3D.x();
    if (dim == 1)
      return points[idx].means3D.y();
    return points[idx].means3D.z();
  }

  template <class BBOX> bool kdtree_get_bbox(BBOX &) const { return false; }
};

using KDTree = flann::Index<flann::L2_Simple<double>>;

// 构建KD树
// void buildKDTree(const GSPoint3D &points, KDTree *index) {
//   PointCloudAdaptor adaptor(points);
//   flann::Matrix<double> dataset(new double[points.size() * 3], points.size(),
//                                 3);

//   for (size_t i = 0; i < points.size(); ++i) {
//     dataset[i][0] = points[i].means3D.x();
//     dataset[i][1] = points[i].means3D.y();
//     dataset[i][2] = points[i].means3D.z();
//   }

//   *index = KDTree(dataset, flann::KDTreeSingleIndexParams());
//   index->buildIndex();
// }

KDTree buildKDTree(const GSPoint3D &cloud) {
  // 方法1
  // PointCloudAdaptor adaptor(cloud);
  // KDTree index(adaptor, flann::KDTreeSingleIndexParams());
  // index.buildIndex();
  // return index;

  // 方法2：显式构造数据矩阵
  flann::Matrix<double> dataset(new double[cloud.size() * 3], cloud.size(), 3);
  for (size_t i = 0; i < cloud.size(); ++i) {
    dataset[i][0] = cloud[i].means3D.x();
    dataset[i][1] = cloud[i].means3D.y();
    dataset[i][2] = cloud[i].means3D.z();
  }
  KDTree index(dataset, flann::KDTreeSingleIndexParams());
  index.buildIndex();
  delete[] dataset.ptr(); // 仅当FLANN内部复制数据时安全
  return index;

  // // 创建索引时传入参数
  // return KDTree(dataset, flann::KDTreeSingleIndexParams());
}

struct NDTResidual {
  GaussianParam last_3dgs_points;
  GaussianParam current_3dgs_points;

  NDTResidual(const GaussianParam &lst, const GaussianParam &crt)
      : last_3dgs_points(lst), current_3dgs_points(crt) {}

  template <typename T>
  bool operator()(const T *const rvec, const T *const tvec, T *residual) const {

    T R[9];
    ceres::AngleAxisToRotationMatrix(rvec, R);

    T p_last[3];
    p_last[0] = T(last_3dgs_points.means3D.x());
    p_last[1] = T(last_3dgs_points.means3D.y());
    p_last[2] = T(last_3dgs_points.means3D.z());

    T p_transformed[3];
    p_transformed[0] =
        R[0] * p_last[0] + R[1] * p_last[1] + R[2] * p_last[2] + tvec[0];
    p_transformed[1] =
        R[3] * p_last[0] + R[4] * p_last[1] + R[5] * p_last[2] + tvec[1];
    p_transformed[2] =
        R[6] * p_last[0] + R[7] * p_last[1] + R[8] * p_last[2] + tvec[2];

    // 计算残差（各维度独立处理）
    // T scale = T(1.0) / T(current_3dgs_points.log_scales);
    // residual[0] =
    //     (transformed_point.x() - T(current_3dgs_points.means3D.x())) *
    //     T(abs(current_3dgs_points.log_scales - last_3dgs_points.log_scales));
    // residual[1] =
    //     (transformed_point.y() - T(current_3dgs_points.means3D.y())) *
    //     T(abs(current_3dgs_points.log_scales - last_3dgs_points.log_scales));
    // residual[2] =
    //     (transformed_point.z() - T(current_3dgs_points.means3D.z())) *
    //     T(abs(current_3dgs_points.log_scales - last_3dgs_points.log_scales));

    // residual[0] = (transformed_point.x() -
    // T(current_3dgs_points.means3D.x())) *
    //               T(abs(exp(current_3dgs_points.log_scales) -
    //                     exp(last_3dgs_points.log_scales)));
    // residual[1] = (transformed_point.y() -
    // T(current_3dgs_points.means3D.y())) *
    //               T(abs(exp(current_3dgs_points.log_scales) -
    //                     exp(last_3dgs_points.log_scales)));
    // residual[2] = (transformed_point.z() -
    // T(current_3dgs_points.means3D.z())) *
    //               T(abs(exp(current_3dgs_points.log_scales) -
    //                     exp(last_3dgs_points.log_scales)));
    T scale = T(abs(exp(current_3dgs_points.log_scales) -
                    exp(last_3dgs_points.log_scales)));
    residual[0] =
        (T(p_transformed[0]) - T(current_3dgs_points.means3D.x())) * scale;
    residual[1] =
        (T(p_transformed[1]) - T(current_3dgs_points.means3D.y())) * scale;
    residual[2] =
        (T(p_transformed[2]) - T(current_3dgs_points.means3D.z())) * scale;

    return true;
  }

  // Ceres 需要使用的工厂函数
  static ceres::CostFunction *Create(const GaussianParam last_3dgs_points,
                                     const GaussianParam current_3dgs_points) {
    return new ceres::AutoDiffCostFunction<NDTResidual, 3, 3, 3>(
        new NDTResidual(last_3dgs_points, current_3dgs_points));
  }
};

void SaveLastAndCurrentPointCorrespondences(const GSPoint3D last_3dgs_points,
                                            const GSPoint3D current_3dgs_points,
                                            std::vector<int> correspondences) {
  // 保存last帧点云
  std::ofstream last_file("last_points.csv");
  for (const auto &pt : last_3dgs_points) {
    last_file << pt.means3D.x() << "," << pt.means3D.y() << ","
              << pt.means3D.z() << "\n";
  }

  // 保存current帧点云
  std::ofstream current_file("current_points.csv");
  for (const auto &pt : current_3dgs_points) {
    current_file << pt.means3D.x() << "," << pt.means3D.y() << ","
                 << pt.means3D.z() << "\n";
  }

  // 保存对应关系
  std::ofstream corr_file("correspondences.txt");
  for (auto idx : correspondences) {
    corr_file << idx << "\n";
  }
}

void CeresOptimize(const GSPoint3D last_3dgs_points,
                   const GSPoint3D current_3dgs_points, cv::Mat &rvec,
                   cv::Mat &tvec) {

  if (rvec.empty()) {
    rvec = cv::Mat::zeros(3, 1, CV_64F); // 旋转向量未初始化，则设为 (0,0,0)
  } else if (rvec.type() != CV_64F) {
    rvec.convertTo(rvec, CV_64F); // 确保数据类型正确
  }

  if (tvec.empty()) {
    tvec = cv::Mat::zeros(3, 1, CV_64F); // 平移向量未初始化，则设为 (0,0,0)
  } else if (tvec.type() != CV_64F) {
    tvec.convertTo(tvec, CV_64F);
  }

  double rvec_data[3] = {rvec.at<double>(0, 0), rvec.at<double>(1, 0),
                         rvec.at<double>(2, 0)};
  double tvec_data[3] = {tvec.at<double>(0, 0), tvec.at<double>(1, 0),
                         tvec.at<double>(2, 0)};

  std::cout << "rvec_data" << rvec_data[0] << "," << rvec_data[1] << ","
            << rvec_data[2] << std::endl;
  std::cout << "tvec_data" << tvec_data[0] << "," << tvec_data[1] << ","
            << tvec_data[2] << std::endl;
  // 当前帧kdtree
  // KDTree current_frame_kdtree;
  // buildKDTree(current_3dgs_points, &current_frame_kdtree);

  KDTree current_frame_kdtree = buildKDTree(current_3dgs_points);

  std::vector<int> correspondences;
  for (const auto &lst_pt : last_3dgs_points) {

    // 转换当前源点到目标坐标系
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat last_point_mat = (cv::Mat_<double>(3, 1) << lst_pt.means3D.x(),
                              lst_pt.means3D.y(), lst_pt.means3D.z());
    cv::Mat T = tvec.clone();
    cv::Mat transformed_point_mat = R * last_point_mat + T;

    // 寻找最近邻
    // flann::Matrix<double> query(new double[3], 1, 3);
    auto query_data = std::make_unique<double[]>(3); // 智能指针管理
    flann::Matrix<double> query(query_data.get(), 1, 3);
    query[0][0] = transformed_point_mat.at<double>(0);
    query[0][1] = transformed_point_mat.at<double>(1);
    query[0][2] = transformed_point_mat.at<double>(2);

    // flann::Matrix<int> indices(new int[1], 1, 1);
    // flann::Matrix<double> dists(new double[1], 1, 1);

    auto indices_data = std::make_unique<int[]>(1);
    flann::Matrix<int> indices(indices_data.get(), 1, 1);

    auto dists_data = std::make_unique<double[]>(1);
    flann::Matrix<double> dists(dists_data.get(), 1, 1);

    // 检查问题：
    // std::cout << "current_frame_kdtree.size: " << current_frame_kdtree.size()
    //           << std::endl;
    std::cout << "transformed_position: " << transformed_point_mat.at<double>(0)
              << "," << transformed_point_mat.at<double>(1) << ","
              << transformed_point_mat.at<double>(2) << std::endl
              << "current_3dgs_points: "
              << current_3dgs_points[indices[0][0]].means3D.x() << ","
              << current_3dgs_points[indices[0][0]].means3D.y() << ","
              << current_3dgs_points[indices[0][0]].means3D.z() << std::endl;

    current_frame_kdtree.knnSearch(query, indices, dists, 1,
                                   flann::SearchParams());

    correspondences.push_back(indices[0][0]);
    // delete[] query.ptr();
    // delete[] indices.ptr();
    // delete[] dists.ptr();
  }

  SaveLastAndCurrentPointCorrespondences(last_3dgs_points, current_3dgs_points,
                                         correspondences);

  // 构建 Ceres 优化问题
  ceres::Problem problem;
  for (size_t i = 0; i < last_3dgs_points.size(); ++i) {
    // std::cout << "last_3dgs_points[i].x: " << last_3dgs_points[i].means3D.x()
    // << std::endl
    //           << "last_3dgs_points[i].y: " << last_3dgs_points[i].means3D.y()
    //           << std::endl
    //           << "last_3dgs_points[i].z: " << last_3dgs_points[i].means3D.z()
    //           << std::endl
    //           << "current_3dgs_points.x: "
    //           << current_3dgs_points[correspondences[i]].means3D.x() <<
    //           std::endl
    //           << "current_3dgs_points.y: "
    //           << current_3dgs_points[correspondences[i]].means3D.y() <<
    //           std::endl
    //           << "current_3dgs_points.z: "
    //           << current_3dgs_points[correspondences[i]].means3D.z() <<
    //           std::endl;
    ceres::CostFunction *cost_function = NDTResidual::Create(
        last_3dgs_points[i], current_3dgs_points[correspondences[i]]);
    problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0),
                             rvec_data, tvec_data);
  }

  // Ceres 求解器设置
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = false; // 关闭 Ceres 输出
  options.max_num_iterations = 100;

  // 运行 Ceres
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << "summary.termination_type: " << summary.termination_type
            << std::endl;

  // std::cout << "Iteration " << iter << ":\n" << summary.BriefReport() <<
  // "\n";

  // 将优化结果存回 OpenCV 的 rvec, tvec
  rvec = (cv::Mat_<double>(3, 1) << rvec_data[0], rvec_data[1], rvec_data[2]);
  tvec = (cv::Mat_<double>(3, 1) << tvec_data[0], tvec_data[1], tvec_data[2]);

  // 统一 rvec 方向
  // rvec = -rvec; // 旋转方向不一致，翻转 rvec
}

void CalculatePose(const SLAMDataset &dataset) {
  // 初始化位姿
  cv::Mat global_pose = cv::Mat::eye(4, 4, CV_64F);

  std::vector<cv::Mat> all_poses; // 保存所有帧的全局姿态
  all_poses.push_back(global_pose.clone());

  // 获取frame个数
  // int frame_num = ;
  std::fstream f;
  f.open("zyh_pose.txt", std::ios::out);
  for (uint32_t i = 0; i < 588; ++i) {
    if (i == 0)
      continue;

    // 提取两帧3DGS点
    const FrameData *last_frame_points_data = dataset.get_frame(i - 1);
    const FrameData *current_frame_points_data = dataset.get_frame(i);

    // 使用3DGS位姿初始化旋转平移
    Eigen::Quaternionf normalized_quat =
        current_frame_points_data->cam_unnorm_rots.normalized();
    Eigen::Matrix3d eigen_rot_mat =
        normalized_quat.toRotationMatrix().cast<double>();
    cv::Mat rvec;
    cv::Mat cv_rot_mat;
    cv::eigen2cv(eigen_rot_mat, cv_rot_mat);
    cv::Rodrigues(cv_rot_mat, rvec);

    cv::Mat tvec;
    Eigen::Matrix<double, 3, 1> eigen_trans =
        current_frame_points_data->cam_trans.cast<double>();
    cv::eigen2cv(eigen_trans, tvec);

    cv::Mat rvec_ceres = rvec.clone();
    cv::Mat tvec_ceres = tvec.clone();

    CeresOptimize(last_frame_points_data->gaussians,
                  current_frame_points_data->gaussians, rvec_ceres, tvec_ceres);

    // 将rvec和tvec转换为4x4变换矩阵
    cv::Mat R;
    cv::Rodrigues(rvec_ceres, R);

    cv::Mat global_R = global_pose(cv::Rect(0, 0, 3, 3));
    cv::Mat global_t = global_pose(cv::Rect(3, 0, 1, 3));

    // 计算新旋转和新平移
    cv::Mat updated_t = global_t + global_R * tvec;
    cv::Mat updated_R = global_R * R; // 旋转直接相乘

    // 将旋转和平移存回 global_pose
    updated_R.copyTo(global_pose(cv::Rect(0, 0, 3, 3))); // 更新旋转部分
    updated_t.copyTo(global_pose(cv::Rect(3, 0, 1, 3))); // 更新平移部分

    // 将逆矩阵存入 all_poses
    all_poses.push_back(global_pose.clone());

    //输出位姿到文件
    std::cout << "Frame " << i << " pose:\n" << global_pose << std::endl;

    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 4; ++k) {
        f << global_pose.at<double>(j, k);
        if (j == 2 && k == 3) {
          f << std::endl;
        } else {
          f << ",";
        }
      }
    }
  }
  f.close();
}

void TestKDTree() {
  // KDTREE 测试
  GSPoint3D cloud;
  for (int i = 0; i < 10; ++i) {
    GaussianParam param;
    param.means3D = Eigen::Vector3f(i, i * 2, i * 3); // 生成不同位置的测试数据
    cloud.push_back(param);
  }
  // 构建KD-Tree
  KDTree index = buildKDTree(cloud);

  // 准备查询点（查找最近的点）
  flann::Matrix<double> query(new double[3], 1, 3);
  query[0][0] = 4.5;  // x坐标
  query[0][1] = 9.0;  // y坐标（匹配第5个点的y=10）
  query[0][2] = 13.5; // z坐标（匹配第5个点的z=15）

  // 准备接收结果的容器
  std::vector<int> indices(1);
  std::vector<double> dists(1);
  flann::Matrix<int> indices_mat(indices.data(), 1, 1);
  flann::Matrix<double> dists_mat(dists.data(), 1, 1);
  std::cout << "abc" << std::endl;
  // 执行搜索
  index.knnSearch(query, indices_mat, dists_mat, 1, flann::SearchParams());

  // 输出结果
  std::cout << "Nearest point index: " << indices[0] << std::endl;
  std::cout << "Distance: " << dists[0] << std::endl;

  delete[] query.ptr(); // 清理查询矩阵内存
}

int main() {

  // std::string data_path = "../npz/params.npz";
  // SLAMDataset dataset(data_path);

  // 测试获取帧数据
  // uint32_t test_frame_id = 0; // 测试第 5 帧的数据
  // const FrameData *frame_data = dataset.get_frame(test_frame_id);

  // std::cout << "frame_id: " << test_frame_id << std::endl
  //           << "cam_rot: " << frame_data->cam_unnorm_rots.coeffs() <<
  //           std::endl
  //           << "cam_tra: " << frame_data->cam_trans << std::endl;
  // for (int i = 0; i < 100; ++i) {
  //   std::cout << "point x: " << frame_data->gaussians[i].means3D.x()
  //             << std::endl
  //             << "log_scales: " << frame_data->gaussians[i].log_scales
  //             << std::endl;
  // }

  // KDTree 测试
  // TestKDTree();

  std::string data_path = "../npz/params.npz";
  SLAMDataset dataset(data_path);

  CalculatePose(dataset);

  return 0;
}
