#include <iostream>
#include <vector>
#include <unordered_map>
#include <map>
#include <cmath>
#include <limits>
#include <string>
#include <algorithm>

// Eigen 相关头文件
#include <Eigen/Core>
#include <Eigen/Geometry>

// Ceres 相关头文件
#include <ceres/ceres.h>
#include <ceres/rotation.h>

// cnpy 用于加载 npz 文件
#include "cnpy.h"

// FLANN 相关头文件
#include <flann/flann.hpp>

// =================== 数据结构定义 ===================

// 高斯点参数结构体
struct GaussianParam {
  Eigen::Vector3f means3D;                   // XYZ 坐标
  Eigen::Vector3f rgb_colors;                // RGB 颜色 [0,1]
  Eigen::Quaternion<float> unnorm_rotations; // 四元数 (qx, qy, qz, qw)
  float logit_opacities;                     // 不透明度 logit 值
  float log_scales;                          // 对数尺度参数

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// 每一帧数据
struct FrameData {
  Eigen::Quaternion<float> cam_unnorm_rots; // 相机旋转（四元数）
  Eigen::Vector3f cam_trans;                // 相机平移
  std::vector<GaussianParam> gaussians;     // 本帧所有的高斯点

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// =================== 数据集类定义 ===================

class SLAMDataset {
public:
  explicit SLAMDataset(const std::string &data_path) {
    load_data(data_path);
    std::cout << "Loaded " << frame_map_.size() << " frames." << std::endl;
  }

  // 根据帧索引（或时间戳）获取对应的 FrameData
  const FrameData* get_frame(uint32_t timestep) const {
    auto it = frame_map_.find(timestep);
    return (it != frame_map_.end()) ? &it->second : nullptr;
  }

private:
  std::unordered_map<uint32_t, FrameData> frame_map_;
  Eigen::Matrix3f intrinsics_;
  Eigen::Matrix4f init_w2c_;

  // 将数组中相同数值的索引分组
  std::vector<std::vector<int>> groupIndicesByValue(const int *data, size_t size) {
    std::map<int, std::vector<int>> indexMap;
    for (size_t i = 0; i < size; ++i) {
      indexMap[data[i]].push_back(i);
    }
    std::vector<std::vector<int>> result;
    for (const auto &pair : indexMap) {
      result.push_back(pair.second);
    }
    return result;
  }

  // 加载 npz 文件中的数据
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
    int* Timestep_data = Array_Timestep.data<int>();
    std::vector<std::vector<int>> changePositions =
        groupIndicesByValue(Timestep_data, Array_Timestep.shape[0]);

    std::cout << "Number of frames: " << num_frames << std::endl;
    std::cout << "Change positions size: " << changePositions.size() << std::endl;

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

  // 根据时间戳对高斯点进行分组，并加载对应的高斯点参数
  std::vector<GaussianParam> load_and_distribute_gaussians(
      const cnpy::NpyArray &Array_Means3D,
      const cnpy::NpyArray &Array_Rgb_Colors,
      const cnpy::NpyArray &Array_Unnorm_Rotations,
      const cnpy::NpyArray &Array_Logit_Opacities,
      const cnpy::NpyArray &Array_Log_Scales, int now_frame_id,
      std::vector<std::vector<int>> &changePositions, int scale_type) {
    std::vector<GaussianParam> gaussians;
    for (size_t a = 0; a < changePositions[now_frame_id].size(); a++) {
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

// =================== 利用 FLANN 计算对应关系 ===================

// 使用 FLANN 建立 KD-Tree，并对每个源帧高斯点查询目标帧最近邻，返回 (source_index, target_index) 对
std::vector<std::pair<int, int>> computeCorrespondencesFLANN(
    const std::vector<GaussianParam>& targetGaussians, 
    const std::vector<GaussianParam>& sourceGaussians,
    float maxDistance) {
  std::vector<std::pair<int, int>> correspondences;

  // 构造目标帧点云数据，存储为连续的 float 数组，尺寸为 [n_points x 3]
  size_t targetSize = targetGaussians.size();
  std::vector<float> target_data(targetSize * 3);
  for (size_t i = 0; i < targetSize; ++i) {
    target_data[i * 3 + 0] = targetGaussians[i].means3D.x();
    target_data[i * 3 + 1] = targetGaussians[i].means3D.y();
    target_data[i * 3 + 2] = targetGaussians[i].means3D.z();
  }

  // 使用 FLANN 构造矩阵，行数为目标点个数，列数为3
  flann::Matrix<float> dataset(&target_data[0], targetSize, 3);
  // 构造 KD-Tree 索引，使用 L2 距离，KDTreeIndexParams 中的参数表示树的个数
  flann::Index<flann::L2<float>> index(dataset, flann::KDTreeIndexParams(4));
  index.buildIndex();

  // 对每个源帧点进行查询
  size_t sourceSize = sourceGaussians.size();
  for (size_t i = 0; i < sourceSize; ++i) {
    Eigen::Vector3f query_pt = sourceGaussians[i].means3D;
    float query[3] = { query_pt.x(), query_pt.y(), query_pt.z() };

    // knnSearch 查询最近邻
    int nn = 1;
    std::vector<int> indices(nn);
    std::vector<float> dists(nn);
    flann::Matrix<int> indicesMat(&indices[0], 1, nn);
    flann::Matrix<float> distsMat(&dists[0], 1, nn);

    index.knnSearch(flann::Matrix<float>(query, 1, 3), indicesMat, distsMat, nn, flann::SearchParams(32));

    // distsMat 中返回的是平方距离
    if (dists[0] < maxDistance * maxDistance) {
      correspondences.push_back(std::make_pair(static_cast<int>(i), indices[0]));
    }
  }
  return correspondences;
}

// =================== Ceres 优化残差构造 ===================

// 定义残差项：将源帧高斯点经过变换后与目标帧对应点之间的欧式距离作为残差
struct NDTResidual {
  NDTResidual(const Eigen::Vector3f &target_mean, const Eigen::Vector3f &source_point)
      : target_mean(target_mean), source_point(source_point) {}

  template <typename T>
  bool operator()(const T* const angle_axis, const T* const translation, T* residual) const {
    T p[3];
    p[0] = T(source_point.x());
    p[1] = T(source_point.y());
    p[2] = T(source_point.z());

    T p_rotated[3];
    ceres::AngleAxisRotatePoint(angle_axis, p, p_rotated);

    p_rotated[0] += translation[0];
    p_rotated[1] += translation[1];
    p_rotated[2] += translation[2];

    residual[0] = p_rotated[0] - T(target_mean.x());
    residual[1] = p_rotated[1] - T(target_mean.y());
    residual[2] = p_rotated[2] - T(target_mean.z());
    return true;
  }

  Eigen::Vector3f target_mean;
  Eigen::Vector3f source_point;
};

// =================== 主函数 ===================

int main() {
  // 修改为你的 npz 文件路径
  std::string file_path = "path/to/your/data.npz";
  SLAMDataset dataset(file_path);

  // 选取相邻两帧（例如帧 0 和帧 1）
  const FrameData* frame0 = dataset.get_frame(0); // 目标帧
  const FrameData* frame1 = dataset.get_frame(1); // 源帧

  if (!frame0 || !frame1) {
    std::cerr << "Error: Cannot load frame data!" << std::endl;
    return -1;
  }

  // ---------- 利用 FLANN 建立对应关系 ----------
  float maxDistance = 0.5f;  // 根据数据设置匹配阈值
  auto flann_correspondences = computeCorrespondencesFLANN(frame0->gaussians, frame1->gaussians, maxDistance);
  std::cout << "FLANN correspondences: " << flann_correspondences.size() << " pairs found." << std::endl;
  for (const auto &pair : flann_correspondences) {
    std::cout << "Source index: " << pair.first << " -> Target index: " << pair.second << std::endl;
  }

  // ---------- 利用 FLANN 对应关系进行 Ceres 位姿优化 ----------
  size_t num_correspondences = flann_correspondences.size();
  if (num_correspondences == 0) {
    std::cerr << "No correspondences found for optimization!" << std::endl;
    return -1;
  }

  // 初始猜测：将源帧转换到目标帧，采用角轴（3 个参数）和平移（3 个参数）
  double angle_axis[3] = {0.0, 0.0, 0.0};
  double translation[3] = {0.0, 0.0, 0.0};

  ceres::Problem problem;
  for (size_t i = 0; i < num_correspondences; ++i) {
    int src_idx = flann_correspondences[i].first;
    int tgt_idx = flann_correspondences[i].second;
    const Eigen::Vector3f &source_point = frame1->gaussians[src_idx].means3D;
    const Eigen::Vector3f &target_point = frame0->gaussians[tgt_idx].means3D;
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<NDTResidual, 3, 3, 3>(
            new NDTResidual(target_point, source_point));
    problem.AddResidualBlock(cost_function, nullptr, angle_axis, translation);
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << std::endl;
  std::cout << "Optimized angle-axis: [" << angle_axis[0] << ", " << angle_axis[1]
            << ", " << angle_axis[2] << "]" << std::endl;
  std::cout << "Optimized translation: [" << translation[0] << ", "
            << translation[1] << ", " << translation[2] << "]" << std::endl;

  // 将角轴转换为旋转矩阵和四元数
  double angle = std::sqrt(angle_axis[0]*angle_axis[0] + angle_axis[1]*angle_axis[1] + angle_axis[2]*angle_axis[2]);
  Eigen::Vector3d axis(0, 0, 1);
  if (angle > 1e-6) {
    axis = Eigen::Vector3d(angle_axis[0], angle_axis[1], angle_axis[2]) / angle;
  }
  Eigen::AngleAxisd optimized_rotation(angle, axis);
  Eigen::Quaterniond optimized_quat(optimized_rotation);
  std::cout << "Optimized rotation (quaternion): " << optimized_quat.coeffs().transpose() << std::endl;

  return 0;
}
