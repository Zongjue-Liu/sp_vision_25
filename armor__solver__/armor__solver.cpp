#include<iostream>
#include <vector>
#include <limits>
#include <yaml-cpp/yaml.h>
#include"armor__solve.hpp"

namespace HNU_NHS_Vision::auto_aim{
constexpr double LIGHTBAR_LENGTH = 56e-3;     
constexpr double BIG_ARMOR_WIDTH = 230e-3;    
constexpr double SMALL_ARMOR_WIDTH = 135e-3;  //定义基本物理尺寸

const std::vector<cv::Point3f> BIG_ARMOR_POINTS{
  {0, BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, BIG_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}};
const std::vector<cv::Point3f> SMALL_ARMOR_POINTS{
  {0, SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, LIGHTBAR_LENGTH / 2},
  {0, -SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2},
  {0, SMALL_ARMOR_WIDTH / 2, -LIGHTBAR_LENGTH / 2}}; //定义角点坐标
//这个函数在创建 Solver 对象时运行，负责加载所有必要的配置文件。
Solver::Solver(const std::string & config_path) 
  : R_gimbal2world_(Eigen::Matrix3d::Identity())
{
  try {
    auto yaml = YAML::LoadFile(config_path);  // 文件不存在会在这里抛出

    // 检查必要字段是否存在
    const std::vector<std::string> required_keys = {
      "R_gimbal2imubody", "R_camera2gimbal", "t_camera2gimbal",
      "camera_matrix", "distort_coeffs"
    };
    for (const auto & key : required_keys) {
      if (!yaml[key]) {
        throw std::runtime_error("Missing key in config: " + key);
      }
    }

    auto R_gimbal2imubody_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
    auto R_camera2gimbal_data  = yaml["R_camera2gimbal"].as<std::vector<double>>();
    auto t_camera2gimbal_data  = yaml["t_camera2gimbal"].as<std::vector<double>>();

    // 检查数据长度是否正确（3x3矩阵需要9个元素）
    if (R_gimbal2imubody_data.size() != 9 || R_camera2gimbal_data.size() != 9) {
      throw std::runtime_error("Rotation matrix data must have 9 elements");
    }
    if (t_camera2gimbal_data.size() != 3) {
      throw std::runtime_error("Translation vector must have 3 elements");
    }

    R_gimbal2imubody_ = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_gimbal2imubody_data.data());
    R_camera2gimbal_  = Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(R_camera2gimbal_data.data());
    t_camera2gimbal_  = Eigen::Matrix<double, 3, 1>(t_camera2gimbal_data.data());

    auto camera_matrix_data  = yaml["camera_matrix"].as<std::vector<double>>();
    auto distort_coeffs_data = yaml["distort_coeffs"].as<std::vector<double>>();

    if (camera_matrix_data.size() != 9) {
      throw std::runtime_error("camera_matrix must have 9 elements");
    }

    Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_matrix(camera_matrix_data.data());
    Eigen::Matrix<double, 1, 5> distort_coeffs(distort_coeffs_data.data());
    cv::eigen2cv(camera_matrix, camera_matrix_);
    cv::eigen2cv(distort_coeffs, distort_coeffs_);

  } catch (const YAML::BadFile & e) {
    throw std::runtime_error("Cannot open config file: " + config_path);
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(std::string("YAML parse error: ") + e.what());
  }
  // std::runtime_error 不拦截，让上层调用者处理
}

Eigen::Matrix3d Solver::R_gimbal2world() const { return R_gimbal2world_; }
//此函数用于更新 R_gimbal2world_ 矩阵
void Solver::set_R_gimbal2world(const Eigen::Quaterniond & q)
{
  Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
  R_gimbal2world_ =R_imubody2imuabs * R_gimbal2imubody_;// R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;做出修改
}
//主解算函数
void Solver::solve(Armor & armor) const
{
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  cv::Vec3d rvec, tvec;
  bool success = cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);
  if (!success) return;
  //坐标变换链
  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  //armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  //armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  //armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
  armor.ypr_in_gimbal = Eigen::Vector3d::Zero();
  armor.ypr_in_world  = Eigen::Vector3d::Zero();
  armor.ypr_in_gimbal[0] = std::atan2(R_armor2gimbal(1, 0), R_armor2gimbal(0, 0));
  armor.ypr_in_world[0]  = std::atan2(R_armor2world(1, 0),  R_armor2world(0, 0));
  // 平衡不做yaw优化，因为pitch假设不成立
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);
  if (is_balance) return;

  optimize_yaw(armor);
}
//重投影，用于验证姿态解算的准确性和优化特定参数
std::vector<cv::Point2f> Solver::reproject_armor(
  const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const
{
  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);
                                  //计算假设的姿态矩阵 R_armor2world
  auto pitch = (name == ArmorName::outpost) ? -15.0 * CV_PI / 180.0 : 15.0 * CV_PI / 180.0;
  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  
  const Eigen::Matrix3d R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };

  //坐标变换到相机系
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  //转换为 OpenCV 格式
  cv::Vec3d rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, rvec);
  cv::Vec3d tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // 执行重投影
  std::vector<cv::Point2f> image_points;
  const auto & object_points = (type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;
  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, distort_coeffs_, image_points);
  return image_points;
}

double Solver::outpost_reprojection_error(Armor armor, const double & pitch) const
{
  // solve
  const auto & object_points =
    (armor.type == ArmorType::big) ? BIG_ARMOR_POINTS : SMALL_ARMOR_POINTS;

  cv::Vec3d rvec, tvec;
  bool success = cv::solvePnP(
    object_points, armor.points, camera_matrix_, distort_coeffs_, rvec, tvec, false,
    cv::SOLVEPNP_IPPE);
  if (!success) return std::numeric_limits<double>::max();

  Eigen::Vector3d xyz_in_camera;
  cv::cv2eigen(tvec, xyz_in_camera);
  armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
  armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;

  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);
  Eigen::Matrix3d R_armor2camera;
  cv::cv2eigen(rmat, R_armor2camera);
  Eigen::Matrix3d R_armor2gimbal = R_camera2gimbal_ * R_armor2camera;
  Eigen::Matrix3d R_armor2world = R_gimbal2world_ * R_armor2gimbal;
  //armor.ypr_in_gimbal = tools::eulers(R_armor2gimbal, 2, 1, 0);
  //armor.ypr_in_world = tools::eulers(R_armor2world, 2, 1, 0);

  //armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);

  //auto yaw = armor.ypr_in_world[0];
  double yaw = std::atan2(R_armor2world(1, 0), R_armor2world(0, 0));
  auto xyz_in_world = armor.xyz_in_world;

  auto sin_yaw = std::sin(yaw);
  auto cos_yaw = std::cos(yaw);

  auto sin_pitch = std::sin(pitch);
  auto cos_pitch = std::cos(pitch);

  // clang-format off
  const Eigen::Matrix3d _R_armor2world {
    {cos_yaw * cos_pitch, -sin_yaw, cos_yaw * sin_pitch},
    {sin_yaw * cos_pitch,  cos_yaw, sin_yaw * sin_pitch},
    {         -sin_pitch,        0,           cos_pitch}
  };
  // clang-format on

  // get R_armor2camera t_armor2camera
  const Eigen::Vector3d & t_armor2world = xyz_in_world;
  Eigen::Matrix3d _R_armor2camera =
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose() * _R_armor2world;
  Eigen::Vector3d t_armor2camera =
    R_camera2gimbal_.transpose() * (R_gimbal2world_.transpose() * t_armor2world - t_camera2gimbal_);

  // get rvec tvec
  cv::Vec3d _rvec;
  cv::Mat R_armor2camera_cv;
  cv::eigen2cv(_R_armor2camera, R_armor2camera_cv);
  cv::Rodrigues(R_armor2camera_cv, _rvec);
  cv::Vec3d _tvec(t_armor2camera[0], t_armor2camera[1], t_armor2camera[2]);

  // reproject
  std::vector<cv::Point2f> image_points;
  cv::projectPoints(object_points, _rvec, _tvec, camera_matrix_, distort_coeffs_, image_points);

  auto error = 0.0;
  for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  return error;
}

void Solver::optimize_yaw(Armor & armor) const
{
  double gimbal_yaw = std::atan2(R_gimbal2world_(1, 0), R_gimbal2world_(0, 0));

  auto limit_rad = [](double rad) -> double {
    while (rad >  CV_PI) rad -= 2 * CV_PI;
    while (rad < -CV_PI) rad += 2 * CV_PI;
    return rad;
  };

  constexpr double SEARCH_RANGE = 140;  // degree
  double yaw0 = limit_rad(gimbal_yaw - SEARCH_RANGE / 2 * CV_PI / 180.0);
  double min_error = 1e10;
  double best_yaw = gimbal_yaw;  // 用云台yaw作为初始值

  for (int i = 0; i < SEARCH_RANGE; i++) {
    double yaw = limit_rad(yaw0 + i * CV_PI / 180.0);
    double inclined = (i - SEARCH_RANGE / 2) * CV_PI / 180.0;
    double error = armor_reprojection_error(armor, yaw, inclined);
    if (error < min_error) {
      min_error = error;
      best_yaw = yaw;
    }
  }

  armor.yaw_raw = best_yaw;        // 记录优化前的原始值（暂时也是best_yaw）
  armor.ypr_in_world[0] = best_yaw;
}

double Solver::armor_reprojection_error(
  const Armor & armor, double yaw, const double & inclined) const
{
  auto image_points = reproject_armor(armor.xyz_in_world, yaw, armor.type, armor.name);
  //auto error = 0.0;
  //for (int i = 0; i < 4; i++) error += cv::norm(armor.points[i] - image_points[i]);
  // auto error = SJTU_cost(image_points, armor.points, inclined);
   return SJTU_cost(image_points, armor.points, inclined);
}

double Solver::SJTU_cost(
  const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
  const double & inclined) const
{
  auto square = [](double x) { return x * x; };

  auto get_abs_angle = [](const Eigen::Vector2d & a, const Eigen::Vector2d & b) -> double {
    double cos_angle = a.dot(b) / (a.norm() * b.norm());
    cos_angle = std::clamp(cos_angle, -1.0, 1.0);
    return std::abs(std::acos(cos_angle));
  };

  std::size_t size = cv_refs.size();
  std::vector<Eigen::Vector2d> refs;
  std::vector<Eigen::Vector2d> pts;
  for (std::size_t i = 0u; i < size; ++i) {
    refs.emplace_back(cv_refs[i].x, cv_refs[i].y);
    pts.emplace_back(cv_pts[i].x, cv_pts[i].y);
  }

  double cost = 0.0;
  for (std::size_t i = 0u; i < size; ++i) {
    std::size_t p = (i + 1u) % size;

    Eigen::Vector2d ref_d = refs[p] - refs[i];
    Eigen::Vector2d pt_d  = pts[p]  - pts[i];

    // 防止除以零（边长为0时跳过）
    if (ref_d.norm() < 1e-6) continue;

    double pixel_dis =
      (0.5 * ((refs[i] - pts[i]).norm() + (refs[p] - pts[p]).norm()) +
       std::fabs(ref_d.norm() - pt_d.norm())) /
      ref_d.norm();

    double angular_dis = get_abs_angle(ref_d, pt_d);

    double cost_i =
      square(pixel_dis  * std::sin(inclined)) +
      square(angular_dis * std::cos(inclined)) * 2.0;

    cost += std::sqrt(cost_i);
  }
  return cost;
}

std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f> & worldPoints) const
{
  // 世界系到相机系的变换
  Eigen::Matrix3d R_world2camera = 
    R_camera2gimbal_.transpose() * R_gimbal2world_.transpose();
  Eigen::Vector3d t_world2camera = 
    -R_camera2gimbal_.transpose() * t_camera2gimbal_;
  // 即：R_camera2gimbal^T * (-t_camera2gimbal_)，世界原点在gimbal系下偏移为0

  // 过滤相机背后的点
  std::vector<cv::Point3f> valid_points;
  for (const auto & wp : worldPoints) {
    Eigen::Vector3d p(wp.x, wp.y, wp.z);
    Eigen::Vector3d p_in_camera = R_world2camera * p + t_world2camera;
    if (p_in_camera.z() > 0) {
      valid_points.push_back(wp);
    }
  }

  if (valid_points.empty()) return {};

  // 旋转矩阵转旋转向量
  cv::Mat R_cv, rvec, tvec;
  cv::eigen2cv(R_world2camera, R_cv);
  cv::Rodrigues(R_cv, rvec);
  cv::eigen2cv(t_world2camera, tvec);

  std::vector<cv::Point2f> pixel_points;
  cv::projectPoints(valid_points, rvec, tvec, camera_matrix_, distort_coeffs_, pixel_points);
  return pixel_points;
}
}
