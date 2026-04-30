# 解算模块开发记录

**项目**：27赛季自瞄框架 — HNU_NHS_Vision  
**模块**：空间位置解算（Solver）  
**文件**：`armor__solve.cpp` / `armor__solve.hpp`  
**依赖库**：OpenCV、Eigen3、yaml-cpp

---

## 模块职责

Solver 负责将识别模块输出的装甲板图像角点，解算为空间三维坐标和姿态角，供 tracker 和 planner 使用。

### 坐标变换链

相机系 → 云台系 → 世界系（IMU惯性系）

### 输出字段

| 字段 | 含义 | 单位 |
|------|------|------|
| `xyz_in_gimbal` | 装甲板在云台系下的位置 | m |
| `xyz_in_world` | 装甲板在世界系下的位置 | m |
| `ypr_in_gimbal[0]` | 装甲板在云台系下的 yaw | rad |
| `ypr_in_world[0]` | 装甲板在世界系下的 yaw（优化后） | rad |
| `yaw_raw` | yaw 优化前的原始值 | rad |

---

## 函数说明

### `Solver(const std::string & config_path)`

构造函数，从 YAML 文件加载相机内参、畸变系数及各坐标系旋转矩阵。

**YAML 需包含字段：**

```yaml
R_gimbal2imubody: [9个数，3x3行优先]
R_camera2gimbal:  [9个数，3x3行优先]
t_camera2gimbal:  [3个数，单位m]
camera_matrix:    [9个数，3x3行优先]
distort_coeffs:   [5个数，k1,k2,p1,p2,k3]
```

---

### `set_R_gimbal2world(const Eigen::Quaterniond & q)`

根据 IMU 四元数更新云台到世界系的旋转矩阵。

```cpp
R_gimbal2world_ = R_imubody2imuabs * R_gimbal2imubody_;
```

---

### `solve(Armor & armor)`

主解算函数，流程：

1. `solvePnP`（IPPE 方法）解算装甲板 3D 位置
2. 坐标变换：相机系 → 云台系 → 世界系
3. 提取 yaw 角（atan2 从旋转矩阵直接提取）
4. 平衡步兵跳过 yaw 优化；其余调用 `optimize_yaw`

---

### `optimize_yaw(Armor & armor)`

在云台 yaw 附近 ±70° 范围内（共 140 步，步长 1°）搜索最优 yaw，使重投影误差最小。

---

### `reproject_armor(...)`

给定世界系位置和 yaw，构造假设姿态矩阵，将装甲板角点重投影回图像。pitch 固定假设：

- 哨兵：-15°
- 其他：+15°

---

### `armor_reprojection_error(armor, yaw, inclined)`

调用 `reproject_armor` 后用 `SJTU_cost` 计算误差，供 `optimize_yaw` 使用。

---

### `SJTU_cost(cv_refs, cv_pts, inclined)`

精细重投影误差代价函数，对每条边分别计算像素位置误差和角度误差，用 `inclined`（装甲板倾斜角）加权融合：

```
cost_i = (pixel_dis × sin(inclined))² + (angular_dis × cos(inclined))² × 2
```

- 正对相机时：角度误差权重大
- 侧对相机时：位置误差权重大

---

### `outpost_reprojection_error(armor, pitch)`

哨兵专用代价函数，pitch 作为待优化参数传入，用于搜索最优 pitch。

---

### `world2pixel(worldPoints)`

将世界系下的一批 3D 点投影到图像像素坐标，自动过滤相机背后（z < 0）的点。

---

## 修改记录

### Bug 修复

| 问题 | 原因 | 修复方式 |
|------|------|---------|
| `set_R_gimbal2world` 旋转矩阵计算错误 | 原式多乘一项，形成相似变换，物理意义不对 | 改为 `R_imubody2imuabs * R_gimbal2imubody_` |
| `solvePnP` 未检查返回值 | 失败时 rvec/tvec 为垃圾值，导致错误传播 | 接收返回值，失败时直接 return |
| `ypr` 赋值顺序错误 | 先赋 yaw 后清零，yaw 被覆盖 | 改为先 `Zero()` 初始化再赋值 |
| `world2pixel` 传旋转矩阵给 projectPoints | `projectPoints` 需要旋转向量 | 加 `cv::Rodrigues` 转换 |
| `world2pixel` 平移向量计算冗余 | 乘以零向量项多余 | 化简为 `-R_camera2gimbal_.transpose() * t_camera2gimbal_` |
| `SJTU_cost` 缺少除零保护 | `ref_d.norm()` 为零时崩溃 | 加 `< 1e-6` 判断跳过 |

### 依赖替换（tools:: → 标准库）

| 原调用 | 替换方式 |
|--------|---------|
| `tools::eulers(R, 2, 1, 0)` | `std::atan2(R(1,0), R(0,0))` 提取 yaw |
| `tools::xyz2ypd(...)` | 删除（后续模块暂不使用） |
| `tools::limit_rad(rad)` | lambda：while 循环限制到 [-π, π] |
| `tools::get_abs_angle(a, b)` | lambda：`acos(clamp(dot/norms, -1, 1))` |
| `tools::square(x)` | lambda：`x * x` |

### 代码质量

- YAML 构造函数加入完整异常处理（`BadFile`、`Exception`、字段缺失、长度校验）
- 删除旧构造函数注释块
- 删除 `armor_reprojection_error` 中的死代码
- 修正 `outpost` 拼写错误（原为 `oupost`）
- `outpost_reprojection_error` 和 `world2pixel` 补全 `const` 限定

---

## 注意事项

- `armor.center` 不是对角线交点，不能作为实际几何中心使用
- 平衡步兵（3、4、5号大装甲板）跳过 yaw 优化，因为固定 pitch 假设不成立
- YAML 配置文件由相机标定和机械安装测量提供，是解算的基础，缺少时构造函数会抛出异常
- `ypr_in_gimbal` 和 `ypr_in_world` 目前只有 `[0]`（yaw）有意义，`[1]`、`[2]` 初始化为 0
