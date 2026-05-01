# ByteTrack 风格 Tracker 修改记录

本文记录本次基于 ByteTrack 思路对自瞄 `Tracker` 的修改方案、当前已改位置、逻辑前后对比和后续注意事项。

## 1. 修改目标

原来的 tracker 逻辑主要依赖单阶段匹配：

```text
检测器输出 armors
  -> tracker 按 name/type 找当前目标
  -> 找到就 solver_.solve + target_.update
  -> 找不到就 return false
  -> 状态机进入 temp_lost 或 lost
```

问题：

```text
遮挡、运动模糊、曝光异常时，YOLO 可能仍然能输出低置信度框
但如果检测器或 tracker 只接受高分框，这些低分框不会被利用
目标容易提前进入 temp_lost / lost
```

ByteTrack 的核心思想：

```text
不要过早丢掉低置信度检测框
先用高分框匹配已有目标
高分框失败后，再用低分框尝试救回已有目标
低分框只能救旧目标，不能创建新目标
```

本次 tracker 目标：

```text
单阶段 name/type 匹配
  -> 高分框第一阶段匹配
  -> 低分框第二阶段补救匹配
  -> 低分框必须接近 EKF 预测位置
```

## 2. 配置文件修改

文件：

```text
configs/standard3.yaml
```

修改位置：

```text
第 39-41 行
```

新增内容：

```yaml
track_high_thresh: 0.6
track_low_thresh: 0.15
new_target_thresh: 0.7
```

含义：

```text
track_high_thresh
  高分框阈值。大于等于该值的检测结果进入第一阶段匹配。

track_low_thresh
  低分框阈值。大于等于该值但低于 high 阈值的检测结果进入第二阶段补救匹配。

new_target_thresh
  新目标创建阈值。只有大于等于该值的检测结果才能创建新目标。
```

逻辑变化：

```text
原来：
  tracker 不区分高分框和低分框

现在：
  confidence >= 0.6       -> 高分候选
  0.15 <= confidence < 0.6 -> 低分候选
  confidence < 0.15       -> tracker 不使用
  confidence < 0.7        -> 不能创建新目标
```

## 3. tracker.hpp 修改

文件：

```text
tasks/auto_aim/tracker.hpp
```

### 3.1 新增阈值成员变量

修改位置：

```text
第 40-42 行
```

新增代码：

```cpp
double track_high_thresh_;
double track_low_thresh_;
double new_target_thresh_;
```

修改前：

```cpp
int min_detect_count_;
int max_temp_lost_count_;
int detect_count_;
int temp_lost_count_;
int outpost_max_temp_lost_count_;
```

修改后：

```cpp
int min_detect_count_;
int max_temp_lost_count_;
int detect_count_;
int temp_lost_count_;
double track_high_thresh_;
double track_low_thresh_;
double new_target_thresh_;
int outpost_max_temp_lost_count_;
```

作用：

```text
Tracker 内部保存 ByteTrack 两阶段匹配所需的三个阈值
```

### 3.2 新增 match_target 函数声明

修改位置：

```text
第 57 行
```

新增代码：

```cpp
bool match_target(const Armor & armor, bool use_strict_gate) const;
```

作用：

```text
判断候选 armor 是否真的接近当前 EKF 预测的 target_
```

逻辑：

```text
高分框调用 match_target(..., false)
  -> 使用较宽松距离门限

低分框调用 match_target(..., true)
  -> 使用更严格距离门限
```

## 4. tracker.cpp 修改

文件：

```text
tasks/auto_aim/tracker.cpp
```

### 4.1 新增 algorithm 头文件

修改位置：

```text
第 10 行
```

新增代码：

```cpp
#include <algorithm>
```

原因：

```text
match_target 中使用 std::min
建议显式 include <algorithm>
```

当前注意：

```text
当前文件中该 include 前面有一个多余空格，建议后续整理为 #include <algorithm>
```

### 4.2 构造函数读取 ByteTrack 阈值

修改位置：

```text
第 29-36 行
```

新增代码：

```cpp
track_high_thresh_ =
  yaml["track_high_thresh"].IsDefined() ? yaml["track_high_thresh"].as<double>() : 0.6;

track_low_thresh_ =
  yaml["track_low_thresh"].IsDefined() ? yaml["track_low_thresh"].as<double>() : 0.15;

new_target_thresh_ =
  yaml["new_target_thresh"].IsDefined() ? yaml["new_target_thresh"].as<double>() : 0.7;
```

修改前：

```cpp
min_detect_count_ = yaml["min_detect_count"].as<int>();
max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
normal_temp_lost_count_ = max_temp_lost_count_;
```

修改后：

```cpp
min_detect_count_ = yaml["min_detect_count"].as<int>();
max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();

track_high_thresh_ =
  yaml["track_high_thresh"].IsDefined() ? yaml["track_high_thresh"].as<double>() : 0.6;

track_low_thresh_ =
  yaml["track_low_thresh"].IsDefined() ? yaml["track_low_thresh"].as<double>() : 0.15;

new_target_thresh_ =
  yaml["new_target_thresh"].IsDefined() ? yaml["new_target_thresh"].as<double>() : 0.7;

normal_temp_lost_count_ = max_temp_lost_count_;
```

作用：

```text
优先读取 YAML 配置
如果 YAML 没写，则使用默认值
```

### 4.3 set_target 禁止低分框创建新目标

修改位置：

```text
第 243-250 行
```

新增代码：

```cpp
armors.remove_if([this](const Armor & armor) {
  return armor.confidence < new_target_thresh_;
});
```

修改前逻辑：

```text
state_ == lost
  -> 只要 armors 非空
  -> 取 armors.front()
  -> 创建新 Target
```

修改后逻辑：

```text
state_ == lost
  -> 先删除 confidence < new_target_thresh_ 的候选
  -> 只允许高可信候选创建新 Target
```

箭头对比：

```text
原来：
  任意通过检测器的 armor -> 可能创建新目标

现在：
  armor.confidence >= new_target_thresh_ -> 才能创建新目标
  armor.confidence < new_target_thresh_  -> 不能创建新目标
```

原因：

```text
ByteTrack 允许低分框救旧目标
但不允许低分框创建新目标
否则容易误锁低分误检
```

### 4.4 update_target 改为两阶段匹配

修改位置：

```text
第 283-348 行
```

原始逻辑：

```cpp
target_.predict(t);

int found_count = 0;
for (const auto & armor : armors) {
  if (armor.name != target_.name || armor.type != target_.armor_type) continue;
  found_count++;
}

if (found_count == 0) return false;

for (auto & armor : armors) {
  if (armor.name != target_.name || armor.type != target_.armor_type) continue;

  solver_.solve(armor);
  target_.update(armor);
}

return true;
```

原始逻辑流程：

```text
预测 target_
  -> 找 name/type 相同的 armor
  -> 找不到 return false
  -> 找到就全部 update
  -> return true
```

当前新逻辑：

```cpp
target_.predict(t);

std::list<Armor> high_score_armors;
std::list<Armor> low_score_armors;

for (const auto & armor : armors) {
  if (armor.name != target_.name || armor.type != target_.armor_type) continue;

  if (armor.confidence >= track_high_thresh_) {
    high_score_armors.push_back(armor);
  } else if (armor.confidence >= track_low_thresh_) {
    low_score_armors.push_back(armor);
  }
}

for (auto & armor : high_score_armors) {
  solver_.solve(armor);

  if (!match_target(armor, false)) continue;

  target_.update(armor);
  return true;
}

for (auto & armor : low_score_armors) {
  solver_.solve(armor);

  if (!match_target(armor, true)) continue;

  target_.update(armor);
  return true;
}

return false;
```

新逻辑流程：

```text
预测 target_
  -> 遍历当前帧 armors
  -> 只保留 name/type 与当前 target_ 一致的候选
  -> 按 confidence 分成 high_score_armors 和 low_score_armors
  -> 第一轮：尝试高分框
      -> solve
      -> match_target 宽松门限
      -> 成功则 update 并 return true
  -> 第二轮：尝试低分框
      -> solve
      -> match_target 严格门限
      -> 成功则 update 并 return true
  -> 两轮都失败
      -> return false
      -> 状态机进入 temp_lost 逻辑
```

箭头对比：

```text
原来：
  name/type 相同 -> 直接 update

现在：
  name/type 相同
    -> confidence >= high_thresh -> 高分匹配
    -> confidence >= low_thresh  -> 低分补救
    -> 还要通过 match_target 位置门限
```

为什么最后要 `return false`：

```text
update_target 返回 bool

true:
  本帧成功用检测结果更新了目标

false:
  本帧没有找到可用候选
  状态机应该认为目标暂时丢失
```

### 4.5 新增 match_target

修改位置：

```text
第 352-370 行
```

新增代码逻辑：

```cpp
bool Tracker::match_target(const Armor & armor, bool strict) const
{
  if (armor.name != target_.name || armor.type != target_.armor_type) {
    return false;
  }

  auto predicted_armors = target_.armor_xyza_list();

  double min_distance = 1e10;
  for (const auto & predicted_armor : predicted_armors) {
    double distance = (armor.xyz_in_world - predicted_armor.head<3>()).norm();
    min_distance = std::min(min_distance, distance);
  }

  double distance_gate = strict ? 0.35 : 0.7;

  return min_distance < distance_gate;
}
```

作用：

```text
防止低分误检直接更新 target_
```

判断流程：

```text
候选 armor
  -> name/type 必须与当前 target_ 一致
  -> 取 EKF 预测出的所有装甲板位置
  -> 计算候选 armor.xyz_in_world 到预测装甲板的最小 3D 距离
  -> 距离小于门限才认为匹配成功
```

门限解释：

```text
strict == false
  -> 高分框
  -> 使用 0.7 m 门限
  -> 因为模型较可信，允许位置误差稍大

strict == true
  -> 低分框
  -> 使用 0.35 m 门限
  -> 因为低分框误检概率更高，必须更接近预测位置
```

箭头对比：

```text
原来：
  检测框 name/type 一样 -> 直接 update

现在：
  检测框 name/type 一样
    -> solver_.solve 得到 3D 位置
    -> 与 EKF 预测位置比较
    -> 距离足够近才 update
```

当前注意：

```text
当前代码中 match_target 函数头被拆成两行：

bool Tracker::match_target(const Armor & armor, bool strict) 
const

建议后续整理为：

bool Tracker::match_target(const Armor & armor, bool strict) const

以减少编译器/格式化器误判。
```

## 5. 总体前后逻辑对比

### 修改前

```text
track()
  -> 如果 lost
       -> set_target(armors)
       -> 任意排序靠前 armor 可初始化 target
  -> 如果非 lost
       -> update_target(armors)
       -> 只按 name/type 找 armor
       -> 找到直接 update
       -> 找不到 return false
  -> state_machine(found)
```

### 修改后

```text
track()
  -> 如果 lost
       -> set_target(armors)
       -> 删除 confidence < new_target_thresh 的 armor
       -> 只有高可信 armor 可初始化 target
  -> 如果非 lost
       -> update_target(armors)
       -> EKF 先 predict
       -> 按 name/type 过滤候选
       -> confidence >= track_high_thresh 进入高分队列
       -> track_low_thresh <= confidence < track_high_thresh 进入低分队列
       -> 高分队列先匹配
       -> 高分失败后低分队列补救
       -> match_target 检查 3D 位置是否接近预测
       -> 成功 update 并 return true
       -> 全失败 return false
  -> state_machine(found)
```

## 6. 与 YOLO26 的关系

当前 tracker 逻辑已经为低分候选预留了入口。

但如果 YOLO26 的 C++ 后处理仍然提前删除低分框，例如：

```text
score < 0.7 直接丢弃
confidence < 0.8 直接丢弃
```

那么：

```text
low_score_armors 基本会一直为空
第二阶段补救逻辑不会真正发挥作用
```

后续需要和 YOLO26 同学确认：

```text
1. 是否能把低分候选传给 tracker
2. 是否保留 armor.confidence
3. 是否只删除极低分候选，例如 confidence < 0.15
4. NMS 是否会提前删掉遮挡时有用的低分框
```

推荐后处理目标：

```text
confidence < 0.15
  -> 删除

0.15 <= confidence < 0.6
  -> 保留，作为低分补救候选

confidence >= 0.6
  -> 保留，作为高分候选
```

## 7. 当前未完成或需注意事项

1. 当前尚未在 Windows 本地编译。

原因：

```text
当前 PowerShell 环境未安装 cmake/ninja/make
工程依赖 OpenVINO、OpenCV、相机 SDK 等，更适合在 Ubuntu/WSL/机器人开发环境编译
```

2. 当前代码建议在 Ubuntu 编译前整理两处格式：

```cpp
#include <algorithm>
```

以及：

```cpp
bool Tracker::match_target(const Armor & armor, bool strict) const
{
```

3. 旧逻辑注释目前仍保留。

影响：

```text
不影响功能
但影响阅读
后续稳定后建议删除旧注释
```

4. 当前只修改了 `configs/standard3.yaml`。

如果实际运行使用的是其他配置文件，例如：

```text
configs/sentry.yaml
configs/standard4.yaml
configs/uav.yaml
```

也需要同步增加：

```yaml
track_high_thresh: 0.6
track_low_thresh: 0.15
new_target_thresh: 0.7
```

5. 低分候选是否能进入 tracker，取决于 YOLO26 后处理代码是否配合修改。

## 8. 后续 Ubuntu 编译命令

在装好依赖的 Ubuntu/WSL 环境中：

```bash
cmake -S . -B build
cmake --build build -j$(nproc)
```

如果安装了 Ninja：

```bash
cmake -S . -B build -G Ninja
cmake --build build -j$(nproc)
```

如果只想优先检查自瞄模块：

```bash
cmake --build build --target auto_aim -j$(nproc)
```

