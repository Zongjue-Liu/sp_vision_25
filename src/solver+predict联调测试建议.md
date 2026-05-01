# solver+predict联调测试建议
## 📄 Auto-Aim 模块测试说明（Solver + Predictor）
### 🎯 一、测试目标

本次测试用于验证以下模块功能是否正常：

1. Solver（PnP解算）是否输出稳定三维坐标
2. Predictor（KF）是否能正确进行状态估计与预测
3. Solver → Predictor 数据链路是否打通
### 🧱 二、测试环境要求
1️⃣ 基础环境
✔ Visual Studio 2022（必须安装 C++ 开发工具）
✔ CMake ≥ 3.10
2️⃣ 第三方依赖
（1）OpenCV
要求：
版本：OpenCV 4.x（当前使用 4.12.0）
路径：D:\opencv\build
必须已完成：
✔ 项目成功编译（无 error）
✔ OpenCV 已配置（DLL 可用）记得配置系统变量！！！！！
✔ yaml-cpp 已接入（或使用无yaml版本）
在根目录创建congfig文件夹，再复制同济的standard3.yaml进去
✔ 可运行 test_solver_predictor.exe
### ▶️ 三、测试运行方式
在 PowerShell 执行：
cd D:\HNU_RM\HNU_NHS_Vision\build\Debug
.\test_solver_predictor.exe
### 📊 四、测试内容与预期结果
4.1 静态目标测试（基础验证）
输入（当前默认）：
armor.points 固定矩形
预期输出：
pos: 稳定（不变化）
vel: 0 0 0
pred: 与 pos 相同
state: DETECTING → TRACKING
判定标准：
✔ 数值稳定
✔ 无 NaN / Inf
✔ 状态能进入 TRACKING
4.2 动态目标测试（关键测试）
修改测试代码：
float dx = i * 5;

armor.points = {
    {600 + dx, 300},
    {680 + dx, 300},
    {680 + dx, 380},
    {600 + dx, 380}
};
预期输出：
pos: 持续变化
vel: 非零（x方向为主）
pred: 大于 pos（向前预测）
state: TRACKING
判定标准：
✔ vel ≠ 0
✔ pred 在 pos 前方
✔ 无明显抖动或跳变
### 🧠 五、常见问题排查
❌ 1. pos 全为 0

原因：

Solver 未正确解算（PnP失败）

排查：

检查点顺序是否正确：
左上 → 右上 → 右下 → 左下
❌ 2. vel 一直为 0

原因：

输入数据没有变化（静态测试）

解决：

改为动态测试（见 4.2）
❌ 3. pred ≈ pos（无预测效果）

原因：

速度为 0 或 KF 未收敛
❌ 4. 数值跳变 / 爆炸

原因：

✔ 点顺序错误
✔ 相机参数异常
✔ 单位不一致
❌ 5. state 一直 DETECTING

原因：

KF 还未稳定（前几帧正常）
### 🧪 六、推荐测试流程
1. 运行程序 → 验证静态输出
2. 修改为动态目标 → 验证预测能力
3. 观察 pos / vel / pred 是否合理
4. 检查 state 是否进入 TRACKING
🚀 七、测试结论标准

测试通过需满足：

✔ Solver 输出稳定三维坐标
✔ Predictor 能估计速度
✔ Predictor 能进行前向预测
✔ 系统状态进入 TRACKING
## 🧠 八、当前系统能力说明

当前模块已实现：

✔ 三维位置解算（PnP）
✔ 状态估计（Kalman Filter）
✔ 目标运动预测
✔ 完整数据链路（Solver → Predictor）
🔜 九、后续扩展方向（非本次测试）
- 接入真实 detect 模块
- 延迟补偿（aimPoint）
- 多目标跟踪
