#include "trajectory.hpp"
#include <cmath>
#include <limits>

namespace tools
{
constexpr double g = 9.7833;

Trajectory::Trajectory(const double v0, const double d, const double h)
{
  auto a = g * d * d / (2 * v0 * v0);
  auto b = -d;
  auto c = a + h;
  auto delta = b * b - 4 * a * c;

  if (delta < 0) {
    unsolvable = true;
    return;
  }

  unsolvable = false;
  auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
  auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  auto pitch_1 = std::atan(tan_pitch_1);
  auto pitch_2 = std::atan(tan_pitch_2);
  auto t_1 = d / (v0 * std::cos(pitch_1));
  auto t_2 = d / (v0 * std::cos(pitch_2));

  pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
  fly_time = (t_1 < t_2) ? t_1 : t_2;
}



TrajectoryWithDrag::TrajectoryWithDrag(double v0, double d, double h, double k);
{

    // 计算飞行时间（核心物理量反解）
    double calcTime(double v0, double d, double k, double theta)
    {
        double cos_t = std::cos(theta); //cos_t可能为0

        double denom = v0 * cos_t; //水平速度分量
        double ratio = k * d / denom; //列公式

        // 防止非法（当ratio大于1时，log(1-ratio)会变成负无穷）
        if (ratio >= 1.0)
            return -1.0;

        return -std::log(1.0 - ratio) / k;
    }

    // 残差函数
    double f(double v0, double d, double h, double k, double theta)
    {
        double t = calcTime(v0, d, k, theta);
        if (t < 0)
            return std::numeric_limits<double>::infinity();

        double z = v0 * std::sin(theta) * t - 0.5 * g * t * t;
        //将非法角度映射成无穷大
        return z - h;
        //返回残差（高度差）
    }

    //开始求解
    void solve(double v0, double d, double h, double k)
    {

        //搜索范围初始化(英雄要增大范围)
        double left = -1;   // -60°
        double right = 1;   // +60°

        bool found = false;

        double prev_theta = left;
        double prev_f = f(v0, d, h, k, prev_theta);

        //遍历100个点，用介值定理，找到合法区间
        for (int i = 1; i <= 100; i++)
        {
            double theta = left + (right - left) * i / 100.0;
            double cur_f = f(v0, d, h, k, theta);

            if (std::isfinite(prev_f) && std::isfinite(cur_f) &&
                prev_f * cur_f <= 0)
            {
                left = prev_theta;
                right = theta;
                found = true;
                break;
            }

            prev_theta = theta;
            prev_f = cur_f;
        }

        if (!found)
        {
            unsolvable = true;
            return;
        }

        //开始暴力的二分 
        for (int i = 0; i < 60; i++)
        {
            double mid = 0.5 * (left + right);//二分区间
            double val = f(v0, d, h, k, mid);//残差

            if (!std::isfinite(val))//如果残差无穷大，判定为非法
            {
                right = mid;//则更新右边界
                continue;//并跳过本次迭代
            }

            if (val > 0)//如果残差大于0，更新右边界
                right = mid;
            else//如果残差小于0，更新左边界
                left = mid;
        }

        pitch = 0.5 * (left + right);//输出结果（这个是合法区间的中点）

        //最后确定飞行时间
        fly_time = calcTime(v0, d, k, pitch);

        if (fly_time < 0)//如果飞行时间小于0
            unsolvable = true;//则输出结果为非法
    }
}


}  // namespace tools