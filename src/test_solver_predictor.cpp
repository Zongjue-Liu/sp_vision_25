#include <iostream>
#include <chrono>
#include <thread>
#include <Eigen/Dense>

#include "predictor.hpp"
#include "../solve/armor__solver.hpp"

using namespace HNU_NHS_Vision::auto_aim;

int main()
{
    std::cout << "Pipeline test start\n";

    Solver solver("../../config/standard3.yaml");
    Predictor predictor;

    
    // 云台姿态（先用单位四元数）
    solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());

    // 构造一个“假装甲板”（图像中心的小矩形）
    Armor armor;
    armor.type = ArmorType::small;
    armor.name = ArmorName::one;
    armor.points = {
        {600, 300}, // 左上
        {680, 300}, // 右上
        {680, 380}, // 右下
        {600, 380}  // 左下
    };

    for (int i = 0; i < 10; ++i)
    {
        // 1) 解算三维
        solver.solve(armor);

        // 2) 喂给 KF（时间戳用 now）
        auto now = std::chrono::steady_clock::now();
        predictor.update(armor.xyz_in_world, now);

        // 3) 打印
        auto pos = predictor.filteredPos();
        auto vel = predictor.velocity();
        auto pred = predictor.predictedPos(0.05); // 预测50ms后

        std::cout << "frame " << i << "\n";
        std::cout << "pos: " << pos.transpose() << "\n";
        std::cout << "vel: " << vel.transpose() << "\n";
        std::cout << "pred: " << pred.transpose() << "\n";
        std::cout << "state: " << predictor.stateString() << "\n\n";

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "Press Enter to exit...";
    std::cin.get();
    return 0;
}
