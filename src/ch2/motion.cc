/*
 * @Description  :
 * @Author       : zhiwei chen
 * @Date         : 2023-06-04 20:23:33
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2023-06-05 22:35:53
 */
// /*
//  * @Description  :
//  * @Author       : zhiwei chen
//  * @Date         : 2023-05-15 16:19:03
//  * @LastEditors  : zhiwei chen
//  * @LastEditTime : 2023-05-21 20:20:38
//  */
// //
// // Created by xiang on 22-12-29.
// //

// #include <fstream>
// #include <iostream>

// #include <gflags/gflags.h>
// #include <glog/logging.h>

// #include "common/eigen_types.h"
// #include "common/math_utils.h"
// #include "tools/ui/pangolin_window.h"

// /// 本节程序演示一个正在作圆周运动的车辆
// /// 车辆的角速度与线速度可以在flags中设置

// DEFINE_double(angular_velocity, 10.0, "角速度（角度）制");
// DEFINE_double(linear_velocity, 5.0, "车辆前进线速度 m/s");
// DEFINE_bool(use_quaternion, false, "是否使用四元数计算");

// // TAG add gravitational acceleration
// DEFINE_double(g, 9.81, "gravitational acceleration");

// int main(int argc, char** argv)
// {
//     google::InitGoogleLogging(argv[0]);
//     FLAGS_stderrthreshold = google::INFO;
//     FLAGS_colorlogtostderr = true;
//     google::ParseCommandLineFlags(&argc, &argv, true);

//     /// 可视化
//     sad::ui::PangolinWindow ui;
//     if (ui.Init() == false)
//     {
//         return -1;
//     }

//     double angular_velocity_rad = FLAGS_angular_velocity * sad::math::kDEG2RAD;  // 弧度制角速度
//     SE3 pose;                                                                    // TWB表示的位姿
//     Vec3d omega(0, 0, angular_velocity_rad);                                     // 角速度矢量
//     Vec3d v_body(FLAGS_linear_velocity, 0, 0);                                   // 本体系速度
//     const double dt = 0.05;                                                      // 每次更新的时间

//     std::ofstream ofs;
//     std::string filename = "/home/work/workspace/homework/slam_in_autonomous_driving/src/ch2/pose.txt";
//     ofs.open(filename.c_str(), std::ios::out);

//     while (ui.ShouldQuit() == false)
//     {
//         // 更新自身位置
//         // TAG update v_body
//         v_body.z() -= (FLAGS_g * dt);
//         Vec3d v_world = pose.so3() * v_body;
//         pose.translation() += v_world * dt;

//         // 更新自身旋转
//         if (FLAGS_use_quaternion)
//         {
//             Quatd q = pose.unit_quaternion() * Quatd(1, 0.5 * omega[0] * dt, 0.5 * omega[1] * dt, 0.5 * omega[2] * dt);
//             q.normalize();
//             pose.so3() = SO3(q);
//         }
//         else
//         {
//             pose.so3() = pose.so3() * SO3::exp(omega * dt);
//         }

//         LOG(INFO) << "pose: " << pose.translation().transpose();
//         static double time = 0;
//         time += dt;
//         ofs << std::fixed << std::setprecision(5) << time << " " << pose.translation().transpose().x() << " "
//             << pose.translation().transpose().y() << " " << pose.translation().transpose().z()
//             << " " << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
//         ui.UpdateNavState(sad::NavStated(0, pose, v_world));

//         usleep(dt * 1e6);
//     }

//     ui.Quit();
//     return 0;
// }

#include <Eigen/Dense>  // 导入 Eigen 库
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

using namespace Eigen;

enum States
{
    POSITION,
    VELOCITY
};

class KinematicKalman
{
  private:
    Matrix2d A;  // 状态转移矩阵
    Matrix2d H;  // 观测矩阵
    Matrix2d Q;  // 状态转移噪声协方差矩阵
    Matrix2d R;  // 观测噪声协方差矩阵
    Matrix2d P;  // 状态估计误差协方差矩阵
    Vector2d x;  // 状态估计向量

  public:
    KinematicKalman(double dt, double position_variance, double velocity_variance)
    {
        // 初始化卡尔曼滤波器的参数和初始状态
        A << 1, dt,
            0, 1;

        H << 1, 0,
            0, 1;

        Q << pow(dt, 4) / 4, pow(dt, 3) / 2,
            pow(dt, 3) / 2, pow(dt, 2);

        R << position_variance, 0,
            0, velocity_variance;

        P << position_variance, 0,
            0, velocity_variance;

        x << 0, 0;
    }

    void predict()
    {
        x = A * x;
        P = A * P * A.transpose() + Q;
    }

    void update(double measurement)
    {
        Vector2d y = Vector2d(measurement, 0) - H * x;
        Matrix2d S = H * P * H.transpose() + R;
        Matrix2d K = P * H.transpose() * S.inverse();

        x = x + K * y;
        P = (Matrix2d::Identity() - K * H) * P;
    }

    double getPosition() const
    {
        return x(POSITION);
    }
};

int main()
{
    double dt = 0.01;                 // 时间步长
    double position_variance = 0.1;   // 位置观测噪声方差
    double velocity_variance = 0.01;  // 速度观测噪声方差

    double simulation_time = 5.0;  // 模拟时间
    int num_steps = static_cast<int>(simulation_time / dt);

    KinematicKalman kf(dt, position_variance, velocity_variance);

    double x = 0.0;
    double v = 0.0;

    std::vector<double> positions;
    std::vector<double> measurements;
    std::vector<double> kf_estimates;

    for (int i = 0; i < num_steps; ++i)
    {
        positions.push_back(x);

        // 生成模拟观测值
        double measurement = x;
        measurements.push_back(measurement);
        // 记录开始时间
        std::chrono::high_resolution_clock::time_point start_time = std::chrono::high_resolution_clock::now();

        kf.predict();
        kf.update(measurement);

        // 记录结束时间
        std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
        // 计算时间差，以毫秒为单位
        std::chrono::duration<double, std::milli> execution_time = end_time - start_time;

        // 打印运行时间
        std::cout << "代码执行时间：" << std::setprecision(10) << std::fixed << execution_time.count() / 1000 << "秒" << std::endl;

        // 打印预测位置和速度
        double kf_estimate = kf.getPosition();
        kf_estimates.push_back(kf_estimate);

        // 更新模拟状态
        x += v * dt;
        v = std::sin(i * dt * 5.0);
    }

    // // 打印模拟结果
    // for (int i = 0; i < num_steps; ++i)
    // {
    //     std::cout << "Step " << i << ": "
    //               << "True Position = " << positions[i] << ", "
    //               << "Measurement = " << measurements[i] << ", "
    //               << "KF Estimate = " << kf_estimates[i] << std::endl;
    // }

    return 0;
}