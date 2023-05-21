/*
 * @Description  :
 * @Author       : zhiwei chen
 * @Date         : 2023-05-15 16:19:03
 * @LastEditors  : zhiwei chen
 * @LastEditTime : 2023-05-21 20:20:38
 */
//
// Created by xiang on 22-12-29.
//

#include <fstream>
#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/eigen_types.h"
#include "common/math_utils.h"
#include "tools/ui/pangolin_window.h"

/// 本节程序演示一个正在作圆周运动的车辆
/// 车辆的角速度与线速度可以在flags中设置

DEFINE_double(angular_velocity, 10.0, "角速度（角度）制");
DEFINE_double(linear_velocity, 5.0, "车辆前进线速度 m/s");
DEFINE_bool(use_quaternion, false, "是否使用四元数计算");

// TAG add gravitational acceleration
DEFINE_double(g, 9.81, "gravitational acceleration");

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    /// 可视化
    sad::ui::PangolinWindow ui;
    if (ui.Init() == false)
    {
        return -1;
    }

    double angular_velocity_rad = FLAGS_angular_velocity * sad::math::kDEG2RAD;  // 弧度制角速度
    SE3 pose;                                                                    // TWB表示的位姿
    Vec3d omega(0, 0, angular_velocity_rad);                                     // 角速度矢量
    Vec3d v_body(FLAGS_linear_velocity, 0, 0);                                   // 本体系速度
    const double dt = 0.05;                                                      // 每次更新的时间

    std::ofstream ofs;
    std::string filename = "/home/work/workspace/homework/slam_in_autonomous_driving/src/ch2/pose.txt";
    ofs.open(filename.c_str(), std::ios::out);

    while (ui.ShouldQuit() == false)
    {
        // 更新自身位置
        // TAG update v_body
        v_body.z() -= (FLAGS_g * dt);
        Vec3d v_world = pose.so3() * v_body;
        pose.translation() += v_world * dt;

        // 更新自身旋转
        if (FLAGS_use_quaternion)
        {
            Quatd q = pose.unit_quaternion() * Quatd(1, 0.5 * omega[0] * dt, 0.5 * omega[1] * dt, 0.5 * omega[2] * dt);
            q.normalize();
            pose.so3() = SO3(q);
        }
        else
        {
            pose.so3() = pose.so3() * SO3::exp(omega * dt);
        }

        LOG(INFO) << "pose: " << pose.translation().transpose();
        static double time = 0;
        time += dt;
        ofs << std::fixed << std::setprecision(5) << time << " " << pose.translation().transpose().x() << " "
            << pose.translation().transpose().y() << " " << pose.translation().transpose().z()
            << " " << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;
        ui.UpdateNavState(sad::NavStated(0, pose, v_world));

        usleep(dt * 1e6);
    }

    ui.Quit();
    return 0;
}