//
// Created by xiang on 23-1-19.
//

#include "ch4/g2o_types.h"

#include "common/g2o_types.h"

namespace sad
{

EdgeInertial::EdgeInertial(std::shared_ptr<IMUPreintegration> preinteg, const Vec3d& gravity, double weight)
    : preint_(preinteg), dt_(preinteg->dt_)
{
    resize(6);  // 6个关联顶点
    grav_ = gravity;
    setInformation(preinteg->cov_.inverse() * weight);
}

void EdgeInertial::computeError()
{
    auto* p1 = dynamic_cast<const VertexPose*>(_vertices[0]);
    auto* v1 = dynamic_cast<const VertexVelocity*>(_vertices[1]);
    auto* bg1 = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
    auto* ba1 = dynamic_cast<const VertexAccBias*>(_vertices[3]);
    auto* p2 = dynamic_cast<const VertexPose*>(_vertices[4]);
    auto* v2 = dynamic_cast<const VertexVelocity*>(_vertices[5]);

    Vec3d bg = bg1->estimate();
    Vec3d ba = ba1->estimate();

    const SO3 dR = preint_->GetDeltaRotation(bg);
    const Vec3d dv = preint_->GetDeltaVelocity(bg, ba);
    const Vec3d dp = preint_->GetDeltaPosition(bg, ba);

    /// 预积分误差项（4.41）
    const Vec3d er = (dR.inverse() * p1->estimate().so3().inverse() * p2->estimate().so3()).log();
    Mat3d RiT = p1->estimate().so3().inverse().matrix();
    const Vec3d ev = RiT * (v2->estimate() - v1->estimate() - grav_ * dt_) - dv;
    const Vec3d ep = RiT * (p2->estimate().translation() - p1->estimate().translation() - v1->estimate() * dt_ - grav_ * dt_ * dt_ / 2) - dp;
    _error << er, ev, ep;
}

void EdgeInertial::linearizeOplus()
{
    auto* p1 = dynamic_cast<const VertexPose*>(_vertices[0]);
    auto* v1 = dynamic_cast<const VertexVelocity*>(_vertices[1]);
    auto* bg1 = dynamic_cast<const VertexGyroBias*>(_vertices[2]);
    auto* ba1 = dynamic_cast<const VertexAccBias*>(_vertices[3]);
    auto* p2 = dynamic_cast<const VertexPose*>(_vertices[4]);
    auto* v2 = dynamic_cast<const VertexVelocity*>(_vertices[5]);

    Vec3d bg = bg1->estimate();
    Vec3d ba = ba1->estimate();
    Vec3d dbg = bg - preint_->bg_;

    // 一些中间符号
    const SO3 R1 = p1->estimate().so3();
    const SO3 R1T = R1.inverse();
    const SO3 R2 = p2->estimate().so3();

    auto dR_dbg = preint_->dR_dbg_;
    auto dv_dbg = preint_->dV_dbg_;
    auto dp_dbg = preint_->dP_dbg_;
    auto dv_dba = preint_->dV_dba_;
    auto dp_dba = preint_->dP_dba_;

    // 估计值
    Vec3d vi = v1->estimate();
    Vec3d vj = v2->estimate();
    Vec3d pi = p1->estimate().translation();
    Vec3d pj = p2->estimate().translation();

    const SO3 dR = preint_->GetDeltaRotation(bg);
    const SO3 eR = SO3(dR).inverse() * R1T * R2;
    const Vec3d er = eR.log();
    const Mat3d invJr = SO3::jr_inv(eR);
    Mat3d RiT = p1->estimate().so3().inverse().matrix();
    const Vec3d dv = preint_->GetDeltaVelocity(bg, ba);
    const Vec3d dp = preint_->GetDeltaPosition(bg, ba);

    /// 雅可比矩阵
    /// 注意有3个index, 顶点的，自己误差的，顶点内部变量的
    /// 变量顺序：pose1(R1,p1), v1, bg1, ba1, pose2(R2,p2), v2
    /// 残差顺序：eR, ev, ep，残差顺序为行，变量顺序为列

    //       | R1 | p1 | v1 | bg1 | ba1 | R2 | p2 | v2 |
    //  vert | 0       | 1  | 2   | 3   | 4       | 5  |
    //  col  | 0    3  | 0  | 0   | 0   | 0    3  | 0  |
    //    row
    //  eR 0 |
    //  ev 3 |
    //  ep 6 |

    /// 残差对R1, 9x3
    _jacobianOplus[0].setZero();
    // dR/dR1, 4.42
    _jacobianOplus[0].block<3, 3>(0, 0) = -invJr * (R2.inverse() * R1).matrix();
    // dv/dR1, 4.47
    _jacobianOplus[0].block<3, 3>(3, 0) = SO3::hat(R1T * (vj - vi - grav_ * dt_));
    // dp/dR1, 4.48d
    _jacobianOplus[0].block<3, 3>(6, 0) = SO3::hat(R1T * (pj - pi - v1->estimate() * dt_ - 0.5 * grav_ * dt_ * dt_));

    /// 残差对p1, 9x3
    // dp/dp1, 4.48a
    _jacobianOplus[0].block<3, 3>(6, 3) = -R1T.matrix();

    /// 残差对v1, 9x3
    _jacobianOplus[1].setZero();
    // dv/dv1, 4.46a
    _jacobianOplus[1].block<3, 3>(3, 0) = -R1T.matrix();
    // dp/dv1, 4.48c
    _jacobianOplus[1].block<3, 3>(6, 0) = -R1T.matrix() * dt_;

    /// 残差对bg1
    _jacobianOplus[2].setZero();
    // dR/dbg1, 4.45
    _jacobianOplus[2].block<3, 3>(0, 0) = -invJr * eR.inverse().matrix() * SO3::jr((dR_dbg * dbg).eval()) * dR_dbg;
    // dv/dbg1
    _jacobianOplus[2].block<3, 3>(3, 0) = -dv_dbg;
    // dp/dbg1
    _jacobianOplus[2].block<3, 3>(6, 0) = -dp_dbg;

    /// 残差对ba1
    _jacobianOplus[3].setZero();
    // dv/dba1
    _jacobianOplus[3].block<3, 3>(3, 0) = -dv_dba;
    // dp/dba1
    _jacobianOplus[3].block<3, 3>(6, 0) = -dp_dba;

    /// 残差对pose2
    _jacobianOplus[4].setZero();
    // dr/dr2, 4.43
    _jacobianOplus[4].block<3, 3>(0, 0) = invJr;
    // dp/dp2, 4.48b
    _jacobianOplus[4].block<3, 3>(6, 3) = R1T.matrix();

    /// 残差对v2
    _jacobianOplus[5].setZero();
    // dv/dv2, 4,46b
    _jacobianOplus[5].block<3, 3>(3, 0) = R1T.matrix();  // OK

    Eigen::Matrix<double, 9, 24> J;
    J.block<9, 6>(0, 0) = _jacobianOplus[0];
    J.block<9, 3>(0, 6) = _jacobianOplus[1];
    J.block<9, 3>(0, 9) = _jacobianOplus[2];
    J.block<9, 3>(0, 12) = _jacobianOplus[3];
    J.block<9, 6>(0, 15) = _jacobianOplus[4];
    J.block<9, 3>(0, 21) = _jacobianOplus[5];

    const double eps = 1e-6;
    /// 残差对R1, 9x3
    // dR/dR1, 4.42
    auto dR_dR1_error_J = [&]()
    {
        Eigen::Matrix<double, 3, 3> numerical_jacobian;
        // 计算每个分量上的变化率
        for (int i = 0; i < 3; ++i)
        {
            Eigen::Vector3d perturbation_R = Eigen::Vector3d::Zero();
            perturbation_R(i) = eps;

            const SO3 R1_1 = R1 * Sophus::SO3d::exp(perturbation_R);
            const SO3 eR1 = SO3(dR).inverse() * (R1_1.inverse()) * R2;
            const Vec3d er1 = eR1.log();

            const SO3 R1_2 = R1 * Sophus::SO3d::exp(-perturbation_R);
            const SO3 eR2 = SO3(dR).inverse() * (R1_2.inverse()) * R2;
            const Vec3d er2 = eR2.log();

            Vec3d dv = er1 - er2;
            numerical_jacobian.col(i) = dv / (2 * eps);

            LOG(INFO) << std::endl
                      << "dR/dR1: "
                      << std::endl
                      << "Analytical Jacobian is"
                      << std::endl
                      << -invJr * (R2.inverse() * R1).matrix()
                      << std::endl
                      << "Numerical Jacobian is"
                      << std::endl
                      << numerical_jacobian
                      << std::endl;
        }
    };
    // dv/dR1, 4.47
    auto dv_dR1_error_J = [&]()
    {
        Eigen::Matrix<double, 3, 3> numerical_jacobian;
        // 计算每个分量上的变化率
        for (int i = 0; i < 3; ++i)
        {
            Eigen::Vector3d perturbation_R = Eigen::Vector3d::Zero();
            perturbation_R(i) = eps;

            const SO3 R1_1 = R1 * Sophus::SO3d::exp(perturbation_R);
            Mat3d RiT1 = R1_1.inverse().matrix();
            const Vec3d ev1 = RiT1 * (v2->estimate() - v1->estimate() - grav_ * dt_) - dv;

            const SO3 R1_2 = R1 * Sophus::SO3d::exp(-perturbation_R);
            Mat3d RiT2 = R1_2.inverse().matrix();
            const Vec3d ev2 = RiT2 * (v2->estimate() - v1->estimate() - grav_ * dt_) - dv;

            Vec3d dv = ev1 - ev2;
            numerical_jacobian.col(i) = dv / (2 * eps);

            LOG(INFO) << std::endl
                      << "dv/dR1: "
                      << std::endl
                      << "Analytical Jacobian is"
                      << std::endl
                      << SO3::hat(R1T * (vj - vi - grav_ * dt_))
                      << std::endl
                      << "Numerical Jacobian is"
                      << std::endl
                      << numerical_jacobian
                      << std::endl;
        }
    };
    // dp/dR1, 4.48d
    auto dp_dR1_error_J = [&]()
    {
        Eigen::Matrix<double, 3, 3> numerical_jacobian;
        // 计算每个分量上的变化率
        for (int i = 0; i < 3; ++i)
        {
            Eigen::Vector3d perturbation_R = Eigen::Vector3d::Zero();
            perturbation_R(i) = eps;

            const SO3 R1_1 = R1 * Sophus::SO3d::exp(perturbation_R);
            Mat3d RiT1 = R1_1.inverse().matrix();
            const Vec3d ep1 = RiT1 * (p2->estimate().translation() - p1->estimate().translation() - v1->estimate() * dt_ - grav_ * dt_ * dt_ / 2) - dp;

            const SO3 R1_2 = R1 * Sophus::SO3d::exp(-perturbation_R);
            Mat3d RiT2 = R1_2.inverse().matrix();
            const Vec3d ep2 = RiT2 * (p2->estimate().translation() - p1->estimate().translation() - v1->estimate() * dt_ - grav_ * dt_ * dt_ / 2) - dp;

            Vec3d dv = ep1 - ep2;
            numerical_jacobian.col(i) = dv / (2 * eps);

            LOG(INFO) << std::endl
                      << "dp/dR1: "
                      << std::endl
                      << "Analytical Jacobian is"
                      << std::endl
                      << SO3::hat(R1T * (pj - pi - v1->estimate() * dt_ - 0.5 * grav_ * dt_ * dt_))
                      << std::endl
                      << "Numerical Jacobian is"
                      << std::endl
                      << numerical_jacobian
                      << std::endl;
        }
    };

    /// 残差对p1, 9x3
    // dp/dp1, 4.48a
    auto dp_dp1_error_J = [&]()
    {
        Eigen::Matrix<double, 3, 3> numerical_jacobian;
        // 计算每个分量上的变化率
        for (int i = 0; i < 3; ++i)
        {
            Vec3d p1_1 = p1->estimate().translation();
            p1_1(i) += eps;
            const Vec3d ep1 = RiT * (p2->estimate().translation() - p1_1 - v1->estimate() * dt_ - grav_ * dt_ * dt_ / 2) - dp;

            Vec3d p1_2 = p1->estimate().translation();
            p1_2(i) -= eps;
            const Vec3d ep2 = RiT * (p2->estimate().translation() - p1_2 - v1->estimate() * dt_ - grav_ * dt_ * dt_ / 2) - dp;

            Vec3d dv = ep1 - ep2;
            numerical_jacobian.col(i) = dv / (2 * eps);
        }

        LOG(INFO) << std::endl
                  << "dp/dp1: "
                  << std::endl
                  << "Analytical Jacobian is"
                  << std::endl
                  << -R1T.matrix()
                  << std::endl
                  << "Numerical Jacobian is"
                  << std::endl
                  << numerical_jacobian
                  << std::endl;
    };

    /// 残差对v1, 9x3
    // dv/dv1, 4.46a
    auto dv_dv1_error_J = [&]()
    {
        Eigen::Matrix<double, 3, 3> numerical_jacobian;
        // 计算每个分量上的变化率
        for (int i = 0; i < 3; ++i)
        {
            Vec3d v1_1 = v1->estimate();
            v1_1(i) += eps;
            const Vec3d ev1 = RiT * (v2->estimate() - v1_1 - grav_ * dt_) - dv;

            Vec3d v1_2 = v1->estimate();
            v1_2(i) -= eps;
            const Vec3d ev2 = RiT * (v2->estimate() - v1_2 - grav_ * dt_) - dv;

            Vec3d dv = ev1 - ev2;
            numerical_jacobian.col(i) = dv / (2 * eps);
        }

        LOG(INFO) << std::endl
                  << "dv/dv1: "
                  << std::endl
                  << "Analytical Jacobian is"
                  << std::endl
                  << -R1T.matrix()
                  << std::endl
                  << "Numerical Jacobian is"
                  << std::endl
                  << numerical_jacobian
                  << std::endl;
    };
    // dp/dv1, 4.48c
    auto dp_dv1_error_J = [&]()
    {
        Eigen::Matrix<double, 3, 3> numerical_jacobian;
        // 计算每个分量上的变化率
        for (int i = 0; i < 3; ++i)
        {
            Vec3d v1_1 = v1->estimate();
            v1_1(i) += eps;
            const Vec3d ep1 = RiT * (p2->estimate().translation() - p1->estimate().translation() - v1_1 * dt_ - grav_ * dt_ * dt_ / 2) - dp;

            Vec3d v1_2 = v1->estimate();
            v1_2(i) -= eps;
            const Vec3d ep2 = RiT * (p2->estimate().translation() - p1->estimate().translation() - v1_2 * dt_ - grav_ * dt_ * dt_ / 2) - dp;

            Vec3d dv = ep1 - ep2;
            numerical_jacobian.col(i) = dv / (2 * eps);
        }

        LOG(INFO) << std::endl
                  << "dp/dv1: "
                  << std::endl
                  << "Analytical Jacobian is"
                  << std::endl
                  << -R1T.matrix() * dt_
                  << std::endl
                  << "Numerical Jacobian is"
                  << std::endl
                  << numerical_jacobian
                  << std::endl;
    };

    /// 残差对bg1
    // dR/dbg1, 4.45
    auto dR_dbg1_error_J = [&]()
    {
        Eigen::Matrix<double, 3, 3> numerical_jacobian;
        // 计算每个分量上的变化率
        for (int i = 0; i < 3; ++i)
        {
            Vec3d bg1 = bg;
            bg1(i) += eps;
            const SO3 dR1 = preint_->GetDeltaRotation(bg1);
            const Vec3d er1 = (dR1.inverse() * p1->estimate().so3().inverse() * p2->estimate().so3()).log();

            Vec3d bg2 = bg;
            bg2(i) -= eps;
            const SO3 dR2 = preint_->GetDeltaRotation(bg2);
            const Vec3d er2 = (dR2.inverse() * p1->estimate().so3().inverse() * p2->estimate().so3()).log();

            Vec3d dv = er1 - er2;
            numerical_jacobian.col(i) = dv / (2 * eps);
        }

        LOG(INFO) << std::endl
                  << "dR/dbg1: "
                  << std::endl
                  << "Analytical Jacobian is"
                  << std::endl
                  << -invJr * eR.inverse().matrix() * SO3::jr((dR_dbg * dbg).eval()) * dR_dbg
                  << std::endl
                  << "Numerical Jacobian is"
                  << std::endl
                  << numerical_jacobian
                  << std::endl;
    };
    // dv/dbg1
    auto dv_dbg1_error_J = [&]()
    {
        Eigen::Matrix<double, 3, 3> numerical_jacobian;
        // 计算每个分量上的变化率
        for (int i = 0; i < 3; ++i)
        {
            Vec3d bg1 = bg;
            bg1(i) += eps;
            const Vec3d dv1 = preint_->GetDeltaVelocity(bg1, ba);
            const Vec3d ev1 = RiT * (v2->estimate() - v1->estimate() - grav_ * dt_) - dv1;

            Vec3d bg2 = bg;
            bg2(i) -= eps;
            const Vec3d dv2 = preint_->GetDeltaVelocity(bg2, ba);
            const Vec3d ev2 = RiT * (v2->estimate() - v1->estimate() - grav_ * dt_) - dv2;

            Vec3d dv = ev1 - ev2;
            numerical_jacobian.col(i) = dv / (2 * eps);
        }

        LOG(INFO) << std::endl
                  << "dv/dbg1: "
                  << std::endl
                  << "Analytical Jacobian is"
                  << std::endl
                  << -dv_dbg
                  << std::endl
                  << "Numerical Jacobian is"
                  << std::endl
                  << numerical_jacobian
                  << std::endl;
    };
    // dp/dbg1
    auto dp_dbg1_error_J = [&]()
    {
        Eigen::Matrix<double, 3, 3> numerical_jacobian;
        // 计算每个分量上的变化率
        for (int i = 0; i < 3; ++i)
        {
            Vec3d bg1 = bg;
            bg1(i) += eps;
            const Vec3d dp1 = preint_->GetDeltaPosition(bg1, ba);
            const Vec3d ep1 = RiT * (p2->estimate().translation() - p1->estimate().translation() - v1->estimate() * dt_ - grav_ * dt_ * dt_ / 2) - dp1;

            Vec3d bg2 = bg;
            bg2(i) -= eps;
            const Vec3d dp2 = preint_->GetDeltaPosition(bg2, ba);
            const Vec3d ep2 = RiT * (p2->estimate().translation() - p1->estimate().translation() - v1->estimate() * dt_ - grav_ * dt_ * dt_ / 2) - dp2;

            Vec3d dv = ep1 - ep2;
            numerical_jacobian.col(i) = dv / (2 * eps);
        }

        LOG(INFO) << std::endl
                  << "dp/dbg1: "
                  << std::endl
                  << "Analytical Jacobian is"
                  << std::endl
                  << -dp_dbg
                  << std::endl
                  << "Numerical Jacobian is"
                  << std::endl
                  << numerical_jacobian
                  << std::endl;
    };

    /// 残差对ba1
    // dv/dba1
    auto dv_dba1_error_J = [&]()
    {
        Eigen::Matrix<double, 3, 3> numerical_jacobian;
        // 计算每个分量上的变化率
        for (int i = 0; i < 3; ++i)
        {
            Vec3d ba1 = ba;
            ba1(i) += eps;
            const Vec3d dv1 = preint_->GetDeltaVelocity(bg, ba1);
            const Vec3d ev1 = RiT * (v2->estimate() - v1->estimate() - grav_ * dt_) - dv1;

            Vec3d ba2 = ba;
            ba2(i) -= eps;
            const Vec3d dv2 = preint_->GetDeltaVelocity(bg, ba2);
            const Vec3d ev2 = RiT * (v2->estimate() - v1->estimate() - grav_ * dt_) - dv2;

            Vec3d dv = ev1 - ev2;
            numerical_jacobian.col(i) = dv / (2 * eps);
        }

        LOG(INFO) << std::endl
                  << "dv/dba1: "
                  << std::endl
                  << "Analytical Jacobian is"
                  << std::endl
                  << -dv_dba
                  << std::endl
                  << "Numerical Jacobian is"
                  << std::endl
                  << numerical_jacobian
                  << std::endl;
    };
    // dp/dba1
    auto dp_dba1_error_J = [&]()
    {
        Eigen::Matrix<double, 3, 3> numerical_jacobian;
        // 计算每个分量上的变化率
        for (int i = 0; i < 3; ++i)
        {
            Vec3d ba1 = ba;
            ba1(i) += eps;
            const Vec3d dp1 = preint_->GetDeltaPosition(bg, ba1);
            const Vec3d ep1 = RiT * (p2->estimate().translation() - p1->estimate().translation() - v1->estimate() * dt_ - grav_ * dt_ * dt_ / 2) - dp1;

            Vec3d ba2 = ba;
            ba2(i) -= eps;
            const Vec3d dp2 = preint_->GetDeltaPosition(bg, ba2);
            const Vec3d ep2 = RiT * (p2->estimate().translation() - p1->estimate().translation() - v1->estimate() * dt_ - grav_ * dt_ * dt_ / 2) - dp2;

            Vec3d dv = ep1 - ep2;
            numerical_jacobian.col(i) = dv / (2 * eps);
        }

        LOG(INFO) << std::endl
                  << "dp/dba1: "
                  << std::endl
                  << "Analytical Jacobian is"
                  << std::endl
                  << -dp_dba
                  << std::endl
                  << "Numerical Jacobian is"
                  << std::endl
                  << numerical_jacobian
                  << std::endl;
    };

    /// 残差对pose2
    // dr/dr2, 4.43
    // _jacobianOplus[4].block<3, 3>(0, 0) = invJr;
    auto dR_dR2_error_J = [&]()
    {
        Eigen::Matrix<double, 3, 3> numerical_jacobian;
        // 计算每个分量上的变化率
        for (int i = 0; i < 3; ++i)
        {
            Eigen::Vector3d perturbation_R = Eigen::Vector3d::Zero();
            perturbation_R(i) = eps;

            const SO3 R2_1 = R2 * Sophus::SO3d::exp(perturbation_R);
            const SO3 eR1 = SO3(dR).inverse() * R1T * R2_1;
            const Vec3d er1 = eR1.log();

            const SO3 R2_2 = R2 * Sophus::SO3d::exp(-perturbation_R);
            const SO3 eR2 = SO3(dR).inverse() * R1T * R2_2;
            const Vec3d er2 = eR2.log();

            Vec3d dv = er1 - er2;
            numerical_jacobian.col(i) = dv / (2 * eps);

            LOG(INFO) << std::endl
                      << "dR/dR2: "
                      << std::endl
                      << "Analytical Jacobian is"
                      << std::endl
                      << invJr
                      << std::endl
                      << "Numerical Jacobian is"
                      << std::endl
                      << numerical_jacobian
                      << std::endl;
        }
    };
    // dp/dp2, 4.48b
    auto dp_dp2_error_J = [&]()
    {
        Eigen::Matrix<double, 3, 3> numerical_jacobian;
        // 计算每个分量上的变化率
        for (int i = 0; i < 3; ++i)
        {
            Vec3d p2_1 = p2->estimate().translation();
            p2_1(i) += eps;
            const Vec3d ep1 = RiT * (p2_1 - p1->estimate().translation() - v1->estimate() * dt_ - grav_ * dt_ * dt_ / 2) - dp;

            Vec3d p2_2 = p2->estimate().translation();
            p2_2(i) -= eps;
            const Vec3d ep2 = RiT * (p2_2 - p1->estimate().translation() - v1->estimate() * dt_ - grav_ * dt_ * dt_ / 2) - dp;

            Vec3d dv = ep1 - ep2;
            numerical_jacobian.col(i) = dv / (2 * eps);
        }

        LOG(INFO) << std::endl
                  << "dp/dp2: "
                  << std::endl
                  << "Analytical Jacobian is"
                  << std::endl
                  << R1T.matrix()
                  << std::endl
                  << "Numerical Jacobian is"
                  << std::endl
                  << numerical_jacobian
                  << std::endl;
    };

    /// 残差对v2
    // dv/dv2, 4,46b
    auto dv_dv2_error_J = [&]()
    {
        Eigen::Matrix<double, 3, 3> numerical_jacobian;
        // 计算每个分量上的变化率
        for (int i = 0; i < 3; ++i)
        {
            Vec3d v2_1 = v2->estimate();
            v2_1(i) += eps;
            const Vec3d ev1 = RiT * (v2_1 - v1->estimate() - grav_ * dt_) - dv;

            Vec3d v2_2 = v2->estimate();
            v2_2(i) -= eps;
            const Vec3d ev2 = RiT * (v2_2 - v1->estimate() - grav_ * dt_) - dv;

            Vec3d dv = ev1 - ev2;
            numerical_jacobian.col(i) = dv / (2 * eps);
        }

        LOG(INFO) << std::endl
                  << "dv/dv2: "
                  << std::endl
                  << "Analytical Jacobian is"
                  << std::endl
                  << R1T.matrix()
                  << std::endl
                  << "Numerical Jacobian is"
                  << std::endl
                  << numerical_jacobian
                  << std::endl;
    };

    // dR_dR1_error_J();
    // dv_dR1_error_J();
    // dp_dR1_error_J();

    // dp_dp1_error_J();

    // dv_dv1_error_J();
    // dp_dv1_error_J();

    // dR_dbg1_error_J();
    // dv_dbg1_error_J();
    // dp_dbg1_error_J();

    // dv_dba1_error_J();
    // dp_dba1_error_J();

    // dR_dR2_error_J();
    // dp_dp2_error_J();

    // dv_dv2_error_J();
}

}  // namespace sad