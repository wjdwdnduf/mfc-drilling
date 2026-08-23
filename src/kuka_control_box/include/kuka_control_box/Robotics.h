#ifndef ROBOTICS_H
#define ROBOTICS_H

#include <Eigen/Dense>
#include <vector>

using namespace Eigen;

namespace Robotics {
    bool NearZero(double val);
    VectorXd so3ToVec(const MatrixXd& so3mat);
    VectorXd se3ToVec(const MatrixXd& se3mat);
    MatrixXd VecToso3(const VectorXd& omg);
    MatrixXd VecTose3(const VectorXd& V);
    MatrixXd MatrixExp3(const MatrixXd& so3mat);
    MatrixXd MatrixExp6(const MatrixXd& se3mat);
    MatrixXd FKinBody(const MatrixXd& M, const MatrixXd& Blist, const VectorXd& thetalist);
    MatrixXd FKinSpace(const MatrixXd& M, const MatrixXd& Slist, const VectorXd& thetalist);
    MatrixXd Adjoint(const MatrixXd& T);
    Matrix4d TransInv(const Matrix4d& T);
    Matrix3d MatrixLog3(const Matrix3d& R);
    Matrix4d MatrixLog6(const Matrix4d& T);
    MatrixXd JacobianBody(const MatrixXd& Blist, const VectorXd& thetalist);
    MatrixXd JacobianSpace(const MatrixXd& Slist, const VectorXd& thetalist);
    double wrapToNearest(double angle, double reference, double min_rad, double max_rad);

    // Inverse Kinematics
    std::vector<Eigen::VectorXd> IkinSpace(const Eigen::VectorXd& thetainit,
                                           const Eigen::Matrix3d& R,
                                           const Eigen::Vector3d& P);

    // Joint 각도 범위 내에서 초기값에 가까운 해를 선택
    // double wrapToNearest(double angle, double reference, double min_rad, double max_rad);
    std::vector<VectorXd> kuka_ik(const Matrix3d& R, const Vector3d& P);
}

#endif // ROBOTICS_H
