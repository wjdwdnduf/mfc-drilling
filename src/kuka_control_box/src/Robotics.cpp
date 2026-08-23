#include "kuka_control_box/Robotics.h"
#include <cmath>
#include <iostream>

namespace Robotics {
    
    bool NearZero(double val) {
        return std::abs(val) < 1e-6;
    }

    VectorXd so3ToVec(const MatrixXd& so3mat) {
        VectorXd omg(3);
        omg << so3mat(2, 1), so3mat(0, 2), so3mat(1, 0);
        return omg;
    }

    VectorXd se3ToVec(const MatrixXd& se3mat) {
        VectorXd V(6);
        V << se3mat(2, 1), se3mat(0, 2), se3mat(1, 0),
            se3mat(0, 3), se3mat(1, 3), se3mat(2, 3);
        return V;
    }

    MatrixXd VecToso3(const VectorXd& omg) {
        MatrixXd so3mat(3, 3);
        so3mat << 0, -omg(2), omg(1),
                omg(2), 0, -omg(0),
                -omg(1), omg(0), 0;
        return so3mat;
    }

    MatrixXd VecTose3(const VectorXd& V) {
        MatrixXd se3mat = MatrixXd::Zero(4, 4);
        se3mat.block<3, 3>(0, 0) = VecToso3(V.head(3));
        se3mat.block<3, 1>(0, 3) = V.tail(3);
        return se3mat;
    }

    MatrixXd MatrixExp3(const MatrixXd& so3mat) {
        VectorXd omgtheta = so3ToVec(so3mat);
        MatrixXd R = MatrixXd::Identity(3, 3);
        if (NearZero(omgtheta.norm())) {
            return R;
        } else {
            double theta = omgtheta.norm();
            MatrixXd omgmat = so3mat / theta;
            R = R + sin(theta) * omgmat + (1 - cos(theta)) * omgmat * omgmat;
        }
        return R;
    }

    MatrixXd MatrixExp6(const MatrixXd& se3mat) {
        VectorXd omgtheta = so3ToVec(se3mat.block<3, 3>(0, 0));
        MatrixXd T = MatrixXd::Identity(4, 4);
        if (NearZero(omgtheta.norm())) {
            T.block<3, 1>(0, 3) = se3mat.block<3, 1>(0, 3);
        } else {
            double theta = omgtheta.norm();
            MatrixXd omgmat = se3mat.block<3, 3>(0, 0) / theta;
            T.block<3, 3>(0, 0) = MatrixExp3(se3mat.block<3, 3>(0, 0));
            T.block<3, 1>(0, 3) = (Matrix3d::Identity() * theta + (1 - cos(theta)) * omgmat + (theta - sin(theta)) * omgmat * omgmat) * se3mat.block<3, 1>(0, 3) / theta;
        }
        return T;
    }

    MatrixXd FKinBody(const MatrixXd& M, const MatrixXd& Blist, const VectorXd& thetalist) {
        MatrixXd T = M;
        for (int i = 0; i < thetalist.size(); ++i) {
            T = T * MatrixExp6(VecTose3(Blist.col(i) * thetalist(i)));
        }
        return T;
    }

    MatrixXd FKinSpace(const MatrixXd& M, const MatrixXd& Slist, const VectorXd& thetalist) {
        MatrixXd T = M;
        for (int i = thetalist.size() - 1; i >= 0; --i) {
            T = MatrixExp6(VecTose3(Slist.col(i) * thetalist(i))) * T;
        }
        return T;
    }

    MatrixXd Adjoint(const MatrixXd& T) {
        MatrixXd R = T.block<3, 3>(0, 0);
        MatrixXd AdT = MatrixXd::Zero(6, 6);
        VectorXd p = T.block<3, 1>(0, 3);
        MatrixXd K = VecToso3(p) * R;
        AdT.block<3, 3>(0, 0) = R;
        AdT.block<3, 3>(0, 3) = MatrixXd::Zero(3, 3);
        AdT.block<3, 3>(3, 0) = K;
        AdT.block<3, 3>(3, 3) = R;
        return AdT;
    }

    MatrixXd JacobianBody(const MatrixXd& Blist, const VectorXd& thetalist) {
        int num_joints = Blist.cols();
        MatrixXd Jb = Blist;
        MatrixXd T = MatrixXd::Identity(4, 4);
        for (int i = num_joints - 2; i >= 0; i--) {
            VectorXd alist = (-1) * Blist.col(i + 1) * thetalist(i + 1);
            T = T * MatrixExp6(VecTose3(alist));
            Jb.col(i) = Adjoint(T) * Blist.col(i);
        }
        return Jb;
    }

    MatrixXd JacobianSpace(const MatrixXd& Slist, const VectorXd& thetalist) {
        int n = thetalist.size();
        MatrixXd Js = Slist;
        Matrix4d T = Matrix4d::Identity();

        for (int i = 1; i < n; ++i) {
            VectorXd se3vec = Slist.col(i - 1) * thetalist(i - 1);
            T = T * MatrixExp6(VecTose3(se3vec));
            Js.col(i) = Adjoint(T) * Slist.col(i);
        }

        return Js;
    }

    Matrix4d TransInv(const Matrix4d& T) {
        Matrix3d R = T.block<3, 3>(0, 0);
        Vector3d p = T.block<3, 1>(0, 3);

        Matrix4d T_inv = Matrix4d::Identity();
        T_inv.block<3, 3>(0, 0) = R.transpose();
        T_inv.block<3, 1>(0, 3) = -R.transpose() * p;

        return T_inv;
    }


    Matrix3d MatrixLog3(const Matrix3d& R) {
        double acos_input = (R.trace() - 1.0) / 2.0;

        if (acos_input >= 1.0) {
            return Matrix3d::Zero();  // Identity rotation
        }
        else if (acos_input <= -1.0) {
            Vector3d omg;
            if (!NearZero(1 + R(2,2))) {
                omg << R(0,2), R(1,2), 1 + R(2,2);
            } else if (!NearZero(1 + R(1,1))) {
                omg << R(0,1), 1 + R(1,1), R(2,1);
            } else {
                omg << 1 + R(0,0), R(1,0), R(2,0);
            }
            omg.normalize();
            return VecToso3(M_PI * omg);
        }
        else {
            double theta = std::acos(std::clamp(acos_input, -1.0, 1.0));
            return theta / (2.0 * std::sin(theta)) * (R - R.transpose());
        }
    }


    Matrix4d MatrixLog6(const Matrix4d& T) {
        Matrix3d R = T.block<3,3>(0,0);
        Vector3d p = T.block<3,1>(0,3);
        Matrix3d omgmat = MatrixLog3(R);

        Matrix4d se3mat = Matrix4d::Zero();

        if (omgmat.isZero(1e-6)) {
            // Pure translation case
            se3mat.block<3,1>(0,3) = p;
        } else {
            double theta = std::acos(std::clamp((R.trace() - 1.0) / 2.0, -1.0, 1.0));
            Matrix3d term = Matrix3d::Identity() - omgmat / 2.0
                + ((1.0 / theta) - (0.5 / tan(theta / 2.0))) * (omgmat * omgmat) / theta;

            se3mat.block<3,3>(0,0) = omgmat;
            se3mat.block<3,1>(0,3) = term * p;
        }

        return se3mat;
    }
    //--------------------------------------------------------------
    // helper: joint 각도를 범위(min,max) 안에서 reference(초기값)에
    //          가장 가까운 등가 각도로 wrap
    //--------------------------------------------------------------
    inline double wrapToNearest(double angle,
                                double reference,
                                double min_rad,
                                double max_rad)
    {
        const double TWO_PI = 2.0 * M_PI;

        // reference에 가장 가까운 2π 배수만큼 먼저 이동
        double wrapped = angle + std::round((reference - angle) / TWO_PI) * TWO_PI;

        // 아직 범위 밖이면 한두 번 더 밀어서 집어넣기
        while (wrapped < min_rad) wrapped += TWO_PI;
        while (wrapped > max_rad) wrapped -= TWO_PI;

        // 그래도 벗어나면 최종 클램프
        return std::clamp(wrapped, min_rad, max_rad);
    }

    /* ======  Helper : (R,P) -> 4×4 변환행렬  ===================== */
    static inline Eigen::Matrix4d makeT(const Eigen::Matrix3d& R,
                                        const Eigen::Vector3d& P)
    {
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = P;
        return T;
    }

    /* ======  Step‑Scaling IK (joint‑limit 포함) =================== */
    std::vector<Eigen::VectorXd> IkinSpace(const Eigen::VectorXd& thetainit,
                                        const Eigen::Matrix3d& R,
                                        const Eigen::Vector3d& P)
    {
        using namespace Eigen;
        constexpr double EPS_POS = 1e-3;   // 1 mm
        constexpr double EPS_ATT = 1e-3;   // ~0.06°
        constexpr int    MAX_IT  = 10000;
        constexpr double KAPPA   = 0.25;   // 스텝 감쇠

        /* joint limits (rad) */
        std::vector<std::pair<double,double>> jl = {
            {-185,185}, {-140, 5}, {-120,168},
            {-350,350}, {-122.5,122.5}, {-350,350}};
        for (auto& p: jl){ p.first*=M_PI/180; p.second*=M_PI/180; }

        /* 고정 M, Slist */
        Matrix4d M;
        M << 0,0,-1,2.94,
            0,1, 0,0,
            1,0, 0,0.76,
            0,0, 0,1;

        Matrix<double,6,6> S;
        S.col(0) << 0,0,-1,    0,    0,    0;
        S.col(1) << 0,1, 0,-0.645,   0, 0.330;
        S.col(2) << 0,1, 0,-0.645,   0, 1.480;
        S.col(3) <<-1,0, 0,    0,-0.76,   0;
        S.col(4) << 0,1, 0,-0.76,   0, 2.7;
        S.col(5) <<-1,0, 0,    0,-0.76,   0;

        /* 반복 */
        VectorXd th = thetainit;
        for (int it=0; it<MAX_IT; ++it)
        {
            Matrix4d Tsb = FKinSpace(M,S,th);
            VectorXd Vs  = Adjoint(Tsb) *
                        se3ToVec(MatrixLog6(TransInv(Tsb)*makeT(R,P)));

            if (Vs.head<3>().norm()<EPS_POS && Vs.tail<3>().norm()<EPS_ATT)
                break;                          // 수렴

            VectorXd dth = KAPPA *
                JacobianSpace(S,th).completeOrthogonalDecomposition().solve(Vs);

            /* step‑scaling */
            double alpha = 1.0;
            for (int j=0;j<6;++j){
                double next = th[j] + dth[j];
                if (next < jl[j].first && dth[j] < 0)
                    alpha = std::min(alpha,
                            (jl[j].first - th[j]) / dth[j]);
                else if (next > jl[j].second && dth[j] > 0)
                    alpha = std::min(alpha,
                            (jl[j].second - th[j]) / dth[j]);
            }
            th += alpha * dth;
        }
        return {th};       // 한 솔루션
    }








    std::vector<VectorXd> kuka_ik(const Matrix3d& R06, const Vector3d& p) {
        // 링크 파라미터 초기화
        VectorXd D(6);
        D << 0.645, 0, 0, 1.220, 0, 0.240;
        VectorXd A(6);
        A << -0.330, 1.150, -0.115, 0, 0, 0;

        // 솔루션 저장용 벡터
        std::vector<VectorXd> sols;
        Vector3d w6(0, 0, -D(5));

        // 시작 계산
        Vector3d c = p - R06 * w6;
        double px = c(0);
        double py = c(1);
        double pz = c(2);

        double d1 = D(0);
        double L2 = A(1);
        double L3 = std::sqrt(D(3) * D(3) + A(2) * A(2));

        // theta1 계산 (2개)
        double theta1_1 = std::atan2(py, px);
        double theta1_2 = std::atan2(-py, -px);
        double c1_1 = std::cos(theta1_1), s1_1 = std::sin(theta1_1);
        double c1_2 = std::cos(theta1_2), s1_2 = std::sin(theta1_2);

        theta1_1 = -theta1_1;
        theta1_2 = -theta1_2;

        // theta3 계산
        double c3_1 = ((px - 0.330 * c1_1) * (px - 0.330 * c1_1) +
                       (py - 0.330 * s1_1) * (py - 0.330 * s1_1) +
                       (pz - d1) * (pz - d1) - L2 * L2 - L3 * L3) / (2 * L2 * L3);

        if (c3_1 > 1) {
            c3_1 = ((px + 0.330 * c1_1) * (px + 0.330 * c1_1) +
                    (py + 0.330 * s1_1) * (py + 0.330 * s1_1) +
                    (pz - d1) * (pz - d1) - L2 * L2 - L3 * L3) / (2 * L2 * L3);
        }

        double s3_1 = std::sqrt(1 - c3_1 * c3_1);
        double theta3_1_1 = std::atan2(s3_1, c3_1);
        double theta3_1_2 = std::atan2(-s3_1, c3_1);
        double theta3_1_3 = -theta3_1_1;
        double theta3_1_4 = -theta3_1_2;

        double c3_1_1 = std::cos(theta3_1_1);
        double s3_1_1 = std::sin(theta3_1_1);
        double c3_1_2 = std::cos(theta3_1_2);
        double s3_1_2 = std::sin(theta3_1_2);


        theta3_1_1 = theta3_1_1 + std::atan2(0.115, 1.220);
        theta3_1_2 = theta3_1_2 + std::atan2(0.115, 1.220);
        theta3_1_3 = theta3_1_3 + std::atan2(0.115, 1.220);
        theta3_1_4 = theta3_1_4 + std::atan2(0.115, 1.220);

        double c3_2 = ((px - 0.330 * c1_2) * (px - 0.330 * c1_2) +
                       (py - 0.330 * s1_2) * (py - 0.330 * s1_2) +
                       (pz - d1) * (pz - d1) - L2 * L2 - L3 * L3) / (2 * L2 * L3);

        if (c3_2 > 1) {
            c3_2 = ((px + 0.330 * c1_2) * (px + 0.330 * c1_2) +
                    (py + 0.330 * s1_2) * (py + 0.330 * s1_2) +
                    (pz - d1) * (pz - d1) - L2 * L2 - L3 * L3) / (2 * L2 * L3);
        }

        double s3_2 = std::sqrt(1 - c3_2 * c3_2);
        double theta3_2_1 = std::atan2(s3_2, c3_2);
        double theta3_2_2 = std::atan2(-s3_2, c3_2);
        double theta3_2_3 = -theta3_2_1;
        double theta3_2_4 = -theta3_2_2;

        double c3_2_1 = std::cos(theta3_2_1);
        double s3_2_1 = std::sin(theta3_2_1);
        double c3_2_2 = std::cos(theta3_2_2);
        double s3_2_2 = std::sin(theta3_2_2);

        theta3_2_1 = theta3_2_1 + std::atan2(0.115, 1.220);
        theta3_2_2 = theta3_2_2 + std::atan2(0.115, 1.220);
        theta3_2_3 = theta3_2_3 + std::atan2(0.115, 1.220);
        theta3_2_4 = theta3_2_4 + std::atan2(0.115, 1.220);

        // theta2 계산 및 솔루션 추가
        std::vector<VectorXd> theta(4, VectorXd(3));
        Matrix2d A2;
        Vector2d B2;

        // theta1_1, theta3_1_1
        A2 << L2 + L3 * c3_1_1, L3 * s3_1_1,
              L3 * s3_1_1, -(L2 + L3 * c3_1_1);
        B2 << c1_1 * px + s1_1 * py - 0.330,
              pz - d1;
        Vector2d c2s2 = A2.colPivHouseholderQr().solve(B2);
        double theta2 = std::atan2(c2s2(1), c2s2(0));
        theta[0] << theta1_1, theta2, theta3_1_3;

        // theta1_1, theta3_1_2
        A2 << L2 + L3 * c3_1_2, L3 * s3_1_2,
              L3 * s3_1_2, -(L2 + L3 * c3_1_2);
        c2s2 = A2.colPivHouseholderQr().solve(B2);
        theta2 = std::atan2(c2s2(1), c2s2(0));
        theta[1] << theta1_1, theta2, theta3_1_4;

        // theta1_2, theta3_2_1
        A2 << L2 + L3 * c3_2_1, L3 * s3_2_1,
              L3 * s3_2_1, -(L2 + L3 * c3_2_1);
        B2 << c1_2 * px + s1_2 * py - 0.330,
              pz - d1;
        c2s2 = A2.colPivHouseholderQr().solve(B2);
        theta2 = std::atan2(c2s2(1), c2s2(0));
        theta[2] << theta1_2, theta2, theta3_2_3;

        // theta1_2, theta3_2_2
        A2 << L2 + L3 * c3_2_2, L3 * s3_2_2,
              L3 * s3_2_2, -(L2 + L3 * c3_2_2);
        c2s2 = A2.colPivHouseholderQr().solve(B2);
        theta2 = std::atan2(c2s2(1), c2s2(0));
        theta[3] << theta1_2, theta2, theta3_2_4;

        for (int i = 0; i < 4; ++i) {
            VectorXd thetalist = theta[i];
            if (thetalist(0) <= 185 * M_PI / 180 && thetalist(0) >= -185 * M_PI / 180 &&
                thetalist(1) <= -5 * M_PI / 180 && thetalist(1) >= -140 * M_PI / 180 &&
                thetalist(2) <= 168 * M_PI / 180 && thetalist(2) >= -120 * M_PI / 180) {
                sols.push_back(thetalist);
            }
        }

        // 전방 운동학 계산을 위한 변환 행렬 및 스크류 리스트 초기화
        Matrix4d M;
        M << 0, 0, -1, 2.7,
             0, 1, 0, 0,
             1, 0, 0, 0.76,
             0, 0, 0, 1;

        Matrix<double, 6, 1> S1, S2, S3;
        S1 << 0, 0, -1, 0, 0, 0;
        S2 << 0, 1, 0, -0.645, 0, 0.330;
        S3 << 0, 1, 0, -0.645, 0, 1.480;


        Matrix<double, 6, 3> Slist;
        Slist << S1, S2, S3;

        // theta4, theta5, theta6 계산
        std::vector<VectorXd> f_sols;
        for (const auto& sol : sols) {
            Matrix4d T03 = FKinSpace(M, Slist, sol);
            Matrix3d R03 = T03.block<3, 3>(0, 0);
            Matrix3d R36 = R03.transpose() * R06;

            double theta4_1 = std::atan2(R36(1, 2), R36(0, 2));
            double theta5_1 = std::atan2(std::sqrt(R36(1, 2) * R36(1, 2) + R36(0, 2) * R36(0, 2)), R36(2, 2));
            double theta6_1 = std::atan2(R36(2, 1), -R36(2, 0))-2 * M_PI;

            // 첫 번째 솔루션 검증 및 저장
            if (theta4_1 <= 350 * M_PI / 180 && theta4_1 >= -350 * M_PI / 180 &&
                theta5_1 <= 122.5 * M_PI / 180 && theta5_1 >= -122.5 * M_PI / 180 &&
                theta6_1 <= 350 * M_PI / 180 && theta6_1 >= -350 * M_PI / 180) {
                VectorXd full_sol(6);
                full_sol << sol(0), sol(1), sol(2), theta4_1, theta5_1, theta6_1;
                f_sols.push_back(full_sol);
                break;
            }

            // 두 번째 솔루션 계산 및 검증
            double theta4_2 = theta4_1 + M_PI;
            double theta5_2 = -theta5_1;
            double theta6_2 = theta6_1 + 3* M_PI;

            if (theta4_2 <= 350 * M_PI / 180 && theta4_2 >= -350 * M_PI / 180 &&
                theta5_2 <= 122.5 * M_PI / 180 && theta5_2 >= -122.5 * M_PI / 180 &&
                theta6_2 <= 350 * M_PI / 180 && theta6_2 >= -350 * M_PI / 180) {
                VectorXd full_sol(6);
                full_sol << sol(0), sol(1), sol(2), theta4_2, theta5_2, theta6_2;
                f_sols.push_back(full_sol);
                break;
            }


        }

        return f_sols;
    }
}
