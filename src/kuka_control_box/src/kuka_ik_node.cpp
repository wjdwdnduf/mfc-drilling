#include "rclcpp/rclcpp.hpp"
#include "kuka_control_box_srvs/srv/kuka_transform_input.hpp"
#include "kuka_control_box_srvs/srv/kuka_joint.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "Eigen/Dense"
#include "kuka_control_box/Robotics.h"
#include <iostream>
#include "sensor_msgs/msg/joint_state.hpp"   // joint_states 구독
#include <optional>                          // std::optional

using namespace std::chrono_literals;
class KukaIKServiceServer : public rclcpp::Node
{
public:
    KukaIKServiceServer() : Node("kuka_ik_node")
    {
        // IK 계산을 위한 서비스 서버 생성
        service_ = this->create_service<kuka_control_box_srvs::srv::KukaTransformInput>(
            "kuka_transform_input",
            std::bind(&KukaIKServiceServer::compute_ik_callback, this, std::placeholders::_1, std::placeholders::_2));

        // 실제 로봇에 조인트 명령을 보내기 위한 서비스 클라이언트 생성
        client_ = this->create_client<kuka_control_box_srvs::srv::KukaJoint>("kuka_joint");

        // Joint 클라이언트가 준비될 때까지 대기
        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "서비스 대기 중에 인터럽트가 발생했습니다. 종료합니다.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "kuka_joint 서비스가 준비될 때까지 대기 중...");
        }

        // joint_states 구독 → current_theta_ 저장
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            [this](const sensor_msgs::msg::JointState::SharedPtr msg)
            {
                if (msg->position.size() >= 6) {
                    current_theta_.emplace(6);
                    for (int i = 0; i < 6; ++i)
                        (*current_theta_)[i] = msg->position[i];
                }
            });


        RCLCPP_INFO(this->get_logger(), "서비스 서버가 준비되었으며 요청을 기다리고 있습니다.");
    }

private:
    void compute_ik_callback(
        const std::shared_ptr<kuka_control_box_srvs::srv::KukaTransformInput::Request> request,
        std::shared_ptr<kuka_control_box_srvs::srv::KukaTransformInput::Response> response)
    {
        // 기본적으로 성공 상태로 설정
        response->success = true;
        // joint_states 가 아직 없으면 실패 반환
        if (!current_theta_.has_value()) {
            RCLCPP_WARN(this->get_logger(), "joint_states 수신 전입니다.");
            response->success = false;
            return;
        }
        Eigen::VectorXd thetainit = *current_theta_;

        RCLCPP_INFO(this->get_logger(), "서비스 요청 수신 (IK 계산 시작)");

        // 요청에서 Transform 정보를 추출
        const auto &transform = request->target_transform.transform;
        const auto &translation = transform.translation;
        const auto &rotation = transform.rotation;

        double x = translation.x;
        double y = translation.y;
        double z = translation.z;

        Eigen::Quaterniond q(rotation.w, rotation.x, rotation.y, rotation.z);
        Eigen::Matrix3d R = q.toRotationMatrix();

        Eigen::Vector3d P(x, y, z);

        RCLCPP_INFO(this->get_logger(), "IK 계산 중...");
        std::vector<Eigen::VectorXd> solutions = Robotics::IkinSpace(thetainit, R, P);

        if (!solutions.empty())
        {
            // IK 계산 성공 시 각도 값 설정
            RCLCPP_INFO(this->get_logger(), "IK 계산 성공: %ld개의 솔루션 발견됨.", solutions.size());

            std::vector<double> positions_in_degrees(6);
            for (int i = 0; i < 6; ++i)
            {
                positions_in_degrees[i] = solutions[0][i] * (180.0 / M_PI);
                RCLCPP_INFO(this->get_logger(), "Joint %d: %f degrees", i + 1, positions_in_degrees[i]);
            }

            // KukaJoint 서비스에 요청 생성
            auto joint_request = std::make_shared<kuka_control_box_srvs::srv::KukaJoint::Request>();
            joint_request->a1 = positions_in_degrees[0];
            joint_request->a2 = positions_in_degrees[1];
            joint_request->a3 = positions_in_degrees[2];
            joint_request->a4 = positions_in_degrees[3];
            joint_request->a5 = positions_in_degrees[4];
            joint_request->a6 = positions_in_degrees[5];

            RCLCPP_INFO(this->get_logger(), "KukaJoint 서비스에 명령을 보내는 중...");

            // Joint 서버로부터 응답을 기다린 후 성공/실패 처리
            auto future_result = client_->async_send_request(joint_request,
                [this, response](rclcpp::Client<kuka_control_box_srvs::srv::KukaJoint>::SharedFuture result_future)
                {
                    auto result = result_future.get();
                    if (result->success)
                    {
                        RCLCPP_INFO(this->get_logger(), "KukaEkiJointServer: Joint command 성공.");
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "KukaEkiJointServer: Joint command 실패.");
                        response->success = false; // Joint 서버에서 실패 시 실패로 설정
                    }

                    // 응답을 반환하는 시점 로그
                    RCLCPP_INFO(this->get_logger(), "응답을 반환합니다 (성공 여부: %s)", response->success ? "True" : "False");
                });

        }
        else
        {
            // IK 계산 실패 시 바로 실패 반환
            RCLCPP_WARN(this->get_logger(), "IK 솔루션을 찾지 못했습니다.");
            response->success = false; // IK 실패 시 클라이언트에게 실패 반환
            // 실패 시 응답 반환 로그
            RCLCPP_INFO(this->get_logger(), "IK 실패로 인한 즉시 응답 반환 (False)");
        }
    }

    rclcpp::Service<kuka_control_box_srvs::srv::KukaTransformInput>::SharedPtr service_;
    rclcpp::Client<kuka_control_box_srvs::srv::KukaJoint>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    std::optional<Eigen::VectorXd> current_theta_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // MultiThreadedExecutor 사용
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<KukaIKServiceServer>();
    executor.add_node(node);

    executor.spin();

    rclcpp::shutdown();
    return 0;
}
