#include "approach/pid_controller.hpp"
#include <cmath>

void PIDController::setParameters(float kp_x, float kp_y, float kp_theta,
                                  float ki_x, float ki_y, float ki_theta,
                                  float kd_x, float kd_y, float kd_theta,
                                  float base_to_rotationcore) {
  this->kp_x = kp_x;
  this->kp_y = kp_y;
  this->kp_theta = kp_theta;
  this->ki_x = ki_x;
  this->ki_y = ki_y;
  this->ki_theta = ki_theta;
  this->kd_x = kd_x;
  this->kd_y = kd_y;
  this->kd_theta = kd_theta;
  this->base_to_rotationcore = base_to_rotationcore;
}

geometry_msgs::msg::Twist
PIDController::compute_control(const SE2Error &se2_error, float dt) {
  geometry_msgs::msg::Twist cmd_vel;
  if (dt <= 0.0f)
    return cmd_vel; // 방어 코드

  // 1. 센서 기준 에러를 회전 중심(CoR) 기준으로 변환
  float e_x = se2_error.x + base_to_rotationcore;
  float e_y = se2_error.y;

  float e_theta = se2_error.degree_theta;

  // 2. PID 연산
  se2_error_sum.x += e_x * dt;
  se2_error_sum.y += e_y * dt;
  se2_error_sum.degree_theta += e_theta * dt;

  float d_x = (e_x - se2_error_prev.x) / dt;
  float d_y = (e_y - se2_error_prev.y) / dt;
  float d_theta = (e_theta - se2_error_prev.degree_theta) / dt;

  se2_error_prev = {e_x, e_y, e_theta};

  float control_x = (kp_x * e_x) + (ki_x * se2_error_sum.x) + (kd_x * d_x);
  float control_y = (kp_y * e_y) + (ki_y * se2_error_sum.y) + (kd_y * d_y);
  float control_theta = (kp_theta * e_theta) +
                        (ki_theta * se2_error_sum.degree_theta) +
                        (kd_theta * d_theta);

  // 3. 기구학 모델(Lyapunov 기반) 방정식 적용
  // 종방향 속도 (단순 거리 오차 제어)
  float v_x = control_x;

  // 회전 속도 (각조향 복구 + 횡방향 복구)
  // 선속도 v_x가 존재할 때 횡방향 오차 제어가 활성화되어 선에 부드럽게 진입하게
  // 함
  float w_z = control_theta + control_y;

  // Limits
  float max_v = 0.05f; // 로봇의 최대 직진 속도 (m/s)
  float max_w = 0.2f;  // 로봇의 최대 회전 속도 (rad/s)
  // 절대값 계산
  float abs_v = std::abs(v_x);
  float abs_w = std::abs(w_z);
  // 각각 한계치 대비 얼마나 초과했는지 비율 계산 (1.0 이하면 정상)
  float scale_v = (abs_v > max_v) ? (abs_v / max_v) : 1.0f;
  float scale_w = (abs_w > max_w) ? (abs_w / max_w) : 1.0f;
  // 두 초과 비율 중 더 큰 것(더 많이 오버한 쪽)을 기준으로 삼기
  float max_scale = std::max(scale_v, scale_w);
  // 비율에 맞춰 두 속도를 똑같이 줄여주기 (곡률 v/w 유지)
  if (max_scale > 1.0f) {
    v_x = v_x / max_scale;
    w_z = w_z / max_scale;
  }
  cmd_vel.linear.x = v_x;
  cmd_vel.angular.z = w_z;

  return cmd_vel;
}
