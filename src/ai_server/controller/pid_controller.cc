#include "pid_controller.h"

namespace ai_server {
namespace controller {

const double pid_controller::kp_ = 1.0;
const double pid_controller::ki_ = 0.1;
const double pid_controller::kd_ = 0.0;

// model::command::velocityの四則演算のオーバロード
const velocity_t operator+(const velocity_t& vel1, const velocity_t& vel2) {
  return {vel1.vx + vel2.vx, vel1.vy + vel2.vy, vel1.omega + vel2.omega};
}

const velocity_t operator-(const velocity_t& vel1, const velocity_t& vel2) {
  return {vel1.vx - vel2.vx, vel1.vy - vel2.vy, vel1.omega - vel2.omega};
}

const velocity_t operator*(const double& c, const velocity_t& vel) {
  return {c * vel.vx, c * vel.vy, c * vel.omega};
}

const velocity_t operator/(const velocity_t& vel, const double& c) {
  return {vel.vx / c, vel.vy / c, vel.omega / c};
}

pid_controller::pid_controller(double cycle) : cycle_(cycle) {
  max_velocity_     = 3000.0;
  max_acceleration_ = 3000.0;
  for (int i = 0; i < 2; i++) {
    up_[i] = {0.0, 0.0, 0.0};
    ui_[i] = {0.0, 0.0, 0.0};
    ud_[i] = {0.0, 0.0, 0.0};
    u_[i]  = {0.0, 0.0, 0.0};
    e_[i]  = {0.0, 0.0, 0.0};
  }
}

velocity_t pid_controller::update(const model::robot& robot, const position_t& setpoint) {
  // 位置偏差
  position_t ep;
  ep.x     = setpoint.x - robot.x();
  ep.y     = setpoint.y - robot.y();
  ep.theta = setpoint.theta - robot.theta();

  // 目標速度
  velocity_t set_velocity;
  set_velocity.vx    = ep.x / cycle_;
  set_velocity.vy    = ep.y / cycle_;
  set_velocity.omega = ep.theta - cycle_;

  // 速度偏差
  e_[0].vx    = set_velocity.vx - robot.vx();
  e_[0].vy    = set_velocity.vy - robot.vy();
  e_[0].omega = set_velocity.omega - robot.omega();

  // 制御計算
  calculate();

  return u_[0];
}

velocity_t pid_controller::update(const model::robot& robot, const velocity_t& setpoint) {
  // 現在偏差
  e_[0].vx    = setpoint.vx - u_[1].vx;
  e_[0].vy    = setpoint.vy - u_[1].vy;
  e_[0].omega = setpoint.omega - u_[1].omega;

  // 制御計算
  calculate();

  return u_[0];
}

void pid_controller::calculate() {
  // 双一次変換
  // s=(2/T)*(Z-1)/(Z+1)としてPIDcontrollerを離散化
  // C=Kp+Ki/s+Kds
  up_[0] = kp_ * e_[0];
  ui_[0] = ki_ * cycle_ * (e_[0] + e_[1]) / 2 + ui_[1];
  ud_[0] = 2 * kd_ * (e_[0] - e_[1]) / cycle_ - ud_[1];
  u_[0]  = u_[1] + cycle_ * (up_[0] + ui_[0] + ud_[0]);

  // 加速度，速度制限
  double u_angle = atan2(u_[0].vy, u_[0].vx);                 // 今回指令速度の方向
  double u_speed = sqrt(pow(u_[0].vx, 2) + pow(u_[0].vy, 2)); // 今回指令速度の大きさ
  double preceding_speed = sqrt(pow(u_[1].vx, 2) + pow(u_[1].vy, 2)); // 前回指令速度の大きさ
  // 加速度制限
  if ((u_speed - preceding_speed) / cycle_ > max_acceleration_) { // 加速時で最大加速度超過
    u_speed = preceding_speed + (max_acceleration_ * cycle_);
  } else if ((preceding_speed - u_speed) / cycle_ <
             -max_acceleration_) { // 減速時で最大加速度超過
    u_speed = preceding_speed - (max_acceleration_ * cycle_);
  }
  // 速度制限
  if (u_speed > max_velocity_) { // 最大速度超過
    u_speed = max_velocity_;
  }
  u_[0].vx = u_speed * cos(u_angle); // 成分速度再計算
  u_[0].vy = u_speed * sin(u_angle);

  // 値の更新
  up_[1] = up_[0];
  ui_[1] = ui_[0];
  ud_[1] = ud_[0];
  u_[1]  = u_[0];
  e_[1]  = e_[0];
}

} // controller
} // ai_server
