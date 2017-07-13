#include <cmath>

#include "ai_server/game/action/receive.h"
#include "ai_server/util/math/to_vector.h"
#include "ai_server/util/math.h"

namespace ai_server {
namespace game {
namespace action {
void receive::set_dribble(int dribble) {
  dribble_ = dribble;
}
int receive::dribble() {
  return dribble_;
}
void receive::set_passer(unsigned int passer_id) {
  passer_id_ = passer_id;
}
unsigned int receive::passer() {
  return passer_id_;
}
model::command receive::execute() {
  //それぞれ自機を生成
  model::command command(id_);

  //ドリブルさせる
  command.set_dribble(dribble_);

  const auto& robots = is_yellow_ ? world_.robots_yellow() : world_.robots_blue();
  if (!robots.count(id_) || !robots.count(passer_id_)) {
    command.set_velocity({0.0, 0.0, 0.0});
    return command;
  }

  const auto& robot      = robots.at(id_);
  const auto robot_pos   = util::math::position(robot);
  const auto robot_theta = util::wrap_to_pi(robot.theta());
  const auto ball_pos    = util::math::position(world_.ball());
  const auto ball_vec    = util::math::velocity(world_.ball());

  const auto& passer      = robots.at(passer_id_);
  const auto passer_pos   = util::math::position(passer);
  const auto passer_theta = util::wrap_to_pi(passer.theta());

  //ボールがめっちゃ近くに来たら受け取ったと判定
  //現状だとボールセンサに反応があるか分からないので
  if ((robot_pos - ball_pos).norm() < 120) {
    flag_ = true;
    command.set_velocity({0.0, 0.0, 0.0});
    return command;
  }

  decltype(util::math::position(passer)) normalize; //正面に移動したい対象の単位ベクトル
  decltype(util::math::position(passer)) position; //正面に移動したい対象の位置

  if (ball_vec.norm() < 500) { // passerの正面に移動したい
    normalize = Eigen::Vector2d{std::cos(passer_theta), std::sin(passer_theta)};
    position  = passer_pos;
  } else { //ボールの移動予測地点に移動したい
    normalize = ball_vec.normalized();
    position  = ball_pos;
  }

  //対象とreceiverの距離
  const auto length = robot_pos - position;
  //内積より,対象と自分の直交する位置
  const auto dot = normalize.dot(length);

  //目標位置と角度
  const auto target  = (position + dot * normalize);
  const auto to_ball = ball_pos - robot_pos;
  const auto theta   = std::atan2(to_ball.y(), to_ball.x());

  //位置から速度へ
  const auto target_vec{(target - robot_pos) * 3};

  const auto omega = theta - robot_theta;
  command.set_velocity({target_vec.x(), target_vec.y(), omega});

  flag_ = false;
  return command;
}
bool receive::finished() const {
  return flag_;
}
}
}
}