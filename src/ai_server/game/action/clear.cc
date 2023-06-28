#include <cmath>
#include "clear.h"
#include "ai_server/util/math/angle.h"
#include "ai_server/util/math/to_vector.h"
#include "ai_server/model/motion/walk_forward.h"
#include "ai_server/model/motion/turn_left.h"
#include "ai_server/model/motion/turn_right.h"

#include <iostream>//omega表示のため
#include "ai_server/util/math/distance.h"//distance表示のため
#include "ai_server/util/math/detail/direction.h"//direction表示のため
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include "ai_server/util/math/geometry.h"
#include <boost/math/constants/constants.hpp>

using namespace std;
namespace ai_server::game::action {
clear::clear(context& ctx, unsigned int id) : base(ctx, id) {}

bool clear::finished() const {
  // 終了条件は設定しないため常に false
  return false;
}
model::command clear::execute() {
 model::command command{};
 // 自チームのロボットの情報を取得するプログラム
 const auto our_robots = model::our_robots(world(), team_color());
 // 自分の情報がなければ終了
 if (!our_robots.count(id_)) return command;
 // 自分の情報を取得
 const auto robot = our_robots.at(id_);
 // 自分とボールの位置を取得
 const Eigen ::Vector2d ene_goal_pos(world().field().x_min(),0.0);
 
 const auto target_0 = ene_goal_pos;
 const auto robot_pos = util::math::position(robot);
 const auto ball_pos = util::math::position(world().ball());      // mw
 
 auto rad = 150;
 auto hiru = 0;
 // 前進
 command.set_motion(std::make_shared<model::motion::walk_forward>());
 // 向きが合っていなければ回転 (前進のモーションはキャンセルされる)
Eigen::Vector2d pos = ball_pos - rad * (ball_pos-target_0).normalized();
 constexpr double rot_th = 0.5;
 auto omega = util::math::direction_from(util::math::direction(pos,robot_pos),robot.theta());
 auto dista = util::math::distance(ball_pos,robot_pos);
 auto dire = util::math::direction(ball_pos,robot_pos);
 if (rot_th < omega ) {
 command.set_motion(std::make_shared<model::motion::turn_left>());
 } else if (omega < -rot_th) {
    command.set_motion(std::make_shared<model::motion::turn_right>());
 }
 cout << "omega" << omega << "\n";
 cout << "2店の距離"  << dista << "\n";
 cout << "2店の角度"  << dire << "\n";
 cout << "pos" << pos <<"\n";
 return command;
}

} // namespace ai_server::game::action