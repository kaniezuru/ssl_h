#ifndef AI_SERVER_CONTROLLER_STATE_FEEDBACK_CONTROLLER_H
#define AI_SERVER_CONTROLLER_STATE_FEEDBACK_CONTROLLER_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <vector>

#include "ai_server/controller/detail/smith_predictor.h"
#include "ai_server/controller/detail/velocity_generator.h"
#include "base.h"

namespace ai_server {
namespace controller {

class state_feedback_controller : public base {
  /* ------------------------------------
  ロボットの状態を3*3行列で表す
         x,    vx,    ax
         y,    vy,    ay
     theta, omega, alpha

  各要素を列ベクトルで
  ------------------------------------ */

private:
  double cycle_;                    // 制御周期
  const static double k_;           // 極,収束の速さ
  static const double zeta_;        // モデルパラメータζ
  static const double omega_;       // モデルパラメータω
  Eigen::Vector3d kp_;              // 比例ゲイン(x,y,rotate)
  Eigen::Vector3d ki_;              // 積分ゲイン(x,y,rotate)
  Eigen::Vector3d kd_;              // 微分ゲイン(x,y,rotate)
  Eigen::Matrix3d estimated_robot_; // 推定ロボット状態
  Eigen::Vector3d up_[2];           // 操作量(比例,1フレーム前まで)
  Eigen::Vector3d ui_[2];           // 操作量(積分,1フレーム前まで)
  Eigen::Vector3d ud_[2];           // 操作量(微分,1フレーム前まで)
  Eigen::Vector3d u_[2];            // 操作量(1フレーム前まで)
  Eigen::Vector3d e_[2];            // 偏差(1フレーム前まで)
  detail::velocity_generator velocity_generator_[2];
  detail::smith_predictor smith_predictor_;

  // レギュレータ部
  void calculate_regulator(const model::robot& robot);

  // フィールド基準座標系からロボット基準座標系に変換
  Eigen::Vector3d convert(const Eigen::Vector3d& raw, const double robot_theta);

public:
  // コンストラクタ
  explicit state_feedback_controller(double cycle);

  // 制御入力更新関数
  velocity_t update(const model::robot& robot, const position_t& setpoint);
  velocity_t update(const model::robot& robot, const velocity_t& setpoint);
};

} // controller
} // ai_server

#endif // AI_SERVER_CONTROLLER_STATE_FEEDBACK_CONTROLLER_H
