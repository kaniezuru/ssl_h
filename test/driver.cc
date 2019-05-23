#define BOOST_TEST_DYN_LINK

#include <chrono>
#include <memory>
#include <stdexcept>
#include <vector>
#include <boost/asio.hpp>
#include <boost/test/unit_test.hpp>

#include "ai_server/controller/base.h"
#include "ai_server/driver.h"
#include "ai_server/model/team_color.h"
#include "ai_server/model/updater/world.h"
#include "ai_server/sender/base.h"

#include "ssl-protos/vision/wrapperpacket.pb.h"

using namespace std::chrono_literals;
namespace controller = ai_server::controller;
namespace model      = ai_server::model;
namespace sender     = ai_server::sender;

BOOST_TEST_DONT_PRINT_LOG_VALUE(model::team_color)

BOOST_AUTO_TEST_SUITE(driver)

BOOST_AUTO_TEST_CASE(set_team_color) {
  boost::asio::io_service io_service{};
  ai_server::model::updater::world wu{};

  {
    // コンストラクタでチームカラーが正しく設定されているか
    ai_server::driver d{io_service, 1s, wu, model::team_color::blue};
    BOOST_TEST(d.team_color() == model::team_color::blue);

    // チームカラーが変更できるか
    d.set_team_color(model::team_color::yellow);
    BOOST_TEST(d.team_color() == model::team_color::yellow);
  }

  {
    ai_server::driver d{io_service, 1s, wu, model::team_color::yellow};
    BOOST_TEST(d.team_color() == model::team_color::yellow);

    d.set_team_color(model::team_color::blue);
    BOOST_TEST(d.team_color() == model::team_color::blue);
  }
}

BOOST_AUTO_TEST_CASE(register_robot) {
  boost::asio::io_service io_service{};
  ai_server::model::updater::world wu{};
  ai_server::driver d{io_service, std::chrono::seconds{1}, wu, model::team_color::yellow};

  ai_server::model::command cmd0{0};
  ai_server::model::command cmd1{1};

  // ロボットが登録されていない状態でupdate_command()を呼ぶとエラー
  BOOST_CHECK_THROW(d.update_command(cmd0), std::runtime_error);
  BOOST_CHECK_THROW(d.update_command(cmd1), std::runtime_error);

  // まだ登録されていない
  BOOST_TEST(!d.registered(0));
  BOOST_TEST(!d.registered(1));

  // ID0のロボットを登録
  BOOST_CHECK_NO_THROW(d.register_robot(0, nullptr, nullptr));
  BOOST_CHECK_NO_THROW(d.update_command(cmd0));
  BOOST_TEST(d.registered(0));

  // ID1のロボットを登録
  BOOST_CHECK_NO_THROW(d.register_robot(1, nullptr, nullptr));
  BOOST_CHECK_NO_THROW(d.update_command(cmd1));
  BOOST_TEST(d.registered(1));

  // 登録したロボットが正常に削除されるか
  d.unregister_robot(0);
  BOOST_CHECK_THROW(d.update_command(cmd0), std::runtime_error);
  BOOST_CHECK_NO_THROW(d.update_command(cmd1));
  BOOST_TEST(!d.registered(0));
  d.unregister_robot(1);
  BOOST_CHECK_THROW(d.update_command(cmd1), std::runtime_error);
  BOOST_TEST(!d.registered(1));
}

struct mock_controller : public controller::base {
  bool is_stable() const {
    return stable_flag_;
  }

  bool executed_ = false;

protected:
  controller::velocity_t update(const model::robot&, const controller::position_t&) {
    executed_ = true;
    return {};
  }
  controller::velocity_t update(const model::robot&, const controller::velocity_t& v) {
    executed_ = true;
    return v;
  }
};

struct mock_sender : public sender::base {
  std::vector<model::command> commands_;

  void send_command(const model::command& command) {
    commands_.push_back(command);
  }
};

BOOST_AUTO_TEST_CASE(velocity_limit) {
  boost::asio::io_service io_service{};
  ai_server::model::updater::world wu{};
  ai_server::driver d{io_service, std::chrono::seconds{1}, wu, model::team_color::blue};

  // 適当なControllerをいくつか初期化し登録する
  // 後からアクセスできるように参照を残しておく
  auto c1_ptr    = std::make_unique<mock_controller>();
  auto c2_ptr    = std::make_unique<mock_controller>();
  const auto& c1 = *c1_ptr;
  const auto& c2 = *c2_ptr;
  d.register_robot(1, std::move(c1_ptr), nullptr);
  d.register_robot(2, std::move(c2_ptr), nullptr);

  // 変更前
  BOOST_TEST(c1.velocity_limit() == std::numeric_limits<double>::max());
  BOOST_TEST(c2.velocity_limit() == std::numeric_limits<double>::max());

  // 変更してみる
  d.set_velocity_limit(123);
  BOOST_TEST(c1.velocity_limit() == 123);
  BOOST_TEST(c2.velocity_limit() == 123);

  // さらに変更してみる
  d.set_velocity_limit(456);
  BOOST_TEST(c1.velocity_limit() == 456);
  BOOST_TEST(c2.velocity_limit() == 456);

  // もとに戻してみる
  d.set_velocity_limit(std::numeric_limits<double>::max());
  BOOST_TEST(c1.velocity_limit() == std::numeric_limits<double>::max());
  BOOST_TEST(c2.velocity_limit() == std::numeric_limits<double>::max());
}

BOOST_AUTO_TEST_CASE(set_stable) {
  boost::asio::io_service io_service{};
  ai_server::model::updater::world wu{};
  ai_server::driver d{io_service, 100us, wu, model::team_color::blue};

  auto c1_ptr    = std::make_unique<mock_controller>();
  const auto& c1 = *c1_ptr;
  d.register_robot(1, std::move(c1_ptr), nullptr);

  auto c2_ptr    = std::make_unique<mock_controller>();
  const auto& c2 = *c2_ptr;
  d.register_robot(2, std::move(c2_ptr), nullptr);

  d.set_stable(true);
  BOOST_TEST(c1.is_stable());
  BOOST_TEST(c2.is_stable());

  d.set_stable(false);
  BOOST_TEST(!c1.is_stable());
  BOOST_TEST(!c2.is_stable());
}

BOOST_AUTO_TEST_CASE(main_loop) {
  boost::asio::io_service io_service{};
  ai_server::model::updater::world wu{};
  ai_server::driver d{io_service, 100us, wu, model::team_color::blue};

  auto c1_ptr = std::make_unique<mock_controller>();
  auto s1_ptr = std::make_unique<mock_sender>();
  auto& c1    = *c1_ptr;
  auto& s1    = *s1_ptr;
  d.register_robot(1, std::move(c1_ptr), std::move(s1_ptr));

  auto c2_ptr = std::make_unique<mock_controller>();
  auto s2_ptr = std::make_unique<mock_sender>();
  auto& c2    = *c2_ptr;
  auto& s2    = *s2_ptr;
  d.register_robot(2, std::move(c2_ptr), std::move(s2_ptr));

  // ロボットが検出されていないとき命令は送信されない
  io_service.run_one();
  BOOST_TEST(!c1.executed_);
  BOOST_TEST(s1.commands_.empty());
  BOOST_TEST(!c2.executed_);
  BOOST_TEST(s2.commands_.empty());

  // blue の ID 1 が見えるようにしてみる
  {
    ssl_protos::vision::Packet p{};

    auto md = p.mutable_detection();
    md->set_camera_id(0);

    auto r = md->add_robots_blue();
    r->set_robot_id(1);
    r->set_x(0);
    r->set_y(0);
    r->set_orientation(0);
    r->set_confidence(100);

    wu.update(p);
  }

  // ID 1 に対して命令が送られる
  // 命令の初期値は停止 (速度0)
  io_service.run_one();
  {
    BOOST_TEST(c1.executed_);
    BOOST_TEST(s1.commands_.size() == 1);
    const auto& c = s1.commands_.back();
    BOOST_TEST(c.id() == 1);
    const auto v = std::get_if<ai_server::model::command::velocity_t>(&c.setpoint());
    BOOST_TEST(v != nullptr);
    BOOST_TEST(v->vx == 0);
    BOOST_TEST(v->vy == 0);
    BOOST_TEST(v->omega == 0);
    c1.executed_ = false;
    s1.commands_.clear();
  }
  BOOST_TEST(!c2.executed_);
  BOOST_TEST(s2.commands_.empty());

  // チームカラーを yellow に変更
  d.set_team_color(model::team_color::yellow);

  // ロボットが検出されていないとき命令は送信されない
  io_service.run_one();
  BOOST_TEST(!c1.executed_);
  BOOST_TEST(s1.commands_.empty());
  BOOST_TEST(!c2.executed_);
  BOOST_TEST(s2.commands_.empty());

  // yellow の ID 1 が見えるようにしてみる
  {
    ssl_protos::vision::Packet p{};

    auto md = p.mutable_detection();
    md->set_camera_id(0);

    auto r = md->add_robots_yellow();
    r->set_robot_id(1);
    r->set_x(0);
    r->set_y(0);
    r->set_orientation(0);
    r->set_confidence(100);

    wu.update(p);
  }

  // ID 1 に対して命令が送られる
  io_service.run_one();
  {
    BOOST_TEST(c1.executed_);
    BOOST_TEST(s1.commands_.size() == 1);
    const auto& c = s1.commands_.back();
    BOOST_TEST(c.id() == 1);
    const auto v = std::get_if<ai_server::model::command::velocity_t>(&c.setpoint());
    BOOST_TEST(v != nullptr);
    BOOST_TEST(v->vx == 0);
    BOOST_TEST(v->vy == 0);
    BOOST_TEST(v->omega == 0);
    c1.executed_ = false;
    s1.commands_.clear();
  }
  BOOST_TEST(!c2.executed_);
  BOOST_TEST(s2.commands_.empty());

  // ID 1 の命令を更新してみる
  {
    model::command c{1};
    c.set_velocity({1, 2, 3});
    d.update_command(c);
  }

  // ID 1 に対して命令が送られる
  io_service.run_one();
  {
    BOOST_TEST(c1.executed_);
    BOOST_TEST(s1.commands_.size() == 1);
    const auto& c = s1.commands_.back();
    BOOST_TEST(c.id() == 1);
    const auto v = std::get_if<ai_server::model::command::velocity_t>(&c.setpoint());
    BOOST_TEST(v != nullptr);
    BOOST_TEST(v->vx == 1);
    BOOST_TEST(v->vy == 2);
    BOOST_TEST(v->omega == 3);
    c1.executed_ = false;
    s1.commands_.clear();
  }
  BOOST_TEST(!c2.executed_);
  BOOST_TEST(s2.commands_.empty());

  // yellow の ID 2 も見えるようにしてみる
  {
    ssl_protos::vision::Packet p{};

    auto md = p.mutable_detection();
    md->set_camera_id(1);

    auto r = md->add_robots_yellow();
    r->set_robot_id(2);
    r->set_x(0);
    r->set_y(0);
    r->set_orientation(0);
    r->set_confidence(100);

    wu.update(p);
  }

  // ID 1, ID 2 に対して命令が送られる
  io_service.run_one();
  {
    BOOST_TEST(c1.executed_);
    BOOST_TEST(s1.commands_.size() == 1);
    const auto& c = s1.commands_.back();
    BOOST_TEST(c.id() == 1);
    const auto v = std::get_if<ai_server::model::command::velocity_t>(&c.setpoint());
    BOOST_TEST(v != nullptr);
    BOOST_TEST(v->vx == 1);
    BOOST_TEST(v->vy == 2);
    BOOST_TEST(v->omega == 3);
    c1.executed_ = false;
    s1.commands_.clear();
  }
  {
    BOOST_TEST(c2.executed_);
    BOOST_TEST(s2.commands_.size() == 1);
    const auto& c = s2.commands_.back();
    BOOST_TEST(c.id() == 2);
    const auto v = std::get_if<ai_server::model::command::velocity_t>(&c.setpoint());
    BOOST_TEST(v != nullptr);
    BOOST_TEST(v->vx == 0);
    BOOST_TEST(v->vy == 0);
    BOOST_TEST(v->omega == 0);
    c2.executed_ = false;
    s2.commands_.clear();
  }
}

BOOST_AUTO_TEST_SUITE_END()
