#ifndef AI_SERVER_MODEL_WORLD_UPDATER_H
#define AI_SERVER_MODEL_WORLD_UPDATER_H

#include <algorithm>
#include <functional>
#include <memory>
#include <mutex>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "world.h"

#include "ssl-protos/vision/wrapperpacket.pb.h"

namespace ai_server {
namespace model {

class world_updater {
  /// <camera id, Ball>
  using ball_with_camera_id =
      std::tuple<unsigned int,
                 google::protobuf::RepeatedPtrField<ssl_protos::vision::Ball>::const_iterator>;
  /// <camera id, Robot>
  using robot_with_camera_id =
      std::tuple<unsigned int,
                 google::protobuf::RepeatedPtrField<ssl_protos::vision::Robot>::const_iterator>;
  /// ロボットのデータ更新用のハッシュテーブルの型 <robot_id, <camera_id, Robot>>
  using robots_table = std::unordered_multimap<unsigned int, robot_with_camera_id>;

  mutable std::mutex mutex_;

  model::world world_;

  /// カメラ台数分の最新のdetectionパケットを保持する
  std::unordered_map<unsigned int, ssl_protos::vision::Frame> detection_packets_;

public:
  /// @brief                  WorldModelを取得する
  const model::world& world_model() const;

  /// @brief                  内部の状態を更新する
  /// @param packet           SSL-Visionのパース済みパケット
  void update(const ssl_protos::vision::Packet& packet);

private:
  /// @brief                  detectionパケットを処理し, ボールやロボットの情報を更新する
  /// @param detection        SSL-Visionのdetectionパケット
  void process_packet(const ssl_protos::vision::Frame& detection);

  /// @brief                  geometryパケットを処理し, フィールドの情報を更新する
  /// @param geometry         SSL-Visionのgeometryパケット
  void process_packet(const ssl_protos::vision::Geometry& geometry);

  /// @brief                  ballsから最終的なボールの情報を生成する
  /// @param camera_id        最新のパケットのカメラID
  /// @param captured_time    最新のパケットのキャプチャされた時間
  /// @param balls            検出されたボールのリスト
  /// @param prev_data        前のデータ
  model::ball build_ball_data(unsigned int camera_id,
                              std::chrono::high_resolution_clock::time_point captured_time,
                              const std::vector<ball_with_camera_id>& balls,
                              const model::ball& prev_data);

  /// @brief                  tableにrobotsを追加する
  /// @param table            ロボットのデータ更新用のハッシュテーブル
  /// @param camera_id        robotsが検出されたカメラのID
  /// @param robots           tableに追加したいロボットのデータ
  static void add_robots_to_table(
      robots_table& table, unsigned int camera_id,
      const google::protobuf::RepeatedPtrField<ssl_protos::vision::Robot>& robots);

  /// @brief                  tableから最終的なロボットのリストを生成する
  /// @param is_yellow        ロボットは黄色か
  /// @param camera_id        最新のパケットのカメラID
  /// @param captured_time    最新のパケットのキャプチャされた時間
  /// @param table            ロボットのデータ更新用のハッシュテーブル
  /// @param prev_data        前のデータ
  model::world::robots_list build_robots_list(
      bool is_yellow, unsigned int camera_id,
      std::chrono::high_resolution_clock::time_point captured_time, const robots_table& table,
      const model::world::robots_list& prev_data);
};

} // namespace model
} // namespace ai_server

#endif // AI_SERVER_MODEL_WORLD_UPDATER_H
