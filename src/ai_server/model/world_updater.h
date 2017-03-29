#ifndef AI_SERVER_MODEL_WORLD_UPDATER_H
#define AI_SERVER_MODEL_WORLD_UPDATER_H

#include <tuple>
#include <mutex>
#include <unordered_map>

#include "world.h"

#include "ssl-protos/vision/wrapperpacket.pb.h"

namespace ai_server {
namespace model {

class world_updater {
  /// <camera id, Ball>
  using ball_with_camera_id_t =
      std::tuple<unsigned int,
                 google::protobuf::RepeatedPtrField<ssl_protos::vision::Ball>::const_iterator>;
  /// <camera id, Robot>
  using robot_with_camera_id_t =
      std::tuple<unsigned int,
                 google::protobuf::RepeatedPtrField<ssl_protos::vision::Robot>::const_iterator>;
  /// ロボットのデータ更新用のハッシュテーブルの型 <robot_id, <camera_id, Robot>>
  using robots_table_t = std::unordered_multimap<unsigned int, robot_with_camera_id_t>;

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
  /// @param balls            検出されたボールのリスト
  /// @param prev_data        前のデータ
  static model::ball build_ball_data(const std::vector<ball_with_camera_id_t>& balls,
                                     const model::ball& prev_data);

  /// @brief                  tableにrobotsを追加する
  /// @param table            ロボットのデータ更新用のハッシュテーブル
  /// @param camera_id        robotsが検出されたカメラのID
  /// @param robots           tableに追加したいロボットのデータ
  static void add_robots_to_table(
      robots_table_t& table, unsigned int camera_id,
      const google::protobuf::RepeatedPtrField<ssl_protos::vision::Robot>& robots);

  /// @brief                  tableから最終的なロボットのリストを生成する
  /// @param table            ロボットのデータ更新用のハッシュテーブル
  /// @param prev_data        前のデータ
  static std::unordered_map<unsigned int, model::robot> build_robot_list(
      const robots_table_t& table,
      const std::unordered_map<unsigned int, model::robot>& prev_data);
};

} // namespace model
} // namespace ai_server

#endif // AI_SERVER_MODEL_WORLD_UPDATER_H
