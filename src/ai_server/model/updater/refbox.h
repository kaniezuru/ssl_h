#ifndef AI_SERVER_MODEL_UPDATER_REFBOX_H
#define AI_SERVER_MODEL_UPDATER_REFBOX_H

#include <shared_mutex>
#include "ai_server/model/refbox.h"

// 前方宣言
namespace ssl_protos {
namespace refbox {
class Referee;
}
}

namespace ai_server {
namespace model {
namespace updater {

/// @class   refbox
/// @brief   SSL RefBoxのRefereeパケットでRefBoxの情報を更新する
class refbox {
  mutable std::shared_timed_mutex mutex_;
  model::refbox refbox_;

public:
  refbox();

  refbox(const refbox&) = delete;
  refbox& operator=(const refbox&) = delete;

  /// @brief          RefereeパケットでRefBoxの情報を更新する
  /// @param referee  SSL Referee BoxのRefereeパケット
  void update(const ssl_protos::refbox::Referee& referee);

  /// @brief          値を取得する
  model::refbox value() const;
};

} // namespace updater
} // namespace model
} // namespace ai_server

#endif // AI_SERVER_MODEL_UPDATER_REFBOX_H