#include <algorithm>

#include "ai_server/game/action/no_operation.h"
// mw 0829 #include "ai_server/model/motion/halt.h"

#include "halt.h"
 
namespace ai_server::game::formation {

halt::halt(context& ctx, const std::vector<unsigned int>& ids) : base{ctx} {
  std::transform(ids.cbegin(), ids.cend(), std::back_inserter(actions_),
                 [this](auto id) { return make_action<action::no_operation>(id); });
}

std::vector<std::shared_ptr<action::base>> halt::execute() {
 // mw 0829  command.set_motion(std::make_shared<model::motion::halt>());
  return actions_;
}

} // namespace ai_server::game::formation
