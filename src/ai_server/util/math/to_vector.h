#ifndef AI_SERVER_UTIL_MATH_TO_VECTOR_H
#define AI_SERVER_UTIL_MATH_TO_VECTOR_H

#include <type_traits>

#include <Eigen/Dense>

#include "ai_server/model/command.h"

namespace ai_server {
namespace util {
namespace math {

/// @brief     メンバ関数x(), y()を持つオブジェクトを2次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Matrix<x y共通の型, 2, 1>{obj.x(), obj.y()}
template <class T>
inline auto position(const T& obj)
    -> Eigen::Matrix<std::common_type_t<decltype(obj.x()), decltype(obj.y())>, 2, 1> {
  return {obj.x(), obj.y()};
}

/// @brief     position_tを2次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Vector2d{obj.x, obj.y}
inline Eigen::Vector2d position(const model::command::position_t& obj) {
  return {obj.x, obj.y};
}

/// @brief     メンバ関数vx(), vy()を持つオブジェクトを2次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Matrix<vx vy共通の型, 2, 1>{obj.vx(), obj.vy()}
template <class T>
inline auto velocity(const T& obj)
    -> Eigen::Matrix<std::common_type_t<decltype(obj.vx()), decltype(obj.vy())>, 2, 1> {
  return {obj.vx(), obj.vy()};
}

/// @brief     velocity_tを2次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Vector2d{obj.x, obj.y}
inline Eigen::Vector2d velocity(const model::command::velocity_t& obj) {
  return {obj.vx, obj.vy};
}

/// @brief     メンバ関数ax(), ay()を持つオブジェクトを2次元のベクトル型に変換する
/// @param obj 変換したいオブジェクト
/// @return    Eigen::Matrix<ax ay共通の型, 2, 1>{obj.ax(), obj.ay()}
template <class T>
inline auto acceleration(const T& obj)
    -> Eigen::Matrix<std::common_type_t<decltype(obj.ax()), decltype(obj.ay())>, 2, 1> {
  return {obj.ax(), obj.ay()};
}

} // namespace math
} // namespace util
} // namespace ai_server

#endif // AI_SERVER_UTIL_MATH_TO_VECTOR_H
