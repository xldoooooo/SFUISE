/**
 * @file eigen_types.h
 * @brief Eigen 容器类型别名，支持内存对齐
 */
#pragma once

#include <deque>
#include <map>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>

namespace Eigen {

/**
 * @brief 支持 Eigen 内存对齐的 std::vector
 */
template <typename T>
using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

/**
 * @brief 支持 Eigen 内存对齐的 std::deque
 */
template <typename T>
using aligned_deque = std::deque<T, Eigen::aligned_allocator<T>>;

/**
 * @brief 支持 Eigen 内存对齐的 std::map
 */
template <typename K, typename V>
using aligned_map = std::map<K, V, std::less<K>,
                    Eigen::aligned_allocator<std::pair<K const, V>>>;

/**
 * @brief 支持 Eigen 内存对齐的 std::unordered_map
 */
template <typename K, typename V>
using aligned_unordered_map = std::unordered_map<K, V, std::hash<K>, std::equal_to<K>,
                              Eigen::aligned_allocator<std::pair<K const, V>>>;

}  // namespace Eigen
