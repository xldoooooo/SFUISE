/**
 * @file Accumulator.h
 * @brief 稀疏矩阵累加器 - 用于构建 Hessian 和梯度
 */
#pragma once

#include <array>
#include <unordered_map>

#include <Eigen/CholmodSupport>
#include <Eigen/Dense>
#include <Eigen/Sparse>

/**
 * @brief 稀疏 Cholesky 分解类型别名
 */
template <class T>
using SparseLLT = Eigen::CholmodSupernodalLLT<T>;

/**
 * @brief 基于哈希表的稀疏累加器
 */
class SparseHashAccumulator
{
public:
    using VectorX = Eigen::VectorXd;
    using MatrixX = Eigen::MatrixXd;
    using Triplet = Eigen::Triplet<double>;
    using SparseMatrix = Eigen::SparseMatrix<double>;

    /// 添加 Hessian 块
    template <int ROWS, int COLS, typename Derived>
    void addH(int si, int sj, const Eigen::MatrixBase<Derived>& data)
    {
        EIGEN_STATIC_ASSERT_MATRIX_SPECIFIC_SIZE(Derived, ROWS, COLS);
        auto [it, inserted] = hash_map.try_emplace({si, sj, ROWS, COLS}, data);
        if (!inserted) it->second += data;
    }

    /// 添加梯度块
    template <int ROWS, typename Derived>
    void addB(int i, const Eigen::MatrixBase<Derived>& data)
    {
        b.segment<ROWS>(i) += data;
    }

    /// 构建稀疏矩阵
    void setup_solver()
    {
        std::vector<Triplet> triplets;
        triplets.reserve(hash_map.size() * 36 + b.rows());

        for (const auto& [key, mat] : hash_map) {
            for (int i = 0; i < mat.rows(); i++) {
                for (int j = 0; j < mat.cols(); j++) {
                    triplets.emplace_back(key[0] + i, key[1] + j, mat(i, j));
                }
            }
        }
        // 添加对角线保护
        for (int i = 0; i < b.rows(); i++) {
            triplets.emplace_back(i, i, std::numeric_limits<double>::min());
        }

        smm.resize(b.rows(), b.rows());
        smm.setFromTriplets(triplets.begin(), triplets.end());
    }

    VectorX Hdiagonal() const { return smm.diagonal(); }
    VectorX& getB() { return b; }

    /// 求解线性系统
    VectorX solve(const VectorX* diagonal = nullptr) const
    {
        SparseMatrix sm = smm;
        if (diagonal) sm.diagonal() += *diagonal;
        return SparseLLT<SparseMatrix>(sm).solve(b);
    }

    void reset(int opt_size)
    {
        hash_map.clear();
        b.setZero(opt_size);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    using KeyT = std::array<int, 4>;

    struct KeyHash {
        size_t operator()(const KeyT& c) const
        {
            size_t seed = 0;
            for (int v : c) {
                seed ^= v + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            }
            return seed;
        }
    };

    std::unordered_map<KeyT, MatrixX, KeyHash> hash_map;
    VectorX b;
    SparseMatrix smm;
};
