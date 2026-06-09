#pragma once
#include <Eigen/Eigen>
#include <sophus/so3.hpp>
#include "commons.h"

static constexpr int kNominalStateDim = 27;
static constexpr int kNoiseDim = 24;
static constexpr int kFootPositionStartIdx = 15;
static constexpr int kFootPositionDim = 12;

using MNoiseD = Eigen::Matrix<double, kNoiseDim, kNoiseDim>;
using M6D = Eigen::Matrix<double, 6, 6>;

using V6D = Eigen::Matrix<double, 6, 1>;

using MStateD = Eigen::Matrix<double, kNominalStateDim, kNominalStateDim>;
using VStateD = Eigen::Matrix<double, kNominalStateDim, 1>;
using MStateXNoiseD = Eigen::Matrix<double, kNominalStateDim, kNoiseDim>;

M3D Jr(const V3D &inp);
M3D JrInv(const V3D &inp);

struct SharedState
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MStateD H = MStateD::Zero();
    VStateD b = VStateD::Zero();
    double res = 1e10;
    bool valid = false;
    size_t iter_num = 0;
};
struct Input
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc;
    V3D gyro;
    Input() = default;
    Input(V3D &a, V3D &g) : acc(a), gyro(g) {}
    Input(double a1, double a2, double a3, double g1, double g2, double g3) : acc(a1, a2, a3), gyro(g1, g2, g3) {}
};
struct State
{
    static double gravity;
    M3D r_wi = M3D::Identity();
    V3D t_wi = V3D::Zero();
    V3D v = V3D::Zero();
    V3D ba = V3D::Zero();
    V3D bg = V3D::Zero();
    V3D p_f1 = V3D::Zero();
    V3D p_f2 = V3D::Zero();
    V3D p_f3 = V3D::Zero();
    V3D p_f4 = V3D::Zero();
    V3D g = V3D(0.0, 0.0, -9.81);

    void initGravityDir(const V3D &gravity_dir) { g = gravity_dir.normalized() * State::gravity; }

    void operator+=(const VStateD &delta);

    VStateD operator-(const State &other) const;

    friend std::ostream &operator<<(std::ostream &os, const State &state);
};

using loss_func = std::function<void(State &, SharedState &)>;
using stop_func = std::function<bool(const VStateD &)>;

class IESKF
{
public:
    IESKF() = default;
    void setMaxIter(size_t iter) { m_max_iter = iter; }
    void setLossFunction(loss_func func) { m_loss_func = func; }
    void setStopFunction(stop_func func) { m_stop_func = func; }

    void predict(const Input &inp, double dt, const MNoiseD &Q);

    void update();

    State &x() { return m_x; }

    MStateD &P() { return m_P; }

private:
    size_t m_max_iter = 10;
    State m_x;
    MStateD m_P;
    loss_func m_loss_func;
    stop_func m_stop_func;
    MStateD m_F;
    MStateXNoiseD m_G;
};
