#include "ieskf.h"

double State::gravity = 9.81;

M3D Jr(const V3D &inp)
{
    return Sophus::SO3d::leftJacobian(inp).transpose();
}
M3D JrInv(const V3D &inp)
{
    return Sophus::SO3d::leftJacobianInverse(inp).transpose();
}

void State::operator+=(const V15D &delta)
{
    r_wi *= Sophus::SO3d::exp(delta.segment<3>(0)).matrix();
    t_wi += delta.segment<3>(3);
    v += delta.segment<3>(6);
    bg += delta.segment<3>(9);
    ba += delta.segment<3>(12);
}

V15D State::operator-(const State &other) const
{
    V15D delta = V15D::Zero();
    delta.segment<3>(0) = Sophus::SO3d(other.r_wi.transpose() * r_wi).log();
    delta.segment<3>(3) = t_wi - other.t_wi;
    delta.segment<3>(6) = v - other.v;
    delta.segment<3>(9) = bg - other.bg;
    delta.segment<3>(12) = ba - other.ba;
    return delta;
}

std::ostream &operator<<(std::ostream &os, const State &state)
{
    os << "==============START===============" << std::endl;
    os << "r_wi: " << state.r_wi.eulerAngles(2, 1, 0).transpose() << std::endl;
    os << "t_wi: " << state.t_wi.transpose() << std::endl;
    os << "v: " << state.v.transpose() << std::endl;
    os << "bg: " << state.bg.transpose() << std::endl;
    os << "ba: " << state.ba.transpose() << std::endl;
    os << "g: " << state.g.transpose() << std::endl;
    os << "===============END================" << std::endl;

    return os;
}

void IESKF::predict(const Input &inp, double dt, const M12D &Q)
{
    V15D delta = V15D::Zero();
    delta.segment<3>(0) = (inp.gyro - m_x.bg) * dt;
    delta.segment<3>(3) = m_x.v * dt;
    delta.segment<3>(6) = (m_x.r_wi * (inp.acc - m_x.ba) + m_x.g) * dt;

    m_F.setIdentity();
    m_F.block<3, 3>(0, 0) = Sophus::SO3d::exp(-(inp.gyro - m_x.bg) * dt).matrix();
    m_F.block<3, 3>(0, 9) = -Jr((inp.gyro - m_x.bg) * dt) * dt;
    m_F.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;
    m_F.block<3, 3>(6, 0) = -m_x.r_wi * Sophus::SO3d::hat(inp.acc - m_x.ba) * dt;
    m_F.block<3, 3>(6, 12) = -m_x.r_wi * dt;

    m_G.setZero();
    m_G.block<3, 3>(0, 0) = -Jr((inp.gyro - m_x.bg) * dt) * dt;
    m_G.block<3, 3>(6, 3) = -m_x.r_wi * dt;
    m_G.block<3, 3>(9, 6) = Eigen::Matrix3d::Identity() * dt;
    m_G.block<3, 3>(12, 9) = Eigen::Matrix3d::Identity() * dt;

    m_x += delta;
    m_P = m_F * m_P * m_F.transpose() + m_G * Q * m_G.transpose();
}

void IESKF::update()
{
    State predict_x = m_x;
    SharedState shared_data;
    shared_data.iter_num = 0;
    shared_data.res = 1e10;
    V15D delta = V15D::Zero();
    M15D H = M15D::Identity();
    V15D b;

    for (size_t i = 0; i < m_max_iter; i++)
    {
        m_loss_func(m_x, shared_data);
        if (!shared_data.valid)
            break;
        H.setZero();
        b.setZero();
        delta = m_x - predict_x;
        M15D J = M15D::Identity();
        J.block<3, 3>(0, 0) = JrInv(delta.segment<3>(0));
        H += J.transpose() * m_P.inverse() * J;
        b += J.transpose() * m_P.inverse() * delta;

        H.block<6, 6>(0, 0) += shared_data.H;
        b.block<6, 1>(0, 0) += shared_data.b;

        delta = -H.inverse() * b;

        m_x += delta;
        shared_data.iter_num += 1;

        if (m_stop_func(delta))
            break;
    }

    M15D L = M15D::Identity();
    // L.block<3, 3>(0, 0) = JrInv(delta.segment<3>(0));
    L.block<3, 3>(0, 0) = Jr(delta.segment<3>(0));
    m_P = L * H.inverse() * L.transpose();
}
