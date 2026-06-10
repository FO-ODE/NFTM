#include "ieskf.h"

#include <iomanip>
#include <iostream>

double State::gravity = 9.81;

M3D Jr(const V3D &inp)
{
    return Sophus::SO3d::leftJacobian(inp).transpose();
}
M3D JrInv(const V3D &inp)
{
    return Sophus::SO3d::leftJacobianInverse(inp).transpose();
}

void State::operator+=(const VStateD &delta)
{
    r_wi *= Sophus::SO3d::exp(delta.segment<3>(0)).matrix();
    t_wi += delta.segment<3>(3);
    v += delta.segment<3>(6);
    ba += delta.segment<3>(9);
    bg += delta.segment<3>(12);
    p_f1 += delta.segment<3>(kFootPositionStartIdx);
    p_f2 += delta.segment<3>(kFootPositionStartIdx + 3);
    p_f3 += delta.segment<3>(kFootPositionStartIdx + 6);
    p_f4 += delta.segment<3>(kFootPositionStartIdx + 9);
}

VStateD State::operator-(const State &other) const
{
    VStateD delta = VStateD::Zero();
    delta.segment<3>(0) = Sophus::SO3d(other.r_wi.transpose() * r_wi).log();
    delta.segment<3>(3) = t_wi - other.t_wi;
    delta.segment<3>(6) = v - other.v;
    delta.segment<3>(9) = ba - other.ba;
    delta.segment<3>(12) = bg - other.bg;
    delta.segment<3>(kFootPositionStartIdx) = p_f1 - other.p_f1;
    delta.segment<3>(kFootPositionStartIdx + 3) = p_f2 - other.p_f2;
    delta.segment<3>(kFootPositionStartIdx + 6) = p_f3 - other.p_f3;
    delta.segment<3>(kFootPositionStartIdx + 9) = p_f4 - other.p_f4;
    return delta;
}

std::ostream &operator<<(std::ostream &os, const State &state)
{
    os << "==============START===============" << std::endl;
    os << "r_wi: " << state.r_wi.eulerAngles(2, 1, 0).transpose() << std::endl;
    os << "t_wi: " << state.t_wi.transpose() << std::endl;
    os << "v: " << state.v.transpose() << std::endl;
    os << "ba: " << state.ba.transpose() << std::endl;
    os << "bg: " << state.bg.transpose() << std::endl;
    os << "p_f1: " << state.p_f1.transpose() << std::endl;
    os << "p_f2: " << state.p_f2.transpose() << std::endl;
    os << "p_f3: " << state.p_f3.transpose() << std::endl;
    os << "p_f4: " << state.p_f4.transpose() << std::endl;
    os << "g: " << state.g.transpose() << std::endl;
    os << "===============END================" << std::endl;

    return os;
}

void IESKF::configureStateLog(bool enabled, const std::string &path, bool flush_each_write)
{
    m_state_log_enabled = enabled;
    m_state_log_flush = flush_each_write;

    if (m_state_log.is_open())
        m_state_log.close();

    if (!m_state_log_enabled)
        return;

    m_state_log.open(path, std::ios::out | std::ios::trunc);
    if (!m_state_log.is_open())
    {
        std::cerr << "Failed to open IESKF state log: " << path << std::endl;
        m_state_log_enabled = false;
        return;
    }

    m_state_log << "stage,event,time,iter,valid,res,delta_norm,"
                << "r00,r01,r02,r10,r11,r12,r20,r21,r22,"
                << "t_x,t_y,t_z,"
                << "v_x,v_y,v_z,"
                << "ba_x,ba_y,ba_z,"
                << "bg_x,bg_y,bg_z,"
                << "pf1_x,pf1_y,pf1_z,"
                << "pf2_x,pf2_y,pf2_z,"
                << "pf3_x,pf3_y,pf3_z,"
                << "pf4_x,pf4_y,pf4_z,"
                << "g_x,g_y,g_z,"
                << "contact0,contact1,contact2,contact3,"
                << "force0,force1,force2,force3"
                << '\n';
}

void IESKF::setDebugContext(const std::string &stage, double time)
{
    if (!m_state_log_enabled)
        return;

    m_debug_stage = stage;
    m_debug_time = time;
}

void IESKF::setContactDebug(const V4D &contact_state, const V4D &foot_force)
{
    if (!m_state_log_enabled)
        return;

    m_debug_contact_state = contact_state;
    m_debug_foot_force = foot_force;
}

void IESKF::logState(const std::string &event, int iter, bool valid, double res, const VStateD *delta)
{
    if (!m_state_log_enabled || !m_state_log.is_open())
        return;

    const double delta_norm = delta ? delta->norm() : 0.0;
    m_state_log << std::setprecision(15)
                << m_debug_stage << ','
                << event << ','
                << m_debug_time << ','
                << iter << ','
                << (valid ? 1 : 0) << ','
                << res << ','
                << delta_norm;

    for (int row = 0; row < 3; row++)
        for (int col = 0; col < 3; col++)
            m_state_log << ',' << m_x.r_wi(row, col);

    auto write_vec3 = [this](const V3D &vec)
    {
        m_state_log << ',' << vec.x() << ',' << vec.y() << ',' << vec.z();
    };

    write_vec3(m_x.t_wi);
    write_vec3(m_x.v);
    write_vec3(m_x.ba);
    write_vec3(m_x.bg);
    write_vec3(m_x.p_f1);
    write_vec3(m_x.p_f2);
    write_vec3(m_x.p_f3);
    write_vec3(m_x.p_f4);
    write_vec3(m_x.g);
    for (int i = 0; i < 4; i++)
        m_state_log << ',' << m_debug_contact_state(i);
    for (int i = 0; i < 4; i++)
        m_state_log << ',' << m_debug_foot_force(i);
    m_state_log << '\n';

    if (m_state_log_flush)
        m_state_log.flush();
}

void IESKF::predict(const Input &inp, double dt, const MNoiseD &Q)
{
    VStateD delta = VStateD::Zero();
    delta.segment<3>(0) = (inp.gyro - m_x.bg) * dt;
    delta.segment<3>(3) = m_x.v * dt;
    delta.segment<3>(6) = (m_x.r_wi * (inp.acc - m_x.ba) + m_x.g) * dt;

    m_F.setIdentity();
    m_F.block<3, 3>(0, 0) = Sophus::SO3d::exp(-(inp.gyro - m_x.bg) * dt).matrix();
    m_F.block<3, 3>(0, 12) = -Jr((inp.gyro - m_x.bg) * dt) * dt;
    m_F.block<3, 3>(3, 6) = Eigen::Matrix3d::Identity() * dt;
    m_F.block<3, 3>(6, 0) = -m_x.r_wi * Sophus::SO3d::hat(inp.acc - m_x.ba) * dt;
    m_F.block<3, 3>(6, 9) = -m_x.r_wi * dt;

    m_G.setZero();
    m_G.block<3, 3>(0, 0) = -Jr((inp.gyro - m_x.bg) * dt) * dt;
    m_G.block<3, 3>(6, 3) = -m_x.r_wi * dt;
    m_G.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * dt;
    m_G.block<3, 3>(12, 6) = Eigen::Matrix3d::Identity() * dt;
    m_G.block<3, 3>(kFootPositionStartIdx, 12) = Eigen::Matrix3d::Identity() * dt;
    m_G.block<3, 3>(kFootPositionStartIdx + 3, 15) = Eigen::Matrix3d::Identity() * dt;
    m_G.block<3, 3>(kFootPositionStartIdx + 6, 18) = Eigen::Matrix3d::Identity() * dt;
    m_G.block<3, 3>(kFootPositionStartIdx + 9, 21) = Eigen::Matrix3d::Identity() * dt;

    m_x += delta;
    m_P = m_F * m_P * m_F.transpose() + m_G * Q * m_G.transpose();
    logState("propagation", -1, true, 0.0, &delta);
}

void IESKF::update()
{
    State predict_x = m_x;
    SharedState shared_data;
    shared_data.iter_num = 0;
    shared_data.res = 1e10;
    VStateD delta = VStateD::Zero();
    MStateD H = MStateD::Identity();
    VStateD b;
    bool updated = false;

    for (size_t i = 0; i < m_max_iter; i++)
    {
        m_loss_func(m_x, shared_data);
        logState("update_linearized", static_cast<int>(i), shared_data.valid, shared_data.res, nullptr);
        if (!shared_data.valid)
            break;
        updated = true;
        H.setZero();
        b.setZero();
        delta = m_x - predict_x;
        MStateD J = MStateD::Identity();
        J.block<3, 3>(0, 0) = JrInv(delta.segment<3>(0));
        H += J.transpose() * m_P.inverse() * J;
        b += J.transpose() * m_P.inverse() * delta;

        H += shared_data.H;
        b += shared_data.b;

        delta = -H.inverse() * b;

        m_x += delta;
        logState("update_applied", static_cast<int>(i), true, shared_data.res, &delta);
        shared_data.iter_num += 1;

        if (m_stop_func(delta))
            break;
    }

    if (!updated)
        return;

    MStateD L = MStateD::Identity();
    // L.block<3, 3>(0, 0) = JrInv(delta.segment<3>(0));
    L.block<3, 3>(0, 0) = Jr(delta.segment<3>(0));
    m_P = L * H.inverse() * L.transpose();
    logState("update_final", static_cast<int>(shared_data.iter_num), true, shared_data.res, &delta);
}
