#include "contact_processor.h"

#include <cmath>

ContactProcessor::ContactProcessor(Config &config, std::shared_ptr<IESKF> kf) : m_config(config), m_kf(kf)
{
    resetContactState();
    m_lowstate_cache.clear();
}

void ContactProcessor::process(SyncPackage &package)
{
    prepare(package);

    if (!m_config.contact_enable || !m_has_lowstate || !m_has_imu || !hasActiveContact())
        return;

    m_kf->setLossFunction([&](State &s, SharedState &d)
                          { updateLossFunc(s, d); });
    m_kf->setStopFunction([&](const VStateD &delta) -> bool
                          { return delta.segment<15>(0).norm() < 1e-5 && delta.segment<kFootPositionDim>(kFootPositionStartIdx).norm() < 1e-5; });
    m_kf->update();
}

void ContactProcessor::prepare(SyncPackage &package)
{
    cacheLowStates(package);
    cacheImus(package);
    updateContactState();
    if (m_has_lowstate)
        m_kf->setContactDebug(m_contact_state, m_latest_lowstate.foot_force);
    initializeNewContacts();
}

void ContactProcessor::updateLossFunc(State &state, SharedState &share_data)
{
    share_data.H.setZero();
    share_data.b.setZero();
    share_data.res = 0.0;
    share_data.valid = false;

    if (!m_config.contact_enable || !m_has_lowstate || !m_has_imu || !hasActiveContact())
        return;

    const M3D &R_wb = state.r_wi;
    const M3D R_bw = R_wb.transpose();
    const V3D omega = m_latest_gyro - state.bg;
    int residual_num = 0;

    for (int foot_idx = 0; foot_idx < 4; foot_idx++)
    {
        if (m_contact_state(foot_idx) < 0.5)
            continue;

        const V3D p_f_rel = footRelativePosition(foot_idx);
        const V3D v_f_rel = footRelativeVelocity(foot_idx);
        const V3D &p_f = footPosition(state, foot_idx);
        const int foot_state_idx = kFootPositionStartIdx + 3 * foot_idx;

        // Formula (16): contact velocity residual in world frame.
        const V3D velocity_body_term = v_f_rel + omega.cross(p_f_rel);
        const V3D h_cv = state.v + R_wb * velocity_body_term;

        Eigen::Matrix<double, 3, kNominalStateDim> J_cv;
        J_cv.setZero();
        J_cv.block<3, 3>(0, 0) = -R_wb * Sophus::SO3d::hat(velocity_body_term);
        J_cv.block<3, 3>(0, 6) = M3D::Identity();
        J_cv.block<3, 3>(0, 12) = R_wb * Sophus::SO3d::hat(p_f_rel);

        share_data.H += J_cv.transpose() * m_config.contact_velocity_cov_inv * J_cv;
        share_data.b += J_cv.transpose() * m_config.contact_velocity_cov_inv * h_cv;
        share_data.res += h_cv.squaredNorm();
        residual_num += 3;

        // Formula (17): contact position residual in body frame.
        const V3D predicted_p_f_rel = R_bw * (state.t_wi - p_f);
        const V3D h_cp = p_f_rel - predicted_p_f_rel;

        Eigen::Matrix<double, 3, kNominalStateDim> J_cp;
        J_cp.setZero();
        J_cp.block<3, 3>(0, 0) = -Sophus::SO3d::hat(predicted_p_f_rel);
        J_cp.block<3, 3>(0, 3) = -R_bw;
        J_cp.block<3, 3>(0, foot_state_idx) = R_bw;

        share_data.H += J_cp.transpose() * m_config.contact_position_cov_inv * J_cp;
        share_data.b += J_cp.transpose() * m_config.contact_position_cov_inv * h_cp;
        share_data.res += h_cp.squaredNorm();
        residual_num += 3;
    }

    if (residual_num == 0)
        return;

    share_data.res /= static_cast<double>(residual_num);
    share_data.valid = true;
}

void ContactProcessor::cacheLowStates(SyncPackage &package)
{
    m_lowstate_cache.clear();
    m_lowstate_cache.insert(m_lowstate_cache.end(), package.lowstates.begin(), package.lowstates.end());

    if (m_lowstate_cache.empty())
        return;

    m_latest_lowstate = m_lowstate_cache.back();
    m_last_process_time = m_latest_lowstate.time;
    m_has_lowstate = true;
}

void ContactProcessor::cacheImus(SyncPackage &package)
{
    if (package.imus.empty())
        return;

    m_latest_gyro = package.imus.back().gyro;
    m_has_imu = true;
}

void ContactProcessor::updateContactState()
{
    if (!m_has_lowstate)
    {
        resetContactState();
        return;
    }

    m_prev_contact_state = m_contact_state;
    m_contact_state.setZero();

    for (int i = 0; i < 4; i++)
    {
        if (m_latest_lowstate.foot_force(i) >= m_config.contact_force_threshold)
            m_contact_state(i) = 1.0;
    }
}

void ContactProcessor::initializeNewContacts()
{
    if (!m_config.contact_enable || !m_has_lowstate)
        return;

    State &state = m_kf->x();
    for (int foot_idx = 0; foot_idx < 4; foot_idx++)
    {
        if (m_contact_state(foot_idx) < 0.5 || m_prev_contact_state(foot_idx) >= 0.5)
            continue;

        // Formula (15) inverted: p_f_i = p_wb - R_wb * p_f_i^rel.
        footPosition(state, foot_idx) = state.t_wi - state.r_wi * footRelativePosition(foot_idx);
    }
}

bool ContactProcessor::hasActiveContact() const
{
    return m_contact_state.maxCoeff() > 0.5;
}

V3D ContactProcessor::footRelativePosition(int foot_idx) const
{
    const int joint_idx = 3 * foot_idx;
    const V3D q = m_latest_lowstate.joint_pos.segment<3>(joint_idx);
    const double q_ab = q(0);
    const double q_hip = q(1);
    const double q_knee = q(2);
    const double q_hip_knee = q_hip + q_knee;

    const double side_sign = (foot_idx == 1 || foot_idx == 3) ? 1.0 : -1.0;
    const double l_ab = m_config.contact_abduction_link;
    const double l_thigh = m_config.contact_thigh_link;
    const double l_calf = m_config.contact_calf_link;
    const double leg_extension = l_thigh * std::cos(q_hip) + l_calf * std::cos(q_hip_knee);

    V3D p_leg;
    p_leg.x() = -l_thigh * std::sin(q_hip) - l_calf * std::sin(q_hip_knee);
    p_leg.y() = side_sign * l_ab * std::cos(q_ab) + leg_extension * std::sin(q_ab);
    p_leg.z() = side_sign * l_ab * std::sin(q_ab) - leg_extension * std::cos(q_ab);

    const V3D p_base_to_foot_link = m_config.contact_hip_offsets.segment<3>(joint_idx) + p_leg;
    const V3D p_base_to_foot = p_base_to_foot_link + V3D(0.0, 0.0, -m_config.contact_foot_link_to_ground);
    const V3D p_body_to_foot = m_config.contact_r_base_body.transpose() * (p_base_to_foot - m_config.contact_t_base_body);

    // Formula (15) uses R_wb^T * (p_wb - p_f_i), so expose the body-minus-foot
    // vector in the FAST-LIO body/IMU frame. Unitree FK is computed in base.
    return -p_body_to_foot;
}

V3D ContactProcessor::footRelativeVelocity(int foot_idx) const
{
    const int joint_idx = 3 * foot_idx;
    const V3D q = m_latest_lowstate.joint_pos.segment<3>(joint_idx);
    const V3D dq = m_latest_lowstate.joint_vel.segment<3>(joint_idx);
    const double q_ab = q(0);
    const double q_hip = q(1);
    const double q_knee = q(2);
    const double q_hip_knee = q_hip + q_knee;

    const double side_sign = (foot_idx == 1 || foot_idx == 3) ? 1.0 : -1.0;
    const double l_ab = m_config.contact_abduction_link;
    const double l_thigh = m_config.contact_thigh_link;
    const double l_calf = m_config.contact_calf_link;
    const double leg_extension = l_thigh * std::cos(q_hip) + l_calf * std::cos(q_hip_knee);
    const double leg_extension_dq_hip = -l_thigh * std::sin(q_hip) - l_calf * std::sin(q_hip_knee);
    const double leg_extension_dq_knee = -l_calf * std::sin(q_hip_knee);

    M3D J;
    J.setZero();
    J(0, 1) = -l_thigh * std::cos(q_hip) - l_calf * std::cos(q_hip_knee);
    J(0, 2) = -l_calf * std::cos(q_hip_knee);
    J(1, 0) = -side_sign * l_ab * std::sin(q_ab) + leg_extension * std::cos(q_ab);
    J(1, 1) = leg_extension_dq_hip * std::sin(q_ab);
    J(1, 2) = leg_extension_dq_knee * std::sin(q_ab);
    J(2, 0) = side_sign * l_ab * std::cos(q_ab) + leg_extension * std::sin(q_ab);
    J(2, 1) = -leg_extension_dq_hip * std::cos(q_ab);
    J(2, 2) = -leg_extension_dq_knee * std::cos(q_ab);

    const V3D v_base_to_foot = J * dq;
    const V3D v_body_to_foot = m_config.contact_r_base_body.transpose() * v_base_to_foot;

    // Keep velocity consistent with footRelativePosition(), whose sign follows
    // formula (15) instead of the usual body-to-foot FK convention.
    return -v_body_to_foot;
}

V3D &ContactProcessor::footPosition(State &state, int foot_idx) const
{
    switch (foot_idx)
    {
    case 0:
        return state.p_f1;
    case 1:
        return state.p_f2;
    case 2:
        return state.p_f3;
    default:
        return state.p_f4;
    }
}

const V3D &ContactProcessor::footPosition(const State &state, int foot_idx) const
{
    switch (foot_idx)
    {
    case 0:
        return state.p_f1;
    case 1:
        return state.p_f2;
    case 2:
        return state.p_f3;
    default:
        return state.p_f4;
    }
}

void ContactProcessor::resetContactState()
{
    m_contact_state.setZero();
    m_prev_contact_state.setZero();
    m_latest_lowstate = LowStateData();
    m_latest_gyro.setZero();
    m_has_lowstate = false;
    m_has_imu = false;
    m_last_process_time = 0.0;
}
