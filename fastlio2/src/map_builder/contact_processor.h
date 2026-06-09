#pragma once
#include "commons.h"
#include "ieskf.h"

class ContactProcessor
{
public:
    ContactProcessor(Config &config, std::shared_ptr<IESKF> kf);

    void prepare(SyncPackage &package);

    void process(SyncPackage &package);

    void updateLossFunc(State &state, SharedState &share_data);

    bool hasLowState() const { return m_has_lowstate; }
    const LowStateData &latestLowState() const { return m_latest_lowstate; }

private:
    void cacheLowStates(SyncPackage &package);
    void cacheImus(SyncPackage &package);
    void updateContactState();
    void initializeNewContacts();
    void resetContactState();
    bool hasActiveContact() const;
    V3D footRelativePosition(int foot_idx) const;
    V3D footRelativeVelocity(int foot_idx) const;
    V3D &footPosition(State &state, int foot_idx) const;
    const V3D &footPosition(const State &state, int foot_idx) const;

    Config m_config;
    std::shared_ptr<IESKF> m_kf;
    Vec<LowStateData> m_lowstate_cache;
    LowStateData m_latest_lowstate;
    V4D m_contact_state;
    V4D m_prev_contact_state;
    V3D m_latest_gyro = V3D::Zero();
    double m_last_process_time = 0.0;
    bool m_has_lowstate = false;
    bool m_has_imu = false;
};
