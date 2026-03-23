#include "automap_pro/system/automap_system.h"
#include "automap_pro/v3/map_registry.h"

namespace automap_pro {

void AutoMapSystem::onOdometry(double ts, const Pose3d& pose, const Mat66d& cov) {
    if (v3_context_) {
        v3::RawOdometryEvent ev;
        ev.timestamp = ts;
        ev.pose = pose;
        ev.covariance = cov;
        v3_context_->eventBus()->publish(ev);
    }
}

void AutoMapSystem::onCloud(double ts, const CloudXYZIPtr& cloud) {
    if (v3_context_) {
        v3::RawCloudEvent ev;
        ev.timestamp = ts;
        ev.cloud = cloud;
        v3_context_->eventBus()->publish(ev);
    }
}

void AutoMapSystem::onKFInfo(const LivoKeyFrameInfo& info) {
    if (v3_context_) {
        v3::RawKFInfoEvent ev;
        ev.info = info;
        v3_context_->eventBus()->publish(ev);
    }
}

void AutoMapSystem::onGPS(double ts, double lat, double lon, double alt, double hdop, int sats) {
    if (v3_context_) {
        v3::RawGPSEvent ev;
        ev.timestamp = ts;
        ev.lat = lat; ev.lon = lon; ev.alt = alt; ev.hdop = hdop; ev.sats = sats;
        v3_context_->eventBus()->publish(ev);
    }
}

} // namespace automap_pro
