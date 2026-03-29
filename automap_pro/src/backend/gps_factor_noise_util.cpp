#include "automap_pro/backend/gps_factor_noise_util.h"
#include "automap_pro/core/config_manager.h"

#include <algorithm>
#include <cmath>

namespace automap_pro {
namespace gps_factor_noise {

namespace {

double clamp01(double x) { return std::max(0.0, std::min(1.0, x)); }

/** t in [0,1]: 0 → scale 1, 1 → scale_max */
double linearRampScale(double t, double scale_max) {
    if (scale_max <= 1.0) return 1.0;
    return 1.0 + t * (scale_max - 1.0);
}

}  // namespace

void refineKeyFrameGpsCovariance(Eigen::Matrix3d& cov_io,
                                 const ConfigManager& cfg,
                                 double hdop,
                                 int num_satellites,
                                 const Eigen::Vector3d& z_gps,
                                 const Eigen::Vector3d* z_graph) {
    if (!cov_io.allFinite()) return;

    double hdop_scale = 1.0;
    double sats_scale = 1.0;

    if (cfg.gpsAdaptiveNoiseEnabled()) {
        const double hg = std::max(0.05, cfg.gpsAdaptiveHdopGood());
        const double hp = std::max(hg + 1e-3, cfg.gpsAdaptiveHdopPoor());
        const double s_max_h = std::max(1.0, cfg.gpsAdaptiveVarScaleMaxHdop());

        double h = std::isfinite(hdop) && hdop > 0.0 ? hdop : hp;
        if (h <= hg) {
            hdop_scale = 1.0;
        } else if (h >= hp) {
            hdop_scale = s_max_h;
        } else {
            const double t = (h - hg) / (hp - hg);
            hdop_scale = linearRampScale(t, s_max_h);
        }

        const int sg = std::max(1, cfg.gpsAdaptiveSatsGood());
        const int sp = std::max(0, std::min(sg - 1, cfg.gpsAdaptiveSatsPoor()));
        const double s_max_s = std::max(1.0, cfg.gpsAdaptiveVarScaleMaxSats());

        int ns = num_satellites;
        if (ns <= 0) ns = sp;
        if (ns >= sg) {
            sats_scale = 1.0;
        } else if (ns <= sp) {
            sats_scale = s_max_s;
        } else {
            const double t = static_cast<double>(sg - ns) / static_cast<double>(std::max(1, sg - sp));
            sats_scale = linearRampScale(clamp01(t), s_max_s);
        }
    }

    const double quality_scale = hdop_scale * sats_scale;
    if (quality_scale > 1.0) {
        cov_io(0, 0) *= quality_scale;
        cov_io(1, 1) *= quality_scale;
        cov_io(2, 2) *= quality_scale;
    }

    if (cfg.gpsLocalConsistencyEnabled() && z_graph && z_graph->allFinite()) {
        const Eigen::Vector2d innov(z_gps.x() - z_graph->x(), z_gps.y() - z_graph->y());
        const double innov_xy = innov.norm();

        double sigma_h = std::sqrt(0.5 * (std::max(1e-12, cov_io(0, 0)) + std::max(1e-12, cov_io(1, 1))));
        sigma_h = std::max(0.05, sigma_h);

        const double gate = std::max(0.5, cfg.gpsLocalConsistencyInnovationGateSigma());
        const double threshold = gate * sigma_h;
        const double v_max = std::max(1.0, cfg.gpsLocalConsistencyVarScaleMax());

        if (innov_xy > threshold) {
            const double excess = innov_xy - threshold;
            const double rel = excess / std::max(1e-6, sigma_h);
            double m = 1.0 + rel * rel;
            m = std::min(v_max, m);
            cov_io(0, 0) *= m;
            cov_io(1, 1) *= m;
        }
    }

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (!std::isfinite(cov_io(i, j))) cov_io(i, j) = (i == j) ? 1.0 : 0.0;
        }
    }
}

void applyGpsFactorWeight(Eigen::Matrix3d& cov_io, double factor_weight) {
    if (!cov_io.allFinite()) return;
    const double w = std::max(0.01, factor_weight);
    if (std::abs(w - 1.0) < 1e-9) return;
    cov_io /= (w * w);
}

}  // namespace gps_factor_noise
}  // namespace automap_pro
