#pragma once

#include <string>
#include <memory>
#include <mutex>

#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>

namespace automap_pro {

class ConfigManager {
public:
    static ConfigManager& instance();

    bool loadFromFile(const std::string& path);

    // ── System ──────────────────────────────────────────
    std::string systemName()     const;
    std::string mode()           const;
    std::string logLevel()       const;
    std::string outputDir()      const;
    int         numThreads()     const;
    bool        useGPU()         const;
    int         gpuDeviceId()    const;

    // ── Sensor ──────────────────────────────────────────
    std::string lidarTopic()     const;
    double      lidarFrequency() const;
    double      lidarMaxRange()  const;
    double      lidarMinRange()  const;
    bool        lidarUndistort() const;

    std::string imuTopic()       const;
    double      imuFrequency()   const;
    double      imuGravity()     const;

    bool        cameraEnabled()  const;
    std::string cameraTopic()    const;

    bool        gpsEnabled()     const;
    std::string gpsTopic()       const;
    bool        gpsAutoInitENU() const;

    // ── GPS Fusion ───────────────────────────────────────
    double gpsExcellentHDOP()   const;
    double gpsHighHDOP()        const;
    double gpsMediumHDOP()      const;

    Eigen::Vector3d gpsCovExcellent() const;
    Eigen::Vector3d gpsCovHigh()      const;
    Eigen::Vector3d gpsCovMedium()    const;
    Eigen::Vector3d gpsCovLow()       const;

    bool   gpsConsistencyCheck()      const;
    double gpsChi2Threshold()         const;
    double gpsMaxVelocity()           const;
    bool   gpsJumpDetection()         const;
    double gpsMaxJump()               const;
    int    gpsConsecutiveValid()       const;

    // ── Frontend ─────────────────────────────────────────
    std::string frontendMode()       const;   // "internal" | "external_fast_livo"
    std::string externalFastLivoOdomTopic()  const;
    std::string externalFastLivoCloudTopic() const;
    std::string fastLivo2Config()     const;
    double kfMinTranslation()         const;
    double kfMinRotationDeg()         const;
    double kfMaxInterval()            const;
    double cloudDownsampleResolution() const;

    // ── Submap ────────────────────────────────────────────
    std::string msMappingConfig()     const;
    int    submapMaxKeyframes()       const;
    double submapMaxSpatialExtent()   const;
    double submapMaxTemporalExtent()  const;
    double submapMatchResolution()    const;

    // ── Loop Closure (OverlapTransformer) ─────────────────
    std::string overlapTransformerMode() const;  // "internal" | "external_service"
    std::string overlapDescriptorServiceName() const;
    std::string overlapModelPath()    const;
    int    rangeImageHeight()         const;
    int    rangeImageWidth()          const;
    int    descriptorDim()            const;
    int    loopTopK()                 const;
    double overlapThreshold()         const;
    double loopMinTemporalGap()       const;
    int    loopMinSubmapGap()         const;
    double gpsSearchRadius()          const;

    // ── Loop Closure (TEASER++) ───────────────────────────
    double teaserVoxelSize()          const;
    double fpfhNormalRadius()         const;
    double fpfhFeatureRadius()        const;
    int    fpfhMaxNNNormal()          const;
    int    fpfhMaxNNFeature()         const;
    double teaserNoiseBound()         const;
    double teaserCbar2()              const;
    double teaserRotGNCFactor()       const;
    int    teaserRotMaxIter()         const;
    double teaserRotCostThresh()      const;
    double teaserMinInlierRatio()     const;
    double teaserMaxRMSE()            const;
    double teaserMinFitness()         const;
    bool   teaserICPRefine()          const;
    int    icpMaxIterations()         const;
    double icpMaxCorrDist()           const;
    double icpConvThresh()            const;

    // ── Backend (HBA) ─────────────────────────────────────
    std::string hbaConfig()           const;
    bool   hbaTriggerOnLoop()         const;
    int    hbaTriggerPeriodicSubmaps() const;
    bool   hbaTriggerOnFinish()       const;
    int    hbaMaxIterations()         const;
    double hbaConvergenceThreshold()  const;
    bool   hbaRobustKernel()          const;
    double hbaRobustKernelDelta()     const;
    std::string hbaBridgeExportPath()   const;
    bool   hbaBridgeRunAfterExport()    const;
    std::string hbaBridgeRunCommandTemplate() const;

    // ── Map Output ────────────────────────────────────────
    double mapVoxelSize()             const;
    bool   mapStatFilter()            const;
    int    mapStatFilterMeanK()       const;
    double mapStatFilterStdMul()      const;
    bool   mapTilingEnabled()         const;
    double mapTileSize()              const;
    bool   saveFormatPCD()            const;
    bool   saveFormatPLY()            const;
    bool   saveFormatLAS()            const;

    // ── Visualization ─────────────────────────────────────
    double visPublishRate()           const;
    bool   visShowLoopClosures()      const;
    bool   visShowGPSTrajectory()     const;
    bool   visShowSubmapBoundaries()  const;
    double visGlobalMapDownsample()   const;

    // Raw node access for non-standard params
    YAML::Node getNode(const std::string& key) const;

private:
    ConfigManager() = default;
    ConfigManager(const ConfigManager&) = delete;
    ConfigManager& operator=(const ConfigManager&) = delete;

    YAML::Node root_;
    mutable std::mutex mutex_;

    template<typename T>
    T get(const std::string& path, const T& default_val) const;
};

}  // namespace automap_pro
