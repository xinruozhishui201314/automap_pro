#pragma once

#include <string>
#include <vector>
#include <map>
#include <mutex>

#include "automap_pro/core/data_types.h"

namespace automap_pro {

// ──────────────────────────────────────────────────────────
// SessionManager: handles multi-session mapping
// Tracks which submaps belong to which session,
// loads historical descriptor databases, and manages
// cross-session associations.
// ──────────────────────────────────────────────────────────
class SessionManager {
public:
    struct SessionInfo {
        int session_id;
        std::string data_dir;
        std::vector<int> submap_ids;
        int num_keyframes = 0;
        double t_start = 0.0;
        double t_end   = 0.0;
        bool loaded    = false;
    };

    SessionManager();
    ~SessionManager() = default;

    // Start a new session
    int startNewSession(const std::string& data_dir);

    // Load a historical session from disk
    bool loadSession(const std::string& session_dir, int session_id);

    // Save session metadata
    bool saveSession(int session_id, const std::string& output_dir,
                     const std::vector<SubMap::Ptr>& submaps);

    // Register a submap to a session
    void registerSubmap(int session_id, int submap_id);

    int currentSessionId() const;
    std::vector<SessionInfo> allSessions() const;
    SessionInfo getSession(int session_id) const;

    // Save/load descriptor database for cross-session loop closure
    bool saveDescriptorDB(const std::string& path,
                          const std::vector<SubMap::Ptr>& submaps) const;
    bool loadDescriptorDB(const std::string& path,
                          std::vector<SubMap::Ptr>& submaps);

private:
    mutable std::mutex mutex_;
    int current_session_id_ = 0;
    std::map<int, SessionInfo> sessions_;
};

}  // namespace automap_pro
