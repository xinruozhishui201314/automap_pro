#!/usr/bin/env bash
# Single-parameter ablation checklist for loop closure (no bag replay here).
# Change exactly ONE item in your patched YAML, rerun the same bag, then:
#   ./scripts/quantify_teaser_reject.sh /path/to/new/full.log
#   ./scripts/loop_closure_log_audit.sh /path/to/new/logdir

set -euo pipefail
cat <<'EOF'
=== Loop closure: one-parameter ablation (do one at a time) ===

A) Geometry verification slightly looser (more accepts, more false-loop risk):
   - loop_closure.teaser.min_inlier_ratio: 0.08 -> 0.06
   OR
   - loop_closure.teaser.min_safe_inliers: 18 -> 14

B) Inter keyframe coverage (more candidate pairs):
   - loop_closure.inter_keyframe_sample_step: 3 -> 2
   OR
   - loop_closure.inter_keyframe_top_k_per_submap: 6 -> 10

C) Retrieval softer:
   - loop_closure.overlap_threshold: 0.12 -> 0.10
   OR
   - loop_closure.scancontext.dist_threshold: 0.30 -> 0.35

Post-run greps (expect higher LOOP_ACCEPTED if hypothesis A/B/C helps):
  grep -c '\[INTER_KF\]\[LOOP_ACCEPTED\]' full.log
  grep '\[HBA\]\[LOOP_SUMMARY\]' full.log | tail -5

Rollback: revert the single YAML change.
EOF
