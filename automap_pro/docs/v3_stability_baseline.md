# V3 Stability Baseline

This document defines executable acceptance gates for V3 architecture stability.
Use it after every meaningful code change.

## Scope

- V3 architecture boundaries
- Loop closure default pipeline (OT model -> TEASER)
- Fallback policy strictness
- Concurrency and observability signals

## Mandatory Contract Gates

The following must hold in source/config:

1. Default coarse matching is OT model based (`overlapTransformer.pt`).
2. Default geometry verification is TEASER based.
3. Silent fallback is disabled by default:
   - no automatic ScanContext switch on OT load failure
   - no descriptor fallback by default
   - no SVD geometry fallback by default
4. Flow mode is explicit (`strict` or `safe_degraded`) and readable from config.
5. OT/TEASER unavailability must emit event-level alerts (`LOOP_EVENT` counters + logs).
6. Loop flow observability is present (`LOOP_FLOW`, `TEASER_PATH`, `CONSTRAINT_KPI`).
7. Structured runtime KPI fields exist:
   - queue drop (`desc_queue_drop`, `match_queue_drop`)
   - TEASER concurrency (`teaser_inflight_now`, `teaser_inflight_peak`)
   - TEASER latency (`teaser_p95_ms`)
   - query window acceptance (`accept_ratio_window`)

## Execution

Run from repository root:

```bash
./automap_pro/scripts/check_v3_stability_baseline.sh
```

With runtime log verification:

```bash
./automap_pro/scripts/check_v3_stability_baseline.sh --log "/path/to/full.log"
```

Optional local shortcuts (not for CI):

```bash
./automap_pro/scripts/check_v3_stability_baseline.sh --skip-test-check
./automap_pro/scripts/check_v3_stability_baseline.sh --skip-build-check --skip-test-check
```

## Gate Semantics

- Exit code `0`: build gate + enabled checks pass.
- Exit code non-zero: at least one gate fails (must fix before merge/deploy).

## CI Suggestion

Add this command as a required gate (it already runs build/test gates first):

```bash
./automap_pro/scripts/check_v3_stability_baseline.sh
```

For nightly or integration pipelines that produce logs:

```bash
./automap_pro/scripts/check_v3_stability_baseline.sh --log "$ARTIFACT_LOG"
```

Recommended hard gate script for CI:

```bash
./automap_pro/scripts/ci_v3_gate.sh "$ARTIFACT_LOG"
```

## Runtime Evidence Keywords

When a log is provided, the checker validates the existence of:

- `LOOP_FLOW`
- `TEASER_PATH`
- `CONSTRAINT_KPI][LOOP`

For strict mode validation, it also checks at least one of:

- `LOOP_FLOW][STRICT`
- `descriptor_fallback_disabled`

## Notes

- This baseline intentionally favors stability over permissive fallback behavior.
- If a project phase requires relaxed fallback, treat it as an explicit policy exception and document it in the PR.
