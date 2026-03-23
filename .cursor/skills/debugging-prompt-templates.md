# Skill: Debugging Prompt Templates

## Purpose
A reusable prompt library for investigating bugs and incidents in large projects.

## Templates

### Template 1: Root Cause Analysis First
Do not modify code yet.

Use root-cause analysis and provide:
1. observed issue
2. confirmed facts
3. likely inferences
4. missing information
5. top 3 candidate root causes ranked by likelihood
6. validation steps for each
7. the smallest next debugging step

Focus on root cause, not symptom-level patching.

---

### Template 2: Explicit Uncertainty
Do not jump to implementation.

Analyze the problem and explicitly separate:
- confirmed facts
- likely inferences
- low-confidence assumptions
- missing runtime evidence

Then provide:
- the most likely root causes
- what evidence is needed to confirm each one
- what code/log/config should be inspected next

---

### Template 3: Cross-Module Incident Analysis
Use deep-analysis-sop.

Analyze across:
- call chain
- data flow
- state flow
- exception propagation
- contract boundaries

Do not assume the visible error point is the real cause.

---

### Template 4: Performance Issue Investigation
Do not propose optimizations yet.

First identify:
- likely bottlenecks visible in code
- what must be verified by runtime evidence
- what metrics/logs/traces would best distinguish candidates
- the smallest next validation step

---

### Template 5: Intermittent Failure Investigation
Assume this issue is intermittent.

Check possibilities involving:
- race conditions
- timing-sensitive behavior
- cache inconsistency
- partial state
- retries
- async processing
- environment differences
- historical data anomalies

Rank candidates and explain how to validate each.

---

### Template 6: Impact Review Before Fix
Before modifying code, use impact-scope-review.

List:
- impacted modules
- impacted contracts
- risk areas
- recommended tests
- compatibility concerns

Do not edit code until the review is complete.

---

### Template 7: Controlled Fix Plan
Assume the likely root cause is identified.

Use fix-plan-generator and provide:
- problem summary
- root cause summary
- fix goal
- change boundary
- planned file/module changes
- main risks
- why this plan addresses root cause
- test and rollback suggestions

Do not implement until confirmed.

---

### Template 8: Challenge the Current Diagnosis
Assume your current conclusion may be wrong.

Provide:
- the strongest alternative explanation
- what observed facts are not fully explained
- two competing hypotheses
- what evidence would quickly distinguish them

Do not collapse uncertainty too early.