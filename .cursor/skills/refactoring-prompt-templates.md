# Skill: Refactoring Prompt Templates

## Purpose
A reusable prompt library for planning and reviewing refactors in large projects.

## Templates

### Template 1: Analyze Before Refactoring
Do not modify code yet.

Use large-project-refactor-design and provide:
1. current design problems
2. problem level
3. coupling points
4. hidden constraints
5. minimal-change option
6. balanced evolutionary option
7. ideal target-state option
8. recommended option

---

### Template 2: Refactor with Trade-Off Comparison
Do not give only one refactor suggestion.

Compare:
- minimal option
- balanced option
- ideal option

For each option, explain:
- benefits
- risks
- compatibility impact
- migration complexity
- implementation cost
- whether incremental rollout is possible

Then recommend one.

---

### Template 3: Responsibilities and Boundaries
Analyze this code from these perspectives:
- responsibility separation
- module boundaries
- data model consistency
- duplicated logic
- exception flow
- hidden coupling

Do not rewrite yet. First identify structural problems.

---

### Template 4: Incremental Refactor Plan
Assume we want a safe refactor in a large project.

Provide:
- target design
- what can be improved now
- what should remain unchanged for now
- phased rollout steps
- how to preserve compatibility
- what tests should protect the migration

---

### Template 5: Refactor Without Breaking Contracts
Before proposing implementation, identify:
- external contracts that must remain stable
- internal contracts that may need migration
- data shape changes
- serialization concerns
- backward compatibility risks

Then propose the safest refactor plan.

---

### Template 6: Is This a Refactor or a Rewrite?
Assess whether this task is:
- local cleanup
- focused refactor
- cross-module redesign
- rewrite disguised as refactoring

Explain the risk level and the appropriately scoped plan.

---

### Template 7: Controlled Refactor Plan
Use the refactor analysis and provide:
- recommended option
- file/module plan
- migration order
- fallback considerations
- regression risks
- test strategy

Do not implement until confirmed.

---

### Template 8: Architecture-Level Refactor Review
Evaluate the issue across:
- data flow
- state transitions
- layer responsibilities
- coupling direction
- failure propagation
- observability

Then explain:
- short-term fix
- medium-term refactor
- long-term target architecture