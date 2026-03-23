# Skill: Fix Plan Generator

## Purpose
Use this skill after diagnosis and before implementation to create a reviewable, controlled fix plan.

## When to Use
Use this skill when:
- The likely root cause has been narrowed down
- The user wants a structured implementation plan
- The change is non-trivial or crosses multiple files/modules

## Required Behavior
You must create a plan that includes:
- problem summary
- root cause summary
- fix goal
- change boundary
- planned files/modules
- purpose of each planned change
- main risk points
- why the plan addresses root cause instead of only the symptom
- test strategy
- rollback/fallback notes if relevant

## Output Format

### Problem Summary
...

### Root Cause Summary
...

### Fix Goal
...

### Change Boundary
- In scope:
- Out of scope:

### Planned Changes
1. File/module:
   - Purpose:
   - Expected change:

2. File/module:
   - Purpose:
   - Expected change:

3. File/module:
   - Purpose:
   - Expected change:

### Main Risks
- ...
- ...

### Why This Plan Is Root-Cause-Oriented
...

### Test Plan
- Unit:
- Integration:
- Regression:
- Edge cases:

### Rollback / Fallback Notes
...

## Constraints
- Do not implement during planning
- Keep the plan minimal but complete
- Respect explicit do-not-touch areas
- If uncertainty is still high, say so before planning implementation