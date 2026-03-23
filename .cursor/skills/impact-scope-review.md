# Skill: Impact Scope Review

## Purpose
Use this skill before code changes to assess what may be affected by a bug fix, behavior change, or refactor in a large project.

## When to Use
Use this skill when:
- A non-trivial code change is being planned
- Shared logic or public contracts may be affected
- A change may cross layers or modules
- The user wants a more complete solution than a local patch

## Required Behavior
You must review likely impact across:
- entry points
- service/application logic
- domain logic
- repository/data access
- DTO / VO / entity / schema consistency
- serialization / deserialization / mapping
- cache behavior and cache keys
- transactions and side effects
- async/background jobs
- configuration flags and environment differences
- external contracts
- observability assumptions
- tests, mocks, fixtures, and documentation

## Review Process
1. Summarize the proposed change
2. Identify likely affected modules/layers/files
3. Identify contract and compatibility risks
4. Identify data/state risks
5. Identify operational/runtime risks
6. Recommend test scope
7. Call out remaining uncertainties

## Output Format

### Proposed Change
...

### Potentially Impacted Areas
- Module / layer:
  - Why impacted:
  - Risk level:
  - What to review/update:

- Module / layer:
  - Why impacted:
  - Risk level:
  - What to review/update:

### Contract Risks
- ...
- ...

### Data / State Risks
- ...
- ...

### Operational Risks
- ...
- ...

### Recommended Test Scope
- Unit tests:
- Integration tests:
- Regression tests:
- Manual verification:

### Remaining Uncertainties
- ...
- ...

## Constraints
- Do not implement changes during this review
- Do not limit review to the currently open file
- Explicitly call out hidden blast-radius risks