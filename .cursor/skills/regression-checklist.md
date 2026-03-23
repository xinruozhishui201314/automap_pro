# Skill: Regression Checklist

## Purpose
Use this skill after a fix or refactor to review possible regressions, missed scenarios, and release risks.

## When to Use
Use this skill when:
- Code changes are complete
- A PR is being prepared
- A release is approaching
- A non-trivial change affects shared logic or contracts

## Required Behavior
You must check:
- main success path
- failure path and exception handling
- null / empty / boundary values
- timeout / retry behavior
- concurrency and race conditions
- cache consistency and invalidation
- transaction consistency
- async side effects
- contract compatibility
- legacy behavior compatibility
- test coverage gaps
- rollback/fallback feasibility

## Output Format

### Change Summary
...

### Covered Scenarios
- ...
- ...

### Potentially Missed Scenarios
- ...
- ...

### Recommended Tests
- Unit tests:
- Integration tests:
- Regression tests:
- Manual checks:

### Release Risks
- ...
- ...

### Rollback / Fallback Considerations
- ...
- ...

## Constraints
- Do not assume existing tests are sufficient
- Be explicit about compatibility and release concerns
- Focus on realistic edge cases, not only happy paths