# Skill: Large Project Refactor Design

## Purpose
Use this skill to design safe, practical refactoring strategies for large projects.

## When to Use
Use this skill when:
- Responsibilities are mixed
- Module boundaries are unclear
- Logic is duplicated
- A local patch would increase technical debt
- The user wants options and trade-off analysis

## Required Behavior
You must:
1. Analyze the current design problem before proposing implementation
2. Classify the problem level:
   - function-level
   - class-level
   - module-level
   - architecture-level
3. Identify coupling points and hidden constraints
4. Propose at least 3 options:
   - minimal-change option
   - balanced evolutionary option
   - ideal target-state option
5. Compare options by:
   - benefits
   - risks
   - compatibility impact
   - migration complexity
   - implementation effort
6. Recommend one option
7. Provide phased rollout steps
8. Provide validation strategy

## Output Format

### Current Design Problems
- ...
- ...

### Problem Level
...

### Coupling / Constraint Analysis
- ...
- ...

### Refactor Options

#### Option 1: Minimal Change
- Approach:
- Benefits:
- Risks:
- Compatibility impact:
- Migration complexity:
- Cost:

#### Option 2: Balanced Evolution
- Approach:
- Benefits:
- Risks:
- Compatibility impact:
- Migration complexity:
- Cost:

#### Option 3: Ideal Target State
- Approach:
- Benefits:
- Risks:
- Compatibility impact:
- Migration complexity:
- Cost:

### Recommended Option
...

### Phased Rollout Plan
1. ...
2. ...
3. ...

### Validation Strategy
- Functional validation:
- Regression validation:
- Compatibility validation:

### Deferred Technical Debt
- ...
- ...

## Constraints
- Do not rewrite by default
- Prefer incremental migration in large projects
- Explicitly identify what should not be changed immediately