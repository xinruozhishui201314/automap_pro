# Skill: Deep Analysis SOP

## Purpose
Use this skill for complex issues in large projects when a more thorough and systematic analysis is needed.

## When to Use
Use this skill when:
- The issue spans multiple modules or layers
- The symptom may hide a deeper cause
- The user wants a complete investigation workflow
- Previous analysis was too shallow or too local

## Required Behavior
Follow this SOP strictly unless the user explicitly asks for a narrower task.

## SOP

### Step 1. Define the Problem
Clarify:
- what is happening
- what should happen
- trigger conditions
- whether the issue is stable, intermittent, data-dependent, or environment-dependent

### Step 2. Clarify Scope and Constraints
Identify:
- what modules/layers are involved
- what areas are out of scope
- compatibility or release constraints
- any do-not-touch boundaries

### Step 3. Gather High-Signal Context
Request only the most relevant:
- code
- logs
- traces
- configs
- tests
- recent changes

### Step 4. Map Execution Path
Describe:
- call chain
- data flow
- state flow
- error/exception propagation

### Step 5. Separate Facts from Assumptions
List:
- confirmed facts
- reasonable inferences
- unknowns

### Step 6. Build Candidate Explanations
Propose multiple root causes or design diagnoses, ranked by likelihood or importance.

### Step 7. Design Validation
For each major candidate, specify:
- what to inspect
- what result confirms it
- what result weakens it

### Step 8. Review Impact Scope
Identify:
- modules/contracts affected
- direct and indirect risks
- likely test scope

### Step 9. Form Solution Options
Provide:
- minimal fix
- balanced fix
- ideal long-term approach

### Step 10. Review Regression and Risk
Provide:
- testing strategy
- edge cases
- compatibility concerns
- rollback/fallback considerations

## Output Format

### Step 1. Problem Definition
...

### Step 2. Scope and Constraints
...

### Step 3. High-Signal Context
...

### Step 4. Execution Path
...

### Step 5. Facts vs Assumptions
- Confirmed facts:
- Inferences:
- Unknowns:

### Step 6. Candidate Explanations
1. ...
2. ...
3. ...

### Step 7. Validation Plan
...

### Step 8. Impact Scope
...

### Step 9. Solution Options
- Minimal:
- Balanced:
- Ideal:

### Step 10. Regression and Risk Review
...

## Constraints
- Do not skip directly from symptom to implementation
- Do not collapse uncertainty too early
- Maintain strict separation between evidence and interpretation