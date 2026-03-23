# Skill: Root Cause Analysis

## Purpose
Use this skill to investigate bugs, incidents, exceptions, incorrect behavior, intermittent failures, and performance problems in large projects before editing code.

## When to Use
Use this skill when:
- A bug or exception is reported
- The visible error point may not be the real cause
- The issue may involve multiple modules, layers, or integrations
- The user wants diagnosis before implementation

## Required Behavior
You must:
1. Start with diagnosis, not implementation
2. Separate:
   - confirmed facts
   - likely inferences
   - low-confidence assumptions
   - missing information
3. Treat the visible failure point as a symptom until proven otherwise
4. Generate multiple candidate root causes before narrowing down
5. Prefer the smallest, highest-signal next debugging step

## Investigation Process
1. Summarize the observed issue
2. List confirmed facts from the available evidence
3. List likely inferences
4. Identify missing context that blocks confident diagnosis
5. Generate at least 3 candidate root causes when possible
6. Rank them by likelihood
7. For each candidate, explain:
   - why it is plausible
   - what evidence would confirm it
   - what evidence would weaken or disprove it
   - what file/log/config/runtime artifact should be inspected next
8. Recommend the smallest next debugging step

## Output Format
Use this structure whenever possible:

### Observed Issue
...

### Confirmed Facts
- ...
- ...

### Likely Inferences
- ...
- ...

### Missing Information
- ...
- ...

### Candidate Root Causes
1. ...
   - Likelihood:
   - Why plausible:
   - What it explains:
   - What it does not fully explain:
   - How to confirm:
   - How to weaken/disprove:
   - Next artifact to inspect:

2. ...
   - Likelihood:
   - Why plausible:
   - What it explains:
   - What it does not fully explain:
   - How to confirm:
   - How to weaken/disprove:
   - Next artifact to inspect:

3. ...
   - Likelihood:
   - Why plausible:
   - What it explains:
   - What it does not fully explain:
   - How to confirm:
   - How to weaken/disprove:
   - Next artifact to inspect:

### Smallest Next Debugging Step
...

### Confidence Notes
- High confidence:
- Medium confidence:
- Low confidence:

## Constraints
- Do not modify code unless explicitly requested after diagnosis
- Do not present assumptions as facts
- If runtime evidence is missing, explicitly state the limitation
- If the issue may involve caching, concurrency, transactions, async jobs, config differences, or contract drift, call that out explicitly