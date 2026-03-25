# Agent instructions (Automap Pro)

This file guides AI agents and automation working in this repository. Project-specific Cursor rules live under `.cursor/rules/`.

## Official documentation (required for fixes)

When **diagnosing bugs** or **implementing fixes** that involve behavior outside purely local application code, agents **must consult primary official sources** before treating an approach as final.

**Applies especially to:** language semantics and stdlib; compiler and linker; build systems (CMake, colcon, etc.); third-party libraries and SDKs; ROS and related middleware; OS or distro packages; vendor APIs and hardware interfaces.

**Expectations:**

1. Use **vendor/upstream documentation, manuals, release notes, migration guides, or versioned reference pages** — not memory or unverified secondary articles alone.
2. **Name the source** (document + section or URL) when it materially supports the diagnosis or the patch.
3. If official docs conflict with local assumptions, **surface the conflict**; align the fix with documented contracts unless the repo explicitly documents an intentional deviation or pinned behavior.

**Concrete entry points in this repo:**

- Always-applied: `.cursor/rules/global-engineering.mdc` (§3a), `.cursor/rules/001-core-analysis-methodology.mdc` (PRINCIPLE 1a, PRINCIPLE 7 checklist, anti-pattern on guessing APIs).
- On-demand skill: `.cursor/rules/015-skill-official-documentation.mdc` — invoke for toolchain, library, and migration issues.
- Subagent rules under `.cursor/rules/020-*.mdc` … `024-*.mdc` include matching mandates for root-cause, impact, archaeology, refactor, and verification roles.

Agents should **read the relevant rule or skill file** when the task touches external or platform behavior, then follow it for that session.
