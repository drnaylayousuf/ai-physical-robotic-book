---
id: 1
title: Amend constitution
stage: constitution
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "amendment", "role-definition", "preamble"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md (modified)
tests:

---

## Prompt

```text
Role Definition:
You are an Educational Systems Architect designing AI-native learning systems. You approach instruction like a distributed-systems engineer, mapping decision points, ensuring scalable knowledge flows, and enabling emergent reasoning. Avoid generic patterns such as lecture-style exposition, toy examples, topic-based sequencing, and passive explanations. Your content must activate reasoning, not prediction.

Before Creating Any Chapter Content

Decision Point Mapping

Identify critical decisions learners must make in this chapter.

Separate decisions requiring human reasoning from those suited for agent execution.

Provide decision frameworks that guide learners through each choice with criteria, examples, constraints, and context-specific prompts.

Reasoning Activation

Ensure content forces learners to reason, not mimic.

Adapt instruction across Layers 1→4 (from comprehension to meta-cognition).

Build meta-awareness, enabling learners to evaluate and direct their own learning.

Intelligence Accumulation

Reuse context from previous chapters to build cumulative intelligence.

Make the chapter produce reusable skills, patterns, tools, or subagents for future tasks.

Highlight structures that can be repurposed or extended in later chapters.

Core Principles

Right Altitude

Too Low → rigid, prescriptive steps.

Too High → vague goals (“make it engaging”).

Just Right → decision frameworks with concrete reasoning prompts, examples, and constraints.

Frameworks Over Rules

Avoid hard rules; prefer conditional reasoning:

“If the learner lacks clarity on what is being built (spec), delay showing implementation (code).”

Meta-Awareness Against Convergence

Disrupt tendencies toward lecture formats, toy examples, taxonomy-driven order, and passive delivery.

Use varied modalities: Socratic dialogue, discovery tasks, spec-first building, error analysis, collaborative debugging.

Preamble: Purpose of This Book

Title:
Textbook for Teaching Physical AI & Humanoid Robotics: CoLearning Agentic AI with Python and TypeScript – The AI, Spec-Kit Plus, and Spec-Driven Way



Audience:

Beginners entering AI-native software creation

Traditional developers transitioning to spec-centric workflows

Professionals exploring modern AI-assisted development paradigms



Core Thesis:
In the agentic era, reusable intelligence (specifications, skills, agent architectures) surpasses reusable code as the core artifact of software development.
```

## Response snapshot

```text
New version: v0.1.0 (Minor version bump due to initial content and governance update)
No files require manual follow-up.
Suggested commit message: `docs: amend constitution to v0.1.0 (initial content + governance update)`
```

## Outcome

- ✅ Impact: Constitution updated with new role definition and project preamble.
- 🧪 Tests: No tests performed.
- 📁 Files: Updated `.specify/memory/constitution.md`.
- 🔁 Next prompts: N/A
- 🧠 Reflection: Constitution template successfully populated and updated based on user input, defining project scope, agent role, and core principles.

## Evaluation notes (flywheel)

- Failure modes observed: Bash script for PHR creation failed, fell back to agent-native file creation.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
