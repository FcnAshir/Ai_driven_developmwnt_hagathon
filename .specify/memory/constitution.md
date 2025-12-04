<!-- Sync Impact Report:
Version change: 0.0.0 (initial) -> 1.0.0
Modified principles: None (initial creation)
Added sections: Core Principles, Key Standards, Constraints, Success Criteria, Deliverable Standards, Governance
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: None
-->
# AI/Spec-Driven Book Creation using Docusaurus, Spec-Kit Plus & Claude Code Constitution

## Core Principles

### Spec-Driven Writing
Every chapter, page, and section MUST follow specifications generated through Spec-Kit Plus.

### Technical Accuracy
All explanations MUST be correct, up-to-date, and verifiable through authoritative sources (official docs, academic articles, or standard references).

### Clarity & Accessibility
Writing MUST be understandable for beginner-to-intermediate readers in AI, software development, and DevOps.

### Modularity
Each chapter MUST function as a standalone module that can be iterated independently.

### Tool-Native Workflow
Use Claude Code + Spec-Kit Plus for:
- outline generation
- chapter drafting
- refactoring
- revision control
- consistency enforcement

## Key Standards

### Content Structure
Docusaurus folder structure MUST be followed.
Each chapter MUST include:
- Overview
- Learning Goals
- Core Concepts
- Examples
- Summary
- Review Questions

### Writing Standards
- Style: Clear, educational, and beginner-friendly technical writing.
- Voice: Objective, helpful, and well-structured.
- Reading Level: Flesch-Kincaid Grade 9–11.
- Avoid unnecessary jargon; define all AI-specific terms.

### Technical Standards
All instructions MUST be accurate for 2024–2025 versions of:
- Node.js
- Docusaurus
- GitHub Pages
- Spec-Kit Plus
- Claude Code

### Verification
- Each technical instruction MUST be tested or validated before inclusion.
- All referenced tools, commands, or scripts MUST be executable.

## Constraints

### Book Length
6–10 chapters minimum

### Word Count
10,000 – 20,000 words (entire book)

### Deployment
MUST compile 100% successfully in Docusaurus
MUST deploy without errors to GitHub Pages

### Assets
Include diagrams, code snippets, and walkthroughs where necessary
All images MUST be exportable and consistent with Docusaurus formatting

### Compliance
No copyrighted text
All external references properly cited

## Success Criteria

### Technical Completeness
Book fully builds and deploys on GitHub Pages without warnings or errors.

### Spec Compliance
All chapters strictly follow Spec-Kit Plus specs.
Internal consistency: terminology, structure, code formatting, examples.

### Teaching Quality
A beginner can follow the book end-to-end and complete all tasks.
Each chapter includes working code samples and reproducible steps.

### Documentation Quality
Clear navigation
Search-friendly structure
Proper section hierarchy

### Deliverable Standards
Stable build
GitHub Pages hosted site
Clean repository with versioned content

## Governance
Constitution supersedes all other practices; Amendments require documentation, approval, migration plan.
All PRs/reviews MUST verify compliance; Complexity MUST be justified; Use `.specify/memory/constitution.md` for runtime development guidance.

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04
