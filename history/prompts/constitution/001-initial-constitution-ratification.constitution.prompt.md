---
id: 001
title: Initial Constitution Ratification
stage: constitution
date: 2025-12-17
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: master
user: Waqar Ali
command: /sp.constitution
labels: ["constitution", "project-setup", "rag-chatbot", "docusaurus"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - null
---

## Prompt

Project: AI-Spec–Driven Book with Embedded RAG Chatbot

Purpose:
Author and publish a technical book using Spec-Kit Plus and Claude Code, deploy it via Docusaurus on GitHub Pages, and embed a RAG chatbot that answers questions grounded strictly in the book content or user-selected text.

Core Principles:
- Spec-first, AI-native development
- Technical accuracy, no hallucinations
- Reproducibility and open-source compatibility
- Modular, maintainable architecture

Book Requirements:
- Generated using Spec-Kit Plus + Claude Code
- Framework: Docusaurus
- Deployment: GitHub Pages
- Audience: Intermediate–advanced developers
- Style: Technical, concise, instructional
- Chapters include objectives, examples, summaries
- Code examples must be correct and runnable

RAG Chatbot Requirements:
- Embedded in book UI
- Stack: FastAPI, OpenAI Agents/ChatKit SDKs
- Vector DB: Qdrant Cloud (Free Tier)
- Relational DB: Neon Serverless Postgres
- Indexes full book content
- Answers only from retrieved context
- Strict "selected-text-only" answering mode
- Refuses out-of-scope questions

Context Rules:
- If text is selected: retrieval limited to selection only
- If no selection: retrieval allowed from full book corpus
- Response must indicate context scope

Constraints:
- Free-tier compatible services only
- No hardcoded secrets
- Single-developer deployable

Quality Gates:
- Successful Docusaurus build and GitHub Pages deploy
- Accurate, grounded chatbot responses
- No hallucinated content

Success Criteria:
- Live AI-generated book on GitHub Pages
- Fully functional embedded RAG chatbot
- Reproducible setup via documentation

## Response snapshot

Created project constitution v1.0.0 for "AI-Spec–Driven Book with Embedded RAG Chatbot" with 6 core principles:

1. **Spec-First Development** - All work begins with specifications
2. **Technical Accuracy (No Hallucinations)** - All content must be verifiable
3. **Reproducibility and Open-Source Compatibility** - Reproducible from docs alone
4. **Modular Architecture** - Clear separation of concerns
5. **Grounded Context Retrieval** - Chatbot answers only from indexed content
6. **Free-Tier Sustainability** - All services free-tier compatible

Added Technical Standards, Quality Gates, and Governance sections with versioning policy.

## Outcome

- ✅ Impact: Established project foundation with 6 principles, tech stack, and quality gates
- 🧪 Tests: N/A (constitution document)
- 📁 Files: 1 file created (.specify/memory/constitution.md)
- 🔁 Next prompts: /sp.specify for first feature, project structure setup
- 🧠 Reflection: Constitution captures all user requirements; ready for feature development

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Test constitution compliance during first /sp.plan
