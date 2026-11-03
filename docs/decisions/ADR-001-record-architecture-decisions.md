# ADR-001: Record architecture decisions

- **Status:** accepted
- **Date:** 2026-11-02

## Context

This project is built in phases over several months. Decisions made early (robot design,
interfaces, tooling) shape everything after them, and the reasons behind them are easy to
forget. Reviewers of the repository also want to see *why* the system looks the way it does.

## Decision

Record every significant decision as an Architecture Decision Record (ADR) in
`docs/decisions/`, using [ADR-000-template.md](ADR-000-template.md). ADRs are numbered in
order and never deleted: a changed decision gets a new ADR that supersedes the old one.

A decision is significant if it is hard to reverse, affects more than one package, or
picks one well-known approach over another.

## Alternatives considered

| Option | Why not |
|---|---|
| Decisions only in the project plan | The plan describes intent; ADRs record what was actually decided and when |
| Decisions in commit messages / PRs | Hard to find later; no single place to read the reasoning |

## Consequences

- Each phase PR that makes a significant decision adds or updates an ADR.
- The decision log in the project plan maps to ADRs as they are written.
