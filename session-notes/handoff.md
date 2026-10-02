# Handoff

Last updated: Friday 2 October 2026, evening.

## Current objective

Confirm the plan as a team and start building.

## State

- Challenge 2, navigation without GNSS. Team of six, team name Taipei Drift, repository github.com/dwn97/TaipeiDrift (private).
- The single plan is `docs/PLAN.md`: a navigator, an integrity check and a navigability map, all on open map data, with roles, shared conventions, timeline, gates and demo script. Not yet confirmed by the team.
- Research: `docs/landscape.md` (existing products and gaps), `docs/challenge-2-research.md` (papers, data, reading list), `research/` (two papers as PDF).
- Experiments on real Taiwan data in `experiments/`, results in `docs/experiments.md`. They contain working first versions of the particle filter with wind, terrain matching and camera matching.
- Data: two aerial images of Wufeng, Taichung (`scripts/fetch_aerial.py`) and an elevation strip at 24.05 N (fetched by the terrain experiment).
- Environment: Python 3.12 via uv. Audio and deep-learning libraries were removed when the challenge was fixed.

## Removed on Friday evening

Docs for the challenges not chosen (decision briefing, Challenge 4 and 6 research, per-challenge plans, the nine condensed briefs) and the audio and video fetch script. They remain in the git history. The audio and video datasets are still on Dustin's laptop under `data/raw/` and can be deleted.

## Open issues

- The team has not confirmed the plan or assigned roles.
- Product code is not started. The code structure for six people still has to be set up.
- The second input the mentor named for the cold start is unknown.
- No real flight data. A 360-degree camera or phone recording would add it.
- `docs/PLAN.md` contains a mentor table and `research/` contains other authors' papers. Remove both before the repository is made public. `docs/brief.md` paraphrases the members-only challenge page.
- The native uv is at `~/.local/bin/uv` on Dustin's laptop; the `uv` on the shell path is still the Intel build.
- No project-specific CLAUDE.md yet.

## Next exact step

Team reads `docs/PLAN.md`, confirms or changes it, and writes names into the roles table. Then set up the code structure and get the dead-reckoning baseline plot running, which is the Friday 23:00 gate.
