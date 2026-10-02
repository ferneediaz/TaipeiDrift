# Handoff

Last updated: Friday 2 October 2026, afternoon, after the opening.

## Current objective

Choose the challenge as a team of six, then start building.

## Completed

- Team of six formed, team name Taipei Drift. Repository renamed to github.com/dwn97/TaipeiDrift (private), collaborators invited.
- Decision briefing for the team in `docs/challenge-decision.md`: all nine challenges with data, tools, earlier entries, roles and risks.
- Demo scripts and weekend plans for Challenges 2, 7, 3 and 6 in `docs/plans.md`.
- Timeline, idea filter, slide skeleton and working rules in `docs/playbook.md`. Code freeze Sunday 10:00, Demo Day 13:00.
- Condensed briefs of all nine challenges in `docs/challenges.md`.
- Python 3.12 arm64 environment via uv (`pyproject.toml`, `uv.lock`). Imports verified, PyTorch MPS available, laptop microphone detected.
- Audio datasets for Challenge 7 in `data/raw/`: drone audio 803 MB, ESC-50 846 MB, outdoor test set 152 MB.

- Research on Challenge 2 in `docs/challenge-2-research.md`; two aerial images of one Taichung site (2018, 2020) fetched by `scripts/fetch_aerial.py` and checked: they line up and cover a corridor about 3 km long.
- Research on Challenge 6 in `docs/challenge-6-research.md`.
- Research on Challenge 4 in `docs/challenge-4-research.md`; drone video set (190 MB) in `data/raw/drone_video`.

## Proposal on the table

Challenge 2 (navigation without GPS) with the mentor's design: optical flow with altitude for speed, a sun compass for heading and cold start, map matching for position fixes. Gate on Friday 22:00. Fallback: Challenge 7 (acoustic drone detection), without Challenge 8 as a separate entry. Challenge 4 is a third candidate after the research.

## Open issues

- The team has not voted yet.
- The second item the mentor named for the cold start, besides the position of the sun, is not known. Ask him again.
- Challenge 2 data: aerial images are available; the flight simulator over them is not built. The organiser dataset has not been seen.
- No stereo microphone for direction finding in Challenge 7. Ask whether one can be borrowed.
- Dataset licences need a check with the organisers (see `data/README.md`).
- The mentor table in `docs/challenge-decision.md` must be removed before the repository is made public. `docs/challenges.md` paraphrases the members-only challenge page.
- The native uv is at `~/.local/bin/uv` on Dustin's laptop; the `uv` on the shell path is still the Intel build.
- No project-specific CLAUDE.md yet. Write one once the challenge is fixed.

## Next exact step

Team reads `docs/challenge-decision.md` and votes. Then follow "First hour after choosing" in `docs/playbook.md`. If Challenge 2 is chosen, build the gate test first: simulated flight frames and speed from optical flow against the true speed.
