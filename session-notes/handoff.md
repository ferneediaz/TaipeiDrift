# Handoff

Last updated: 2 October 2026, night before kickoff.

## Current objective

Arrive at kickoff with a working environment, data on disk and a shortlist, then choose the challenge with the team.

## Completed

- All nine challenges read and condensed in `docs/challenges.md`.
- Shortlist with demo, approach, roles and plan in `docs/pitches.md`: Challenge 2, Challenge 7 with 8, Challenge 3. Challenge 6 is reported as taken.
- Weekend playbook in `docs/playbook.md`: timeline with gates, idea filter, demo script, slide skeleton, working rules.
- Python 3.12 arm64 environment via uv (`pyproject.toml`, `uv.lock`). Imports verified, PyTorch MPS available, laptop microphone detected.
- Audio datasets for Challenge 7 in `data/raw/` (drone audio 803 MB, ESC-50 846 MB, outdoor test set 152 MB).
- Evidence from earlier EDTH hackathons added to `docs/pitches.md`. Timeline corrected to Demo Day on Sunday 13:00.
- Repository initialised and pushed to github.com/dwn97/DefenseHackathon (private).

- Team decision briefing in `docs/challenge-decision.md` (all nine challenges, links, roles, risks). Its mentor table must be removed before the repository goes public.

## Files changed

- `README.md`, `pyproject.toml`, `uv.lock`, `.gitignore`
- `docs/challenges.md`, `docs/pitches.md`, `docs/kickoff.md`, `docs/playbook.md`
- `data/README.md`, `scripts/fetch_data.sh`
- `session-notes/handoff.md`

## Open issues

- Challenge not chosen. Team of six formed. Proposal: Challenge 2 with a gate on Friday 22:00, Challenge 7 as fallback.
- Venue unclear: Nangang District (EDTH page) or National Taiwan University (tdth.org). Check the acceptance email.
- Unknown whether pre-written code is allowed, so no product code exists.
- No navigation dataset for Challenge 2. The organisers suggest one; ask for it.
- Dataset licences need a check with the organisers (see `data/README.md`).
- The native uv is at `~/.local/bin/uv`; the `uv` on the shell path is still the Intel build.
- Docker is installed but not running. Needed only for the Challenge 8 benchmark.
- No project-specific CLAUDE.md yet. Write one once the challenge is fixed.

## Next exact step

At kickoff, ask the questions in `docs/kickoff.md`, form the team, pick the challenge with "How to choose" in `docs/pitches.md`. Then run the idea filter and write the demo script from `docs/playbook.md`, and scaffold the chosen challenge.
