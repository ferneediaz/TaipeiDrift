# Handoff

Last updated: 2 October 2026, night before kickoff.

## Current objective

Arrive at kickoff with a working environment, data on disk and a shortlist, then choose the challenge with the team.

## Completed

- All nine challenges read and condensed in `docs/challenges.md`.
- Shortlist with demo, approach, roles and plan in `docs/pitches.md`: Challenge 2, Challenge 7 with 8, Challenge 3. Challenge 6 is reported as taken.
- Python 3.12 arm64 environment via uv (`pyproject.toml`, `uv.lock`). Imports verified, PyTorch MPS available, laptop microphone detected.
- Audio datasets for Challenge 7 in `data/raw/` (drone audio 803 MB, ESC-50 846 MB).
- Repository initialised and pushed to github.com/dwn97/DefenseHackathon (private).

## Files changed

- `README.md`, `pyproject.toml`, `uv.lock`, `.gitignore`
- `docs/challenges.md`, `docs/pitches.md`, `docs/kickoff.md`
- `data/README.md`, `scripts/fetch_data.sh`
- `session-notes/handoff.md`

## Open issues

- Challenge not chosen. Depends on the team formed at kickoff.
- Unknown whether pre-written code is allowed, so no product code exists.
- No navigation dataset for Challenge 2. The organisers suggest one; ask for it.
- Dataset licences need a check with the organisers (see `data/README.md`).
- The native uv is at `~/.local/bin/uv`; the `uv` on the shell path is still the Intel build.
- Docker is installed but not running. Needed only for the Challenge 8 benchmark.
- No project-specific CLAUDE.md yet. Write one once the challenge is fixed.

## Next exact step

At kickoff, ask the questions in `docs/kickoff.md`, form the team, pick the challenge with "How to choose" in `docs/pitches.md`. Then scaffold the chosen challenge and write the demo script.
