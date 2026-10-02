# Taipei Drift

Repository of team Taipei Drift for the Taiwan Defense Tech Hackathon 2026, organised by EDTH and the Unmanned Vehicles R&D Center at National Taiwan University, 2 to 4 October 2026 in Taipei.

We are working on Challenge 2, navigation without GNSS. The plan is proposed and awaits the team's confirmation. The repository holds the shared environment, the datasets, the research and first experiment scripts. Product code has not been started.

## Contents

- [docs/PLAN.md](docs/PLAN.md): **start here**, the plan: what we build, roles, timeline, demo
- [docs/brief.md](docs/brief.md): what the challenge asks for
- [docs/landscape.md](docs/landscape.md): existing products and their limits
- [docs/experiments.md](docs/experiments.md): first experiments on real Taiwan data, with scripts in `experiments/`
- [docs/challenge-2-research.md](docs/challenge-2-research.md): papers, data sources and reading list
- [docs/playbook.md](docs/playbook.md): working rules, slide skeleton and submission checklist
- [data/README.md](data/README.md): datasets and licences
- [research/](research/): the two key papers as PDF
- [session-notes/handoff.md](session-notes/handoff.md): current state and next step

## Setup

Requires [uv](https://docs.astral.sh/uv/). On an Apple Silicon Mac, use the native arm64 build of uv.

```bash
uv sync                              # creates .venv with Python 3.12 and the libraries
source .venv/bin/activate
python scripts/fetch_aerial.py       # two aerial images of Taichung, about 27 MB
python experiments/c_terrain_matching.py   # fetches the elevation strip on first run and prints the terrain results
```

The downloads are small on purpose. The venue Wi-Fi is slow.
