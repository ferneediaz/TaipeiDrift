# Taipei Drift

Repository of team Taipei Drift for the Taiwan Defense Tech Hackathon 2026, organised by EDTH and the Unmanned Vehicles R&D Center at National Taiwan University, 2 to 4 October 2026 in Taipei.

We are working on Challenge 2, navigation without GNSS. The plan is proposed and awaits the team's confirmation. The repository holds the shared environment, the research and experiment scripts on the Mid-Air and ALTO datasets. Product code has not been started.

## Contents

- [docs/TaipeiDrift_Pitch.pdf](docs/TaipeiDrift_Pitch.pdf): the pitch deck for Demo Day, 18 slides: nine for the three-minute pitch, then the backup slides with costs, market, competition and limits
- [docs/pitch-offline/](docs/pitch-offline/): the same deck as a web page that runs without internet, with the demo video; open `index.html` in a browser, the keys are in its `README.txt`
- [docs/findings.md](docs/findings.md): **read first**, what we measured on Friday night and what it changes
- [docs/PLAN.md](docs/PLAN.md): the plan: what we build, roles, timeline, demo
- [docs/brief.md](docs/brief.md): what the challenge asks for
- [docs/method.md](docs/method.md): how image matching and the particle filter work, with worked numbers
- [docs/landscape.md](docs/landscape.md): existing products and their limits
- [docs/experiments.md](docs/experiments.md): earlier experiments on Taiwan data, made before the dataset was chosen, with scripts in `experiments/`
- [docs/challenge-2-research.md](docs/challenge-2-research.md): papers, data sources and reading list
- [docs/playbook.md](docs/playbook.md): working rules, slide skeleton and submission checklist
- [data/README.md](data/README.md): datasets and licences
- [research/](research/): links to the two key papers (the PDFs may not be redistributed)
- [sim/README.md](sim/README.md): ROS 2 + Gazebo simulator with a Mid-Air-like drone (camera, IMU, barometer, GPS)
- [session-notes/handoff.md](session-notes/handoff.md): current state and next step

## Setup

Requires [uv](https://docs.astral.sh/uv/). On an Apple Silicon Mac, use the native arm64 build of uv.

```bash
uv sync                              # creates .venv with Python 3.12 and the libraries
source .venv/bin/activate
```

The datasets are not in the repository. [data/README.md](data/README.md) says how to get Mid-Air (about 10 GB, through a form and `scripts/fetch_midair.sh`) and ALTO (1.73 GB, from Dropbox in a browser).
