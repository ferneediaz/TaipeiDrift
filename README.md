# Taipei Drift

Repository of team Taipei Drift for the Taiwan Defense Tech Hackathon 2026, organised by EDTH and the Unmanned Vehicles R&D Center at National Taiwan University, 2 to 4 October 2026 in Taipei.

The challenge is not chosen yet. This repository currently holds the shared environment, the datasets and the briefing documents. No product code has been written.

## Contents

- [docs/challenge-decision.md](docs/challenge-decision.md): **start here**, the briefing for choosing our challenge, with links to data, tools and earlier entries
- [docs/challenges.md](docs/challenges.md): the nine challenges, condensed
- [docs/landscape.md](docs/landscape.md): existing products, their limits, and the proposal for what we build
- [docs/experiments.md](docs/experiments.md): first experiments on real Taiwan data, laser against camera, and crossing water
- [docs/challenge-2-research.md](docs/challenge-2-research.md): fielded systems, limits and options for the navigation challenge
- [docs/challenge-4-research.md](docs/challenge-4-research.md): existing products, limits and options for the drone detection challenge
- [docs/challenge-6-research.md](docs/challenge-6-research.md): existing products, limits and options for the maritime challenge
- [docs/plans.md](docs/plans.md): target user, demo script and weekend plan for each candidate challenge
- [docs/playbook.md](docs/playbook.md): timeline, idea filter, demo script, slide skeleton and working rules
- [data/README.md](data/README.md): datasets, formats and licences
- [session-notes/handoff.md](session-notes/handoff.md): current state and next step

## Setup

Requires [uv](https://docs.astral.sh/uv/). On an Apple Silicon Mac, use the native arm64 build of uv, otherwise the environment runs under Rosetta and model training is several times slower.

```bash
uv sync                      # creates .venv with Python 3.12 and all libraries
bash scripts/fetch_data.sh   # downloads the audio datasets, about 2 GB
source .venv/bin/activate
```

Check the environment:

```bash
python -c "import platform, torch; print(platform.machine(), torch.backends.mps.is_available())"
```

On an Apple Silicon Mac this should print `arm64 True`.

## Installed libraries

- Shared: numpy, scipy, pandas, matplotlib, scikit-learn, jupyterlab, pytest
- Challenge 2: filterpy, pyproj
- Challenge 7 and 8: librosa, soundfile, sounddevice, torch, torchaudio, onnx, onnxruntime
- Challenge 3: fastapi, uvicorn, pydantic
