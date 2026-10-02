# Working rules and templates

Idea filter, demo script template, slide skeleton, working rules and submission checklist. What we build and the timeline are in [PLAN.md](PLAN.md).

## Idea filter

Before writing code, answer these in writing. If one answer is weak, change the idea or narrow it.

1. Is it feasible by Sunday 10:00 with the people at this table?
2. Is it more than the obvious solution every team will build?
3. Does it solve a real problem for a named user?
4. Can the demo show one input turning into one visible result?
5. Which number proves it works, and what is the baseline for that number?

Write the idea as one sentence of the form "turns X into Y for Z". If that sentence needs an "and", the scope is too wide.

## Demo script template

Write this on Friday and build only what it needs.

- Opening line: who the user is and what goes wrong for them today.
- Step 1: the baseline, shown failing.
- Step 2: our system on the same input.
- Step 3: the number, baseline against ours.
- Step 4: a failure case shown on purpose, and what it means for use.
- Closing line: who would deploy this, on what hardware, at what cost.

Length: ask the organisers how long the demo slot is. Plan for two to three minutes of demo.

## Slide skeleton

Few words per slide. One idea each.

1. The situation, in one picture.
2. The problem, in one sentence.
3. The baseline today, as a chart.
4. The same chart with our system and one number.
5. Live demo, with the recorded video ready as fallback.
6. Where it breaks and why that is acceptable.
7. User, hardware, cost and how it scales.
8. Next steps and the team.

Every brief scores the user and deployment part, so slide 7 is required, not optional.

## Working rules

- One owner per part. Each person can explain their part alone.
- Commit small and often to `main` or to short-lived branches. Merge at least every two hours.
- Shared conventions (units, coordinate frames, file formats) go into the README on Friday.
- Data stays in `data/raw/` and is fetched by script, never committed.
- No secrets in the repository. Use `.env`, which is ignored.
- A number only counts if a script in the repository reproduces it.
- Report weaknesses plainly. Judges from the field will find them anyway.
- No cleanup or refactoring after the freeze.

## Submission and packaging

- README: what it does in one sentence, the headline number, how to run it, known limits.
- Demo video, two to three minutes, linked at the top of the README.
- Slides exported to PDF in `docs/`.
- Each teammate's part named in the README.
- Dataset sources and licences listed.
- A check with the organisers on whether the repository may be public.
