# Weekend playbook

A plan that works for whichever challenge the team picks. The hackathon runs Friday 12:00 to Sunday 17:00. According to the official site (tdth.org), doors open Friday at 12:00, the opening is at 14:00, and Demo Day starts Sunday at 13:00 after arrival at 08:30. Confirm these times and the submission deadline at kickoff.

## Timeline

### Friday

| Time | What | Done when |
|---|---|---|
| 12:00 to 14:00 | Doors open, questions to the organisers, team forming | Team of 3 to 6, challenge and fallback agreed |
| 14:00 to 17:00 | Opening, then idea filter, demo script, roles, setup | Demo script written, every laptop runs the environment |
| 17:00 to 22:00 | Thinnest end-to-end version | Real input goes in, something shows on screen, however crude |

Gate at 22:00: if nothing runs end to end, cut scope before going to sleep.

### Saturday

| Time | What | Done when |
|---|---|---|
| 09:00 to 13:00 | Core method working on real data | First honest number against the baseline |
| 13:00 to 14:00 | Mentor round | Shown to at least two mentors, feedback written down |
| 14:00 to 19:00 | Experiments, failure cases, demo interface | The demo script runs start to finish |
| 19:00 to 22:00 | Record a fallback demo video, draft the slides | Video file saved, slide skeleton filled |

Gate at 14:00: if the core method does not beat the baseline yet, switch to the simplest version that does and spend the afternoon on evaluation and the demo.

### Sunday

| Time | What | Done when |
|---|---|---|
| 08:30 to 10:00 | Bug fixes only, slides, README | Nothing new is being built |
| 10:00 | Code freeze | Demo branch tagged, nobody edits it |
| 10:00 to 13:00 | Rehearse three times with a timer, submit | Submission confirmed |
| 13:00 to 17:00 | Demo Day | |

## Idea filter

Before writing code, answer these in writing. If one answer is weak, change the idea or narrow it.

1. Is it feasible by Sunday 12:00 with the people at this table?
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

Length: check the demo slot at kickoff. Plan for two to three minutes of demo.

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
