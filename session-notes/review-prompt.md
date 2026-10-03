# Prompt: a deep review of the TaipeiDrift repository

Review the repository at `~/Projects/DefenseHackathon-sim` (branch `integration`) as if your job were to
stop us from embarrassing ourselves in front of the jury. It is a hackathon entry for drone navigation
without GNSS; the challenge brief is `docs/brief.md`. The code freeze is Sunday 4 October 10:00, Taipei
time, followed by a 3-minute demo.

Find what is wrong and what is missing: bugs that would change a reported number, claims the evidence does
not support, and gaps a jury would notice. Read the code and the docs and judge for yourself.

## Rules

- Read only: no edits, commits or pushes. Your answer is the report.
- Never run, open or plot `recordings/wufeng_north_90m` or `recordings/wufeng_south_110m`. They are sealed
  for the final test.
- You may run the tests and short scripts (Python: `~/.venvs/defensehackathon/bin/python`). Do not start
  the simulator, and keep any job under about 10 minutes.
- Verify before you report. Mark each finding CONFIRMED (reproduced or traced without doubt) or SUSPECTED.

## Your answer

1. Findings, most severe first: CONFIRMED or SUSPECTED; severity (A: changes a reported number or claim,
   B: could mislead, C: minor); `file:line`; what is wrong; a concrete case where it goes wrong; how you
   verified it; the smallest fix.
2. What is missing for the submission, with an estimate of the work.
3. At most five quick wins before the freeze.

Write plainly, in short sentences.
