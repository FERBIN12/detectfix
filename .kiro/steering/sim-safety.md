---
inclusion: always
---
# Working rules for agents in this repo

- Never start, stop or kill a running simulation, autopilot, RViz or QGroundControl process you did not start yourself.
  Use only the test port 8100 for web servers; 8099 is the live demo.
- Stay inside your task scope; if a failing test is outside it, report it instead of editing it.
- Do not change git config or commit as another person; leave changes uncommitted unless asked.
- Target hardware: RTX 4050 laptop (6 GB VRAM). Keep geometry instanced and per-frame work non-blocking (the Isaac loop
  runs at ~16 fps; anything called per frame must cost a few ms).
