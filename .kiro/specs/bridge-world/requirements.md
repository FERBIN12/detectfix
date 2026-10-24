# Bridge world (Isaac Sim 6.0.1), drones first

## Goal
A photoreal Massachusetts river bridge in Isaac Sim that a quadcopter inspects. The robotics is the data source for the
AI (R6: defect detection + risk-based prioritization). The ROV under the water comes later.

## Requirements
1. **Bridge**: a multi-span concrete highway bridge over a river, like the Merrimack River crossings in the MA bridge
   inventory: ~120 m long, 3 spans, 2 river piers with pier caps, deck with parapets, girders under the deck, abutments.
   Scale in metres, Z up.
2. **Photoreal**: NVIDIA MDL Base materials with world-projected textures (Concrete_Formed / Concrete_Rough for piers and
   girders, Asphalt for the deck, Steel for railings); HDRI sky from `NVIDIA/Assets/Skies` + sun; real-time RTX
   (RaytracedLighting) for the live view; a path-traced mode for hero frames. Reuse helpers in `~/glydcar_isaac/photoreal.py`.
3. **Water**: river surface (Natural/Water MDL) at a fixed level; piers go below it; riverbanks with grass/soil and a few
   NVIDIA vegetation trees.
4. **Visible defects for the AI to find**: decals or geometry on piers and girder ends: cracks, spalling with exposed
   rebar, efflorescence/staining, rust streaks. Each defect has a ground-truth record (id, type, severity, bridge element,
   world position) written to `data/defects_gt.json`.
5. **Drone**: the Isaac quadcopter asset (`Isaac/Robots/IsaacSim/Quadcopter` or `Isaac/Robots/Bitcraze/Crazyflie`), flown
   kinematically along an inspection route: around each pier and under the deck, keeping a **standoff distance** (2–4 m)
   from the structure and covering every face. A gimbal camera on the drone looks at the structure.
6. **Data out**: per waypoint, the drone camera RGB (1280x720) + pose + which element it is looking at, saved to
   `data/flight_<time>/`. These images feed the defect detector.
7. **Live view**: a chase camera and the drone camera, the flight runs in a loop; a `--headless` flag records data only.
8. **Runs on an RTX 4050 (6 GB VRAM)**: keep geometry instanced and textures moderate.

## Out of scope for now
ROV, sonar, scour. The AI (detector, ranking) lives in its own module and reads `data/`.
