# What each sensor does (for the "meet the robots" scenes)
DRONE (Holybro X500, flown by ArduCopter):
- Ouster OS0-128 lidar: a spinning laser with 128 beams; measures the bridge and walls as millions of 3D points.
- Gimbal camera: the inspector's eyes; close-up images of cracks, spalls, stains.
- Thermal camera (simulated in this demo): sees temperature; damp, failing stone reads cooler than sound stone.
- RTK GPS: centimetre position in open sky; under the bridge deck it loses satellites (12 to 7 in our run).
- LED floodlights: light the dark underside of the deck.
ROV (BlueROV2 Heavy, flown by ArduSub):
- DVL, Doppler velocity log (Water Linked A50 class): four sound beams bounce off the riverbed to measure how fast it is
  moving; that is how it navigates with no GPS (about 0.4 m of drift after 60 m in our test).
- Multibeam imaging sonar (130 degrees, 12 m): sees the walls and riverbed by sound, where a camera only sees mud.
- Low-light camera + Lumen lights: confirms what the sonar found, up close.
- Depth sensor: pressure gives exact depth.
- 8 thrusters (the "Heavy"): holds position against current.
