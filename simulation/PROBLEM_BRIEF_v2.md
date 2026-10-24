# Problem v2 (Oct 24): Greater Boston's water tunnels can't be shut off to inspect them

## Pain (MWRA's own words + news)
- About **60% of Metropolitan Boston's daily water** flows through the remaining **19 miles** of the Metropolitan Tunnel System.
- MWRA: "With no redundancy... partial system shutdowns for planned maintenance of the aged infrastructure or unplanned
  emergencies cannot take place."
- MWRA estimates a service outage would cost the region **about $360 million per day** (2024, FEMA method).
- The fix is the Metropolitan Water Tunnel Program: new backup tunnels **so the old ones can finally be taken offline and
  inspected**. FY25 plan: **$2.1B** (other MWRA docs: $1.1B; industry: $1.5B). First construction contract bid target: **2027**.
- It has already gone wrong once: **May 2010**, the MetroWest pipe in Weston broke (failed coupling bolts on a 10-ft pipe,
  only 7 years old). It dumped about **8 million gallons an hour** into the Charles, and nearly **2 million people in 30
  communities, Boston included,** were put under a **boil-water order**.

## Why underwater robots
A robot can inspect a tunnel **while the water keeps flowing**. A Massachusetts lab already proved it: **WHOI (Woods Hole)**
built the AUV that inspected New York City's **45-mile Delaware Aqueduct in service** (2003; 160,000 images in 16 hours).
WHOI's own list of hard problems: keeping the vehicle centered without hitting the walls, tracking it in a concrete tunnel,
cameras and lighting, and launching through deep shafts.

## Our pitch
"Boston can't turn off its water to check its own tunnels, and the backup tunnels are years and $2B away. We built a
robot that inspects them with the water on."

## Demo (reuses our stack)
- Isaac: a flooded concrete tunnel with flow; the ROV holds the centerline against the current using its sonar ranges
  (our station-keeping controller).
- A profiling sonar maps the tunnel wall: cracks, spalling, deposit/tuberculation build-up (our sonar + mosaic code, wall → tunnel ring).
- Each defect is scored; a code rule decides "log / re-inspect / urgent". Urgent goes to the "Needs you" inbox for the operator.
- Output: a defect map along the tunnel's length, the report a utility engineer files.

## Careful
- Don't say MWRA has never inspected the tunnels. Say: "without redundancy, they can't be shut down for maintenance" (MWRA's line).
- Quote the $2.1B as "up to $2.1B (MWRA FY25 plan)"; the estimates vary.
- The 2.5M-people figure is from an industry article, not MWRA. Use the 2010 "nearly 2 million under a boil-water order" instead.
- We are students validating the idea, not a company.

## Sources
- MWRA tunnel program: https://www.mwra.com/mwtp/resources.html
- MWRA staff summary, $360M/day: https://www.mwra.com/media/file/2024-03-13-mwtp-ss-needs-overview
- MWRA board materials, Mar 2024: https://www.mwra.com/sites/default/files/2024-03/2024-3-13%20OS%20Board%20Materials.pdf
- WBUR 2010 break: https://www.wbur.org/news/2010/05/03/water_main_break
- MWRA boil-water order: https://www.mwra.com/01news/2010/boilwaterorder1.htm
- CBS, 2013 settlement: https://www.cbsnews.com/boston/news/mass-water-agency-to-receive-3-1m-in-settlement/
- WHOI tunnel inspection vehicle: https://www2.whoi.edu/site/osl/vehicles/remus-tiv/
- NYC DEP, 2003 AUV run: https://www.nyc.gov/html/dep/html/press_releases/03-31pr.shtml
- Vision Systems: https://www.vision-systems.com/home/article/16737500/submersible-imaging-system-inspects-nyc-water-supply
