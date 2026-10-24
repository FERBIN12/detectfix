# Problem: ghost lobster traps in Massachusetts Bay and Cape Cod Bay

## The problem in one breath
Massachusetts lobstermen lose an estimated **1–5% of their gear every year** (storms, ship traffic, other gear).
In Cape Cod Bay alone, these derelict traps are estimated to **kill 12,500–33,000 lobsters a year**. Lost pots keep
fishing at roughly 3.6–4.8 lobsters per pot per year. Lost rope and traps are also part of the entanglement risk for the
~380 North Atlantic right whales left, which feed in Cape Cod Bay. In January 2026 a right whale calf was found tangled
in gear NOAA traced to a single lobster trap set in Massachusetts state waters.

## How it's done today, and where it breaks
| Step | Today | Where it breaks (from MA's own reports) |
|---|---|---|
| Find | Side-scan sonar surveys (MA DMF developed the method in 2017; CCS used it in Cape Cod Bay) | **Pots and rocks look alike on sonar.** In complex habitat, and in an NH survey, granite blocks looked like traps |
| Confirm | Send a diver or a VideoRay ROV to look | Slow and costly; divers at risk |
| Recover | Grapple dragging or SCUBA divers (Beverly Harbor, Cape Ann) | **Grappling gives "limited and mixed results"** and drags across habitat |
| Permission | **MA and Maine law: nobody may handle another fisherman's gear without express permission** | So gear gets thrown back and waits for a winter cleanup |
| Funding | Project grants of $95k–150k each (NOAA, TRAP program) | Cleanups happen once, in winter, then the gear piles up again |

## Our robot (reuses everything we have)
**Find → confirm → identify owner → ask permission → recover → log.**
1. **Find:** multibeam/imaging sonar flags trap-shaped targets in murky water (our sonar sim + wall mosaic code → seabed).
2. **Confirm:** the ROV approaches, holds station, and a classifier says **trap or rock** (this fixes the #1 failure above).
3. **Identify:** camera reads the trap tag / buoy markings to find the owner (NOAA traced the 2026 whale gear to its owner
   from gear markings; check the exact MA trap-tag rule with a DMF page before saying it on stage).
4. **Permission gate (the human in the loop):** the code rule says "owner's gear, permission required" → one card in the
   "Needs you" inbox → **Fo emails/calls the license holder or harbormaster** (consenting teammate in the demo) → approved.
5. **Recover:** the Alpha 5 arm clips a lift line or cuts the trailing rope; the ROV holds station against the pull.
6. **Log:** a map of every trap found: position, confirmed or not, owner, recovered or left. That map is the deliverable
   a TRAP grantee or DMF actually wants.

## Why this beats the other options for today
| | Ghost traps | Dams | Sewage overflows | Eelgrass | Zebra mussels |
|---|---|---|---|---|---|
| Only-in-Massachusetts story | ✅ Cape Cod Bay, MA lobster fishery, MA law | ✅ but every state has dams | ✅ | ✅ | ✅ Berkshires |
| Hurts people you can name | ✅ lobstermen, whales | ✅ public safety | ✅ health | ~ | ~ |
| A robot is the answer | ✅ confirm + recover underwater | ✅ inspection | ✗ mostly sensors | ~ mapping | ✅ |
| Uses sonar + arm + agent gate + Fo | ✅ all four | sonar only | ✗ | sonar | sonar + arm |
| Demo-able by 5 PM | ✅ (the seabed is simpler than the intake wall) | ✅ | ✗ | ~ | ✅ |
| Fresh data from this year | ✅ Jan 2026 case, Sept 2026 TRAP round | ✗ (2019/2023) | ✅ 2024 | ~ | ✗ (2009) |

## Say / don't say
- Say "estimated" with the 1–5% and 12,500–33,000 numbers (that is how NOAA states them).
- Don't say the robot frees whales. Trained teams do that.
- Don't claim a total count of ghost traps in MA (nobody has one).
- Don't say anyone may touch other people's gear: the permission gate exists because of the law.

## Sources
- NOAA Marine Debris, Cape Cod and Massachusetts Bays project: https://marinedebris.noaa.gov/removal/fishing-derelict-lobster-gear-cape-cod-and-massachusetts-bays
- NOAA blog, teachings from the trash: https://blog.marinedebris.noaa.gov/removing-derelict-fishing-gear-cape-cod-bay-teachings-trash
- MA DMF side-scan methodology (2017): https://www.mass.gov/doc/development-of-side-scan-sonar-methodology-to-survey-derelict-lobster-pots-2017/download
- NOAA side-scan abundance study: https://marinedebris.noaa.gov/prevention/estimating-abundance-derelict-lobster-pots-new-england-using-side-scan-sonar
- Ocean News, sonar plus ROV confirmation (NH): https://oceannews.com/?p=78432
- TRAP program 2025 round: https://news.wm.edu/2025/10/29/national-trap-program-targets-ghostly-issue-with-second-round-of-coastal-clean-up-funding/
- TRAP program, Sept 2026: https://news.wm.edu/2026/09/23/trap-program-turns-lost-fishing-gear-into-lasting-solutions-as-nationwide-impact-grows/
- NOAA right whale updates: https://www.fisheries.noaa.gov/national/endangered-species-conservation/north-atlantic-right-whale-health-updates
- Boston Globe, Jan 2026: https://www.bostonglobe.com/2026/01/21/metro/entangled-right-whale-cape-cod/
