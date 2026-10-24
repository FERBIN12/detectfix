---
inclusion: always
---
# Design rule: code enforces, models explain

Every change to this project must keep this rule. If a request conflicts with
it, say so and propose a design that keeps it.

1. **Models never set facts.** Severity, change status, class, confidence,
   geometry, location, evaluation metrics, hardware status and the sign-off
   state are written only by deterministic code (`severity.py`,
   `correlation.py`, the detectors, `lidar.py`, `evaluation.py`,
   `hardware.py`, `review.py`). Never move a threshold or a decision into a
   prompt.
2. **Every report claim cites a detection ID** (`[D-xxxxxxxxxxxx]`).
   `agents._validate_report` drops lines with an unknown ID or an uncited
   number. Don't weaken it to make LLM output pass; fix the prompt or the
   fallback text instead.
3. **Thresholds are named constants** with a comment saying where the number
   came from (e.g. `severity.DEPTH_MM`, `rov.ROV_MIN_VISIBILITY`,
   `evaluation.DEFAULT_RADIUS_M`).
4. **A deterministic fallback always exists.** Any stage that calls a model
   must still produce a correct result with no network and no API key.
5. **Label what isn't real.** Ground truth (`model_version` starts with
   `ground-truth@`), simulated hardware (`is_simulated`), ROV filler
   (`is_filler`) and generic detections (`sonar_anomaly`) are always marked,
   in the data and in Hyperion.
6. **Tests pin the rule.** Changes to severity, correlation, matching, the
   validator, hardware status or the review gate need a unit test, and a
   property test where a universal statement exists.
7. **Stdlib first.** Mock mode and the tests run with the standard library
   only; third-party imports go inside the function that needs them.
