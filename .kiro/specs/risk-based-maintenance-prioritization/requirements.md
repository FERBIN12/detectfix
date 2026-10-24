# Requirements Document

## Introduction

Risk-based maintenance prioritization ranks the bridge elements of one inspected structure into a work queue. Each element gets a deterministic priority score built from its detections (worst condition state, per-defect severity, change status) and from structure-level factors (age, traffic exposure as ADT, and location: underwater or scour-critical elements weigh more). Structure factors come from a per-bridge JSON config. The first config is for the Craigie Drawbridge (NBI structure number B160134EMDOTNBI).

The feature adds a new deterministic module (`pipeline/priority.py`), a `priority` block in the result JSON written by `run.py` / `pipeline/orchestrator.py` and `run_bag.py`, and a work-queue panel in Hyperion fed by `export_web.py`. It moves the "Risk-based maintenance prioritization" row of the `ARCHITECTURE.md` "Implemented vs. planned" table from planned to implemented.

The feature keeps the project rule "code enforces, models explain". Scores, ranks, factors and weights are computed only by code. LLM agents may only narrate the priority block, and every narrated claim cites detection IDs so that `agents._validate_report` keeps working unchanged. All mission data is simulated (Isaac Sim + ArduPilot SITL), and the structure-config values must carry their provenance. NBI↔SNBI export stays planned and is out of scope.

## Glossary

- **Priority_Module**: The new deterministic module `pipeline/priority.py` that loads the Structure_Config, computes Element_Scores, and builds the Priority_Block.
- **Structure_Config**: A per-bridge JSON file holding structure-level factors (year built, ADT, scour-critical code, per-element location flags) plus provenance fields.
- **Config_Loader**: The function in the Priority_Module that reads and validates a Structure_Config.
- **Structure_Factors**: The validated values taken from the Structure_Config: year built, assessment year, ADT, scour-critical code, the set of elements flagged underwater or scour-critical, and the provenance values (`structure_number`, `structure_name`, `source`, `source_version`, `is_illustrative`).
- **Neutral_Factor**: The factor value the Priority_Module uses for a structure factor that is missing or invalid. A Neutral_Factor neither raises nor lowers an Element_Score relative to the documented baseline. For location flags the Neutral_Factor is false.
- **Detection**: The normalized record defined in `pipeline/schema.py` (`Detection`), after `correlation.run_correlation` and `severity.run_severity` have run. Its fields include `detection_id`, `cls`, `platform`, `confidence`, `severity`, `change_status`, `model_version`, `location.structure.element_id`.
- **Scored_Detection**: Any Detection given to the Priority_Module that is not a Filler_Detection. The requirements write this as "scored Detection".
- **Element**: A bridge element identified by `location.structure.element_id` (for example `DECK-01`, `PIER-02`). Detections with an empty `element_id` belong to the Unassigned_Element.
- **Unassigned_Element**: The reserved element id `UNASSIGNED` that groups detections without an element assignment.
- **Condition_State**: One of `good`, `fair`, `poor`, `severe`, in that order, as defined by `severity.STATES`.
- **Worst_Condition_State**: The highest Condition_State among the scored Detections of one Element.
- **Change_Status**: One of `new`, `existing`, `worsening`, as set by `correlation.tag_change_status`.
- **Element_Score**: The numeric priority score of one Element, computed by the Priority_Module.
- **Score_Bounds**: The documented closed interval [SCORE_MIN, SCORE_MAX] = [0, 100] that every Element_Score lies within.
- **Rank**: The 1-based position of an Element in the Work_Queue.
- **Work_Queue**: The list of Elements ordered by the Tie_Break_Order, held in the Priority_Block as `work_queue`.
- **Tie_Break_Order**: The total order used for ranking: Element_Score descending, then Worst_Condition_State descending, then scored-detection count descending, then `element_id` ascending (code-point order).
- **Factor_Breakdown**: The per-element list of named factors, each with its input value, its contribution to the Element_Score, and the named constant it came from.
- **Rule_Trail**: A plain-language string per Element that lists the rules that produced its Element_Score, in the same style as `Detection.severity_reason`.
- **Priority_Block**: The `priority` object added to the result JSON.
- **Priority_Line**: A report line, drafted by the Report_Agent, that describes one Element of the Work_Queue or states that the Work_Queue is empty. The requirements write this as "priority line".
- **Ground_Truth_Detection**: A Detection whose `model_version` starts with `ground-truth@`.
- **Filler_Detection**: A detection record that carries `is_filler` set to true.
- **Sonar_Anomaly**: A Detection with `cls` equal to `sonar_anomaly` (generic sonar output that an inspector must identify).
- **Report_Agent**: The report-drafting stage in `pipeline/agents.py` (`draft_report`), including its deterministic fallback text.
- **Report_Validator**: The function `agents._validate_report`.
- **Web_Exporter**: The script `export_web.py` that writes `web/data/`.
- **Hyperion**: The web dashboard in `web/index.html`, `web/app.js`, `web/style.css`, served by `serve.py`.
- **Work_Queue_Panel**: The new Hyperion panel that shows the Work_Queue.
- **Test_Suite**: The `unittest` suites in `tests/`.

## Requirements

### Requirement 1: Per-bridge structure config

**User Story:** As an inspection engineer, I want structure-level risk factors kept in a per-bridge JSON file, so that the factors are reviewable and can change without code edits.

#### Acceptance Criteria

1. THE Structure_Config SHALL contain the fields `structure_number`, `structure_name`, `year_built`, `assessment_year`, `adt`, `scour_critical_code`, `elements`, `source`, `source_version`, and `is_illustrative`.
2. THE Structure_Config SHALL hold each field with the following documented type and valid range: `structure_number` a string of 1 to 15 characters (NBI Item 8 length); `structure_name` a string of 1 to 200 characters; `year_built` an integer from 1800 to 2100 inclusive; `assessment_year` an integer from 1800 to 2100 inclusive; `adt` an integer from 0 to 999,999 inclusive vehicles per day (the 6-digit width of NBI Item 29); `scour_critical_code` a single-character string from the NBI Item 113 code set (`0` to `9`, `N`, `T`, `U`); `source` a non-empty string of at most 2,000 characters; `source_version` a non-empty string of at most 100 characters; `is_illustrative` a boolean.
3. THE Structure_Config SHALL hold, under `elements`, an object with one entry per element id, where each element id is a non-empty string, appears at most once, and its entry contains exactly the boolean flags `underwater` and `scour_critical`.
4. THE project SHALL ship a Structure_Config for the Craigie Drawbridge with `structure_number` equal to `B160134EMDOTNBI` and with an `elements` entry for every element id used by the element map of the shipped missions in `fixtures/` and `sample_data/`.
5. IF any of `year_built`, `adt`, or `scour_critical_code` in the Craigie Drawbridge Structure_Config has not been verified against the published NBI record, THEN THE Structure_Config SHALL set `is_illustrative` to true and SHALL name each unverified field in `source` as illustrative.
6. IF every one of `year_built`, `adt`, and `scour_critical_code` in a Structure_Config has been verified against the published NBI record, THEN THE Structure_Config SHALL set `is_illustrative` to false.
7. THE Structure_Config SHALL state in `source`, for each value taken from the NBI record, the NBI item it came from: `year_built` (NBI Item 27), `adt` (NBI Item 29), and `scour_critical_code` (NBI Item 113).
8. THE `config.json` file SHALL hold the path of the Structure_Config that mock mode (`run.py`) loads.
9. WHEN `run_bag.py` is started with the Structure_Config command-line option, THE `run_bag.py` script SHALL load the Structure_Config at the path given by that option.
10. WHEN `run_bag.py` is started without the Structure_Config command-line option, THE `run_bag.py` script SHALL load the Structure_Config at the path held in `config.json`.

### Requirement 2: Config loading and validation

**User Story:** As an inspection engineer, I want a missing or bad structure config to produce a defined, visible result, so that the work queue is never silently wrong.

#### Acceptance Criteria

1. WHEN a Structure_Config is loaded in which every field is present, has its documented type, and lies within its documented valid range (as defined in Requirement 1.2 and checked by criteria 4 to 7 of this Requirement 2), THE Config_Loader SHALL return Structure_Factors equal to the values in the file and SHALL add no warning to the Priority_Block.
2. IF the Structure_Config file is missing or cannot be read, THEN THE Config_Loader SHALL return Neutral_Factors for every structure factor, SHALL add exactly one warning to the Priority_Block saying the structure config was not found at the given path, and SHALL return normally without raising an exception, so that the Priority_Block is still produced.
3. IF the Structure_Config file is not valid JSON, or its top-level value is not a JSON object, THEN THE Config_Loader SHALL return Neutral_Factors for every structure factor, SHALL add exactly one warning to the Priority_Block naming the file path and the parse error or the actual top-level type, and SHALL return normally without raising an exception.
4. IF one or more of the fields `year_built`, `assessment_year`, `adt`, or `scour_critical_code` is missing, has the wrong type, or lies outside its valid range, THEN THE Config_Loader SHALL use the Neutral_Factor for each affected field only, SHALL keep the values of all valid fields, and SHALL add one warning per affected field to the Priority_Block naming that field. The valid ranges are: `year_built` an integer from 1800 to 2100 inclusive; `assessment_year` an integer from 1800 to 2100 inclusive; `adt` an integer from 0 to 999,999 inclusive (the 6-digit width of NBI Item 29); `scour_critical_code` one of the NBI Item 113 codes `0` to `9`, `N`, `T`, or `U`. A boolean value counts as the wrong type for an integer field.
5. IF `elements` is missing or is not a JSON object, or an entry under `elements` has an `underwater` or `scour_critical` flag that is missing or not a boolean, THEN THE Config_Loader SHALL treat each affected flag as false (the Neutral_Factor for location flags), SHALL keep all valid flags, and SHALL add one warning per affected element id and flag to the Priority_Block naming the element id and the flag.
6. IF `assessment_year` is earlier than `year_built`, THEN THE Config_Loader SHALL use the Neutral_Factor for age, SHALL keep both year values in the Structure_Factors, and SHALL add one warning to the Priority_Block naming both `assessment_year` and `year_built`.
7. IF `is_illustrative` is missing or not a boolean, or `source` or `source_version` is missing or not a non-empty string, THEN THE Config_Loader SHALL set `is_illustrative` to true in the Structure_Factors and SHALL add one warning per affected field to the Priority_Block naming that field.
8. THE Config_Loader SHALL compute structure age as `assessment_year` minus `year_built` in whole years, using only those two fields and never the system clock, so that the same file gives the same age on any date.
9. WHEN the Config_Loader adds warnings, THE Config_Loader SHALL emit them in a fixed order that does not depend on the order of keys in the file, so that identical files produce identical warning lists.
10. WHILE the Config_Loader runs, THE Config_Loader SHALL use only the Python standard library and SHALL make no network call.

### Requirement 3: Deterministic element scoring

**User Story:** As an inspection engineer, I want each element scored by fixed rules from its defects and structure factors, so that the priority can be audited and reproduced.

#### Acceptance Criteria

1. WHEN the Priority_Module receives a list of Detections and Structure_Factors, THE Priority_Module SHALL compute exactly one Element_Score for each Element that has at least one scored Detection, where a scored Detection is any Detection in the list that is not a Filler_Detection.
2. WHEN the Priority_Module receives a list of Detections and Structure_Factors, THE Priority_Module SHALL omit from the Work_Queue every Element that has zero scored Detections, including Elements listed in the Structure_Config that have no Detections.
3. THE Priority_Module SHALL compute each Element_Score only from the Worst_Condition_State, the Condition_State of each scored Detection, the Change_Status of each scored Detection, structure age, ADT, and the Element's underwater and scour-critical flags, so that changing any other Detection field (for example `confidence` or `cls`) with every listed input fixed leaves the Element_Score unchanged.
4. IF the Structure_Config flags an Element as `underwater`, or any scored Detection of the Element has `platform` equal to `rov`, THEN THE Priority_Module SHALL treat that Element as underwater.
5. IF the Structure_Config flags an Element as `scour_critical`, THEN THE Priority_Module SHALL treat that Element as scour-critical, and otherwise THE Priority_Module SHALL treat the Element as not scour-critical.
6. IF an Element with at least one scored Detection, other than the Unassigned_Element, has no entry under the Structure_Config `elements`, THEN THE Priority_Module SHALL treat both of that Element's config flags as false, SHALL still score the Element, and SHALL add a warning naming the `element_id` to the Priority_Block.
7. THE Priority_Module SHALL define every weight, threshold, and factor value used in an Element_Score as a named UPPER_CASE module-level constant, with no numeric literal other than 0 or 1 used directly in the scoring arithmetic.
8. THE Priority_Module SHALL carry, next to each constant, a comment that cites the constant's source (NBI coding guide item, AASHTO MBE condition states, or FHWA scour-critical guidance) or states that the number is a project judgement.
9. WHEN the Priority_Module runs two or more times on identical Detections and an identical Structure_Config, THE Priority_Module SHALL produce Priority_Blocks whose JSON serializations with sorted keys are byte-identical.
10. IF a scored Detection has no `severity`, or its `severity` is not one of the Condition_States `good`, `fair`, `poor`, `severe`, THEN THE Priority_Module SHALL score that Detection as Condition_State `fair`, SHALL record the substitution and the original value in the Element's Rule_Trail, and SHALL leave the Detection's stored `severity` unchanged.
11. IF a scored Detection has no `change_status`, or its `change_status` is not one of `new`, `existing`, `worsening`, THEN THE Priority_Module SHALL score that Detection as Change_Status `new`, SHALL record the substitution and the original value in the Element's Rule_Trail, and SHALL leave the Detection's stored `change_status` unchanged.
12. IF a scored Detection has an empty or missing `element_id`, THEN THE Priority_Module SHALL group that Detection under the Unassigned_Element and SHALL treat the Unassigned_Element as neither underwater from config nor scour-critical.
13. WHILE the Priority_Module runs, THE Priority_Module SHALL use only the Python standard library and SHALL make no network call.

### Requirement 4: Score bounds

**User Story:** As an inspection engineer, I want scores on a fixed scale, so that scores from different missions and bridges are comparable.

#### Acceptance Criteria

1. THE Priority_Module SHALL produce every Element_Score as a finite number (not NaN and not infinite) within the closed Score_Bounds [0, 100], where an Element_Score equal to 0 or equal to 100 lies within the Score_Bounds.
2. THE Priority_Module SHALL define SCORE_MIN with value 0 and SCORE_MAX with value 100 as named UPPER_CASE module-level constants, each with a comment stating that the scale is a project judgement.
3. THE Priority_Block SHALL contain the fields `score_min` and `score_max`, whose values equal SCORE_MIN and SCORE_MAX, in every Priority_Block, including a Priority_Block with an empty Work_Queue.
4. FOR ALL lists of 0 to 500 Detections, all Condition_State and Change_Status combinations, and all Structure_Factors with ADT from 0 to 999,999 vehicles per day, structure age from 0 to 300 years, any combination of underwater and scour-critical flags, and any mix of valid values and Neutral_Factors, every Element_Score SHALL satisfy SCORE_MIN ≤ Element_Score ≤ SCORE_MAX (bounds property).
5. IF the sum of an Element's Factor_Breakdown contributions is greater than SCORE_MAX, THEN THE Priority_Module SHALL set the Element_Score to SCORE_MAX and SHALL record in that Element's Rule_Trail that the score was clamped to SCORE_MAX.
6. IF the sum of an Element's Factor_Breakdown contributions is less than SCORE_MIN, THEN THE Priority_Module SHALL set the Element_Score to SCORE_MIN and SHALL record in that Element's Rule_Trail that the score was clamped to SCORE_MIN.
7. IF a Structure_Factor value given to the Priority_Module lies outside the ranges in criterion 4 (ADT above 999,999 or age above 300 years), THEN THE Priority_Module SHALL still produce an Element_Score within the Score_Bounds and SHALL add a warning to the Priority_Block naming the out-of-range factor.

### Requirement 5: Monotonic scoring

**User Story:** As an inspection engineer, I want a worse defect or a riskier location never to lower an element's priority, so that the queue agrees with engineering judgement.

#### Acceptance Criteria

1. FOR ALL Elements, including the Unassigned_Element, and all scored Detections of that Element, THE Priority_Module SHALL produce an Element_Score greater than or equal to the original Element_Score when one Detection's Condition_State is raised by one to three steps in the order `good` < `fair` < `poor` < `severe` and every other Detection and every Structure_Factor is unchanged. A Detection with no `severity` counts as `fair` for this comparison (severity monotonicity property).
2. FOR ALL Elements, THE Priority_Module SHALL produce an Element_Score greater than or equal to the original Element_Score when one scored Detection's Condition_State is raised above the original Worst_Condition_State, so that the Worst_Condition_State goes up by one to three steps, and every other Detection and every Structure_Factor is unchanged (worst-state monotonicity property).
3. FOR ALL Elements and all scored Detections, THE Priority_Module SHALL produce an Element_Score greater than or equal to the original Element_Score when one Detection's Change_Status is changed from `new`, `existing`, or missing to `worsening` and every other input is unchanged (change-status monotonicity property).
4. FOR ALL Elements, THE Priority_Module SHALL produce an Element_Score greater than or equal to the original Element_Score when the Structure_Config `underwater` flag or `scour_critical` flag of that Element is changed from false or absent to true and every other input is unchanged (location monotonicity property).
5. FOR ALL Structure_Factors, THE Priority_Module SHALL produce, for every Element in the Work_Queue, an Element_Score greater than or equal to the original Element_Score when ADT is raised, or when structure age is raised by an earlier `year_built` or a later `assessment_year`, provided that both the original and the raised value lie within the documented valid range of that field and every other input is unchanged (exposure monotonicity property).
6. FOR ALL Elements, THE Priority_Module SHALL produce an Element_Score greater than or equal to the original Element_Score when one scored Detection is added to the Element, whatever its Condition_State, Change_Status, `platform` or `cls` (including a Sonar_Anomaly or a Ground_Truth_Detection), and every other input is unchanged (added-defect monotonicity property).
7. FOR ALL Elements, THE Priority_Module SHALL produce an Element_Score equal to the original Element_Score when one Filler_Detection is added to the Element's detection records and every other input is unchanged (filler-neutrality property).

### Requirement 6: Ranking and tie-breaking

**User Story:** As an inspection engineer, I want a single, stable work-queue order, so that two runs on the same data never disagree about what to fix first.

#### Acceptance Criteria

1. WHEN the Priority_Module builds the Work_Queue, THE Priority_Module SHALL order the Elements by the Tie_Break_Order, comparing Element_Scores as the exact values written to each Element entry's `score` field in the Priority_Block, so that two Elements whose written scores are equal are ordered by the next key of the Tie_Break_Order.
2. WHEN the Priority_Module builds the Work_Queue, THE Priority_Module SHALL include exactly the Elements that have at least one scored Detection, including the Unassigned_Element, and SHALL exclude Elements listed in the Structure_Config that have no scored Detection.
3. WHEN the Priority_Module builds the Work_Queue, THE Priority_Module SHALL assign Ranks 1 to N without gaps or repeats, where N is the number of Elements in the Work_Queue, and SHALL set each Element entry's `rank` equal to that entry's 1-based position in the Work_Queue list.
4. IF there are no scored Detections, THEN THE Priority_Module SHALL produce a Work_Queue with N equal to 0 and SHALL assign no Ranks.
5. THE Priority_Module SHALL rank the Unassigned_Element by the same Tie_Break_Order as every other Element, with no fixed first or last position.
6. FOR ALL lists of Detections, every permutation of the Detection list SHALL produce an identical Work_Queue, identical Ranks, identical Element_Scores, identical Factor_Breakdowns, and identical Rule_Trails (order-independence property).
7. FOR ALL Structure_Configs, every permutation of the `elements` entries SHALL produce an identical Work_Queue, identical Ranks, identical Element_Scores, identical Factor_Breakdowns, and identical Rule_Trails (config order-independence property).
8. FOR ALL Work_Queues, any two distinct Elements SHALL have distinct Ranks, and the Element with the lower Rank SHALL precede the other Element under the Tie_Break_Order (total-order property).

### Requirement 7: Handling of labelled detections

**User Story:** As an inspection engineer, I want ground-truth, filler, and generic sonar detections handled by explicit rules and kept labelled, so that the queue never presents unreal or unidentified data as confirmed findings.

#### Acceptance Criteria

1. IF a detection record is a Filler_Detection, THEN THE Priority_Module SHALL exclude the record from scoring, SHALL leave its `detection_id` out of every Element entry's `detection_ids`, and SHALL list its `detection_id` exactly once under the Priority_Block's excluded detections with the reason `filler`.
2. THE Priority_Module SHALL score a Sonar_Anomaly as a Detection of its Element, using the same scoring rules as any other Detection with the same Condition_State and Change_Status.
3. WHEN an Element has at least one Sonar_Anomaly among its scored Detections, THE Priority_Module SHALL set the flag `needs_identification` to true on that Element's entry and SHALL list every Sonar_Anomaly `detection_id` of that Element exactly once, in ascending code-point order.
4. THE Priority_Module SHALL score a Ground_Truth_Detection as a Detection of its Element, using the same scoring rules as any other Detection with the same Condition_State and Change_Status.
5. WHEN an Element has at least one Ground_Truth_Detection among its scored Detections, THE Priority_Module SHALL set the flag `has_ground_truth` to true on that Element's entry.
6. WHEN a run has at least one scored Detection and every scored Detection in the run is a Ground_Truth_Detection, THE Priority_Block SHALL set the flag `ground_truth_only` to true.
7. IF an Element has no Sonar_Anomaly among its scored Detections, THEN THE Priority_Module SHALL set `needs_identification` to false on that Element's entry and SHALL list no Sonar_Anomaly detection IDs; IF an Element has no Ground_Truth_Detection among its scored Detections, THEN THE Priority_Module SHALL set `has_ground_truth` to false on that Element's entry.
8. IF a run has zero scored Detections or at least one scored Detection that is not a Ground_Truth_Detection, THEN THE Priority_Block SHALL set the flag `ground_truth_only` to false.
9. IF every detection record of an Element is a Filler_Detection, THEN THE Priority_Module SHALL omit that Element from the Work_Queue.

### Requirement 8: Priority block in the result JSON

**User Story:** As an inspection engineer, I want the full priority result written into the mission result JSON, so that each rank can be traced back to its detections and rules.

#### Acceptance Criteria

1. WHEN `run.py` or `run_bag.py` completes successfully, THE result JSON SHALL contain exactly one top-level `priority` object, computed by the Priority_Module from the Detections after `severity.run_severity` has set their `severity` values.
2. THE Priority_Block SHALL hold the Work_Queue as a `work_queue` list that has one entry per Element, ordered by ascending `rank`. Each entry SHALL contain the fields `element_id`, `rank`, `score`, `worst_condition_state`, `factor_breakdown`, `rule_trail`, `detection_ids`, `underwater`, `scour_critical`, `needs_identification`, and `has_ground_truth`.
3. THE Priority_Block SHALL contain the fields `score_min`, `score_max`, `structure_number`, `config_path`, `config_source`, `config_source_version`, `is_illustrative`, `is_simulated`, `warnings`, `excluded_detections`, and `ground_truth_only`. `warnings` and `excluded_detections` SHALL be lists that are empty when nothing applies.
4. THE Priority_Block SHALL hold, in each `factor_breakdown` item, the factor name, the factor's input value, its numeric `contribution`, and the name of the Priority_Module constant it came from. Each `excluded_detections` item SHALL hold the `detection_id` and the exclusion reason.
5. THE Priority_Block SHALL set `is_simulated` to true for every mission loaded from `sample_data/` or `fixtures/`.
6. THE `detection_ids` of each Element entry SHALL contain only `detection_id` values of Detections present in the result JSON's `detections` list.
7. FOR ALL Priority_Blocks, each `detection_id` in the result JSON's `detections` list SHALL appear in exactly one place: either the `detection_ids` of exactly one Element entry or `excluded_detections` (detection coverage property).
8. FOR ALL Element entries, the sum of the `contribution` values in `factor_breakdown`, clamped to the Score_Bounds, SHALL equal `score` within an absolute tolerance of 1e-6 (breakdown consistency property).
9. FOR ALL Priority_Blocks, serializing the Priority_Block to JSON and parsing the JSON SHALL produce an object equal to the original Priority_Block (round-trip property).
10. WHEN there are no scored Detections, THE Priority_Block SHALL contain an empty `work_queue` list and SHALL still contain every field listed in criterion 3, with the structure provenance fields filled from the Structure_Config.
11. IF the Config_Loader returned Neutral_Factors because the Structure_Config was missing or invalid, THEN THE Priority_Block SHALL still be written with every field listed in criterion 3. `config_path` SHALL hold the requested path, any provenance field that could not be read SHALL be null, and the matching warning SHALL appear in `warnings`.

### Requirement 9: Agent narration limits

**User Story:** As an inspection engineer, I want any report text about priority to be checked like every other claim, so that the LLM cannot invent or adjust a rank.

#### Acceptance Criteria

1. THE Priority_Module SHALL compute Element_Scores, Ranks and Factor_Breakdowns without calling an LLM, the Kiro CLI, or any network service, and SHALL produce the same Priority_Block whichever agent backend is configured (Kiro CLI, Claude, local model, or none).
2. THE Report_Agent SHALL receive the Priority_Block as input data only, and SHALL compute no Element_Score, Rank, Worst_Condition_State or factor value of its own.
3. WHERE the Report_Agent includes priority lines in the report, THE Report_Agent SHALL cite in each priority line at least one `detection_id` in the form `[D-xxxxxxxxxxxx]`, and every cited `detection_id` SHALL belong to the `detection_ids` of the Element the line describes.
4. THE Report_Validator SHALL apply the same keep/drop rules to priority lines as to every other report line: it keeps a line whose cited detection IDs are all present in the run's Detections, drops a line that cites any unknown detection ID, and drops a line that has no detection ID and contains a digit. The Report_Validator's rules SHALL NOT be relaxed or bypassed for priority lines.
5. IF an LLM-drafted priority line kept by the Report_Validator states a Rank, Element_Score or Worst_Condition_State that differs from the Priority_Block entry of the Element whose detection IDs it cites, THEN THE Report_Agent SHALL remove that line from the narrative and SHALL list it among the report's dropped lines. The Priority_Block stays unchanged.
6. WHEN the Report_Agent runs with no network and no API key, THE Report_Agent's deterministic fallback text SHALL contain one priority line for each of the first 5 Elements of the Work_Queue in Rank order, or for every Element if the Work_Queue has fewer than 5. Each line SHALL state the Element's Rank, `element_id`, Element_Score and Worst_Condition_State, taken from the Priority_Block, and SHALL cite at least one of that Element's `detection_ids`.
7. WHEN the Report_Validator checks the deterministic fallback text, THE Report_Validator SHALL keep every fallback priority line and drop none of them.
8. IF the Work_Queue is empty, THEN THE Report_Agent's deterministic fallback text SHALL contain a single priority line that states no elements are queued and contains no digits, so that the Report_Validator keeps it.
9. WHEN the Report_Agent returns its output, THE Priority_Block SHALL equal the Priority_Block computed before the Report_Agent ran, compared field by field after JSON serialization, whether the narrative came from an LLM or from the deterministic fallback.

### Requirement 10: Hyperion work-queue panel

**User Story:** As an inspection engineer, I want a work-queue panel in Hyperion, so that I can see which elements to fix first and why.

#### Acceptance Criteria

1. WHEN the Web_Exporter runs on a result JSON that contains a Priority_Block, THE Web_Exporter SHALL write that Priority_Block into `result.json` in `web/data/` as the top-level `priority` object, with every field value equal to the value in the source result JSON.
2. WHEN `result.json` contains a Priority_Block with one or more Elements in the Work_Queue, THE Work_Queue_Panel SHALL show one row per Element, ordered by the `rank` field ascending, with each row showing the Element's `rank`, `element_id`, `score`, `worst_condition_state`, and the number of entries in its `detection_ids`, all taken from the Priority_Block without recomputation, reordering, or re-ranking in the browser.
3. WHEN the user selects an Element row, THE Work_Queue_Panel SHALL show that Element's Factor_Breakdown (each factor's name, input value, contribution, and source constant name), its Rule_Trail, and every entry of its `detection_ids`, and SHALL replace the details of any previously selected Element so that the details of exactly one Element are shown at a time.
4. WHEN `result.json` contains a Priority_Block, THE Work_Queue_Panel SHALL show the structure name, `structure_number`, `config_source`, and `config_source_version`, and SHALL show the text "not provided" in place of any of these fields that is missing or empty.
5. WHILE the Priority_Block has `is_illustrative` set to true, THE Work_Queue_Panel SHALL show the label "Illustrative structure data (not verified against NBI)".
6. WHILE the Priority_Block has `is_simulated` set to true, THE Work_Queue_Panel SHALL show the label "Simulated mission data".
7. WHEN an Element entry has `needs_identification` set to true, THE Work_Queue_Panel SHALL show the label "Needs identification (generic sonar anomaly)" on that Element's row, and WHEN an Element entry has `has_ground_truth` set to true, THE Work_Queue_Panel SHALL show the label "Ground truth (not model output)" on that Element's row.
8. WHEN the Priority_Block has one or more entries in `warnings`, THE Work_Queue_Panel SHALL show the full text of every warning entry, with no entry truncated or omitted.
9. IF `result.json` has no Priority_Block, THEN THE Work_Queue_Panel SHALL show the message "No priority data for this mission" and SHALL show no Element rows.
10. THE Work_Queue_Panel SHALL be implemented in `web/index.html`, `web/app.js` and `web/style.css` with no JavaScript build step and no new third-party script.
11. IF the Priority_Block has an empty Work_Queue, THEN THE Work_Queue_Panel SHALL show a message stating that no elements have scored detections, and SHALL still show the structure fields from criterion 4 and the labels from criteria 5 and 6.
12. WHILE the Priority_Block has `ground_truth_only` set to true, THE Work_Queue_Panel SHALL show a label stating that every scored detection in the run is ground truth and not model output.
13. WHEN the Priority_Block has one or more entries in `excluded_detections`, THE Work_Queue_Panel SHALL show each excluded `detection_id` together with its exclusion reason.

### Requirement 11: Tests

**User Story:** As a maintainer, I want the scoring rules pinned by unit and property tests that run with the standard library, so that a change that breaks a property fails the suite.

#### Acceptance Criteria

1. THE Test_Suite SHALL contain at least one `unittest` unit test for each of the following cases: a valid Structure_Config (Requirement 2.1), a missing Structure_Config file (Requirement 2.2), a Structure_Config that is not valid JSON (Requirement 2.3), a single missing, wrongly typed, or out-of-range field (Requirement 2.4), `assessment_year` earlier than `year_built` (Requirement 2.6), a Detection with no `severity` (Requirement 3.10), a Detection with no `change_status` (Requirement 3.11), Detections with an empty `element_id` grouped under the Unassigned_Element (Requirement 3.12), score bounds (Requirement 4.1), tie-breaking at each of the four levels of the Tie_Break_Order (Requirement 6.1), filler exclusion (Requirement 7.1), Sonar_Anomaly flagging (Requirement 7.3), Ground_Truth_Detection flagging (Requirements 7.5 and 7.6), the presence of every Priority_Block field listed in Requirements 8.2 and 8.3, and an empty Work_Queue when there are no scored Detections (Requirement 8.10).
2. WHERE the `hypothesis` package is installed, THE Test_Suite SHALL run a Hypothesis property test for each of the following properties: bounds (Requirement 4.4), severity monotonicity (Requirement 5.1), worst-state monotonicity (Requirement 5.2), change-status monotonicity (Requirement 5.3), location monotonicity (Requirement 5.4), exposure monotonicity (Requirement 5.5), added-defect monotonicity (Requirement 5.6), filler-neutrality (Requirement 5.7), order-independence (Requirement 6.6), config order-independence (Requirement 6.7), total-order (Requirement 6.8), detection coverage (Requirement 8.7), breakdown consistency (Requirement 8.8), and round-trip (Requirement 8.9).
3. IF the `hypothesis` package is not installed, THEN THE Test_Suite SHALL report each Hypothesis property test as skipped with a skip reason that contains the word `hypothesis`, and SHALL report no failure or error for those tests.
4. THE Test_Suite SHALL test each of the properties listed in Requirement 11.2 with a standard-library randomized test that uses a fixed seed and generates at least 200 cases, so that every property runs whether or not `hypothesis` is installed.
5. THE standard-library randomized tests SHALL generate inputs that include Neutral_Factors, ADT and structure age at the minimum and maximum of their documented valid ranges, Elements with exactly one scored Detection, and Elements with tied Element_Scores.
6. IF a property test finds an input that violates its property, THEN THE Test_Suite SHALL report that test as failed and SHALL include the seed and the failing input in the failure message.
7. THE Test_Suite SHALL contain a test that runs the Report_Validator on the priority lines of the Report_Agent's deterministic fallback text, with no network and no API key, and checks that the Report_Validator keeps every priority line and that each kept line cites at least one `detection_id` from the result JSON's `detections` list.
8. WHEN `python -m unittest discover -s tests` runs with only the standard library installed, THE Test_Suite SHALL finish with zero failures and zero errors.

### Requirement 12: Documentation

**User Story:** As a maintainer, I want the docs to describe the feature as built, so that the docs match the code.

#### Acceptance Criteria

1. THE `ARCHITECTURE.md` "Implemented vs. planned" table SHALL list risk-based maintenance prioritization exactly once, in a row marked implemented that names `pipeline/priority.py`, and SHALL contain no row that lists risk-based maintenance prioritization as planned.
2. THE `ARCHITECTURE.md` "Implemented vs. planned" table SHALL keep the NBI↔SNBI crosswalk export listed as planned, and no documentation file in Requirement 12 SHALL describe the NBI↔SNBI crosswalk export as implemented.
3. THE `PIPELINE.md` file SHALL describe the prioritization step as a numbered flow step placed immediately after the severity step and before the agent steps, and SHALL name its inputs (scored Detections and Structure_Factors) and its output (the Priority_Block).
4. THE `DATA_INPUTS.md` file SHALL list by name every Structure_Config field from Requirement 1.1, each with its type and valid range from Requirement 1.2, and the per-element `underwater` and `scour_critical` flags from Requirement 1.3, each with its type.
5. THE `DATA_INPUTS.md` file SHALL list by name every Priority_Block field from Requirements 8.2 and 8.3, each with its type, and SHALL state the Score_Bounds [0, 100].
6. THE `README.md` file SHALL describe how to select a Structure_Config in mock mode through `config.json` (Requirement 1.8) and in `run_bag.py` through the command-line option and its `config.json` default (Requirements 1.9 and 1.10), and SHALL state where the Work_Queue_Panel appears in Hyperion.
7. THE `README.md` file and the `DATA_INPUTS.md` file SHALL state that the Craigie Drawbridge Structure_Config values marked `is_illustrative` are not verified against the published NBI record and that all mission data in `sample_data/` and `fixtures/` is simulated.
8. THE `DATA_INPUTS.md` file SHALL state the behavior for a missing Structure_Config file, an invalid JSON file, and a single invalid field (Neutral_Factor used and a warning added to the Priority_Block), consistent with Requirement 2.
