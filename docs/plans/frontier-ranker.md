# Frontier ranker plan

Status: step 1 (logging) is in progress. The ranker is not in the product. Defaults do not change.

## Goal

A small learned model suggests which frontier candidate to try next. Motus.NET stays in charge. It does IK, the exact 2 mm collision check, confirmation and replay. Each kept choice must pass the real confirmer. The model only changes the order of candidates. It does not add, remove or skip a candidate. Do not use a language model for geometry.

## Steps

### Step 1: log features (no ranking change)

- Outcome rows (`OutcomeLog.Outcome`) get new trailing fields: `Stand`, `F`, `FeatureVersion`, `Node` and `Pos`. Old fields stay the same. Old readers and the confirm-rate prior still work.
- `Node` is the id of one expansion in `BacktrackSearch.Dfs`. `Pos` is the position of the candidate in the sorted order at that node.
- Each node writes one `OutcomeLog.Sorted` row for each sorted candidate before the first try. These rows have `Ok: null`. `OutcomeLog.Read` skips them, so the prior does not count them. An untried candidate is a sorted row with no outcome row at the same `Node` and `Pos`.
- The search writes these rows only when `TampSettings.OutcomeLogPath` is set. The features use geometry only: no IK, no planner call and no random draw.
- The offline report is `tools/frontier-ranker/rank_report.py`. No project or package references it.

Features (`BambooFrontier.Features`, `FeatureVersion` 1). Distances are viewer metres (Y up) divided by `BambooReachMap.MaxReachM`.

| Index | Feature |
|---|---|
| 0–4 | Mode one-hot: Direct, Land, Walk, Relay, Reposition |
| 5 | Stand to member place distance |
| 6 | Stand to member stock distance |
| 7 | Place height above the stand |
| 8 | Relay only: stand to park place distance (else 0) |
| 9 | Stand is on an anchor (1) or on a member (0) |
| 10 | Placed members / all members |
| 11 | ln(1 + fixed foothold sites within two arm reaches of the stand) |

Features 5–8 are 0 for a reposition. Increase `FeatureVersion` when a feature changes meaning.

### Step 2: collect data (ranker off)

- Run Warren and the 3x3 dome with `OutcomeLogPath` set. Keep `PriorLogPaths` null, so the logged order is the geometric order.
- Add many small generated layout variants. Variety is worth more than repeat runs.
- Do not start a tower run for data. Log the 1x2 tower only when Motus runs it for other work. The tower stall at 6/60 is a stock-pick collision. A ranker cannot fix it.

Run the report:

```
python3 tools/frontier-ranker/rank_report.py <run>.outcomes.jsonl [...]
python3 tools/frontier-ranker/rank_report.py --selftest
```

The report holds out one layout at a time. It compares the geometric order, the confirm-rate prior and a small logistic model. The metrics are top-1 confirm rate per decision and rejections before the first kept candidate. It scores tried candidates only, because untried candidates have no label.

### Step 3: decide

- Wait for about 50 or more logged decisions across layouts.
- Train a linear ranker with 3 to 5 features offline.
- Propose an opt-in ranker (`TampSettings.RankerPath`, default null) only if it is clearly better than the geometric order on held-out layouts.
- The ranker replaces the prior term in the rank. Log the model hash and each suggestion, so the search stays replayable.

## Current status

- Step 1 is on branch `cursor/frontier-ranker-logging`. It is a draft PR with base `cursor/arch-deepening`.
- The existing logs give 8 decisions on 2 layouts. That is too little to train.

## Rules

- Do not change search order or default behaviour. New log fields are permitted.
- Keep Motus.NET RhinoCommon-free.
- Do not loosen the 2 mm collision check. Do not change fixed or grippable canes or stock siting.
- Keep the trainer under `tools/`, out of the shipped packages.
- Speak to Motus before you change library code.
- Do not force-push and do not merge. Keep PRs as drafts.

## Gates

- Warren 7/7 (`BuildSearch_DerivesTheShippedSequence_EveryChoicePlannerConfirmed`).
- `ShippedDefaultPlan_IsIdenticalToLivePlanning`.
- `BambooTrussTests.Tamp.cs`, including `BuildSearch_OutcomeLog_DoesNotChangeTheSearch_AndLogsSortedCandidates` (log on and off give the same search).
- The full core suite. Run it only when no other heavy job runs on the machine.
