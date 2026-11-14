#!/usr/bin/env python3
"""Leave-one-layout-out report for the Motus frontier ranker (step 1, offline only).

Reads outcome logs (TampSettings.OutcomeLogPath JSONL) and scores three candidate orders per decision:
  geometric  the logged order (Rank; geometric when the run had no prior)
  prior      Rank + (-ln p) with the shipped confirm-rate prior (OutcomeLog.Prior) fit on the other layouts
  linear     Rank + (-ln sigmoid(w.F + b)), a logistic model on the F features fit on the other layouts (needs numpy)
Metrics over decisions with at least one tried candidate, using tried candidates only (untried ones have no label):
  top1  the best-scored tried candidate was kept
  rej   rejections before the first kept candidate (all rejections when none was kept)
Not shipped: no project or package references this folder. Stdlib only; numpy only for `linear`.

usage: rank_report.py LOG.jsonl [LOG.jsonl ...]     |     rank_report.py --selftest
"""
import json, math, sys
from collections import defaultdict


def load(paths):
    """Decisions: list of (layout, [cand]) with cand = dict(member, mode, rank, f, ok) in logged order.
    New logs group by (file, Node) and order by Pos; old logs (no Node) group by (file, Placed, State) in file order."""
    groups = defaultdict(dict)
    for path in paths:
        with open(path) as fh:
            for i, line in enumerate(fh):
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                key = (path, r["Node"]) if r.get("Node") is not None else (path, r["Placed"], r["State"])
                pos = r["Pos"] if r.get("Pos") is not None else i
                c = groups[key].setdefault(pos, {"layout": r["Layout"], "member": r["Member"], "mode": r["Mode"],
                                                 "rank": r["Rank"], "f": r.get("F"), "ok": None})
                if r.get("Ok") is not None:
                    c["ok"] = c["ok"] or r["Ok"]  # a second-pass retry that kept it counts as kept
    out = []
    for g in groups.values():
        cands = [g[p] for p in sorted(g)]
        if any(c["ok"] is not None for c in cands):
            out.append((cands[0]["layout"], cands))
    return out


def prior(train):
    """OutcomeLog.Prior on the training decisions; on a held-out layout the member term is always empty."""
    sn = defaultdict(lambda: [0, 0])
    for _, cands in train:
        for c in cands:
            if c["ok"] is not None:
                sn[c["mode"]][0] += c["ok"]; sn[c["mode"]][1] += 1
    return lambda c: -math.log((sn[c["mode"]][0] + 1.0) / (sn[c["mode"]][1] + 2.0))


def linear(train, steps=2000, l2=1.0, lr=0.1):
    """Logistic regression on F (L2, full-batch gradient descent). Returns -ln p, or None without features/numpy."""
    try:
        import numpy as np
    except ImportError:
        return None
    rows = [(c["f"], c["ok"]) for _, cands in train for c in cands if c["ok"] is not None and c["f"]]
    if len({ok for _, ok in rows}) < 2:
        return None
    X = np.array([f for f, _ in rows], float); y = np.array([ok for _, ok in rows], float)
    w = np.zeros(X.shape[1]); b = 0.0
    for _ in range(steps):
        p = 1 / (1 + np.exp(-(X @ w + b)))
        w -= lr * (X.T @ (p - y) / len(y) + l2 * w / len(y)); b -= lr * float(np.mean(p - y))
    def score(c):
        if not c["f"]:
            return 0.0
        p = 1 / (1 + math.exp(-(float(np.dot(w, c["f"])) + b)))
        return -math.log(min(max(p, 0.02), 0.98))  # same clip as the design's rank term
    return score


def metrics(decisions, extra):
    """(top1 rate, mean rejections before the first keep) when candidates are ordered by Rank + extra(c)."""
    top1 = rej = 0
    for _, cands in decisions:
        order = sorted((c for c in cands if c["ok"] is not None), key=lambda c: c["rank"] + extra(c))  # stable
        top1 += order[0]["ok"]
        rej += next((i for i, c in enumerate(order) if c["ok"]), len(order))
    n = max(len(decisions), 1)
    return top1 / n, rej / n


def report(decisions):
    layouts = sorted({l for l, _ in decisions})
    print(f"{len(decisions)} decisions, {sum(c['ok'] is not None for _, cs in decisions for c in cs)} tried candidates, "
          f"{len(layouts)} layouts")
    print("Too little data to trust any learned number until Warren, dome and generated-variant logs exist.\n")
    print("| held-out layout | decisions | geometric top1 / rej | prior top1 / rej | linear top1 / rej |")
    print("|---|---|---|---|---|")
    for held in layouts:
        test = [d for d in decisions if d[0] == held]
        train = [d for d in decisions if d[0] != held]
        cells = [metrics(test, lambda c: 0.0), metrics(test, prior(train))]
        lin = linear(train)
        cells.append(metrics(test, lin) if lin else None)
        fmt = lambda m: "n/a" if m is None else f"{m[0]:.2f} / {m[1]:.2f}"
        print(f"| {held} | {len(test)} | " + " | ".join(fmt(m) for m in cells) + " |")


def selftest():
    def c(mode, rank, ok, f=None):
        return {"layout": "", "member": "m", "mode": mode, "rank": rank, "f": f, "ok": ok}
    # Relays always fail in training; the test decision tries a relay first (rank 0) then a direct move (rank 0.1).
    train = [("A", [c("Relay", 0, False, [1, 0]), c("Direct", 1, True, [0, 1])])] * 5
    test = [("B", [c("Relay", 0, False, [1, 0]), c("Direct", 0.1, True, [0, 1]), c("Land", 2, None, [0, 0])])]
    assert metrics(test, lambda _: 0.0) == (0.0, 1.0)
    assert metrics(test, prior(train)) == (1.0, 0.0)  # relay p = 1/7, direct p = 6/7
    lin = linear(train)
    assert lin is None or metrics(test, lin) == (1.0, 0.0)
    print("selftest ok")


if __name__ == "__main__":
    if sys.argv[1:] == ["--selftest"]:
        selftest()
    elif sys.argv[1:]:
        report(load(sys.argv[1:]))
    else:
        sys.exit(__doc__)
