#!/usr/bin/env python3
"""Rebuild OLD..ORIG as one commit per feature, attributing every changed line to
its originating commit with git blame (forward for added lines, reverse for deleted)."""
import os, re, subprocess, sys, collections
from functools import cache
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from classify import FEATURES, feature_of_subject, path_feature, split_feature, line_overrides
OLD, ORIG, WT = sys.argv[1], sys.argv[2], sys.argv[3]
BODIES = {
 'dev': "Agent guides, Codex/Claude commands and skills, device helper scripts, macOS\nbuild/test support (msgq unsupported-platform guards, spawn workers), favicon.",
 'base': "Custom cereal/opendbc schemas, service rates, params key registry and SCons\nrebuild rules, prebuilt panda firmware with restored ABI/SPI headers, athena\nregistration without konik keys.",
 'tinygrad': "Vendored tinygrad_repo update required by the combined-ONNX modeld runtime.",
 'model': "Newest model (tsfdo) as driving_supercombo.onnx (the superseded split models\nare deleted here); modeld runs the combined model, tinygrad driver-monitoring\nwarp, nv12 helper, capnp leak fix, previous-curvature seed when lateral is off.",
 'hyundai': "Radar track enablement with fallback, hardened Hyundai CAN stopping layer\n(StopReq latch and release, settled set), Active With Gas and block-braking-\nduring-gas-override safety rules, sent-accel telemetry, VIN-verified\npersistent fingerprint fallback in card, fork CAN bounds tests.",
 'stopping': "Stopping V2 stack and V3 service (stop context and arbiter, safety ledger,\nsecure hold, terminal descent, attributed-safety live mode), universal stop\ngovernor, planner stop floors, gas-release fix, creep guard and flat landing,\ntrim hand-off, release rehold, defect bundle, delivered landing, gap-aware\nterminal, entry comfort cap, landing lead tolerance, launch follow, fast\nrelease and starting blip guard, with their flags and tests.",
 'forcecoast': "Force Coast module: activation ramp, far-lead brake spike caps, terminal\ntaper below 1 m/s, rise limiter, driver floor contract, pass-through margin.",
 'following': "Following distance easing gated by lane, stopped-lead distance targets,\nTraffic Mode v2 hooks, Experimental lead boost by personality, far-lead\nbraking conditioned on model and ACC demand, departing-lead launch exemption,\ntesting following refactor compatibility.",
 'lcsurrogate': "radard lane-change lead surrogate (no surrogate below 12 m/s) and its\nfrogpilotRadarState telemetry, with tests.",
 'cem': "Predicted lead trajectory and predictive Conditional Experimental Mode\ntriggers ported from FrogPilot Testing; persistent manual CEM override.",
 'speed': "Set speed snapping and long-press isolation, initial set speed setting,\ngas-override set speed catch-up, curve-aware CSC targets, speed limit\noverride caps outside Poland, mapd binary.",
 'lateral': "Santa Fe torque tuning, curvature latcontrol and desire helper ports,\nconfidence-gated lane centering with offset setting, lagd and torqued\nlearner gates, calibration and pose validity ports.",
 'system': "selfdrived input validity and deviceState tolerance, cruise fault handling,\nmock-fingerprint toggle preservation, loggerd encoder resync and multi-root\ndeleter, hardwared thermal stalls, GPS time in UTC, manager process state\nsync and watchdog threads, UI watchdog diagnostics with backtrace capture,\ncontrol-core isolation for background FrogPilot work, stale telemetry removal.",
 'nvme': "manager_init NVMe guard: probe and mount wait, PCIe reset for an absent\ndevice, fallback LOG_ROOT=/data/realdata_fallback, Offroad_StorageMissing and\nstartup alert, deleter reserve on the fallback disk.",
 'update': "fullupdate.sh staging with on-road live SCC handoff supervisor (verified\nstock SCC takeover, Park support, safe reboot route, boot settle wait),\nlive_update_handoff module and tests, pandad safety hooks, update banners,\ntailscale bootstrap/supervisor, software settings full update action.",
 'ui': "Onroad overlays and lead info layout, Hyundai distance button HUD, stopping\nownership speed color, FrogPilot settings behavior restored on the Testing\nbase, onroad repaint hardening.",
 'stoptools': "Stopping event store, review pack, stop index, sim gate runner with judges'\ngrids, production radard in the sim, drive_report trial rules, KCS plant\nand extractors, ctx replay, plant/friction fitting, harness and census tools.",
 'tools': "Route sync tooling with retention limits, lane-change A/B replay and census,\nlateral and longitudinal analysis tools.",
 'stopdocs': "Architecture, parameters, eval, rollout plan, V3 service plan, universal\nstop program, retrospectives, cycle worklogs, longitudinal tuning worklog,\nreview cursor, archived fits, handover records.",
 'docs': "Lateral tuning records, route refresh process, FrogPilot Testing refresh\nprocess, device NVMe notes, memory bank, and the 2026-10-31 refresh plan,\ndecisions and inventories.",
 'drop_scc14': "Tests of the dormant dynamic SCC14 jerk path (DYNAMIC_SCC14_JERK is False; the\ncar sends the static 3.0/1.0/5.0). The flag, its constants, scc14_jerk_floors\nand the if/else around the static values stay in the hyundai commit: line\nattribution cannot unwrap an if/else. Reverting this commit is the start of\nits phase 5 removal.",
 'drop_stopline': "Reverted entry-bite trial, planner part: the stop line constants, its three\nfunctions, its state and its writers in the planner (SANTA_FE_STOP_LINE\nFalse), with the stop line and band-profile tests. The band-consistent\ngovernor rewrote governor_demand in place, so it and the flag pair stay in\nthe stopping commit (dead while GOVERNOR_BAND_PROFILE is False). Reverting\nthis commit is the start of its phase 5 removal.",
 'drop_ident': "Temporary brake-response identification hook and KCS test mode\n(IDENTIFICATION_HOOK False since 2026-09-27): the hook module, its wiring in\ncontrolsd, card, longcontrol, selfdrived and FrogPilot, the armed param, and\ntests. Reverting this commit is the start of its phase 5 removal.",
 'drop_creep': "Retired post-stop close-gap forward creep (STOPPING_CLOSE_GAP_CREEP_ENABLED\nFalse): constants, helpers, latch state and the stopping-state writer.\nReverting this commit is the start of its phase 5 removal.",
 'drop_wag': "Whole-approach governor research (WHOLE_APPROACH_GOVERNOR off): the\nstopping_governor module, its planner shadow certificate and capture\nreserve, its replay tool, and tests. Reverting this commit is the start of\nits phase 5 removal.",
 'drop_shadow': "Old shadow oracle and learned profile selector: the selector module, the\noracle classes and fitted model in stopping_shadow (the stopping_shadow\ntelemetry channel stays), the orphan pid observer hook, selector tools and\ntests. Reverting this commit is the start of its phase 5 removal.",
 'drop_lookahead': "shouldStop assert-side lookahead (SHOULD_STOP_LOOKAHEAD_S 0.0, off by\ndesign). Reverting this commit is the start of its phase 5 removal.",
 'drop_params': "Orphan params registry rows: CENavigation, TrafficFollow, TrafficJerk*,\nTrafficPersonalityProfile. SLCPriority3 is not orphan (the SLC priority menu\nwrites SLCPriority%1 up to 3). The settings menu residue stays in the base\nand ui commits (one-line key sets, an if/else with equal branches).\nReverting this commit is the start of its phase 5 removal.",
}
def git(*a, **k): return subprocess.run(['git', *a], check=True, capture_output=True, **k).stdout
def gitt(*a): return git(*a, text=True)
fidx = {k: i for i, (k, _) in enumerate(FEATURES)}
N = len(FEATURES)
# commits and their features
commits = gitt('rev-list', '--reverse', f'{OLD}..{ORIG}').split()
subj = dict(l.split('|', 1) for l in gitt('log', '--reverse', '--format=%H|%s', f'{OLD}..{ORIG}').splitlines())
cfeat = {}
for h in commits:
    f = feature_of_subject(subj[h]); assert f, subj[h]; cfeat[h] = fidx[f]
def cf(sha, p):
    f = split_feature(sha, p); return fidx[f] if f else cfeat[sha]
child = {h: (commits[i + 1] if i + 1 < len(commits) else None) for i, h in enumerate(commits)}
OLDF = gitt('rev-parse', OLD).strip(); ORIGF = gitt('rev-parse', ORIG).strip()
def mode_map(rev):
    m = {}
    for l in gitt('ls-tree', '-r', '-z', rev).split('\0'):
        if l: meta, p = l.split('\t', 1); m[p] = meta.split()[0]
    return m
mode_old, mode_new = mode_map(OLD), mode_map(ORIG)
# changed files
status = {}
for l in gitt('diff', '--no-renames', '--name-status', OLD, ORIG).splitlines():
    st, p = l.split('\t', 1); status[p] = st
binary = set()
for l in gitt('diff', '--no-renames', '--numstat', OLD, ORIG).splitlines():
    a, b, p = l.split('\t', 2)
    if a == '-': binary.add(p)
for p in status:
    if mode_new.get(p) == '120000' or mode_old.get(p) == '120000': binary.add(p)
@cache
def blob(rev, p): return git('show', f'{rev}:{p}')
def last_touch_feat(p):
    h = gitt('log', '-1', '--format=%H', f'{OLD}..{ORIG}', '--', p).strip(); return cf(h, p)
HDR = re.compile(rb'^([0-9a-f]{40}) (\d+) (\d+)(?: (\d+))?$')
def blame_lines(args, p, orig=None):
    """returns {final_line_no: sha} from --line-porcelain; orig, if given, collects {final_line_no: line_no in that sha}"""
    out = git('blame', '--line-porcelain', *args, '--', p); res = {}
    for l in out.split(b'\n'):
        m = HDR.match(l)
        if m:
            res[int(m.group(3))] = m.group(1).decode()
            if orig is not None: orig[int(m.group(3))] = int(m.group(2))
    return res
DROPF = {i for i, (k, _) in enumerate(FEATURES) if k.startswith('drop_')}
@cache
def rewrites(h, p):
    """{line in commit h: kept feature} for the lines a drop commit h wrote over lines that a kept commit of the stack wrote.
    Blame alone would move such a line into the drop commit, so reverting it would delete the kept line it replaced."""
    res = {}
    for l in git('show', '--format=', '--no-renames', '-U0', h, '--', p).split(b'\n'):
        m = re.match(rb'^@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@', l)
        if not m: continue
        a, b, c, d = int(m[1]), int(m[2]) if m[2] is not None else 1, int(m[3]), int(m[4]) if m[4] is not None else 1
        if not b or not d: continue
        prev_orig = {}; prev = blame_lines([f'-L{a},{a + b - 1}', f'{OLD}..{h}^'], p, prev_orig)
        # a line an earlier drop commit rewrote counts as the kept line it replaced (a rewrite chain)
        kept = [cf(sha, p) if cf(sha, p) not in DROPF else rewrites(sha, p).get(prev_orig[ln])
                for ln, sha in prev.items() if sha in cfeat]
        kept = [f for f in kept if f is not None]
        if kept:
            for j in range(c, c + d): res[j] = collections.Counter(kept).most_common(1)[0][0]
    return res
fallback_counter = collections.Counter(); rewrite_counter = collections.Counter()
# per-file plan: dict p -> function(i) -> bytes or None
plans = {}; whole = {}
for p, st in status.items():
    pf = path_feature(p)
    if st == 'D':
        h = gitt('log', '--format=%H', '--diff-filter=D', f'{OLD}..{ORIG}', '--', p).split()[0]
        whole[p] = ('D', fidx[pf] if pf else cf(h, p)); continue
    new = blob(ORIG, p)
    old = blob(OLD, p) if st == 'M' else None
    if pf or p in binary or (st == 'M' and old == new):
        whole[p] = ('W', fidx[pf] if pf else last_touch_feat(p)); continue
    fb = last_touch_feat(p)
    fwd_orig = {}; fwd = blame_lines([f'{OLD}..{ORIG}'], p, fwd_orig)
    ovr = {ln: fidx[f] for ln, f in line_overrides(p, new).items()}
    def feat_add(ln, p=p, fwd=fwd, fwd_orig=fwd_orig, ovr=ovr, fb=fb):
        if ln in ovr: return ovr[ln]
        sha = fwd.get(ln)
        if sha in cfeat and cf(sha, p) in DROPF:
            kept = rewrites(sha, p).get(fwd_orig[ln])
            if kept is not None: rewrite_counter[p] += 1; return kept
        if sha in cfeat: return cf(sha, p)
        fallback_counter[p] += 1; return fb
    new_lines = new.splitlines(keepends=True)
    if st == 'A':
        created = cf(gitt('log', '--format=%H', '--diff-filter=A', f'{OLD}..{ORIG}', '--', p).split()[-1], p)
        feats = [max(feat_add(j + 1), created) for j in range(len(new_lines))]
        plans[p] = (lambda i, nl=new_lines, fs=feats, c=created: (b''.join(l for l, f in zip(nl, fs) if f <= i) if i >= c else None))
        continue
    old_lines = old.splitlines(keepends=True)
    rev = blame_lines(['--reverse', f'{OLD}..{ORIG}'], p)
    ovr_old = {ln: fidx[f] for ln, f in line_overrides(p, old, 'old').items()}
    def feat_del(ln, p=p, rev=rev, ovr_old=ovr_old, fb=fb):
        if ln in ovr_old: return ovr_old[ln]
        sha = rev.get(ln); ch = child.get(sha) if sha else None
        if ch in cfeat: return cf(ch, p)
        if sha in cfeat and sha != ORIGF: return cf(sha, p)
        fallback_counter[p] += 1; return fb
    hunks = []
    for l in git('diff', '--no-renames', '-U0', OLD, ORIG, '--', p).split(b'\n'):
        m = re.match(rb'^@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@', l)
        if m:
            a, b, c, d = int(m[1]), int(m[2]) if m[2] is not None else 1, int(m[3]), int(m[4]) if m[4] is not None else 1
            hunks.append((a, b, c, d, [feat_del(j) for j in range(a, a + b)], [feat_add(j) for j in range(c, c + d)]))
    def build(i, ol=old_lines, nl=new_lines, hs=hunks):
        out = []; op = 1
        for a, b, c, d, df, af in hs:
            end = a if b == 0 else a - 1
            out += ol[op - 1:end]; op = end + 1
            for j, f in zip(range(a, a + b), df):
                if f > i: out.append(ol[j - 1])
            op = a + b if b else op
            for j, f in zip(range(c, c + d), af):
                if f <= i: out.append(nl[j - 1])
        out += ol[op - 1:]
        return b''.join(out)
    assert build(-1) == old, p; assert build(N - 1) == new, p
    plans[p] = build
def content(p, i):
    if p in whole:
        kind, f = whole[p]
        if kind == 'D': return blob(OLD, p) if i < f else None
        return blob(ORIG, p) if i >= f else (blob(OLD, p) if status[p] == 'M' else None)
    return plans[p](i)
print(f'files: {len(status)}  blamed: {len(plans)}  whole: {len(whole)}  fallback lines: {sum(fallback_counter.values())} in {len(fallback_counter)} files', flush=True)
for p, n in fallback_counter.most_common(8): print('   fallback', n, p)
for p, n in rewrite_counter.most_common(): print('   kept rewrite', n, p)
os.chdir(WT)
modechg = {p: (fidx[path_feature(p)] if path_feature(p) else last_touch_feat(p)) for p in status if p in mode_old and p in mode_new and mode_old[p] != mode_new[p]}
def mode_at(p, i): return mode_new.get(p) if (p not in modechg or i >= modechg[p]) else mode_old.get(p)
prev = {p: (content(p, -1), mode_at(p, -1)) for p in status}
for i, (k, subject) in enumerate(FEATURES):
    changed = []
    for p in status:
        cur = (content(p, i), mode_at(p, i))
        if cur == prev[p]: continue
        data, mode = cur
        if data is None:
            os.remove(p)
        else:
            os.makedirs(os.path.dirname(p) or '.', exist_ok=True)
            if data != prev[p][0] or not os.path.lexists(p):
                if os.path.lexists(p): os.remove(p)
                if mode == '120000': os.symlink(data.decode(), p)
                else:
                    with open(p, 'wb') as fh: fh.write(data)
            if mode != '120000': os.chmod(p, 0o755 if mode == '100755' else 0o644)
        prev[p] = cur; changed.append(p)
    if not changed: print(f'-- EMPTY {k}'); continue
    for j in range(0, len(changed), 500): git('add', '-A', '-f', '--', *changed[j:j + 500])
    git('commit', '-q', '-m', subject, '-m', BODIES[k])
    print(gitt('log', '-1', '--format=%h').strip(), subject, '(' + gitt('diff', '--shortstat', 'HEAD~1', 'HEAD').strip() + ')', flush=True)
ok = gitt('rev-parse', 'HEAD^{tree}').strip() == gitt('rev-parse', f'{ORIG}^{{tree}}').strip()
print('TREE_IDENTICAL' if ok else 'TREE_DIFFERS'); print('status:', gitt('status', '--porcelain')[:300])
