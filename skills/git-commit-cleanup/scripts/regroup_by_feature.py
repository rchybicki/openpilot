#!/usr/bin/env python3
"""Rebuild OLD..ORIG as one commit per feature, attributing every changed line to
its originating commit with git blame (forward for added lines, reverse for deleted)."""
import os, re, subprocess, sys, collections
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from classify import FEATURES, feature_of_subject, path_feature
OLD, ORIG, WT = sys.argv[1], sys.argv[2], sys.argv[3]
BODIES = {
 'dev': "Agent guides, Codex/Claude commands and skills, memory bank, device helper\nscripts, macOS build/test restore, favicon, refresh process doc.",
 'base': "Custom cereal/opendbc schemas, driver-monitoring service rates, params key\nregistry and SCons rebuild rules, prebuilt panda firmware with restored\nABI/SPI headers, athena registration without konik keys.",
 'tinygrad': "Vendored tinygrad_repo update required by the combined-ONNX modeld runtime.",
 'model': "Newest model (tsfdo) as driving_supercombo.onnx; modeld runs the combined\nmodel, tinygrad driver-monitoring warp, nv12 helper, capnp leak fix.\nSupersedes deep_rl3, divided-rl, michael-rl, Rebel Legion, Rebellious Hope,\nrdf-driving.",
 'hyundai': "Radar track enablement with fallback, Santa Fe tuning, hardened Hyundai CAN\nstopping layer (StopReq release on gas), Active With Gas and block-braking-\nduring-gas-override safety rules, VIN-verified persistent fingerprint\nfallback in card, fork CAN bounds tests.",
 'stopping': "Stopping V2 stack and V3 service (standstill evidence ledger, secure hold,\nterminal descent, attributed-safety live mode), universal stop governor as\nthe live approach law, planner stop floors (roll-in, aim-lane, rest-close,\nre-slam), stop target arbiter/helpers, lead provenance, identification\ndrive hook, longcontrol authority, log schema fields, unit/replay tests.",
 'stoptools': "Stopping event store, review pack, stop index, attributed-safety gate and\nveto tools, ctx replay, plant/friction fitting, harness and census tools.",
 'stopdocs': "Architecture, parameters, eval, rollout plan, V3 service plan, universal\nstop program, retrospectives, cycle worklogs, review cursor, archived fits.",
 'forcecoast': "Force Coast module: activation ramp, far-lead brake spike caps, terminal\ntaper below 1 m/s, rise limiter, driver floor contract, pass-through margin.",
 'following': "Following distance easing gated by lane, stopped-lead distance targets,\nradard lane-change lead surrogate, Experimental lead boost by personality,\nfar-lead braking conditioned on model and ACC demand, testing following\nrefactor compatibility.",
 'speed': "Set speed snapping and long-press isolation, initial set speed setting,\ngas-override set speed catch-up, curve-aware CSC targets, speed limit\noverride caps outside Poland, mapd binary and route sync tooling.",
 'lateral': "Ports of upstream driving updates (curvature latcontrol, lagd, desire\nhelper) and lateral tuning analysis tooling and notes.",
 'system': "selfdrived input validity and deviceState tolerance, cruise fault handling,\nmock-fingerprint toggle preservation, locationd/lagd fixes, loggerd encoder\nresync, hardwared thermal stalls, GPS time in UTC, manager process state\nsync and watchdog threads, UI watchdog diagnostics with backtrace capture,\ncontrol-core isolation for background FrogPilot work.",
 'update': "fullupdate.sh staging with on-road live SCC handoff supervisor (verified\nstock SCC takeover, Park support, safe reboot route, boot settle wait),\nlive_update_handoff module and tests, pandad safety hooks, update banners,\ntailscale bootstrap/supervisor, software settings full update action,\ndevice SSH deploy-key docs.",
 'ui': "Onroad overlays and lead info layout, Hyundai distance button HUD, and\nFrogPilot settings behavior restored on the Testing base.",
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
def blob(rev, p): return git('show', f'{rev}:{p}')
def last_touch_feat(p):
    h = gitt('log', '-1', '--format=%H', f'{OLD}..{ORIG}', '--', p).strip(); return cfeat[h]
HDR = re.compile(rb'^([0-9a-f]{40}) (\d+) (\d+)(?: (\d+))?$')
def blame_lines(args, p):
    """returns {final_line_no: sha} from --line-porcelain"""
    out = git('blame', '--line-porcelain', *args, '--', p); res = {}
    for l in out.split(b'\n'):
        m = HDR.match(l)
        if m: res[int(m.group(3))] = m.group(1).decode()
    return res
fallback_counter = collections.Counter()
# per-file plan: dict p -> function(i) -> bytes or None
plans = {}; whole = {}
for p, st in status.items():
    pf = path_feature(p)
    if st == 'D':
        h = gitt('log', '--format=%H', '--diff-filter=D', f'{OLD}..{ORIG}', '--', p).split()[0]
        whole[p] = ('D', fidx[pf] if pf else cfeat[h]); continue
    new = blob(ORIG, p)
    old = blob(OLD, p) if st == 'M' else None
    if pf or p in binary or (st == 'M' and old == new):
        whole[p] = ('W', fidx[pf] if pf else last_touch_feat(p)); continue
    fb = last_touch_feat(p)
    fwd = blame_lines([f'{OLD}..{ORIG}'], p)
    def feat_add(ln):
        sha = fwd.get(ln)
        if sha in cfeat: return cfeat[sha]
        fallback_counter[p] += 1; return fb
    new_lines = new.splitlines(keepends=True)
    if st == 'A':
        created = cfeat[gitt('log', '--format=%H', '--diff-filter=A', f'{OLD}..{ORIG}', '--', p).split()[-1]]
        feats = [max(feat_add(j + 1), created) for j in range(len(new_lines))]
        plans[p] = (lambda i, nl=new_lines, fs=feats, c=created: (b''.join(l for l, f in zip(nl, fs) if f <= i) if i >= c else None))
        continue
    old_lines = old.splitlines(keepends=True)
    rev = blame_lines(['--reverse', f'{OLD}..{ORIG}'], p)
    def feat_del(ln):
        sha = rev.get(ln); ch = child.get(sha) if sha else None
        if ch in cfeat: return cfeat[ch]
        if sha in cfeat and sha != ORIGF: return cfeat[sha]
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
