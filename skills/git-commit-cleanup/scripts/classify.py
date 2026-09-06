import re, subprocess, sys, collections
OLD = sys.argv[1] if len(sys.argv) > 1 else "d2db3fad4b"; ORIG = sys.argv[2] if len(sys.argv) > 2 else "HEAD"
FEATURES = [  # (key, subject)
 ('dev',      'dev: agent guides, skills, and local workflow tooling'),
 ('base',     'base: schemas, params registry, panda headers, and build compatibility'),
 ('tinygrad', 'tinygrad: bump vendored runtime for the current driving models'),
 ('model',    'tsfdo model with combined-ONNX modeld runtime'),
 ('hyundai',  'hyundai: radar tracks, CAN stopping layer, gas-override safety, and fingerprint fallback'),
 ('stopping', 'stopping: V2/V3 stopping service, universal governor, and planner stop floors'),
 ('stoptools','tools(stopping): offline review, replay, and evaluation tooling'),
 ('stopdocs', 'docs(stopping): program records, worklogs, and design notes'),
 ('forcecoast','force coast: profile ramp, terminal taper, and the driver floor contract'),
 ('following','following: lead easing, lane-change lead surrogate, and Experimental boost'),
 ('speed',    'cruise: set speed and gas-override catch-up, curve speed, speed limit caps, and mapd'),
 ('lateral',  'controls: upstream driving-update ports and lateral tuning'),
 ('system',   'system: runtime liveness, watchdog diagnostics, and service hardening'),
 ('update',   'update: fullupdate with live SCC handoff, tailscale rescue daemon, and device update actions'),
 ('ui',       'ui: onroad overlays, HUD, and settings behavior'),
]
RULES = [  # first match wins; applied to subject
 (r'^(dev:|docs: add FrogPilot Testing refresh|tools: restore local macOS|Add project favicon)', 'dev'),
 (r'^(base:|panda:|params:|scons:|athena:|cereal:|manager: clean stale params|modeld: avoid generated transformations)', 'base'),
 (r'^bump tinygrad', 'tinygrad'),
 (r'^model: refresh tinygrad runtime support', 'model'),
 (r'^controls: cap Force Coast', 'forcecoast'),
 (r'^stopping review:', 'stoptools'),
 (r'^Revert "longcontrol:', 'stopping'),
 (r'^(deep_rl3|Revert "deep_rl3"|Revert deep_rl3|Revert to good deep_rl3|divided-rl|Revert "divided-rl"|michael-rl|rdf-driving|tsfdo|Rebel Legion model|Rebellious Hope model|Op model16|modeld:|dmonitoringmodeld:|Frame drop|Refactor compile_modeld|usbgpu:|lil smooth|Port recent modeld runtime)', 'model'),
 (r'^(hyundai:|Harden Hyundai CAN stopping layer|Longitudinal: |card:|card tests:)', 'hyundai'),
 (r'^(tools\(stopping\)|stopping tools:|review battery:|ctx_replay:|Rebuild stopping eval|Tighten stopping shadow readiness|Integrate stopping shadow cycle reporting)', 'stoptools'),
 (r'^(Pre-drive hardening and rollout plan|Friction-augmented plant model|Correct seg24 diagnosis|docs\(stopping\): cycle-21 record)', 'stopping'),
 (r'^(docs\(stopping\)|docs\+artifacts\(stopping\)|Record |Rollout plan:|Docs: |Rebuild stopping docs|Consolidate the comfort program|Correct seg24 diagnosis|V2 gate: triage|Engaged baseline cycle|worklog|review cursor|Add stopping-review cursor|Review cycle|Pre-drive hardening and rollout plan|Document offline stopping model validation|Fix cycle-exposed measurement defects|Read approach speed from entry|Crank stopping comfort requirement|Measure terminal disc-grab|Fix the aliased terminal-grab gate|Friction-augmented plant model)', 'stopdocs'),
 (r'^(force coast:|force-coast:|Force Coast|controls: honor Force Coast|Cap Force Coast|Cap unnecessary harsh approach|fix: force-coast|Revert "fix: force-coast|Smooth Force Coast|Extend Force Coast)', 'forcecoast'),
 (r'^(Gate highway lead easing|Soften leftmost lead easing|Keep lane change surrogate|Avoid target lane lead surrogate|Release lane change surrogate|Tune experimental no-lead boost|Rebase no-lead boost|Tune Experimental lead boost|Strengthen Experimental boost|Condition far-lead braking|frogpilot: follow testing following refactor|frogpilot: initialize onroad planner|controls: update longitudinal mpc callers|ui: drop removed human following toggle|frogpilot: restore controls settings|Soften decelerating lead approaches)', 'following'),
 (r'^(Snap set speed|Keep cruise decrement|Fix long-press cruise interval|Raise set speed during gas override|cruise:|Add initial set speed setting|Curve-aware CSC|Use a direct curve speed factor|Show CSC training|Cap speed limit overrides|Keep gas SLC override|navigation:)', 'speed'),
 (r'^(Port recent upstream driving updates|Desire keep pulsing is dead|lagd: higher min speed)', 'lateral'),
 (r'^(selfdrived:|hardwared:|loggerd:|locationd:|manager:|timed:|watchdog:|Capture UI watchdog|Keep UI watchdog|Preserve UI watchdog|system:|frogpilot: keep background work|frogpilot: drop stale telemetry|fix: cruise faults|set max cpu core frequency|A valid livePose|ui: never remove AlphaLongitudinalEnabled|ui: gate alert mouse transparency|ui: only repaint FrogPilotOnroadWindow)', 'system'),
 (r'^(fullupdate:|tailscale:|Live SCC handoff|Tap staged update alert|Handle staged update alert|Stage on-road updates|Throttle on-road update|Stabilize live SCC|Fix live restart readiness|Block reengagement during live updates|Restart staged updates|Require SCC handoff|Require Drive for live|Fix live update handoff|Release update lock|Retry failed live update|Keep AOL off during live updates|Accept stock SCC|Require sustained fresh passive|Harden stock SCC verifier|Make live-update banners|Show a banner while a full update|Render all update banners|Keep staging banner|Allow live update handoff in Park|Only show Preparing Restart|Tell the driver what still blocks|Route the settings Reboot button|Close settings on safe reboot|Fix device UI build and ignore pre-boot|ui: add full update action|updater: run full update|ui: compact staged update alert|Document on-road update handoff|docs: device fetches GitHub over SSH|docs: fix persistent device SSH key)', 'update'),
 (r'^(ui: restore onroad overlays|Improve lead info text layout|Smooth Hyundai distance button HUD|Limit Hyundai HUD preview)', 'ui'),
 (r'^(stopping:|planner:|longcontrol:|controls: restore longitudinal stopping|Improve close lead stopping|Brake earlier|Align stopped lead distance|Tighten creeping lead stop|Soften adequate-gap stop tails|Improve stopping hold|Keep close stopped-lead target|Centralize stopped lead target|Hold close stopped lead|Smooth explicit lead stop tails|Fix stopping source signals|Add stopping V2|Wire stop-target arbiter|Soften standstill hold|Publish true lead distance|StopReq stage|Lower StopReq gate|Anti-stiction|SAFETY REVERT|P1 approach cap|Guard off the terminal pre-release|Fix the wrong low-rollout|Stopping-phase planner-aTarget|Flip USE_STOPPING_V2|Forest deletion|Relax downhill queue brake clip|Reject unconfirmed radar tracks|Certify radar-only stop commitment|Revert the stop-target vision gate|Gate stopping service radar-only|Improve stopping authority|Raise the stopping rest-gap|Give re-anchor relief|Revert the cycle-13 pin trigger|stopping tests:)', 'stopping'),
]
# path overrides (apply to file paths regardless of commit)
PATH_RULES = [
 (r'^tinygrad_repo/', 'tinygrad'),
 (r'\.onnx$', 'model'),
 (r'^(tools/stopping/|tools/longitudinal/)', 'stoptools'),
 (r'^(docs/stopping/|docs/stopping_behavior_|docs/longitudinal_tuning_)', 'stopdocs'),
 (r'^(tools/lateral/|docs/lateral_tuning_|selfdrive/controls/lib/latcontrol_)', 'lateral'),
 (r'^(skills/|memory-bank/|\.claude/|\.codex/|AGENTS\.md$|CLAUDE\.md$)', None),  # blame decides (multi-feature docs)
]
def feature_of_subject(s):
    for rx, f in RULES:
        if re.search(rx, s): return f
    return None
def path_feature(p):
    for rx, f in PATH_RULES:
        if re.search(rx, p): return f
    return None
if __name__ == '__main__':
    out = subprocess.check_output(['git','log','--reverse','--format=%H|%s',f'{OLD}..{ORIG}'], text=True)
    cnt = collections.Counter(); unmatched=[]
    for line in out.splitlines():
        h,s = line.split('|',1); f = feature_of_subject(s)
        if f is None: unmatched.append((h[:10],s))
        cnt[f]+=1
    for k,_ in FEATURES: print(f'{cnt[k]:4d} {k}')
    print('unmatched:', len(unmatched))
    for h,s in unmatched: print('  ',h,s)
