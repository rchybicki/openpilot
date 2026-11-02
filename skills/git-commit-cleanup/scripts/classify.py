import ast, re, subprocess, sys, collections
OLD = sys.argv[1] if len(sys.argv) > 1 else "d2db3fad4b"; ORIG = sys.argv[2] if len(sys.argv) > 2 else "HEAD"
FEATURES = [  # (key, subject); order = PLAN.md phase 6 batches, drop units last (phase 5 reverts them)
 ('dev',       'dev: agent guides, skills, and local workflow tooling'),
 ('base',      'base: schemas, params registry, panda headers, and build compatibility'),
 ('tinygrad',  'tinygrad: bump vendored runtime for the current driving models'),
 ('model',     'model: tsfdo combined-ONNX driving model and modeld runtime'),
 ('hyundai',   'hyundai: radar tracks, CAN stopping layer, gas-override safety, and fingerprint fallback'),
 ('stopping',  'stopping: V2/V3 stopping service, universal governor, planner stop floors, and launch follow'),
 ('forcecoast','force coast: profile ramp, terminal taper, and the driver floor contract'),
 ('following', 'following: lead easing, Traffic Mode v2 hooks, Experimental boost, and departing-lead launch'),
 ('lcsurrogate','radard: lane-change lead surrogate and its telemetry'),
 ('cem',       'cem: predictive triggers, lead trajectory prediction, and persistent manual override'),
 ('speed',     'cruise: set speed, gas-override catch-up, curve speed, speed limit caps, and mapd'),
 ('lateral',   'lateral: Santa Fe torque tuning, lane centering, desire state, learner gates, and pose validity'),
 ('system',    'system: runtime liveness, watchdog diagnostics, logger, clock, and service hardening'),
 ('nvme',      'system: NVMe guard with fallback log root and storage alerts'),
 ('update',    'update: fullupdate with live SCC handoff, safe reboot, tailscale rescue daemon, and update UI'),
 ('ui',        'ui: onroad overlays, HUD, stopping ownership color, and settings behavior'),
 ('stoptools', 'tools(stopping): offline review, replay, sim gates, and drive reports'),
 ('tools',     'tools: route sync, lane-change A/B, lateral and longitudinal analysis'),
 ('stopdocs',  'docs(stopping): program records, worklogs, and design notes'),
 ('docs',      'docs: tuning records, route refresh process, device notes, and the FrogPilot refresh plan'),
 ('drop_scc14',   'drop: H06 dynamic SCC14 jerk tests (DYNAMIC_SCC14_JERK False)'),
 ('drop_stopline','drop: D01 planner stop line (SANTA_FE_STOP_LINE False)'),
 ('drop_ident',   'drop: D02 identification hook / KCS test mode (IDENTIFICATION_HOOK False)'),
 ('drop_creep',   'drop: D03 post-stop close-gap creep (STOPPING_CLOSE_GAP_CREEP_ENABLED False)'),
 ('drop_wag',     'drop: D04 whole-approach governor research (WHOLE_APPROACH_GOVERNOR off)'),
 ('drop_shadow',  'drop: D05 old shadow oracle and learned profile selector'),
 ('drop_lookahead','drop: D06 shouldStop lookahead (SHOULD_STOP_LOOKAHEAD_S 0.0)'),
 ('drop_params',  'drop: D07 orphan params registry rows'),
]
RULES = [  # first match wins; applied to subject
 # the 2026-09-06 regroup commits (mixed ones are split further by SPLITS / PATH_RULES / LINE_RULES)
 (r'^dev: agent guides', 'dev'),
 (r'^base: schemas', 'base'),
 (r'^tinygrad: bump', 'tinygrad'),
 (r'^tsfdo model with combined-ONNX', 'model'),
 (r'^hyundai: radar tracks', 'hyundai'),
 (r'^stopping: V2/V3 stopping service', 'stopping'),
 (r'^tools\(stopping\): offline review', 'stoptools'),
 (r'^docs\(stopping\): program records', 'stopdocs'),
 (r'^force coast: profile ramp', 'forcecoast'),
 (r'^following: lead easing', 'following'),
 (r'^cruise: set speed', 'speed'),
 (r'^controls: upstream driving-update ports', 'lateral'),
 (r'^system: runtime liveness', 'system'),
 (r'^update: fullupdate', 'update'),
 (r'^ui: onroad overlays', 'ui'),
 # later work
 (r'^port: lead prediction and predictive CEM', 'cem'),
 (r'^(tools\(lane_change\)|docs\(lane_change\))', 'tools'),
 (r'^radard: (no lane-change surrogate|publish lane-change surrogate)', 'lcsurrogate'),
 (r'^longitudinal: let a departing lead launch', 'following'),
 (r'^modeld: seed previous curvature', 'model'),
 (r'^torqued: drop fit points', 'lateral'),
 (r'^Lane centering', 'lateral'),
 (r'^(Keep macOS analysis workers|Prevent macOS Python aborts)', 'dev'),
 (r'^(Show stopping controller ownership|Clear stale stopping color)', 'ui'),
 (r'^(NVMe guard|Merge misc-nvme-guard)', 'nvme'),
 (r'^route_sync: ', 'tools'),
 (r'^docs: FrogPilot Testing 2026-10-31 refresh', 'docs'),
 # drop units by commit
 (r'^Stopping: (entry-bite trial - stop line|revert the entry-bite trial)', 'drop_stopline'),
 (r'^(Fix calibration input checks and driver handback|Prepare one-attempt brake response calibration|Prepare wheel-button brake response'
  r'|Arm brake response test mode|Release test mode authority|Enable brake response test mode|Set the cruise target to 30 km/h'
  r'|Share the test mode press timer|Run the KCS1 scripted stop program|Let the scripted rep own the wire|Start the next scripted stop'
  r'|Test the rep ownership rule|Add the KCS2 block|Close the test-mode review findings|Test mode|Replace KCS2|KCS2: pair J with M'
  r'|KCS2 fast cycle|Fast cycle: fault frames)', 'drop_ident'),
 # stopping tools (drive_report, sim, review tools, KCS extractors)
 (r'^(Extract KCS1 test reps|Fix the KCS1 extractor|Extractor: hold a ramp|Pre-register the KCS2 decision gates'
  r'|Compare all marked manual stops|Audit stopping CAN response|Separate braking onset from release|Review new stopping routes'
  r'|Stopping tools|Stopping review tools|Stopping sim|WIP drive_report|drive_report|CAPT: drive_report|CAPT trial: act on'
  r'|TRIALFIX: LAND per-drive rule|Merge cyc1013-rest: drive_report)', 'stoptools'),
 # stopping docs
 (r'^(docs\(stopping\)|Stopping docs|Stopping worklog|Stopping program|Worklog|Longitudinal worklog|CAPT docs'
  r'|Record (Radek stopping baseline|native stopping validation|new-route stopping evidence|the car-first evidence path)'
  r'|Define factory cruise comparison)', 'stopdocs'),
 # stopping service and longitudinal control
 (r'^(stopping:|Stopping|stopping_flags|stage-1 guard tests|tests: the HEAD|test_fast_release_to_lanes|Fix acceleration feedback in stopping'
  r'|Limit stopping profile recovery|Enable bounded stopping profile-recovery|Reset stale stopping context|Soften stop entry'
  r'|Predict (stopping response|pending brake requests)|Reduce repeated braking|Reset pre-entry stopping diagnostics'
  r'|Fix factory cruise standstill crash|Send a zero request once|Creep guard|Santa Fe trim hand-off|Trim hand-off'
  r'|WIP DELIVERED_LANDING|DELIVERED_LANDING|GAP_AWARE_TERMINAL|TRIALFIX|ENTRY_COMFORT_CAP|LANDING_LEAD_TOLERANCE|WS TERMINAL'
  r'|LAUNCH-C research prototype|LAUNCH_FOLLOW|FAST_RELEASE_TO_LANES|STARTING_BLIP_GUARD|Astra review acted on: STARTING_BLIP_GUARD'
  r'|Merge cyc10)', 'stopping'),
]
# lines a commit wrote in matching paths go to another feature (splits the coarse 2026-09-06 regroup commits)
SPLITS = [  # (commit sha prefix, path regex, feature)
 ('f3aefffbe7', r'^selfdrive/controls/radard\.py$', 'lcsurrogate'),
 ('f05c127ec2', r'^selfdrive/controls/radard\.py$', 'lcsurrogate'),  # S03 and the far-lead removal stay stopping (LINE_RULES)
 ('f3aefffbe7', r'^frogpilot/controls/lib/conditional_experimental_mode\.py$', 'cem'),
 ('f3aefffbe7', r'^frogpilot/controls/lib/(speed_limit_controller|curve_speed_controller|frogpilot_vcruise)\.py$', 'speed'),
 ('f3aefffbe7', r'^frogpilot/ui/', 'ui'),
 ('147908d4c4', r'^selfdrive/locationd/', 'lateral'),
 ('147908d4c4', r'^system/athena/', 'base'),
 ('147908d4c4', r'^frogpilot/ui/qt/onroad/frogpilot_onroad\.', 'ui'),
 ('ab3464a77d', r'^(cereal/|selfdrive/controls/)', 'stopping'),
 # D01: only the planner stop line is separable; its band-consistent governor rewrote governor_demand in place (kept, dead
 # while GOVERNOR_BAND_PROFILE is False), and the flag pair stays because that derived flag reads SANTA_FE_STOP_LINE
 ('56e1512892', r'^selfdrive/controls/lib/(stopping_service|stopping_flags)\.py$|^selfdrive/controls/lib/tests/test_(stop_governor|stopping_recovery)\.py$', 'stopping'),
 ('1503e80ce1', r'^selfdrive/controls/lib/stopping_flags\.py$', 'stopping'),
]
# whole-file overrides (apply to file paths regardless of commit)
PATH_RULES = [
 (r'^tinygrad_repo/', 'tinygrad'),
 (r'\.onnx$', 'model'),
 # drop units: files that exist only for the unit (and tools that import a dropped module)
 (r'^selfdrive/controls/lib/identification_hook\.py$|^selfdrive/controls/tests/test_identification_(hook|hook_panda|scope)\.py$', 'drop_ident'),
 (r'^selfdrive/controls/lib/tests/test_(stop_line|governor_band_profile)\.py$', 'drop_stopline'),
 (r'^selfdrive/controls/lib/stopping_governor\.py$|^selfdrive/controls/tests/test_whole_approach_governor\.py$'
  r'|^tools/stopping/review/whole_approach_replay\.py$', 'drop_wag'),
 (r'^selfdrive/controls/lib/stopping_profile_selector\.py$|^selfdrive/controls/lib/tests/test_stopping_shadow\.py$'
  r'|^tools/stopping/(train_profile_selector|test_profile_selector|benchmark_controller_variants)\.py$', 'drop_shadow'),
 # single-feature directories
 (r'^(tools/stopping/)', 'stoptools'),
 (r'^(tools/route_sync/|tools/lane_change/|tools/lateral/|tools/longitudinal/)', 'tools'),
 (r'^(docs/stopping/|docs/stopping_behavior_|docs/longitudinal_tuning_)', 'stopdocs'),
 (r'^(docs/|memory-bank/)', 'docs'),
 (r'^(tools/mac_setup\.sh|tools/install_python_dependencies\.sh|msgq_repo/)', 'dev'),
 (r'^system/tailscale/', 'update'),
 (r'^frogpilot/controls/lib/force_coast\.py$', 'forcecoast'),
 (r'^selfdrive/controls/tests/test_radard_lane_change_surrogate\.py$', 'lcsurrogate'),
 (r'^(selfdrive/controls/lib/latcontrol_|opendbc_repo/opendbc/car/torque_data/|selfdrive/controls/lib/lane_centering\.py$'
  r'|selfdrive/controls/tests/test_lane_centering\.py$|selfdrive/controls/lib/desire_helper\.py$|selfdrive/locationd/)', 'lateral'),
 (r'^(skills/|\.claude/|\.codex/|AGENTS\.md$|CLAUDE\.md$)', None),  # blame decides (multi-feature docs)
]
# per-line overrides inside shared files. side 'new' selects ORIG lines (additions), 'old' selects OLD lines (deletions).
#   line:  lines matching the regex
#   block: the largest statement starting on a matching line (Python; decorators and the comment lines directly
#          above included); enclosing: the smallest statement containing a matching line
#   (Python files: an if/for/while/with/try without else whose whole body is selected is selected too)
#   range: (start, end) regexes, from a start line through the next end line, inclusive
LINE_RULES = [  # (path regex, feature, kind, pattern[, side])
 # radard: the 09-06 stopping commit carried the lane-change surrogate; its true lead distance (S03) and far-lead removal stay
 (r'^selfdrive/controls/radard\.py$', 'stopping', 'line',
  r'get_published_lead_distance|# §4\.2\.1: mutation active only|^from openpilot\.frogpilot\.common\.frogpilot_variables import get_frogpilot_toggles$'),
 (r'^selfdrive/controls/radard\.py$', 'stopping', 'line',
  r"lead_dict\['dRel'\] -= frogpilot_plan\.increasedStoppedDistance|^from openpilot\.frogpilot\.common\.frogpilot_variables import THRESHOLD, get_frogpilot_toggles$"
  r'|self\.radarfulFilter = FirstOrderFilter', 'old'),
 (r'^selfdrive/controls/radard\.py$', 'stopping', 'block', r"^  def potential_far_lead\b|^  if low_speed_override and not lead_dict\['status'\] and len\(tracks\) > 0:", 'old'),
 # F06 persistent manual CEM override (the 09-06 following commit carried it)
 (r'^frogpilot/controls/frogpilot_card\.py$', 'cem', 'line', r'set_persistent_ce_override'),
 (r'^frogpilot/controls/frogpilot_card\.py$', 'cem', 'line', r'^from openpilot\.frogpilot\.controls\.lib\.conditional_experimental_mode import CEStatus$', 'old'),
 # H06 dynamic SCC14 jerk: its if/else around the static jerk values stays (phase 5 unwraps it); its tests go
 (r'hyundai/tests/test_can_bounds_fork\.py$', 'drop_scc14', 'block',
  r'^  def test_(nonfinite_accel_with_dynamic_jerk|dynamic_jerk_step_bounded_at_12_7)\b|^class TestSyntheticRampJerk\b|^import math$'),
 # D02 identification hook: whole hook blocks in the shared wiring (the rest of the hook commits' lines follow blame)
 (r'^selfdrive/controls/lib/longcontrol\.py$', 'drop_ident', 'block',
  r'^from openpilot\.selfdrive\.controls\.lib\.identification_hook import|^    if stopping_flags\.IDENTIFICATION_HOOK and self\._service_shadow_scope:'
  r'|^    if self\._id_hook is not None:$|^    if input_hold and self\._id_hook is not None:|^    if self\._id_hook is not None and id_inputs is not None and not input_hold:'),
 (r'^selfdrive/controls/lib/longcontrol\.py$', 'stopping', 'block', r'^    hook_intent = '),  # read by the kept raw_should_stop line
 (r'^selfdrive/controls/controlsd\.py$', 'drop_ident', 'block',
  r'^from openpilot\.selfdrive\.controls\.lib\.identification_hook import|^  def _identification_inputs\b|^        id_inputs=self\._identification_inputs\('
  r'|^ +if stopping_flags\.IDENTIFICATION_HOOK:$|^from openpilot\.selfdrive\.controls\.lib import stopping_flags$'),
 (r'^selfdrive/controls/tests/test_stopping_input_boundary\.py$', 'drop_ident', 'block',
  r'^from openpilot\.selfdrive\.controls\.tests\.test_identification_hook import|^INTENT = |^from openpilot\.common\.realtime import DT_CTRL$'
  r'|^def (_kcs1|_rep|test_motion_fault_during_a_rep_locks_and_keeps_its_braking|test_fault_publishes_the_lock_without_advancing_the_hook'
  r'|test_fault_after_the_intent_keeps_the_finishing_banner_while_moving|test_disengaged_fault_frame_publishes_the_lock)\b'),
 (r'^common/params_keys\.h$', 'drop_ident', 'line', r'"IdentificationTestArmed"'),
 # D03 close-gap creep
 (r'^selfdrive/controls/lib/longcontrol\.py$', 'drop_creep', 'range', (r'^# --- Close-the-gap forward creep behind', r'^STOPPING_CLOSE_GAP_CREEP_ENABLED = ')),
 (r'^selfdrive/controls/lib/longcontrol\.py$', 'drop_creep', 'line', r'^CREEP_[A-Z_]+ += '),
 (r'^selfdrive/controls/lib/longcontrol\.py$', 'drop_creep', 'block', r'^def (stopping_close_gap_creep_|should_apply_stopping_close_gap_creep)'),
 (r'^selfdrive/controls/lib/longcontrol\.py$', 'drop_creep', 'range', (r'^      # Close-the-gap forward creep behind a confirmed STOPPED lead', r'^      # or overspeed\.$')),
 (r'^selfdrive/controls/lib/longcontrol\.py$', 'drop_creep', 'enclosing', r'^        STOPPING_CLOSE_GAP_CREEP_ENABLED$'),
 (r'^selfdrive/controls/lib/longcontrol\.py$', 'drop_creep', 'block',
  r'self\.(creeping|close_gap_creep_standstill_time_s) = |# Close-the-gap forward-creep latch|# oscillate \(a pure per-frame|# drop the close-gap creep latch'),
 (r'^selfdrive/controls/lib/tests/test_longcontrol_fast_release\.py$', 'drop_creep', 'enclosing', r'^  stopping_close_gap_creep_should_arm,$'),
 (r'^selfdrive/controls/lib/tests/test_longcontrol_fast_release\.py$', 'drop_creep', 'block',
  r'^def test_(creep_arms_when_stopped_short_behind_stopped_lead|creep_disarms_at_target|creep_never_for_moving_or_absent_lead_or_force_coast'
  r'|creep_overspeed_disarms|close_gap_creep_[a-z_]+|creep_accel_is_gentle_positive_bounded|creep_true_rest_target_in_bounds_across_isd)\b'),
 (r'^selfdrive/controls/lib/tests/test_stop_target_arbiter\.py$', 'drop_creep', 'line',
  r'^  "stopping_close_gap_creep_(should_arm|should_disarm|accel_target)",$|^  # close-the-gap forward creep behind a confirmed stopped lead \(route'
  r'|^  # stopping-layer consumers gated against the same effective'),
 # D04 whole-approach governor (the longitudinalPlan.wholeApproach* schema fields stay; reverting leaves them at defaults)
 (r'^selfdrive/controls/lib/longitudinal_planner\.py$', 'drop_wag', 'block',
  r'^from openpilot\.selfdrive\.controls\.lib\.stopping_governor import|^WHOLE_APPROACH_[A-Z_]+ = '
  r'|^def (reset_whole_approach_certificate|update_whole_approach_certificate|get_whole_approach_shadow_candidate)\b'
  r'|^  def _(finish_whole_approach|update_whole_approach_shadow)\b|self\._wa_(stop_ctx|lead_auth)\b|reset_whole_approach_certificate\(self\)'
  r'|self\.wa_stats = |self\.whole_approach_(demand|reason|safety_min|deficit) = |self\._finish_whole_approach\(|longitudinalPlan\.wholeApproach'),
 (r'^selfdrive/controls/lib/longitudinal_planner\.py$', 'drop_wag', 'enclosing', r'^      self\._update_whole_approach_shadow\($'),
 (r'^selfdrive/controls/lib/longitudinal_planner\.py$', 'drop_wag', 'line', r'^# Whole-approach governor certificate and shadow'),
 (r'^selfdrive/controls/lib/longitudinal_planner\.py$', 'drop_wag', 'range', (r'^    # WHOLE-APPROACH SHADOW: all inputs', r'^    # telemetry, certification')),
 (r'^selfdrive/controls/lib/stopping_flags\.py$', 'drop_wag', 'line', r'^WHOLE_APPROACH_GOVERNOR = '),
 # D05 shadow oracle (the stopping_shadow telemetry channel, its payload and StoppingShadowInput stay)
 (r'^selfdrive/controls/lib/stopping_shadow\.py$', 'drop_shadow', 'block',
  r'^from openpilot\.selfdrive\.controls\.lib\.stopping_profile_selector import|^from dataclasses import dataclass$|^STOPPING_SHADOW_VERSION|^SHADOW_[A-Z0-9_]+ = '
  r'|^class StoppingShadow(Input|Simulation|Decision|Oracle)\b|^def _(predict_next_accel|score_simulation|clip)\b'),
 (r'^selfdrive/controls/lib/longcontrol\.py$', 'drop_shadow', 'block', r'^  def _populate_pid_stopping_shadow_debug\b|^          self\._populate_pid_stopping_shadow_debug\('),
 (r'^selfdrive/controls/lib/longcontrol\.py$', 'drop_shadow', 'line', r'^  StoppingShadowInput,$'),
 # D06 shouldStop lookahead
 (r'^selfdrive/controls/lib/longitudinal_planner\.py$', 'drop_lookahead', 'block', r'^    if stopping_flags\.SHOULD_STOP_LOOKAHEAD_S > 0\.0:'),
 (r'^selfdrive/controls/lib/stopping_flags\.py$', 'drop_lookahead', 'block', r'^SHOULD_STOP_LOOKAHEAD_S = '),
 (r'^selfdrive/controls/tests/test_(longitudinal_planner|should_stop_hysteresis)\.py$', 'drop_lookahead', 'line', r'^  assert stopping_flags\.SHOULD_STOP_LOOKAHEAD_S == 0\.0$'),
 # D07 orphan params (SLCPriority3 is not orphan: the SLC priority menu writes SLCPriority%1 for 1..3, longitudinal_settings.cc)
 (r'^common/params_keys\.h$', 'drop_params', 'line', r'"(CENavigation|TrafficFollow|TrafficJerk[A-Za-z]*|TrafficPersonalityProfile)"'),
]


def feature_of_subject(s):
    for rx, f in RULES:
        if re.search(rx, s): return f
    return None
def split_feature(sha, path):
    for c, rx, f in SPLITS:
        if sha.startswith(c) and re.search(rx, path): return f
    return None
def path_feature(p):
    for rx, f in PATH_RULES:
        if re.search(rx, p): return f
    return None


def _py_index(text):
    """(statements as (start, end, node) with decorators and the comment lines directly above, parent map); None if no parse"""
    try: tree = ast.parse(text)
    except SyntaxError: return None
    lines = text.splitlines(); parent = {}; stmts = []
    for node in ast.walk(tree):
        for ch in ast.iter_child_nodes(node): parent[ch] = node
        if isinstance(node, ast.stmt):
            s = min([node.lineno] + [d.lineno for d in getattr(node, 'decorator_list', [])])
            ind = len(lines[s - 1]) - len(lines[s - 1].lstrip())
            while s > 1 and lines[s - 2].lstrip().startswith('#') and len(lines[s - 2]) - len(lines[s - 2].lstrip()) == ind: s -= 1
            stmts.append((s, node.end_lineno, node))
    return stmts, parent, lines
def _close_empty_bodies(sel, idx):
    """an if/for/while/with/try whose whole body is selected goes too, while it has no else (an else body stays)"""
    stmts, parent, lines = idx
    def covered(s, e): return all(k in sel or not lines[k - 1].strip() or lines[k - 1].lstrip().startswith('#') for k in range(s, e + 1))
    changed = True
    while changed:
        changed = False
        for s, e, node in stmts:
            if not isinstance(node, (ast.If, ast.For, ast.While, ast.With, ast.Try)) or getattr(node, 'orelse', None): continue
            if all(k in sel for k in range(s, e + 1)): continue
            if covered(node.body[0].lineno, node.body[-1].end_lineno):
                for k in range(s, e + 1): sel[k] = sel.get(node.body[0].lineno) or next(sel[j] for j in range(s, e + 1) if j in sel)
                changed = True
def line_overrides(path, data, side='new'):
    """{line number (1-based): feature key} for the lines LINE_RULES assign in this file version"""
    rules = [r for r in LINE_RULES if re.search(r[0], path) and (r[4] if len(r) > 4 else 'new') == side]
    if not rules: return {}
    text = data.decode('utf-8', 'replace'); lines = text.splitlines(); res = {}
    idx = _py_index(text) if path.endswith('.py') else None
    for _, f, kind, pat, *_ in rules:
        if kind == 'range':
            i = 0
            while i < len(lines):
                if re.search(pat[0], lines[i]):
                    j = i
                    while j < len(lines) - 1 and not re.search(pat[1], lines[j]): j += 1
                    for k in range(i, j + 1): res[k + 1] = f
                    i = j
                i += 1
            continue
        for i, l in enumerate(lines, 1):
            if not re.search(pat, l): continue
            span = (i, i)
            if kind in ('block', 'enclosing') and idx:
                if kind == 'block': cands = [(s, e) for s, e, n in idx[0] if n.lineno == i]
                else: cands = [(s, e) for s, e, n in idx[0] if n.lineno <= i <= n.end_lineno]
                if cands: span = max(cands, key=lambda x: x[1] - x[0]) if kind == 'block' else min(cands, key=lambda x: x[1] - x[0])
            for k in range(span[0], span[1] + 1): res[k] = f
    if idx: _close_empty_bodies(res, idx)
    return res


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
