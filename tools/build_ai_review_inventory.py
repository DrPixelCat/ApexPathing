"""Build a human review queue from local Codex edit records and Java structure.

Reads only session metadata and tool calls from sessions for this repository.
Exports file references, never prompts, source patches, or unrelated history.
This is an evidence inventory, not an AI authorship detector or Java parser.
"""
import argparse
import collections
import datetime
import json
from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'reviews' / 'ai-class-review'

# Review prerequisites can outweigh size: a compact numerical solver still
# requires more background than a similarly sized builder or value object.
DOMAIN_FLOORS = {
    'localizers/util/KalmanFilter.java': (4, 'matrix prediction, covariance invariants, Joseph correction'),
    'localizers/util/AdaptiveKalmanFilter.java': (4, 'adaptive noise estimation and robust sensor weighting'),
    'localizers/util/PoseEstimator.java': (4, 'pose fusion, coordinate frames and covariance propagation'),
    'localizers/BaseLocalizer.java': (4, 'sensor/filter lifecycle, timing and calibration ownership'),
    'trajectory/profile/CoupledMotionProfile.java': (4, 'backward controllability and forward reachability assumptions'),
    'trajectory/profile/DynamicMotionProfile.java': (4, 'acceleration/braking envelopes and time integration'),
    'feedforward/MecanumWheelModel.java': (4, 'coupled wheel authority and body/wheel coordinate conversion'),
    'tuning/follower/sysid/SysIdRegression.java': (3, 'regression conditioning, excitation and fit quality'),
    'tuning/follower/characterization/MotorAuthorityRegression.java': (3, 'parameter identification and physical interpretation'),
    'tuning/follower/tracking/ProfiledTrackingOptimizer.java': (4, 'search objective, trial validity and confirmation ownership'),
}


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True,
                                   encoding='utf-8', errors='replace')


def expanded_text(payload):
    raw = payload.get('arguments', payload.get('input', ''))
    if not isinstance(raw, str):
        return json.dumps(raw)
    chunks = [raw]
    try:
        obj = json.loads(raw)
        if isinstance(obj, dict):
            chunks.extend(v for v in obj.values() if isinstance(v, str))
    except (ValueError, TypeError):
        pass
    # exec calls wrap patches/commands in JS string literals.
    for literal in re.findall(r'"(?:\\.|[^"\\])*"', raw):
        try:
            chunks.append(json.loads(literal))
        except ValueError:
            pass
    return '\n'.join(chunks)


def strip_java(source):
    # Preserve newline count while removing comments and string/char literals.
    return re.sub(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'',
                  lambda m: '\n' * m.group().count('\n'), source, flags=re.S)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--codex-home', type=Path,
                        default=Path.home() / '.codex')
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    paths = sorted((ROOT / 'TeamCode' / 'src').glob('**/*.java'))
    paths = [p for p in paths if '/main/java/' in p.as_posix()
             or '/test/java/' in p.as_posix()]
    paths = [p for p in paths if p.name != 'package-info.java']
    relative = {p.relative_to(ROOT).as_posix(): p for p in paths}
    by_name = collections.defaultdict(list)
    for path in relative:
        by_name[Path(path).name].append(path)

    def resolve(path):
        path = path.strip().replace('\\', '/').rstrip(';,\"\' ')
        start = path.find('TeamCode/')
        if start >= 0:
            path = path[start:]
        if path in relative:
            return path, False
        matches = by_name.get(Path(path).name, [])
        # Old package layout: only map unambiguous basenames.
        if len(matches) == 1:
            return matches[0], True
        return None, False

    titles = {}
    index = args.codex_home / 'session_index.jsonl'
    if index.exists():
        for line in index.open(encoding='utf-8'):
            try:
                row = json.loads(line)
                titles[row['id']] = row.get('thread_name', row['id'])
            except (ValueError, KeyError):
                continue
    evidence = collections.defaultdict(list)
    sessions = {}
    excluded = '01a1172a-7d15-7da1-a207-e9cd1f69ff2b'
    for directory in ('sessions', 'archived_sessions'):
        for session in (args.codex_home / directory).rglob('*.jsonl'):
            with session.open(encoding='utf-8', errors='replace') as stream:
                try:
                    meta = json.loads(next(stream)).get('payload', {})
                except (StopIteration, ValueError):
                    continue
                cwd = str(meta.get('cwd', '')).replace('\\', '/').lower().rstrip('/')
                if cwd != ROOT.as_posix().lower().rstrip('/'):
                    continue
                sid = meta.get('id', meta.get('session_id', session.stem))
                if sid == excluded:
                    continue
                sessions[sid] = {'id': sid, 'title': titles.get(sid, sid),
                                 'started': meta.get('timestamp', '')}
                pending = {}
                for line_number, line in enumerate(stream, 2):
                    try:
                        row = json.loads(line)
                    except ValueError:
                        continue
                    payload = row.get('payload', {})
                    kind = payload.get('type', '')
                    callid = payload.get('call_id')
                    if kind in ('function_call', 'custom_tool_call'):
                        text = expanded_text(payload)
                        candidates = []
                        for match in re.finditer(
                                r'^\*\*\* (Add|Update) File: ([^\r\n]+)\n(.*?)(?=^\*\*\* (?:Add|Update|Delete|End)|\Z)',
                                text, re.M | re.S):
                            operation, oldpath, body = match.groups()
                            changed = [line[1:].strip() for line in body.splitlines()
                                       if line.startswith(('+', '-')) and line[1:].strip()]
                            organization_only = (operation == 'Update' and bool(changed)
                                                 and all(line.startswith(('package ', 'import '))
                                                         for line in changed))
                            candidates.append((oldpath, 'patch-organization' if organization_only
                                               else 'patch-' + operation.lower()))
                        # Conservative shell evidence: literal Path assignments whose
                        # variable is subsequently written. No read-only paths accepted.
                        for match in re.finditer(
                                r'(\w+)\s*=\s*Path\(\s*r?[\"\']([^\"\']+\.java)[\"\']\s*\)', text):
                            variable, oldpath = match.groups()
                            tail = text[match.end():]
                            next_assignment = re.search(r'\b' + re.escape(variable) + r'\s*=\s*Path\(', tail)
                            if next_assignment:
                                tail = tail[:next_assignment.start()]
                            if re.search(r'\b' + re.escape(variable) + r'\.write_(?:text|bytes)\(', tail):
                                candidates.append((oldpath, 'write-command'))
                        for oldpath in re.findall(
                                r'Path\(\s*r?[\"\']([^\"\']+\.java)[\"\']\s*\)\.write_(?:text|bytes)\(', text):
                            candidates.append((oldpath, 'write-command'))
                        if candidates:
                            pending[callid] = (candidates, row.get('timestamp', ''), line_number)
                    elif kind in ('function_call_output', 'custom_tool_call_output') and callid in pending:
                        candidates, timestamp, call_line = pending.pop(callid)
                        output = str(payload.get('output', ''))
                        for oldpath, operation in set(candidates):
                            path, mapped = resolve(oldpath)
                            if not path:
                                continue
                            # Wrapped output can be truncated or contain several calls;
                            # keep ambiguous results separate from confirmed success.
                            if operation.startswith('patch'):
                                success = ('Success. Updated the following files' in output
                                           or '"success": true' in output)
                            else:
                                success = ('"exit_code":0' in output.replace(' ', '')
                                           or '"exit_code": 0' in output
                                           or 'Process exited with code 0' in output)
                            evidence[path].append({'session': sid, 'timestamp': timestamp,
                                                   'line': call_line, 'operation': operation,
                                                   'success_seen': success,
                                                   'historical_path': oldpath.replace('\\', '/'),
                                                   'mapped_by_basename': mapped})

    recent = {}
    for line in git('log', '--since=2026-08-01', '--format=COMMIT %h %as %s',
                    '--name-only', '--', 'TeamCode/src').splitlines():
        if line.startswith('COMMIT '):
            commit = line[7:]
        elif line in relative and line not in recent:
            recent[line] = commit
    dirty = git('status', '--porcelain', '--', 'TeamCode/src')
    dirty_paths = {line[3:].replace('\\', '/') for line in dirty.splitlines()}
    rows = []
    for path, file in relative.items():
        source = file.read_text(encoding='utf-8-sig')
        code = strip_java(source)
        loc = sum(bool(line.strip()) for line in code.splitlines())
        branches = len(re.findall(r'\b(?:if|for|while|case|catch)\b|&&|\|\||\?', code))
        imports = re.findall(r'^import\s+(?:static\s+)?([\w.]+)', source, re.M)
        internal = [i for i in imports if i.split('.')[0] in {
            'geometry', 'controllers', 'drivetrains', 'feedforward', 'follower',
            'localizers', 'paths', 'storage', 'trajectory', 'tuning'}]
        score = round(loc + 3 * branches + 5 * len(internal))
        tier = 1 if score < 100 else 2 if score < 250 else 3 if score < 550 else 4 if score < 1100 else 5
        minimum, prerequisite = DOMAIN_FLOORS.get(path.split('/java/', 1)[1], (1, ''))
        tier = max(tier, minimum)
        ev = sorted(evidence[path], key=lambda e: e['timestamp'])
        confirmed = [e for e in ev if e['success_seen']]
        status = ('Codex edit recorded' if confirmed else 'Write call recorded' if ev
                  else 'Recent Git change only' if path in recent else 'Unknown')
        added = any(e['operation'] == 'patch-add' for e in ev)
        substantive = any(e['operation'] != 'patch-organization' for e in ev)
        involvement = ('Creation call recorded' if added else 'Modification call recorded' if substantive
                       else 'Package/import edits only' if ev else 'Not established')
        rows.append({'path': path, 'name': file.stem, 'involvement': involvement,
                     'scope': 'production' if '/main/' in path else 'test',
                     'kind': 'interface' if re.search(r'\binterface\s+' + file.stem + r'\b', code)
                     else 'enum' if re.search(r'\benum\s+' + file.stem + r'\b', code) else 'class',
                     'loc': loc, 'branches': branches, 'project_imports': len(internal),
                     'score': score, 'tier': tier, 'evidence_status': status,
                     'domain_prerequisite': prerequisite,
                     'uncommitted': path in dirty_paths, 'recent_commit': recent.get(path),
                     'evidence': ev})
    rows.sort(key=lambda r: (r['tier'], r['score'], r['path']))
    previous = OUT / 'REVIEW_QUEUE.md'
    checked = set()
    if previous.exists():
        checked = set(re.findall(r'^- \[x\] \*\*[^\n]*?\(\.\./\.\./(TeamCode/[^)]+)\)',
                                 previous.read_text(encoding='utf-8'), re.M))
    data = {'generated': datetime.date.today().isoformat(), 'head': git('rev-parse', '--short', 'HEAD').strip(),
            'session_count': len(sessions), 'sessions': sessions, 'files': rows}
    (OUT / 'inventory.json').write_text(json.dumps(data, indent=2) + '\n', encoding='utf-8')
    names = {1: 'Small types and helpers', 2: 'Focused logic', 3: 'Several interacting responsibilities',
             4: 'Stateful algorithms and workflows', 5: 'Largest review effort'}
    production = [r for r in rows if r['scope'] == 'production']
    candidates = [r for r in production if r['evidence'] and r['involvement'] != 'Package/import edits only']
    tests = [r for r in rows if r['scope'] == 'test']
    lines = ['# AI-assisted class review queue', '',
             f"Snapshot: {data['generated']}; HEAD `{data['head']}`; current uncommitted source included.", '',
             'The best identifier is provenance: a recorded AI edit, followed by a human review status. '
             'Code style and the Git author are not reliable authorship tests. '
             'A Codex edit proves AI involvement in a file, not that AI originally wrote the entire class. '
             '**Creation call recorded** means an Add File patch exists; the file may have been recreated or moved from older code. '
             '**Modification call recorded** means edits were recorded without creation evidence. '
             'Package/import-only edits are separated into the coverage queue.', '',
             f"Scanned {len(sessions)} available local Codex sessions for this exact repository, "
             f"{len(production)} production Java types/files and {len(tests)} test Java files. "
             'FTC SDK/sample files and package-info.java are excluded. Nested types are reviewed with their enclosing file.', '',
             'Evidence labels:', '',
             '- **Codex edit recorded**: a patch or literal Python write appears in a tool call, and its returned output has a success marker. '
             'A batched command success marker may belong to another command; inspect the cited call when exact provenance matters.',
             '- **Write call recorded**: a concrete patch/write call exists, but success could not be established from the saved output.',
             '- **Recent Git change only**: changed since 2026-08-01; a coverage safety net, not proof of AI involvement.',
             '- **Unknown**: no captured write evidence or recent Git change. This does not mean human-written.', '',
             'Ranking is approximate reading effort, using comment-free nonblank lines (L), branch/operator occurrences (B), '
             'and project imports (I): score = L + 3B + 5I. Thresholds: <100, <250, <550, <1100, then 1100+. '
             'B is a lexical indicator, not cyclomatic complexity. Compact numerical/state-estimation classes have explicit '
             'minimum tiers for the mathematical background they require, noted on their entries. '
             'Ranks are ascending by score within each tier; no dependency ordering is implied.', '',
             'For each file: explain its purpose in your own words; comment units, coordinate frames, invariants, '
             'and non-obvious decisions; remove duplication/dead compatibility code only after understanding callers; '
             'read the relevant tests and run them when behavior changes. Tick the box once you have reviewed, commented, '
             'and cleaned it to your satisfaction. A checked box records human review, never authorship.', '',
             '## Primary queue: recorded Codex involvement', '',
             f'{len(candidates)} production files have recorded writes beyond package/import-only patches. Start here. '
             'An asterisk after the evidence label means the file has uncommitted changes in this snapshot.', '']

    def emit(group):
        rank = 0
        for tier in range(1, 6):
            subset = [r for r in group if r['tier'] == tier]
            if not subset:
                continue
            lines.extend([f"### {tier}. {names[tier]}", ''])
            for row in subset:
                rank += 1
                path = row['path']
                ev = row['evidence']
                selected = next((e for e in ev if e['success_seen']), ev[0] if ev else None)
                label = row['involvement'] + '; ' + row['evidence_status'] + (' *' if row['uncommitted'] else '')
                detail = (f"session `{selected['session']}`, {selected['timestamp'][:10]}, "
                          f"{selected['operation']}, JSONL line {selected['line']}"
                          if selected else f"Git `{row['recent_commit']}`" if row['recent_commit'] else 'no captured provenance')
                mapped = '; historical package mapped by unique filename' if selected and selected['mapped_by_basename'] else ''
                target = '../../' + path
                box = 'x' if path in checked else ' '
                lines.append(f"- [{box}] **{rank}. [{path.split('/java/', 1)[1]}]({target})** "
                             f"— {label}; score {row['score']} (L={row['loc']}, B={row['branches']}, I={row['project_imports']}).")
                lines.append(f"  Evidence: {detail}{mapped}.")
                if row['domain_prerequisite']:
                    lines.append(f"  Review focus: {row['domain_prerequisite']} (domain prerequisite influences tier).")
            lines.append('')

    emit(candidates)
    lines.extend(['## Coverage queue: provenance not established', '',
                  'Review these too if you want full ownership of the library. Recent changes deserve attention, '
                  'but are not classified as AI-generated. Sorted with the same simple-to-complex metric.', ''])
    emit([r for r in production if r not in candidates])
    lines.extend(['## Supporting tests and simulator code', '',
                  'Review the tests alongside the production class they exercise. This separate queue prevents '
                  'small test classes from crowding out production code; simulation infrastructure can itself be complex.', ''])
    emit(tests)
    lines.extend(['## Keeping the list useful', '',
                  'For future AI changes, record the affected file paths and the chat/commit reference in a review ledger. '
                  'Reopen the review checkbox for substantive edits after human review. Prefer file-level tracking '
                  'over an `AI-generated` comment in every class, because most files have mixed authorship.', '',
                  'Regenerate from the repository root with `python tools/build_ai_review_inventory.py`. '
                  'Existing checked boxes are preserved by file path; substantive new edits require you to clear them. '
                  'The scanner recognizes patches and common literal Python writes, not every possible shell/editor operation. '
                  'Missing, cloud-only, other-machine, or differently located sessions and non-Codex AI edits can be missed. '
                  'Basename mapping bridges old package moves only when unambiguous. Evidence contains references only; '
                  'the underlying private chat logs are not copied into this repository.', ''])
    # Checked status parser uses repository paths; links are relative for portability.
    markdown = '\n'.join(lines)
    previous.write_text(markdown, encoding='utf-8')
    counts = collections.Counter(r['evidence_status'] for r in production)
    print(json.dumps({'sessions': len(sessions), 'production': len(production), 'tests': len(tests),
                      'production_evidence': counts, 'tier_counts': dict(collections.Counter(r['tier'] for r in candidates)),
                      'output': str(previous)}, indent=2))


if __name__ == '__main__':
    main()
