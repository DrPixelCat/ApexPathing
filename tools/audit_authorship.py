"""Collect reviewable Git provenance; commit identities are not authorship verdicts.

Run from any directory with ``python tools/audit_authorship.py``. This reads the
working tree and local Git history, but never edits Java or private session logs.
The resulting inventory deliberately separates facts from curated credit decisions.
"""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import re
import subprocess


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'reviews' / 'authorship'


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, encoding='utf-8',
                                   errors='replace', stderr=subprocess.PIPE)


def code_only(source):
    # Keep strings intact: comment-like text inside a string is executable data.
    return re.sub(r'""".*?"""|"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|/\*.*?\*/|//[^\r\n]*',
                  lambda m: '' if m.group().startswith(('/*', '//')) else m.group(),
                  source, flags=re.S)


def inspect(path):
    relative = path.relative_to(ROOT).as_posix()
    raw = path.read_bytes()
    source = raw.decode('utf-8-sig')
    history = []
    for block in git('log', '--follow', '-M20%', '--format=@@@%H\t%as\t%an\t%ae\t%s',
                     '--name-status', '--', relative).split('@@@')[1:]:
        header, _, changes = block.partition('\n')
        commit, date, name, email, subject = header.split('\t', 4)
        history.append(dict(commit=commit, date=date, name=name, email=email,
                            subject=subject, changes=changes.strip().splitlines()))
    original = ''
    origin_path = relative
    if history:
        first = history[-1]
        if first['changes']:
            origin_path = first['changes'][-1].split('\t')[-1]
        original = git('show', first['commit'] + ':' + origin_path)
    tag_history = {}
    current_commit = ''
    for line in git('log', '--follow', '-M20%', '--format=@@@%h', '--unified=0',
                    '-G', '@author', '-p', '--', relative).splitlines():
        if line.startswith('@@@'):
            current_commit = line[3:]
        match = re.search(r'^[+-]\s*\*?\s*@author\s+([^*]+)', line)
        if match:
            tag_history.setdefault(match.group(1).strip(), []).append(current_commit)
    clean = code_only(source)
    declarations = re.findall(r'\b(?:class|interface|enum|record)\s+(\w+)', clean)
    return dict(path=relative, declarations=declarations,
                original_path=origin_path,
                original_authors=re.findall(r'@author\s+([^\r\n*]+)', original),
                existing_authors=re.findall(r'@author\s+([^\r\n*]+)', source),
                history=history,
                original_declarations=re.findall(r'\b(?:class|interface|enum|record)\s+(\w+)', code_only(original)),
                historical_author_tags=tag_history,
                source_sha256=hashlib.sha256(raw).hexdigest(),
                code_sha256=hashlib.sha256(re.sub(r'\s+', '', clean).encode()).hexdigest())


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    paths = sorted(p for module in ('TeamCode', 'FtcRobotController')
                   for p in (ROOT / module / 'src/main/java').rglob('*.java')
                   if p.name != 'package-info.java')
    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
        rows = list(pool.map(inspect, paths))
    ai_path = ROOT / 'reviews/ai-class-review/inventory.json'
    ai = json.loads(ai_path.read_text(encoding='utf-8')) if ai_path.exists() else {}
    by_path = {r['path']: r for r in ai.get('files', [])}
    for row in rows:
        row['recorded_ai_edits'] = by_path.get(row['path'], {}).get('evidence', [])
    data = dict(head=git('rev-parse', 'HEAD').strip(),
                shallow=git('rev-parse', '--is-shallow-repository').strip(),
                ai_inventory_snapshot=ai.get('generated'), files=rows)
    (OUT / 'git-inventory.json').write_text(json.dumps(data, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(dict(files=len(rows), teamcode=sum(r['path'].startswith('TeamCode/') for r in rows),
                          output=str(OUT))))


if __name__ == '__main__':
    main()
