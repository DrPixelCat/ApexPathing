"""Validate the scoped authorship audit without compiling Android dependencies."""
import hashlib
import json
import re
import subprocess

from audit_authorship import ROOT, OUT, code_only
from audit_json import read_json
from update_authorship_docs import offset


def main():
    inventory = read_json(OUT / 'git-inventory.json')
    credits = json.loads((OUT / 'credits.json').read_text(encoding='utf-8'))
    facts = {r['path']: r for r in inventory['files']}
    decisions = {r['path']: r for r in credits['files']}
    types = json.loads(subprocess.check_output(['java', str(ROOT / 'tools/AuthorshipDocs.java'), str(ROOT)],
                                             encoding='utf-8'))
    unchanged = 0
    modified = []
    for path, original in facts.items():
        raw = (ROOT / path).read_bytes()
        source = raw.decode('utf-8')
        assert '<p><b>Authorship:</b>' not in source, f'Authorship audit note remains: {path}'
        fingerprint = hashlib.sha256(re.sub(r'\s+', '', code_only(source)).encode()).hexdigest()
        assert fingerprint == original['code_sha256'], f'Non-comment source changed: {path}'
        if hashlib.sha256(raw).hexdigest() == original['source_sha256']:
            unchanged += 1
        else:
            modified.append(path)
    targeted = sum(r.get('apply_documentation', True) for r in credits['files'])
    for row in types:
        if not row['top_level']:
            continue
        path = row['path']
        source = (ROOT / path).read_bytes().decode('utf-8')
        doc = source[offset(source, row['doc_start']):offset(source, row['doc_end'])] if row['doc_start'] >= 0 else ''
        actual = re.findall(r'@author\s+([^\r\n*]+)', doc)
        assert actual == decisions[path]['authors'], f'JavaDoc/ledger author mismatch: {path}'
        original = facts[path]['existing_authors']
        if path.endswith(('/OTOS.java', '/Pinpoint.java')):
            original = original[:1]
        other_before = [n for n in original if not n.startswith(('Joel H', 'DrPixelCat'))]
        other_after = [n for n in actual if n != 'Joel H']
        if path.endswith('/geometry/Scalar.java'):
            assert other_after == []
        else:
            assert other_before == other_after, f'Unrelated author tags changed: {path}'
        assert all(not n.startswith('DrPixelCat') and not n.startswith('Joel H -') for n in actual)
    updater = json.loads(subprocess.check_output(['python', str(ROOT / 'tools/update_authorship_docs.py')],
                                               encoding='utf-8'))
    assert updater['files_modified'] == 0, updater
    whitespace = subprocess.run(['git', 'diff', '--check', '--', 'TeamCode/src/main/java',
                                 'FtcRobotController/src/main/java', 'README.md'], cwd=ROOT,
                                capture_output=True, text=True)
    assert whitespace.returncode == 0, whitespace.stdout
    previous = json.loads((OUT / 'validation.json').read_text(encoding='utf-8'))
    previous.update(dict(scope=credits['scope'], main_java_files=len(facts), named_types=len(types),
                         missing_type_javadoc=sum(r['doc_start'] < 0 for r in types),
                         javadoc_parse_errors=sum(len(r['doc_errors']) for r in types),
                         non_comment_source_fingerprints_unchanged=True,
                         diff_whitespace_check='passed', updater_idempotence='passed',
                         source_documentation_target_files=targeted,
                         source_files_exactly_matching_pre_task=unchanged,
                         source_files_different_from_pre_task=modified,
                         non_joel_tags_preserved_except_scalar=True,
                         newly_added_non_joel_author_tags=0,
                         authorship_audit_notes_in_java=0,
                         sdk_files_exactly_restored=all(p not in modified for p in facts if p.startswith('FtcRobotController/')),
                         personal_credit_aliases_remaining=0,
                         runtime_tests='Not run: documentation-only changes; Java parser, author tags, and non-comment source fingerprints checked.'))
    (OUT / 'validation.json').write_text(json.dumps(previous, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in previous.items() if k not in ('source_files_different_from_pre_task',)}))


if __name__ == '__main__':
    main()
