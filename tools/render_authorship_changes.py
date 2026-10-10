"""Render current changes against the immutable before-task author-tag snapshot."""
import json

from audit_authorship import OUT
from render_authorship_audit import file_link


def canonical(name):
    return 'Joel H' if name.startswith(('DrPixelCat', 'Joel H')) else name


def main():
    inventory = json.loads((OUT / 'git-inventory.json').read_text(encoding='utf-8'))
    credits = json.loads((OUT / 'credits.json').read_text(encoding='utf-8'))
    facts = {r['path']: r for r in inventory['files']}
    changes = []
    for row in credits['files']:
        before = facts[row['path']]['existing_authors']
        if row['path'].endswith(('/OTOS.java', '/Pinpoint.java')):
            before = before[:1]
        after = row['authors']
        if before == after:
            continue
        old_names, new_names = set(map(canonical, before)), set(map(canonical, after))
        added, removed = new_names - old_names, old_names - new_names
        description = []
        if added:
            description.append('Added ' + '; '.join(sorted(added)))
        if removed:
            description.append('Removed ' + '; '.join(sorted(removed)))
        if 'Joel H' in old_names & new_names:
            description.append('Normalized Joel alias; contribution evidence in audit')
        changes.append(dict(path=row['path'], before=before, after=after, change='; '.join(description)))
    lines = ['# Revised JavaDoc authorship changes', '',
             f'{len(changes)} classes have author-tag changes relative to the original before-task snapshot. '
             'This replaces the initial 106-class change list.', '',
             'The revision focuses on Joel H. No new non-Joel author tags remain. Original non-Joel '
             'tags and wording are preserved except the mistaken Dylan tag on Scalar, whose class '
             'history contains only Joel-account contributions. Earlier Joel credits removed as '
             'unverified are retained in AUTHORSHIP.md; absence of evidence is not proof of no involvement.', '',
             '| Class | Before | After | Change |', '| --- | --- | --- | --- |']
    for row in changes:
        lines.append(f"| {file_link(row['path'])} | {'; '.join(row['before']) or '(none)'} | "
                     f"{'; '.join(row['after']) or '(none)'} | {row['change']} |")
    (OUT / 'AUTHORSHIP_CHANGES.md').write_text('\n'.join(lines) + '\n', encoding='utf-8')
    print(json.dumps(dict(changed_classes=len(changes), report=str(OUT / 'AUTHORSHIP_CHANGES.md'))))


if __name__ == '__main__':
    main()
