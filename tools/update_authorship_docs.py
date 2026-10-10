"""Apply curated author tags to class JavaDoc, without changing Java tokens.

Credit decisions are reviewed data, never inferred from a Git account or AI style.
Requires a JDK with the compiler API. Run after collecting the Git inventory.
"""
import json
import re
import subprocess
import textwrap

from audit_authorship import ROOT, OUT, code_only


def offset(source, utf16_offset):
    return len(source.encode('utf-16-le')[:utf16_offset * 2].decode('utf-16-le'))


def document(old, indent, paragraph, authors, newline, summary):
    content = old[3:-2] if old else summary
    content = re.sub(r'(?m)^[ \t]*\* ?', '', content)
    lines = content.strip().splitlines()
    kept = []
    removing = False
    for line in lines:
        if '<p><b>Authorship:</b>' in line:
            removing = True
        elif removing and (not line.strip() or line.lstrip().startswith('@')):
            removing = False
        if not removing and not line.lstrip().startswith('@author'):
            kept.append(line.rstrip())
    while kept and not kept[-1].strip():
        kept.pop()
    insertion = next((i for i, line in enumerate(kept) if line.lstrip().startswith('@')), len(kept))
    description, tags = kept[:insertion], kept[insertion:]
    while description and not description[-1].strip():
        description.pop()
    body = description
    if paragraph is not None:
        body += [''] + textwrap.wrap('<p><b>Authorship:</b> ' + paragraph, width=94 - len(indent))
    if tags or authors:
        body += [''] + tags + ['@author ' + name for name in authors]
    return '/**' + newline + newline.join(indent + ' *' + (' ' + line if line else '') for line in body) + newline + indent + ' */'


def main():
    credits = json.loads((OUT / 'credits.json').read_text(encoding='utf-8'))
    decisions = {r['path']: r for r in credits['files']}
    result = subprocess.run(['java', str(ROOT / 'tools/AuthorshipDocs.java'), str(ROOT)],
                            capture_output=True, text=True, encoding='utf-8', check=True)
    types = json.loads(result.stdout)
    by_file = {}
    for row in types:
        by_file.setdefault(row['path'], []).append(row)
    modified = []
    for path, declarations in by_file.items():
        decision = decisions[path]
        if not decision.get('apply_documentation', True):
            continue
        file = ROOT / path
        raw = file.read_bytes()
        source = raw.decode('utf-8')
        newline = '\r\n' if '\r\n' in source else '\n'
        before = re.sub(r'\s+', '', code_only(source))
        edits = []
        for row in declarations:
            if not row['top_level'] and not credits.get('document_nested', True):
                continue
            start = offset(source, row['start'])
            ds = offset(source, row['doc_start']) if row['doc_start'] >= 0 else -1
            de = offset(source, row['doc_end']) if row['doc_end'] >= 0 else -1
            old = source[ds:de] if ds >= 0 else ''
            position = ds if ds >= 0 else start
            line_start = source.rfind('\n', 0, position) + 1
            indent = source[line_start:position]
            if indent.strip():
                raise ValueError(f'Unexpected inline declaration/comment: {path} {row["name"]}')
            authors = decision['authors'] if row['top_level'] else re.findall(r'@author\s+([^\r\n*]+)', old)
            paragraph = None
            if credits.get('emit_authorship_notes', False):
                paragraph = decision['javadoc'] if row['top_level'] else decision.get('nested', {}).get(row['name'])
            summary = '{@code ' + row['name'].split('.')[-1] + '} ' + ('type.' if row['top_level'] else 'implementation detail.')
            replacement = document(old, indent, paragraph, authors, newline, summary)
            if ds >= 0:
                edits.append((ds, de, replacement))
            else:
                edits.append((start, start, replacement + newline + indent))
        updated = source
        for start, end, replacement in sorted(edits, reverse=True):
            updated = updated[:start] + replacement + updated[end:]
        if re.sub(r'\s+', '', code_only(updated)) != before:
            raise ValueError(f'Executable Java changed: {path}')
        if updated != source:
            file.write_bytes(updated.encode('utf-8'))
            modified.append(path)
    print(json.dumps(dict(files_modified=len(modified), named_types=len(types))))


if __name__ == '__main__':
    main()
