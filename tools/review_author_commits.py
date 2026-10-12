"""Collect commit-based contribution evidence without using JavaDoc author tags.

Uses normal Git rename detection, ignores merges and comment/format/package/import
changes, and records substantive changes separately from commit-account ownership.
This is a factual aid for a human credit decision, not an authorship classifier.
"""
import concurrent.futures
import json
import re

from audit_authorship import OUT, code_only, git
from audit_json import read_json, write_snapshot


def canonical_account(name, email):
    account = (name + ' ' + email).lower()
    if any(alias in account for alias in ('joelyboi', 'pixelcat', 'pixel cat', 'joel.a.holder')):
        return 'Joel H'
    for aliases, person in [
        (('dyldev', 'dylanb92010', 'dylan b.'), 'Dylan B.'),
        (('sohum', 'paradoxical-duck'), 'Sohum Arora'),
        (('xhaemel', 'xander'), 'Xander Haemel'),
        (('agurnani', 'atharvgurnani'), 'Atharv G.'),
        (('moosecoding',), 'Topher F.'),
    ]:
        if any(alias in account for alias in aliases):
            return person
    return name


def substantive(source):
    source = code_only(source)
    source = re.sub(r'(?m)^\s*(?:package|import)\s+[^;]+;', '', source)
    return re.sub(r'\s+', '', source)


def inspect(path):
    output = git('log', '--follow', '-M50%', '--no-merges', '--no-ext-diff',
                 '--format=@@@%H\t%an\t%ae\t%as\t%s', '--unified=1000000',
                 '-p', '--', path)
    commits = []
    for block in output.split('@@@')[1:]:
        header, _, patch = block.partition('\n')
        commit, name, email, date, subject = header.split('\t', 4)
        before, after = [], []
        old_path = new_path = None
        in_hunk = False
        for line in patch.splitlines():
            if line.startswith('--- '):
                old_path = line[4:].removeprefix('a/')
            elif line.startswith('+++ '):
                new_path = line[4:].removeprefix('b/')
            elif line.startswith('@@ '):
                in_hunk = True
            elif in_hunk:
                if line.startswith((' ', '-')):
                    before.append(line[1:])
                if line.startswith((' ', '+')):
                    after.append(line[1:])
        before, after = '\n'.join(before), '\n'.join(after)
        commits.append(dict(commit=commit, name=name, email=email, date=date,
                            account=canonical_account(name, email), subject=subject,
                            old_path=old_path, new_path=new_path,
                            substantive_change=substantive(before) != substantive(after),
                            types_before=re.findall(r'\b(?:class|interface|enum|record)\s+(\w+)', code_only(before)),
                            types_after=re.findall(r'\b(?:class|interface|enum|record)\s+(\w+)', code_only(after))))
    return dict(path=path, commits=commits,
                substantive_accounts=sorted({c['account'] for c in commits if c['substantive_change']}))


def main():
    inventory = read_json(OUT / 'git-inventory.json')
    paths = [r['path'] for r in inventory['files'] if r['path'].startswith('TeamCode/')]
    with concurrent.futures.ThreadPoolExecutor(max_workers=6) as pool:
        rows = list(pool.map(inspect, paths))
    target = write_snapshot(OUT / 'git-contributions.json', dict(head=git('rev-parse', 'HEAD').strip(),
                                     method='Normal 50% rename detection; no merges; comments, whitespace, package and imports ignored.',
                                     limitations='Git accounts identify committed contributions, not who typed code. Copy/predecessor attribution requires separate evidence.',
                                     files=rows))
    print(json.dumps(dict(files=len(rows), output=str(target))))


if __name__ == '__main__':
    main()
