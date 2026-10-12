"""Render the reviewed credit ledger; never infer credits or edit source code."""
import json

from audit_authorship import ROOT, OUT
from audit_json import read_json


def file_link(path):
    return f'[{path.split("/java/", 1)[-1]}](../../{path})'


def commit_link(commit):
    return f'[`{commit[:8]}`](https://github.com/DrPixelCat/ApexPathing/commit/{commit})'


def guidance_ref(evidence):
    return f"session `{evidence['session']}`, user-message lines " + ', '.join(map(str, evidence['user_message_lines']))


def main():
    credits = json.loads((OUT / 'credits.json').read_text(encoding='utf-8'))
    inventory = read_json(OUT / 'git-inventory.json')
    facts = {r['path']: r for r in inventory['files']}
    contributions = read_json(OUT / 'git-contributions.json')
    commit_facts = {r['path']: r for r in contributions['files']}
    rows = credits['files']
    project = [r for r in rows if r['path'].startswith('TeamCode/')]
    sdk = [r for r in rows if r['path'].startswith('FtcRobotController/')]
    uncertain = [r for r in project if r['joel_credit_unverified']]
    lines = [
        '# Main-source authorship audit', '',
        f"Reviewed {credits['snapshot']}; HEAD `{credits['head'][:8]}`; includes the pre-existing working-tree changes.", '',
        f"Covers {len(project)} Apex Java source files and {len(sdk)} bundled FTC SDK/sample files. "
        'The inventory includes 356 named types. Author-tag changes are limited to the '
        '69 classes relevant to Joel H credit. Added SDK, unrelated-class, and nested-type provenance '
        'notes from the initial audit were withdrawn. ', '',
        '## How to read the credits', '',
        '`@author` credits describe a documented contribution, including an earlier implementation or '
        'a specifically stated design requirement. They do not assert that a person typed every line. '
        'Joel H is credited for the scoped guidance documented in this audit. Class JavaDoc contains '
        'the decided author tags without audit commentary. AI implementation '
        'details are not assigned to Joel H merely because a commit used his account.', '',
        'Git commits are the primary evidence for contributions by people other than Joel. A tag '
        'introduced in Joel\'s commit does not establish another person\'s contribution. The revised '
        'review separates substantive code changes from comment, whitespace, package, and import '
        'changes. A human account does not prove human-written implementation. Recorded Codex changes prove involvement '
        'in a file, not wholesale generation. A creation call can be a split, copy, or recreation. '
        'No percentages of human versus AI authorship can be established from these records.', '',
        'No new non-Joel author tags are added. Other people\'s original tag wording is preserved, '
        'except the mistaken Dylan tag on Scalar. The SparkFun OTOS and Ethan Doak/goBILDA driver credits are retained '
        'on their actual nested drivers. Copyright holders in SDK notices are not automatically '
        'converted into individual authors.', '',
        '## Evidence and method', '',
        '1. Inventoried every Java file under both main source trees and parsed named types with the JDK compiler API.',
        '2. Collected the original history/tag inventory, then separately reviewed normal 50% rename '
        'history without merges and compared substantive code changes. The latter is recorded in git-contributions.json.gz.',
        '3. Checked discontinuities and original non-Joel tags. Predecessor/tag lists are retained '
        'as historical leads in the inventory; they are not automatically promoted to author tags on '
        'rewritten or extracted classes. An earliest followed commit is not necessarily class creation.',
        '4. Cross-referenced the existing AI-edit inventory with direct user messages in repository-specific '
        'local sessions. Approval-review transcripts and assistant-authored suggestions were not treated as '
        'independent user design statements. Approved plans establish requested behavior, not sole invention '
        'of their algorithms.',
        '5. Applied Joel-specific author tags, removed authorship commentary from JavaDoc as requested, '
        'restored unrelated documentation, and checked source fingerprints and author tags with the Java parser.', '',
        'Aliases DrPixelCat, Dr Pixel Cat, DrPixelCat24, JoelyBoi, Joel H / 7842a, and the user-confirmed '
        '7842 variants, and the draft author Joel Holder resolve to **Joel H**. Other aliases are resolved by matching historical credit '
        'and Git identity evidence: Dylan B./DylDev; Sohum Arora/sohum-arora/paradoxical-duck; '
        'Xander Haemel/Xhaemel/Xander-r343; Topher F./MooseCoding; Atharv G./agurnani.', '',
        'The existing AI inventory is dated 2026-10-07 and contains references to 271 local sessions. '
        'Relevant later sessions were also inspected, including the creation of TranslationAxis and DeltaTime. '
        'This is not a complete record of every AI tool, machine, private conversation, or earlier prompt. '
        'Successful batched calls may have ambiguous per-operation success; the JSON retains that distinction. '
        'Session IDs and line references permit local verification; private chat transcripts are not exported.', '',
        '## Correction to the initial audit', '',
        'The user clarified that the review should establish Joel H\'s contribution and leave other '
        'people\'s authorship alone. Initial non-Joel additions in 35 files were withdrawn, along with '
        'team/name normalization and generic predecessor credits. Existing non-Joel tags retain their '
        'original spelling and ordering. A shared predecessor does not automatically confer authorship '
        'of a new helper class.', '',
        'Scalar is the one existing tag corrected under the user\'s explicit rule. It was added in '
        'commit `812e284c` by JoelyBoi and subsequently changed only through that account. Its Dylan '
        'tag did not have independent class-contribution evidence. That tag was removed; Joel H\'s '
        'explicit unitless-Scalar API guidance remains credited.', '',
        'Four preserved non-Joel tags do not have a direct account match in normal followed class '
        'history: Sohum Arora in ApexConstants, Xander Haemel in CoaxialSwerve and Follower, and '
        'Atharv G. in Motor. They were introduced/carried by Dylan\'s commits, rather than Joel-only '
        'new-class commits, and may describe earlier source lineage. They remain unchanged because '
        'the user asked to leave unrelated credits alone; this audit does not newly certify their scope.', '',
        'Wolfpack Machina remains acknowledged as PDS-controller inspiration. Road Runner contributors '
        'are acknowledged for the feedforward/braking and ramp-tuning inspiration recorded in '
        '[the braking implementation note](../../doc/ProfiledBrakingImplementation.md) and '
        '[the SysId source comparison](../../doc/SysIdRoadRunnerComparison.md). WPILib SysId is '
        'credited as workflow inspiration, not as the author of Apex regression code.', '',
        '## Profile-generation draft as design evidence', '',
        'The locally available draft **Controlling Mobile Robots at Full Normalized Utilization on Parametric '
        'Trajectories** names Joel Holder as author (page 1). Its declaration on page 16 reports Gemini '
        'assistance with explanations, wording, and research, and attributes its core concepts and algorithmic '
        'designs to the author. This is affirmative evidence for scoped design credit; it does not prove '
        'that Joel H hand-wrote the Java implementation or establish mathematical novelty.', '',
        'The file was supplied as design guidance in session `019f2428-2020-7842-902c-743f4bd95173`, '
        'user-message lines 6 and 16. The local copy was inspected for this audit; its exact historical '
        'revision at the time of the original chat is not established. The source filename, hash, page '
        'references, and attribution are recorded in `design-sources.json`. Formatting/editing reviewers '
        'listed in the paper are not automatically treated as authors of the Java algorithms.', '',
        '| Class/file | Design credited to Joel H | Draft pages |',
        '| --- | --- | --- |',
    ]
    for r in project:
        for d in r.get('design_sources', []):
            lines.append(f"| {file_link(r['path'])} | {d['role']} | {', '.join(map(str, d['pages']))} |")
    lines += [
        '', '## Older Joel H credits requiring personal confirmation', '',
        f'{len(uncertain)} files previously credited Joel H, but the available records do not establish '
        'a sufficiently specific role. Those credits are preserved here and in credits.json; the '
        'JavaDoc does not assert personal authorship until the role is established. Missing evidence '
        'does not prove that Joel H did not design or write these classes.', '',
        '| Class/file | Historical evidence | Unresolved role |',
        '| --- | --- | --- |',
    ]
    for r in uncertain:
        f = facts[r['path']]
        anchors = ', '.join(commit_link(c) for c in r['evidence_commits']) or 'Working tree only'
        lines.append(f"| {file_link(r['path'])} | {anchors} | Original implementation, explicit AI guidance, and/or design contribution not distinguishable. |")
    lines += ['', 'Confirm a concrete contribution, such as the original API design, an algorithm, '
              'a specific implementation, or explicit instructions given to an AI. Restore `@author Joel H` '
              'with that scope; review or committing code alone should not become implementation credit.', '',
              '## Apex file-by-file ledger', '',
              '“AI recorded” means a non-package/import write call was recorded, including potentially '
              'documentation edits. “Not established” is not a human-written verdict. The full histories '
              'and success flags are in git-inventory.json.gz and credits.json.', '',
              '| File | Preserved tags and Joel scope | Commit contribution evidence | AI evidence |',
              '| --- | --- | --- | --- |']
    for r in project:
        f = facts[r['path']]
        earlier = [a for a in r['authors'] if a != 'Joel H']
        scope = 'Original non-Joel tags: ' + '; '.join(earlier) + '. ' if earlier else ''
        if r.get('design_sources'):
            scope += 'Joel H: supplied draft design for ' + r['design_sources'][0]['role'] + '.'
        elif r['joel_guidance']:
            scope += 'Joel H: ' + r['joel_guidance'][0]['role'] + '.'
        if not scope:
            scope = 'Individual human implementation author unverified.'
        scope = scope.replace('B..', 'B.')
        reviewed = commit_facts.get(r['path'], {})
        first = reviewed.get('commits', [None])[-1] if reviewed.get('commits') else None
        origin = f"{commit_link(first['commit'])} ({first['date']}); substantive accounts: " + ', '.join(reviewed['substantive_accounts']) if first else 'Uncommitted new file'
        ai = 'Recorded: ' + str(len(r['ai_evidence'])) + ' calls' if r['ai_evidence'] else 'Not established'
        lines.append(f"| {file_link(r['path'])} | {scope} | {origin} | {ai} |")
    lines += ['', '## Direct evidence for scoped Joel H credits', '']
    for r in project:
        if not r['joel_guidance']:
            continue
        lines.append('### ' + r['path'].split('/java/')[1])
        lines.append('')
        for g in r['joel_guidance']:
            lines.append('- Joel H ' + g['role'] + '; ' + guidance_ref(g) + '.')
        lines.append('')
    lines += ['## Bundled FTC SDK and sample provenance', '',
              'These sources came with the SDK; the repository importer is not credited as their author. '
              'The original legal notices remain intact. Notice names below identify copyright provenance '
              'and must not be read as a verified list of individual implementation authors.', '',
              '| File | Retained upstream notices |', '| --- | --- |']
    for r in sdk:
        notices = '; '.join(r['upstream_notices']) or 'No individual copyright/author notice in this file; FTC SDK origin retained.'
        lines.append(f"| {file_link(r['path'])} | {notices} |")
    lines += ['', '## Remaining provenance gaps', '',
              '- Exact hand-written versus AI-produced portions cannot be reconstructed for every class. '
              'The ledger leaves these unknown rather than guessing from code style or commit size.',
              '- Octoquad.Driver resembles the Digital Chicken Labs driver family, but an exact donor revision '
              'and copied portions were not established. This remains an upstream-provenance question. '
              'The manufacturer’s [source repository](https://github.com/DigitalChickenLabs/OctoQuad) '
              'establishes the project family, not an exact code-copy match.',
              '',
              '## Maintaining the audit', '',
              'Edit `credits.json` only when new evidence supports a changed contribution. Keep role '
              'descriptions specific. Preserve unrelated original author tags. Use independent Git '
              'contribution evidence for other people; a tag in a Joel-only new-class commit does not '
              'establish their contribution. Record AI changes '
              'and direct guidance separately; approval, review, and ownership do not by themselves prove '
              'hand-written implementation.', '',
              '`python tools/audit_authorship.py` collects a fresh factual snapshot without changing Java. '
              '`python tools/review_author_commits.py` refreshes the separate commit-contribution facts. '
              '`python tools/update_authorship_docs.py` applies only the targeted curated ledger without changing executable Java. '
              '`python tools/render_authorship_audit.py` renders this report. The first command replaces the '
              'inventory snapshot; keep the original snapshot when comparing against this audit’s validation.', '',
              'Generated evidence snapshots are compressed losslessly; the scripts read them directly. '
              'The editable credit ledger, reports, and validation results remain plain text. '
              'See [snapshot storage](SNAPSHOTS.md) for details.', '']
    (OUT / 'AUTHORSHIP.md').write_text('\n'.join(lines), encoding='utf-8')
    print(json.dumps(dict(report=str(OUT / 'AUTHORSHIP.md'), apex_files=len(project), sdk_files=len(sdk),
                          scoped_joel_credits=sum('Joel H' in r['authors'] for r in project),
                          unresolved_joel_credits=len(uncertain))))


if __name__ == '__main__':
    main()
