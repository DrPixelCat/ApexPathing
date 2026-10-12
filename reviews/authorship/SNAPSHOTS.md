# Audit snapshot storage

Large generated evidence snapshots use gzip JSON to keep repository size and pull
request diffs manageable. Compression preserves every byte of the original JSON,
including historical credits, session references, and source fingerprints.

- `git-inventory.json.gz`: source fingerprints, author tags, and Git history.
- `git-contributions.json.gz`: substantive commit-contribution evidence.
- `pre-audit-docs.json.gz`: original JavaDoc snapshots.
- `../ai-class-review/inventory.json.gz`: detailed AI-edit review evidence.

The audit tools read compressed snapshots directly and still accept legacy plain
JSON. Snapshot generators write deterministic `.json.gz` files. The curated
`credits.json`, `design-sources.json`, validation results, and Markdown reports
remain readable and editable without decompression.

To inspect a snapshot from the repository root without changing it:

```powershell
python -c "import gzip; print(gzip.open('reviews/authorship/git-inventory.json.gz', 'rt', encoding='utf-8').read())"
```

Snapshots record a particular audit state. Regenerating them collects current
history and available local sessions; it does not reproduce missing historical
session records. Preserve the archived snapshots when comparing audit results.
