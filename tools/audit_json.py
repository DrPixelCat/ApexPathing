"""Lossless storage for generated audit snapshots; curated ledgers stay plain JSON."""
import gzip
import json


def read_json(path):
    """Read a compressed snapshot, falling back to legacy plain JSON."""
    compressed = path.with_name(path.name + '.gz')
    if compressed.exists():
        return json.loads(gzip.decompress(compressed.read_bytes()))
    return json.loads(path.read_text(encoding='utf-8'))


def write_snapshot(path, data):
    """Write deterministic gzip JSON and return its actual destination."""
    target = path.with_name(path.name + '.gz')
    payload = (json.dumps(data, indent=2) + '\n').encode('utf-8')
    target.write_bytes(gzip.compress(payload, compresslevel=9, mtime=0))
    return target
