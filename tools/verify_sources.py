"""Check bundled source hashes offline. Python standard library only."""
from pathlib import Path
import hashlib
import json

root = Path(__file__).resolve().parents[1]
checked = 0
for name in ['manifest.json', 'mesh_manifest.json']:
    for item in json.loads((root / 'sources' / name).read_text()):
        path = root / item['path']
        if hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
            raise SystemExit(f"Source hash mismatch: {item['path']}")
        checked += 1
print(f'PASS: {checked} preserved source records')
