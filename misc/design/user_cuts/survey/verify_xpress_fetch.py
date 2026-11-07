# usage: python3 -I verify_xpress_fetch.py <index.json> <version> <dest dir>: sha256-checked download, unpacks only shared libraries and licences. Runs nothing from the wheel.
import hashlib, json, sys, urllib.request, zipfile, pathlib
index, version, dest = sys.argv[1], sys.argv[2], pathlib.Path(sys.argv[3])
files = json.load(open(index))['releases'][version]
f = next(x for x in files if 'manylinux' in x['filename'] and 'x86_64' in x['filename'])
dest.mkdir(parents=True, exist_ok=True)
wheel = dest / f['filename']
urllib.request.urlretrieve(f['url'], wheel)
assert hashlib.sha256(wheel.read_bytes()).hexdigest() == f['digests']['sha256'], 'sha256 mismatch'
with zipfile.ZipFile(wheel) as z:
    for n in z.namelist():
        base = n.rsplit('/', 1)[-1]
        if base.startswith('libxprs') or base.startswith('libxprl') or base.endswith('.xpr'):
            z.extract(n, dest)
wheel.unlink()
print(sorted(str(p.relative_to(dest)) for p in dest.rglob('*') if p.is_file()))
