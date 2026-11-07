# usage: python3 -I probe_xpress_fetch.py <index.json> <version> <dest dir>: downloads the wheel, checks its sha256, unpacks the libraries only
import hashlib, json, sys, urllib.request, zipfile, pathlib
index, version, dest = sys.argv[1], sys.argv[2], pathlib.Path(sys.argv[3])
files = json.load(open(index))['releases'][version]
f = next(x for x in files if 'manylinux' in x['filename'] and 'x86_64' in x['filename'])
dest.mkdir(parents=True, exist_ok=True)
wheel = dest / f['filename']
urllib.request.urlretrieve(f['url'], wheel)
h = hashlib.sha256(wheel.read_bytes()).hexdigest()
assert h == f['digests']['sha256'], 'sha256 mismatch'
with zipfile.ZipFile(wheel) as z:
    for n in z.namelist():
        base = n.rsplit('/', 1)[-1]
        if base.startswith('libxprs') or base.startswith('libxprl') or base.startswith('xpauth') or base.endswith('.xpr'):
            z.extract(n, dest)
            print(n)
wheel.unlink()
