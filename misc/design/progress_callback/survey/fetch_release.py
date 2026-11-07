# usage: fetch_release.py <solver> <version>: prints the library a backend would load
import json, sys
from pathlib import Path
sys.path.insert(0, "/home/plaiseek/Projects/mippp/misc/tools")
import compat_matrix as cm
store = Path(sys.argv[3])
cm.DOWNLOADS = store / "downloads"
cm.EXTRACTED = store / "extracted"
solver, want = sys.argv[1], sys.argv[2]
manifest = json.loads(Path("/home/plaiseek/Projects/mippp/misc/tools/compat_manifest.json").read_text())
config = manifest["solvers"][solver]
config["name"] = solver
source = config["source"]
available = cm.discover(source)
if want == "list":
    print(sorted(available, key=cm.version_key)[-40:] if isinstance(available, dict) else available)
    sys.exit(0)
archive, tree = cm.fetch_tree(available[want], solver)
wanted = cm.wrapper_symbols(config)
library = cm.find_library(tree, source["lib_patterns"], wanted)
used, unpinned = cm.fetch_dependencies(source, tree, library.parent)
print(library)
