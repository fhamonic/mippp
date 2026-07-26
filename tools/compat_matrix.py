#!/usr/bin/env python3
"""Build the solver/library-version compatibility table.

For every backend, this downloads released shared libraries, points MIP++ at
each of them through MIPPP_<key>_LIBRARY, runs that backend's test suites and
renders the outcome as markdown.

MIP++ wraps exactly one version of each solver API, so what the table measures
is the *tolerance range of a single wrapper*: which released libraries the
wrapper still loads and drives correctly.

Linux/x86-64 only -- the sources are manylinux wheels and conda-forge linux-64
packages, and the symbol probe shells out to `nm`.

    python3 tools/compat_matrix.py list
    python3 tools/compat_matrix.py run --limit 5
    python3 tools/compat_matrix.py render

Only stdlib is used, plus `nm`, `tar` and `zstd` from the system.
"""

import argparse
import collections
import json
import os
import re
import shutil
import statistics
import subprocess
import sys
import tarfile
import urllib.request
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
MANIFEST = Path(__file__).with_name("compat_manifest.json")
CACHE = ROOT / ".compat-cache"
DOWNLOADS = CACHE / "downloads"
EXTRACTED = CACHE / "extracted"
RESULTS = CACHE / "results"
DEFAULT_BINARY = ROOT / "build" / "test" / "mippp_test"
DEFAULT_OUTPUT = ROOT / "docs" / "solvers" / "compatibility.md"

SOLVERS = json.loads(MANIFEST.read_text())["solvers"]

# construct_api() emits this when MIPPP_REQUIRED_SOLVERS names a solver whose
# api would not construct, see test/test_suites/all.hpp.
LOAD_FAILURE_MARKER = "is listed in MIPPP_REQUIRED_SOLVERS"

# Result markers, also spelled out in the generated legend. Unicode has no
# yellow check mark, so the caution sign carries the "passed, but read the
# notes" case; changing a glyph here changes the whole table and its legend.
PASS_MARK = "✅"
WARN_MARK = "⚠️"
FAIL_MARK = "❌"

# A pass that skipped a large share of its suite is not the clean run the green
# tick implies: MOSEK without a licence skips 62 of 131 tests on license_error
# and would otherwise read exactly like a fully exercised backend.
SKIP_SHARE_WARN = 0.1

# A licence failure is not a wrapper failure: the library loaded and every
# symbol resolved, and the solver itself then refused to run. Matched on the
# text because construct_api() reports only what the exception said, and both
# spellings occur -- "Xpress licensing error", "The license has expired".
LICENCE_ERROR = re.compile(r"licen[cs]", re.I)

# PEP 440 pre-release and dev suffixes. An index lists these beside the real
# releases, but a beta is not a released library: gurobipy 13.0.0b1 is a
# time-limited build that expired on 2025-12-02 and can now only report a
# licence error, and it would still spend a --limit slot owed to a release.
PRE_RELEASE = re.compile(
    r"[-_.]?(a|b|c|rc|alpha|beta|pre|preview|dev)[-_.]?[0-9]*$", re.I
)


# --------------------------------------------------------------------------
# version helpers


def version_key(version):
    """Sort key for dotted versions; non-numeric parts sort before numeric."""
    return tuple(
        int(part) if part.isdigit() else -1 for part in re.split(r"[._\-+]", version)
    )


def newest_first(versions):
    return sorted(versions, key=version_key, reverse=True)


# --------------------------------------------------------------------------
# sources


def get_json(url):
    request = urllib.request.Request(url, headers={"User-Agent": "mippp-compat"})
    with urllib.request.urlopen(request, timeout=60) as response:
        return json.load(response)


def discover_pypi(source):
    """Map version -> archive, from the manylinux x86-64 wheels on PyPI."""
    data = get_json("https://pypi.org/pypi/{}/json".format(source["package"]))
    archives = {}
    for version, files in data["releases"].items():
        wheels = [
            f
            for f in files
            if f["filename"].endswith(".whl")
            and "manylinux" in f["filename"]
            and "x86_64" in f["filename"]
            and not f.get("yanked", False)
        ]
        if not wheels:
            continue
        wheel = sorted(wheels, key=lambda f: f["filename"])[0]
        archives[version] = {"url": wheel["url"], "filename": wheel["filename"]}
    return archives


def discover_conda(source):
    """Map version -> archive, from the conda-forge linux-64 subdir."""
    data = get_json(
        "https://api.anaconda.org/package/conda-forge/{}".format(source["package"])
    )
    archives = {}
    for entry in data["files"]:
        if entry["attrs"].get("subdir") != "linux-64":
            continue
        if not entry["basename"].endswith((".conda", ".tar.bz2")):
            continue
        version = entry["version"]
        build_number = entry["attrs"].get("build_number", 0)
        previous = archives.get(version)
        if previous is not None and previous["build_number"] >= build_number:
            continue
        url = entry["download_url"]
        if url.startswith("//"):
            url = "https:" + url
        archives[version] = {
            "url": url,
            "filename": Path(entry["basename"]).name,
            "build_number": build_number,
        }
    return archives


def discover(source):
    """Map version -> archive, over the *released* linux-64 builds."""
    if source["kind"] == "pypi":
        archives = discover_pypi(source)
    elif source["kind"] == "conda":
        archives = discover_conda(source)
    else:
        raise ValueError("unknown source kind: {}".format(source["kind"]))
    return {v: a for v, a in archives.items() if not PRE_RELEASE.search(v)}


def select_versions(source, limit):
    """The `limit` newest published versions, newest first."""
    archives = discover(source)
    return {version: archives[version] for version in newest_first(archives)[:limit]}


# --------------------------------------------------------------------------
# fetching


def download(url, destination):
    if destination.exists():
        return destination
    destination.parent.mkdir(parents=True, exist_ok=True)
    partial = destination.with_name(destination.name + ".part")
    request = urllib.request.Request(url, headers={"User-Agent": "mippp-compat"})
    with urllib.request.urlopen(request, timeout=300) as response:
        with open(partial, "wb") as out:
            shutil.copyfileobj(response, out)
    partial.rename(destination)
    return destination


def extract(archive, destination):
    if destination.exists():
        return destination
    staging = destination.with_name(destination.name + ".tmp")
    shutil.rmtree(staging, ignore_errors=True)
    staging.mkdir(parents=True)
    name = archive.name
    if name.endswith((".whl", ".conda")):
        with zipfile.ZipFile(archive) as zf:
            zf.extractall(staging)
        # A .conda is a zip holding zstd tarballs; only pkg-* carries the files.
        for inner in staging.glob("pkg-*.tar.zst"):
            subprocess.run(
                ["tar", "--zstd", "-xf", str(inner), "-C", str(staging)], check=True
            )
            inner.unlink()
    elif name.endswith(".tar.bz2"):
        with tarfile.open(archive, "r:bz2") as tf:
            try:
                tf.extractall(staging, filter="data")
            except TypeError:  # python < 3.12
                tf.extractall(staging)
    else:
        raise ValueError("unsupported archive: {}".format(name))
    destination.parent.mkdir(parents=True, exist_ok=True)
    staging.rename(destination)
    return destination


def fetch_tree(archive_info, group):
    """Download and unpack one archive; returns `(archive, tree)`.

    The tree is named after the archive rather than after the version: two
    packages can publish the same version -- `xpress` and `xpresslibs` both
    ship a 9.9.1 -- and an unpacked tree one of them left behind must never be
    mistaken for the other's, since extraction skips a destination that exists.
    """
    archive = download(archive_info["url"], DOWNLOADS / group / archive_info["filename"])
    return archive, extract(archive, EXTRACTED / group / archive.name)


def discard(archive, tree):
    shutil.rmtree(tree, ignore_errors=True)
    archive.unlink(missing_ok=True)


def capture(command, env=None):
    """Stdout of `command`; probes read it and never care about the status."""
    return subprocess.run(command, capture_output=True, text=True, env=env).stdout


# --------------------------------------------------------------------------
# symbols


def wrapper_symbols(config):
    """The functions the wrapper resolves, from its F(...) X-macro list."""
    name = config["name"]
    header = (
        ROOT
        / "include"
        / "mippp"
        / "solvers"
        / name
        / config["wrapper"]
        / "{}_api.hpp".format(name)
    )
    text = header.read_text()
    return set(re.findall(r"^\s+F\(([A-Za-z_][A-Za-z0-9_]*)\s*,", text, re.M))


def exported_symbols(library):
    """Exported names, or None when `nm` could not read the library.

    A solver library with no dynamic symbols at all is not a thing, so an empty
    result means the probe failed -- reporting that as "every symbol missing"
    would be a confident lie about a library that loads perfectly well.
    """
    output = capture(["nm", "--dynamic", "--defined-only", str(library)])
    # Drop any ELF version tag: Xpress exports XPRSaddcols@@XPRS, and dlsym()
    # resolves the default version under the bare name the wrapper asks for.
    return {
        name.split("@")[0]
        for name in re.findall(r"^\S+\s+\S\s+(\S+)$", output, re.M)
    } or None


def fetch_dependencies(source):
    """Extract the runtime packages a solver library needs but does not ship.

    conda-forge splits the COIN-OR stack and SCIP's NLP backends across
    packages, so libCbcSolver arrives without libCgl and libscip without
    libipopt. Their newest build is enough: MIP++ never calls into them, they
    only have to resolve.
    """
    directories = []
    for package in source.get("depends", []):
        archives = select_versions({"kind": "conda", "package": package}, 1)
        if not archives:
            continue
        version, archive_info = next(iter(archives.items()))
        _, tree = fetch_tree(archive_info, "_deps")
        if (tree / "lib").is_dir():
            directories.append(tree / "lib")
        else:
            # conda-forge splits tools from libraries: `scotch` ships only
            # binaries, the shared objects are in `libscotch`.
            print(
                "    warning: {} ships no lib/, did you mean lib{}?".format(
                    package, package
                )
            )
    return directories


def library_env(library, extra_dirs):
    """An environment resolving `library` and its siblings, prepended.

    The dependencies shipped beside the *downloaded* library must win over any
    local install, and dropping the inherited entries would take the compiler
    runtime with them.
    """
    environment = dict(os.environ)
    environment["LD_LIBRARY_PATH"] = os.pathsep.join(
        p
        for p in [str(library.parent)]
        + [str(d) for d in extra_dirs]
        + [environment.get("LD_LIBRARY_PATH", "")]
        if p
    )
    return environment


def unresolved_sonames(library, extra_dirs):
    """Sonames `ldd` cannot resolve -- a missing package, not a bad wrapper."""
    output = capture(["ldd", str(library)], library_env(library, extra_dirs))
    return sorted(set(re.findall(r"^\s*(\S+) => not found", output, re.M)))


def find_library(root, patterns, wanted):
    """Best candidate among `patterns`: the one exporting the most of `wanted`.

    Picking by symbol count rather than by name settles cases like Cbc, which
    ships both libCbc.so and libCbcSolver.so where only the latter carries the
    C API.
    """
    candidates = []
    for pattern in patterns:
        candidates.extend(sorted(root.glob(pattern)))
    best = None
    for candidate in candidates:
        if not candidate.is_file():
            continue
        found = len(wanted & (exported_symbols(candidate) or set()))
        if best is None or found > best[0]:
            best = (found, candidate)
    return best[1] if best else None


# --------------------------------------------------------------------------
# running


def available_prefixes(binary):
    """Test-suite prefixes compiled into the binary (TEST_SOURCE is sticky)."""
    return {
        line.split("/")[0]
        for line in capture([str(binary), "--gtest_list_tests"]).splitlines()
        if line and not line.startswith(" ")
    }


def run_tests(binary, config, library, output, timeout, extra_dirs):
    environment = library_env(library, extra_dirs)
    environment["MIPPP_{}_LIBRARY".format(config["key"])] = str(library)
    # Testing mismatched versions is the point, so silence the warning; and make
    # a failure to load a hard failure rather than a silent skip.
    environment["MIPPP_NO_VERSION_WARNING"] = "1"
    environment["MIPPP_REQUIRED_SOLVERS"] = config["key"]
    # Pin the fuzz seed: rows are compared against each other, and an unlucky
    # random seed would otherwise pin a rare flake on whichever version drew it.
    environment["MIPPP_FUZZ_SEED"] = "20260725"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.unlink(missing_ok=True)
    try:
        completed = subprocess.run(
            [
                str(binary),
                "--gtest_filter={}".format(config["gtest_filter"]),
                "--gtest_output=json:{}".format(output),
            ],
            env=environment,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except subprocess.TimeoutExpired:
        return {"timed_out": True, "report": None}
    # No report means the run died before GoogleTest could write one -- a solver
    # that abort()s takes the process with it, which is not the same as a hang.
    # Keep the solver's own diagnostics and the test it died in; drop the
    # GoogleTest progress chatter that would otherwise crowd out both.
    lines = [
        line.strip()
        for line in (completed.stdout + completed.stderr).splitlines()
        if line.strip() and not re.match(r"^\[\s*(OK|-+|=+)\s*\]", line.strip())
    ]
    return {
        "timed_out": False,
        "returncode": completed.returncode,
        "tail": lines[-3:],
        "report": json.loads(output.read_text()) if output.exists() else None,
    }


def summarize(report):
    """Fold a gtest json report into counts, and detect a load failure."""
    passed = failed = skipped = 0
    failed_tests = []
    load_error = None
    skip_reasons = collections.Counter()
    for suite in report.get("testsuites", []):
        for test in suite.get("testsuite", []):
            name = "{}.{}".format(suite["name"], test["name"])
            failures = test.get("failures", [])
            if failures:
                failed += 1
                for failure in failures:
                    message = failure.get("failure", "")
                    if LOAD_FAILURE_MARKER in message:
                        load_error = message.strip().splitlines()[-1]
                if len(failed_tests) < 8:
                    failed_tests.append(name)
            elif test.get("result") == "SKIPPED" or test.get("status") == "NOTRUN":
                skipped += 1
                for note in test.get("skipped", []):
                    # GoogleTest prefixes the reason with "<file>:<line>", so
                    # what GTEST_SKIP() was actually given is the last line.
                    lines = note.get("message", "").strip().splitlines()
                    if lines:
                        skip_reasons[lines[-1].strip()] += 1
            else:
                passed += 1
    return {
        "passed": passed,
        "failed": failed,
        "skipped": skipped,
        "failed_tests": failed_tests,
        "load_error": load_error,
        # Only the commonest: a suite that skips wholesale does so for one
        # reason, and that reason is the whole story of the row.
        "skip_reason": skip_reasons.most_common(1)[0][0] if skip_reasons else None,
    }


# --------------------------------------------------------------------------
# driver


def select_configs(selection, include_commercial):
    configs = []
    for name, config in SOLVERS.items():
        if selection and name not in selection:
            continue
        if not selection and not include_commercial and config["tier"] != "open-source":
            continue
        configs.append(dict(config, name=name))
    return configs


def solver_version(library, source, package_version):
    """The solver's own version.

    For conda-forge the package version is the solver version. For a wheel it
    is the binding's -- PySCIPOpt 5.2.1 ships libscip 9.1 -- so it is read off
    the library filename, and flagged as approximate when that fails.
    """
    if source["kind"] == "conda":
        return package_version, False
    pattern = source.get("version_regex")
    match = re.search(pattern, library.name) if pattern else None
    if match:
        return match.group(1), False
    return package_version, True


def also_shipped_by(row, package_version):
    row.setdefault("also_packages", []).append(package_version)


def process(config, versions, args):
    source = config["source"]
    wanted = wrapper_symbols(config)
    extra_dirs = fetch_dependencies(source)
    rows = []
    # Several package versions often ship one library -- gurobipy 12.0.0 to
    # 12.0.3 all carry libgurobi120.so. Test each library once and record the
    # other packages on that row instead of repeating an identical one.
    tested = {}
    for version, archive_info in versions.items():
        row = {
            "solver": config["name"],
            "package": source["package"],
            "package_version": version,
        }
        print("  {} {}".format(source["package"], version), flush=True)
        try:
            archive, tree = fetch_tree(archive_info, config["name"])
        except Exception as error:  # network, zstd, corrupt archive
            row["status"] = "fetch-error"
            row["detail"] = str(error)
            rows.append(row)
            continue

        library = find_library(tree, source["lib_patterns"], wanted)
        if library is None:
            row["status"] = "no-library"
            # Name what the archive does ship: whether the library sits under
            # an unexpected name or is simply absent is the whole diagnosis,
            # and the pattern alone never says which.
            shipped = sorted({p.name for p in tree.rglob("*.so*") if p.is_file()})
            row["detail"] = "no candidate matched {}; archive ships {}".format(
                source["lib_patterns"],
                ", ".join(shipped[:4]) if shipped else "no shared object",
            )
            rows.append(row)
            if not args.keep:
                discard(archive, tree)
            continue

        # Report the real file: conda ships lib<name>.so as a symlink onto the
        # versioned soname, which is what actually gets loaded.
        row["library"] = library.resolve().name
        row["version"], row["approximate"] = solver_version(library, source, version)

        # Versions are walked newest first, so the row already recorded is the
        # one to keep; this package is merely another way to obtain it.
        identity = (row["library"], library.resolve().stat().st_size)
        if identity in tested:
            also_shipped_by(tested[identity], version)
            print("    same library as {}, not re-tested".format(
                tested[identity]["package_version"]))
            if not args.keep:
                discard(archive, tree)
            continue
        tested[identity] = row
        rows.append(row)
        exported = exported_symbols(library)
        row["missing_symbols"] = sorted(wanted - exported) if exported else []
        row["unresolved"] = unresolved_sonames(library, extra_dirs)

        outcome = run_tests(
            args.binary,
            config,
            library,
            RESULTS / config["name"] / "{}.json".format(version),
            args.timeout,
            extra_dirs,
        )
        if outcome["timed_out"]:
            row["status"] = "timeout"
        elif outcome["report"] is None:
            row["status"] = "crash"
            code = outcome["returncode"]
            row["detail"] = "{}; {}".format(
                "killed by signal {}".format(-code) if code < 0 else "exit {}".format(code),
                " / ".join(outcome["tail"]) or "no output",
            )
        else:
            row.update(summarize(outcome["report"]))
            if row["load_error"]:
                row["status"] = "load-error"
            elif row["failed"]:
                row["status"] = "partial"
            elif row["passed"] == 0:
                row["status"] = "no-tests"
            else:
                row["status"] = "ok"

        if not args.keep:
            discard(archive, tree)
    return rows


# --------------------------------------------------------------------------
# rendering


def skip_baseline(rows):
    """How many skips are normal for this solver, over its working versions.

    Capability skips come from the wrapper, not the library, so they are
    normally identical across a solver's versions -- which makes the siblings a
    yardstick for spotting the version that lost one. The typical row, not the
    cheapest, sets it: some skips are decided at runtime rather than by
    capability -- the time-limit test bows out when the solver closes the
    instance before the limit can bite -- so the luckiest row would otherwise
    turn every healthy sibling yellow over a one-test timing difference.

    Only clean rows qualify: a version that fails everything skips nothing.
    """
    counts = [r["skipped"] for r in rows if r["status"] == "ok"]
    return statistics.median_high(counts) if counts else 0


def unusual_skips(row, baseline):
    """Whether this row skipped more than a clean pass should.

    Two ways to qualify: skipping tests that every other version of the same
    solver ran -- this version lost a capability -- or skipping so much of the
    suite that what did pass says little about the library.
    """
    skipped = row.get("skipped") or 0
    if not skipped:
        return False
    total = row.get("passed", 0) + row.get("failed", 0) + skipped
    return skipped > baseline or skipped >= SKIP_SHARE_WARN * total


def status_cell(row, baseline=0):
    status = row["status"]
    if status in ("ok", "partial"):
        # Always the tests that passed, out of every test the run accounted
        # for; the marker, not the number, carries the verdict.
        if row["failed"]:
            mark = FAIL_MARK
        elif unusual_skips(row, baseline):
            mark = WARN_MARK
        else:
            mark = PASS_MARK
        total = row["passed"] + row["failed"] + row["skipped"]
        return "{} {}/{}".format(mark, row["passed"], total)
    if status == "load-error":
        # An unresolved soname is a gap in the download environment, not an
        # incompatible wrapper -- say so rather than blame the solver version.
        if row.get("unresolved"):
            return "{} not tested (dependency missing)".format(WARN_MARK)
        if LICENCE_ERROR.search(row.get("load_error") or ""):
            return "{} not tested (licence)".format(WARN_MARK)
        if row.get("missing_symbols"):
            return "{} will not load ({} symbols missing)".format(
                FAIL_MARK, len(row["missing_symbols"])
            )
        return "{} will not load".format(FAIL_MARK)
    # Nothing ran, so nothing is proven either way -- only the outcomes that
    # are the library's own fault get a cross.
    return {
        "crash": "{} aborts".format(FAIL_MARK),
        "timeout": "{} timed out".format(FAIL_MARK),
        "no-library": "{} no library in archive".format(WARN_MARK),
        "fetch-error": "{} download failed".format(WARN_MARK),
        "no-tests": "{} no test ran".format(WARN_MARK),
    }.get(status, status)


def collapse_duplicates(rows):
    """Drop rows that would render identically, keeping the newest package.

    A run already skips re-testing a library it has seen, but results merged
    from separate runs can still collide -- and two packages shipping the same
    library must not become two identical lines in the table.
    """
    kept = []
    seen = {}
    for row in sorted(
        rows, key=lambda r: version_key(r["package_version"]), reverse=True
    ):
        key = (
            row["solver"],
            row.get("version", row["package_version"]),
            row.get("library"),
            row["status"],
            row.get("passed"),
            row.get("failed"),
            row.get("skipped"),
        )
        if key in seen:
            also_shipped_by(seen[key], row["package_version"])
            continue
        seen[key] = row
        kept.append(row)
    return kept


def package_span(row):
    """`oldest-newest` over the packages that resolve to this same library."""
    versions = sorted(
        {row["package_version"], *row.get("also_packages", [])}, key=version_key
    )
    if len(versions) < 2:
        return None
    return "{}-{}".format(versions[0], versions[-1])


def api_message(load_error):
    """What the solver itself said, without construct_api()'s framing."""
    _, _, tail = load_error.partition("could not be constructed: ")
    return (tail or load_error).strip()


def row_notes(row, source, baseline=0):
    notes = []
    span = package_span(row)
    if span:
        notes.append("{} {}".format(source["package"], span))
    if row.get("unresolved"):
        notes.append("needs {}".format(", ".join(row["unresolved"])))
    if row["status"] == "load-error" and LICENCE_ERROR.search(row.get("load_error") or ""):
        notes.append(api_message(row["load_error"]))
    # Only when the load failed: a library that ran the suite resolved every
    # symbol by definition, whatever the static probe thinks.
    elif row["status"] == "load-error" and row.get("missing_symbols"):
        notes.append("missing: {}".format(", ".join(row["missing_symbols"][:4])))
    elif row["status"] == "partial" and row.get("failed_tests"):
        notes.append("failing: {}".format(", ".join(row["failed_tests"][:3])))
    if row.get("skipped"):
        note = "{} skipped".format(row["skipped"])
        # Why, but only when the skips are the story: one capability test
        # bowing out explains itself, a whole suite doing so does not.
        if row.get("skip_reason") and (
            row["status"] == "no-tests" or unusual_skips(row, baseline)
        ):
            note += ": {}".format(row["skip_reason"])
        notes.append(note)
    if row.get("detail"):
        notes.append(row["detail"])
    return "; ".join(notes) or "-"


def render(rows, output):
    by_solver = {}
    # Discovery skips pre-releases, but results merged from an earlier run keep
    # whatever that run found, and a beta must not linger in the table.
    released = [r for r in rows if not PRE_RELEASE.search(r["package_version"])]
    for row in collapse_duplicates(released):
        by_solver.setdefault(row["solver"], []).append(row)

    lines = [
        "# Solver version compatibility",
        "",
        "<!-- Generated by tools/compat_matrix.py -- do not edit by hand. -->",
        "",
        "MIP++ wraps one version of each solver API and loads the library at",
        "runtime, so the question this table answers is: **which released",
        "versions does that wrapper still drive correctly?**",
        "",
        "Each row is a published library, downloaded and pointed at through",
        "`MIPPP_<solver>_LIBRARY`, with the backend's full test suite run",
        "against it on Linux/x86-64. The result column counts the tests that",
        "passed out of every test accounted for:",
        "",
        "- {} every test passed".format(PASS_MARK),
        "- {} some test failed".format(FAIL_MARK),
        "- {} passed, but an unusual number of tests skipped, or nothing ran "
        "at all -- the notes say which".format(WARN_MARK),
        "",
        "Capability skips are counted in the total, so a backend that cannot do",
        "quadratic objectives still shows those tests. The fuzz seed is pinned",
        "so that rows differ only by the library under test.",
        "",
    ]
    for name in sorted(by_solver):
        config = SOLVERS[name]
        source = config["source"]
        # The gtest prefix doubles as the solver's display name (HiGHS, SoPlex).
        lines.append("## {}".format(config["gtest_filter"].rstrip("*")))
        lines.append("")
        lines.append(
            "Wrapper `{}`, from `{}` ({}).".format(
                config["wrapper"], source["package"], source["kind"]
            )
        )
        if config.get("note"):
            lines.append("")
            lines.append("!!! warning")
            lines.append("    {}.".format(config["note"].capitalize()))
        lines.append("")
        lines.append("| version | library | result | notes |")
        lines.append("| --- | --- | --- | --- |")
        baseline = skip_baseline(by_solver[name])
        for row in sorted(
            by_solver[name],
            key=lambda r: version_key(r.get("version", r["package_version"])),
            reverse=True,
        ):
            version = row.get("version", row["package_version"])
            lines.append(
                "| `{}` | `{}` | {} | {} |".format(
                    "~{}".format(version) if row.get("approximate") else version,
                    row.get("library", "-"),
                    status_cell(row, baseline),
                    row_notes(row, source, baseline),
                )
            )
        lines.append("")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines) + "\n")
    print("wrote {}".format(output))


# --------------------------------------------------------------------------


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["list", "run", "render"])
    parser.add_argument("--solvers", help="comma-separated subset")
    parser.add_argument("--limit", type=int, default=5, help="versions per solver")
    parser.add_argument("--commercial", action="store_true", help="include commercial")
    parser.add_argument("--binary", type=Path, default=DEFAULT_BINARY)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--timeout", type=int, default=900)
    parser.add_argument("--keep", action="store_true", help="keep downloads")
    args = parser.parse_args()

    results_file = RESULTS / "compat.json"
    if args.command == "render":
        if not results_file.exists():
            sys.exit("no results yet, run `compat_matrix.py run` first")
        render(json.loads(results_file.read_text()), args.output)
        return

    selection = set(args.solvers.split(",")) if args.solvers else None
    configs = select_configs(selection, args.commercial)

    if args.command == "list":
        for config in configs:
            # The count is every published version; only the names are cut down
            # to --limit, which is what a `run` would actually test.
            versions = newest_first(discover(config["source"]))
            print(
                "{:8} {:14} {:3} versions: {}".format(
                    config["name"],
                    config["source"]["package"],
                    len(versions),
                    ", ".join(versions[: args.limit]),
                )
            )
        return

    if not args.binary.exists():
        sys.exit("{} not found, build it with `make test` first".format(args.binary))
    prefixes = available_prefixes(args.binary)

    rows = []
    for config in configs:
        prefix = config["gtest_filter"].rstrip("*")
        if not any(p.startswith(prefix) for p in prefixes):
            print(
                "{}: not compiled into {} (TEST_SOURCE is sticky in the CMake "
                "cache), skipping".format(config["name"], args.binary.name)
            )
            continue
        print("{}:".format(config["name"]), flush=True)
        rows.extend(process(config, select_versions(config["source"], args.limit), args))

    # Merge rather than replace, so re-running one solver keeps the rest of the
    # table instead of silently emptying it.
    merged = []
    if results_file.exists():
        refreshed = {(r["solver"], r["package_version"]) for r in rows}
        merged = [
            r
            for r in json.loads(results_file.read_text())
            if (r["solver"], r["package_version"]) not in refreshed
        ]
    merged.extend(rows)
    results_file.parent.mkdir(parents=True, exist_ok=True)
    results_file.write_text(json.dumps(merged, indent=2))
    print("wrote {} ({} rows)".format(results_file, len(merged)))
    render(merged, args.output)


if __name__ == "__main__":
    main()
