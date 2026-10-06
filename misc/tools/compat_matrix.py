#!/usr/bin/env python3
"""Build the solver/library-version compatibility table.

For every backend, this downloads released shared libraries, points MIP++ at
each of them through MIPPP_<key>_LIBRARY, runs that backend's test suites and
renders the outcome as markdown.

MIP++ drives each solver through one implementation that adapts at runtime to
a range of releases, so what the table measures is how far that one
implementation reaches: the releases it drives in full, those it supports
partially -- the model core passes, and the members a release lacks throw --
and those it does not drive. Each row is also held against the claim the
backend makes about its release, the validated_versions and supported_versions
its version test records.

Linux/x86-64 only -- the sources are manylinux wheels and conda-forge linux-64
packages, and the symbol probe shells out to `nm`.

    python3 misc/tools/compat_matrix.py list
    python3 misc/tools/compat_matrix.py run --limit 5
    python3 misc/tools/compat_matrix.py render

Only stdlib is used, plus `nm`, `tar` and `zstd` from the system.
"""

import argparse
import collections
import functools
import json
import os
import re
import shutil
import statistics
import subprocess
import sys
import tarfile
import traceback
import urllib.request
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = Path(__file__).with_name("compat_manifest.json")
CACHE = ROOT / ".compat-cache"
DOWNLOADS = CACHE / "downloads"
EXTRACTED = CACHE / "extracted"
RESULTS = CACHE / "results"
DEFAULT_BINARY = ROOT / "build" / "test" / "mippp_test"
DEFAULT_OUTPUT = ROOT / "docs" / "solvers" / "compatibility.md"

SOLVERS = json.loads(MANIFEST.read_text())["solvers"]

# model_test fails every test with this when MIPPP_REQUIRED_SOLVERS names a
# solver whose api would not construct, see test/test_suites/all.hpp.
LOAD_FAILURE_MARKER = "is listed in MIPPP_REQUIRED_SOLVERS"

# ...and skips every test with this prefix when the backend cannot run at all.
# Under MIPPP_REQUIRED_SOLVERS that is a licence refused while the api loads or
# by the first model; a licence refused by a solve skips that test alone,
# without the prefix, and the rest of the suite still says something.
UNAVAILABLE_MARKER = "backend unavailable: "

# Every solver file instantiates this test, so a filter selecting none means
# the backend is not in the binary, or the filter does not name its suites.
# Binaries built before supported_versions existed call it
# loaded_release_is_a_validated_one, and their reports must still read.
VERSION_TEST = re.compile(r"_api\.loaded_release_is_a_(supported|validated)_one$")

# What the version test records as GoogleTest properties, which the json report
# writes as extra string fields of the test's object, on a skipped test too.
# The claims are recorded before the library loads, so a row whose library
# reports no version still carries them, but release_support only once a
# version was there to classify. A claim reads "1.14:1.16,2:3":
# comma-separated `from:before` pairs, each bound spelled by solver_version's
# to_string(), which leaves trailing zero components out.
CLAIM_PROPERTIES = ("release_support", "validated_versions", "supported_versions")

# How the version test skips a library that reports no version (SoPlex).
NO_VERSION = re.compile(r"reports no version$")

# The shared suites skip with this prefix when the loaded release lacks the
# member a test calls -- it threw feature_unavailable_error -- on a release
# outside validated_versions; on a validated one the same throw fails the test,
# except on a library reporting no version, which claim_mismatch() catches.
# Such skips are what a partially supported row is made of, so they never
# count as the backend being unavailable, nor as the capability skips the
# baseline compares across versions.
LACKS_MARKER = "release lacks: "

# The lp_model / milp_model / qp_model core every partially supported release
# must still pass: a lacking skip in these typed suites breaks the claim
# rather than making the row partial. Matched against a component of the
# instantiated suite name, "HiGHS_milp/MilpModelTest/0".
CORE_SUITES = {"LpModelTest", "MilpModelTest", "QpModelTest"}

# Result markers, also spelled out in the generated legend. Unicode has no
# yellow check mark, so the caution sign carries the "passed, but read the
# notes" case, and the boxed check a partially supported release; changing a
# glyph here changes the whole table and its legend.
PASS_MARK = "✅"
PARTIAL_MARK = "☑️"
WARN_MARK = "⚠️"
FAIL_MARK = "❌"

# A pass that skipped a large share of its suite is not the clean run the green
# tick implies: MOSEK without a licence skips every test that solves on
# license_error and would otherwise read exactly like a fully exercised backend.
SKIP_SHARE_WARN = 0.1

# A licence failure is not a wrapper failure: the library loaded and every
# symbol resolved, and the solver itself then refused to run. Matched on the
# text because the skip carries only what the exception said, and both
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


def release_number(version):
    """`version` as parse_solver_version() reads it, or None when no number leads.

    Three components, absent ones 0, the rest ignored: version_key() would put
    "8.1" before "8.1.0" and "22.1.0.1" after "22.1", where a claim's bound
    and the C++ comparison see one release.
    """
    match = re.match(r"\d+(\.\d+)*", version)
    if not match:
        return None
    numbers = [int(part) for part in match.group(0).split(".")][:3]
    return tuple(numbers + [0] * (3 - len(numbers)))


def claim_ranges(claim):
    """The `(from, before)` release numbers of a recorded claim, or None.

    None when a pair does not parse, which classifies nothing rather than
    reading a garbled claim as one that excludes the row.
    """
    ranges = []
    for pair in filter(None, claim.split(",")):
        bounds = [release_number(bound) for bound in pair.split(":")]
        if len(bounds) != 2 or None in bounds:
            return None
        ranges.append(tuple(bounds))
    return ranges


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


@functools.lru_cache(maxsize=None)
def conda_archives(package):
    """Every linux-64 archive conda-forge publishes for `package`."""
    data = get_json("https://api.anaconda.org/package/conda-forge/{}".format(package))
    archives = []
    for entry in data["files"]:
        if entry["attrs"].get("subdir") != "linux-64":
            continue
        if not entry["basename"].endswith((".conda", ".tar.bz2")):
            continue
        url = entry["download_url"]
        if url.startswith("//"):
            url = "https:" + url
        archives.append(
            {
                "version": entry["version"],
                "build_number": entry["attrs"].get("build_number", 0),
                "url": url,
                "filename": Path(entry["basename"]).name,
            }
        )
    return archives


def archive_stem(archive_info):
    """`<package>-<version>-<build>`, the name conda pins a build by."""
    return re.sub(r"\.(conda|tar\.bz2)$", "", archive_info["filename"])


def discover_conda(source):
    """Map version -> archive, from the conda-forge linux-64 subdir."""
    archives = {}
    for archive_info in conda_archives(source["package"]):
        previous = archives.get(archive_info["version"])
        if previous is None or archive_info["build_number"] > previous["build_number"]:
            archives[archive_info["version"]] = archive_info
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
        # A .conda is a zip holding zstd tarballs: pkg-* carries the files and
        # info-* the metadata, among it the recipe that host_pins() reads.
        for inner in [*staging.glob("pkg-*.tar.zst"), *staging.glob("info-*.tar.zst")]:
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


def host_pins(tree):
    """`{package: [version, build]}` a conda archive was compiled against.

    index.json only gives ranges -- `coin-or-cgl >=0.60,<0.61` -- but a library
    must run against what it was built with: Cgl 0.60.7 changed the layout of a
    class Cbc inlines, and a Cbc built against Cgl 0.60.6 aborts on 0.60.10.
    The rendered recipe lists the exact builds. None when the archive carries
    no recipe, so that nothing passes for pinned.
    """
    recipe = tree / "info" / "recipe"
    for name in ("meta.yaml", "rendered_recipe.yaml"):  # conda-build, rattler-build
        if (recipe / name).is_file():
            return recipe_host_pins((recipe / name).read_text())
    return None


def recipe_host_pins(text):
    """The host pins of a rendered recipe, read by indentation.

    conda-build writes `- name version build` under requirements/host, and
    rattler-build one record per package under finalized_dependencies/host/
    resolved. Only the layout these two write is understood, not YAML at large.
    """
    pins = {}
    records = []
    keys = []  # (indent, key) of the mappings enclosing the line
    for line in text.splitlines():
        content = line.lstrip(" ")
        if not content or content.startswith("#"):
            continue
        indent = len(line) - len(content)
        item = content.startswith("- ")
        if item:
            content, indent = content[2:], indent + 2
        while keys and keys[-1][0] >= indent:
            keys.pop()
        path = [key for _, key in keys]
        key, colon, value = content.partition(":")
        if path == ["requirements", "host"] and item and not colon:
            name, *pin = content.split()
            pins[name] = pin[:2]
        elif path == ["finalized_dependencies", "host", "resolved"]:
            if item:
                records.append({})
            if records and key in ("name", "version", "build"):
                records[-1][key] = value.strip()
        if colon and not value.strip():
            keys.append((indent, key))
    for record in records:
        if "name" in record and "version" in record:
            pins[record["name"]] = [record["version"], record.get("build")][
                : 2 if record.get("build") else 1
            ]
    return pins


def dependency_archive(package, pin):
    """The archive of `package` to run beside a row, and whether it is pinned.

    The pinned build if conda-forge still has it, else the newest of the
    pinned version, else the newest of all -- which is also the choice for a
    package the recipe does not build against.
    """
    archives = conda_archives(package)
    if pin:
        for archive_info in archives:
            if archive_stem(archive_info) == "-".join([package] + pin):
                return archive_info, True
        same_version = [a for a in archives if a["version"] == pin[0]]
        if same_version:
            return max(same_version, key=lambda a: a["build_number"]), len(pin) == 1
    newest = select_versions({"kind": "conda", "package": package}, 1)
    return next(iter(newest.values()), None), not pin


def link_shared_objects(source, directory):
    """Hard-link the shared objects of `source` into `directory`.

    Names already there are kept, so a row's own files always win.
    """
    for entry in source.iterdir():
        target = directory / entry.name
        if not re.search(r"\.so(\.\d+)*$", entry.name) or os.path.lexists(target):
            continue
        if entry.is_symlink():
            target.symlink_to(os.readlink(entry))
        elif entry.is_file():
            try:
                os.link(entry, target)
            except OSError:  # another filesystem
                shutil.copy2(entry, target)


def fetch_dependencies(source, tree, directory):
    """Lay the packages a library needs but does not ship out beside it.

    conda-forge splits the COIN-OR stack and SCIP's NLP backends across
    packages, so libCbcSolver arrives without libCgl and libscip without
    libipopt. Each comes at the build the row's recipe pinned, linked into the
    library's directory as one conda environment would hold them: a conda
    library finds its siblings through an RPATH of $ORIGIN, which the loader
    searches before LD_LIBRARY_PATH, so nothing on the host under the same
    soname can stand in for them. What no package listed in `depends`
    provides, BLAS among it, comes from the host.

    Returns the archives used and those that are not the pinned build.
    """
    pins = host_pins(tree)
    used, unpinned = [], []
    for package in source.get("depends", []):
        archive_info, pinned = dependency_archive(package, (pins or {}).get(package))
        if archive_info is None:
            continue
        _, dependency = fetch_tree(archive_info, "_deps")
        used.append(archive_stem(archive_info))
        if pins is None or not pinned:
            unpinned.append(archive_stem(archive_info))
        if (dependency / "lib").is_dir():
            link_shared_objects(dependency / "lib", directory)
        else:
            # conda-forge splits tools from libraries: `scotch` ships only
            # binaries, the shared objects are in `libscotch`.
            print(
                "    warning: {} ships no lib/, did you mean lib{}?".format(
                    package, package
                )
            )
    return used, unpinned


def library_env(library):
    """An environment resolving `library`, its directory prepended.

    The prepended directory serves a library without an RPATH, and the
    inherited entries stay: dropping them would take the compiler runtime the
    test binary needs with them.
    """
    environment = dict(os.environ)
    environment["LD_LIBRARY_PATH"] = os.pathsep.join(
        p for p in [str(library.parent), environment.get("LD_LIBRARY_PATH", "")] if p
    )
    return environment


# The C and C++ runtime, which every process takes from the host.
RUNTIME = re.compile(r"^(libc|libm|libdl|libpthread|librt|libstdc\+\+|libgcc_s)\.so")


def resolve(library):
    """`(unresolved, host)` sonames of `library`'s dependency closure.

    The unresolved ones are a missing package, not a bad wrapper. The host
    ones resolved outside the library's directory, past the C and C++ runtime:
    the system BLAS by design, and anything a `depends` list forgot.
    """
    output = capture(["ldd", str(library)], library_env(library))
    unresolved = sorted(set(re.findall(r"^\s*(\S+) => not found", output, re.M)))
    directory = library.parent.resolve()
    host = sorted(
        {
            name
            for name, path in re.findall(r"^\s*(\S+) => (/\S+)", output, re.M)
            if not RUNTIME.match(name) and directory not in Path(path).resolve().parents
        }
    )
    return unresolved, host


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


def selected_tests(binary, gtest_filter):
    """The `Suite.Test` names the binary runs under `gtest_filter`.

    GoogleTest evaluates the filter itself, so a manifest entry can use any
    of its syntax. A binary that does not start exits here: listing nothing
    would read as every backend left out of the build.
    """
    completed = subprocess.run(
        [str(binary), "--gtest_filter={}".format(gtest_filter), "--gtest_list_tests"],
        capture_output=True,
        text=True,
    )
    if completed.returncode != 0:
        output = (completed.stderr or completed.stdout).strip().splitlines()
        sys.exit(
            "{} does not start: {}".format(
                binary, output[0] if output else "exit {}".format(completed.returncode)
            )
        )
    tests = []
    suite = None
    # Typed suites and tests carry a "# TypeParam = ..." comment.
    for line in completed.stdout.splitlines():
        name = line.split("#")[0].strip()
        if not name:
            continue
        if line.startswith(" "):
            tests.append(suite + name)
        else:
            suite = name
    return tests


def compiled_in(binary, config):
    return any(
        VERSION_TEST.search(test)
        for test in selected_tests(binary, config["gtest_filter"])
    )


def run_tests(binary, config, library, output, timeout):
    environment = library_env(library)
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


def load_failure_reason(message):
    """What the loader said, without model_test's framing or its advice.

    The loader closes on how MIPPP_<key>_LIBRARY takes precedence, the same
    paragraph on every row.
    """
    _, _, reason = message.partition("could not be constructed: ")
    lines = []
    for line in reason.strip().splitlines():
        if line.startswith("An explicit path passed to load()"):
            break
        lines.append(line.strip())
    return " ".join(lines)


def skip_reason(message):
    """What GTEST_SKIP() was given, or where it was called when bare.

    GoogleTest puts "<file>:<line>" on the first line. A reason's own first
    line is its cause; whatever follows is advice.
    """
    lines = [line.strip() for line in message.splitlines() if line.strip()]
    return lines[1] if len(lines) > 1 else lines[0] if lines else None


def summarize(report):
    """Fold a gtest json report into counts and claims; detect a load failure."""
    passed = failed = skipped = lacking = 0
    failed_tests = []
    core_lacking = []
    load_error = None
    reports_no_version = False
    claims = {}
    skip_reasons = collections.Counter()
    unavailable_reasons = collections.Counter()
    lacking_reasons = collections.Counter()
    # Stripped like the reasons are: an empty one loses its trailing space.
    unavailable_marker = UNAVAILABLE_MARKER.rstrip()
    lacks_marker = LACKS_MARKER.rstrip()
    for suite in report.get("testsuites", []):
        core = not CORE_SUITES.isdisjoint(suite["name"].split("/"))
        for test in suite.get("testsuite", []):
            name = "{}.{}".format(suite["name"], test["name"])
            version_test = VERSION_TEST.search(name)
            if version_test:
                claims = {key: test[key] for key in CLAIM_PROPERTIES if key in test}
            failures = test.get("failures", [])
            if failures:
                failed += 1
                for failure in failures:
                    message = failure.get("failure", "")
                    if LOAD_FAILURE_MARKER in message:
                        load_error = load_failure_reason(message)
                if len(failed_tests) < 8:
                    failed_tests.append(name)
            elif test.get("result") == "SKIPPED" or test.get("status") == "NOTRUN":
                skipped += 1
                lacks = False
                for note in test.get("skipped", []):
                    reason = skip_reason(note.get("message", ""))
                    if reason and reason.startswith(unavailable_marker):
                        reason = reason[len(unavailable_marker) :].strip()
                        unavailable_reasons[reason or "no reason given"] += 1
                    elif reason and reason.startswith(lacks_marker):
                        reason = reason[len(lacks_marker) :].strip()
                        lacking_reasons[reason or "no reason given"] += 1
                        lacks = True
                    elif reason:
                        skip_reasons[reason] += 1
                        if version_test and NO_VERSION.search(reason):
                            reports_no_version = True
                if lacks:
                    lacking += 1
                    if core:
                        core_lacking.append(name)
            else:
                passed += 1
    return {
        "passed": passed,
        "failed": failed,
        "skipped": skipped,
        "failed_tests": failed_tests,
        "load_error": load_error,
        "unavailable_reason": (
            unavailable_reasons.most_common(1)[0][0] if unavailable_reasons else None
        ),
        # Only the commonest: a suite that skips wholesale does so for one
        # reason, and that reason is the whole story of the row.
        "skip_reason": skip_reasons.most_common(1)[0][0] if skip_reasons else None,
        "lacking": lacking,
        # Every reason, commonest first: each names a different missing member.
        # They quote the library's path, which row_notes() strips.
        "lacking_reasons": dict(lacking_reasons.most_common()),
        "core_lacking": core_lacking,
        "reports_no_version": reports_no_version,
        **claims,
    }


# --------------------------------------------------------------------------
# driver


def select_configs(selection, include_commercial):
    unknown = sorted(selection - SOLVERS.keys()) if selection else []
    if unknown:
        sys.exit(
            "no solver {} in {}, which has {}".format(
                ", ".join(unknown), MANIFEST.name, ", ".join(SOLVERS)
            )
        )
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
    """Yield each row once its outcome is known.

    A row at a time, so that a solver whose run breaks off halfway still
    keeps the versions it got through.
    """
    source = config["source"]
    wanted = wrapper_symbols(config)
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
            yield row
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
            if not args.keep:
                discard(archive, tree)
            yield row
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
        try:
            row["dependencies"], unpinned = fetch_dependencies(
                source, tree, library.parent
            )
        except Exception as error:  # an index or a download
            row["status"] = "fetch-error"
            row["detail"] = "dependencies: {}".format(error)
            if not args.keep:
                discard(archive, tree)
            yield row
            continue
        if unpinned:
            row["unpinned"] = unpinned
        tested[identity] = row
        exported = exported_symbols(library)
        row["missing_symbols"] = sorted(wanted - exported) if exported else []
        row["unresolved"], row["host_libraries"] = resolve(library)
        if row["host_libraries"]:
            print("    from the host: {}".format(", ".join(row["host_libraries"])))

        outcome = run_tests(
            args.binary,
            config,
            library,
            RESULTS / config["name"] / "{}.json".format(version),
            args.timeout,
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
                row["status"] = "failing"
            elif row["unavailable_reason"]:
                row["status"] = "unavailable"
            elif row["passed"] == 0:
                row["status"] = "no-tests"
            else:
                row["status"] = "ok"

        if not args.keep:
            discard(archive, tree)
        yield row


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
    Nor do partially supported ones, even counting their other skips alone:
    those differ from a full release's too -- HiGHS below its native IIS floor
    runs the floor test a full release skips -- and the HiGHS releases without
    native IIS outnumber those with it, so their counts would set the
    yardstick, and either turn every full release yellow or hide one that lost
    a capability.
    """
    counts = [
        other_skips(r) for r in rows if r["status"] == "ok" and not r.get("lacking")
    ]
    return statistics.median_high(counts) if counts else 0


def other_skips(row):
    """The skips that are not a member the release lacks."""
    return (row.get("skipped") or 0) - (row.get("lacking") or 0)


def unusual_skips(row, baseline):
    """Whether this row skipped more than a clean pass should.

    Two ways to qualify: skipping tests that every other version of the same
    solver ran -- this version lost a capability -- or skipping so much of the
    suite that what did pass says little about the library. The tests of a
    member the release lacks are its partial support, not unusual skips.
    """
    skipped = other_skips(row)
    if not skipped:
        return False
    total = row.get("passed", 0) + row.get("failed", 0) + (row.get("skipped") or 0)
    return skipped > baseline or skipped >= SKIP_SHARE_WARN * total


def package_support(row):
    """The release_support of a library that reports no version, from its package.

    SoPlex never says which release it is, so its version test skips and the
    claim goes unchecked at runtime: only this reading of the package version
    holds it honest. None when the run recorded no claim, or the version
    shown is the binding's rather than the solver's.
    """
    if row.get("approximate") or "validated_versions" not in row:
        return None
    release = release_number(row.get("version", row["package_version"]))
    validated = claim_ranges(row["validated_versions"])
    supported = claim_ranges(row.get("supported_versions", row["validated_versions"]))
    if release is None or validated is None or supported is None:
        return None
    if any(start <= release < end for start, end in validated):
        return "validated"
    if any(start <= release < end for start, end in supported):
        return "partial"
    return "untested"


def claim_mismatch(row):
    """How a row that ran contradicts its backend's claim, or None.

    The version test fails on a release outside both lists; this catches what
    it cannot: the skips, read against the release_support it recorded, and
    the release of a library reporting no version.
    """
    if row.get("core_lacking"):
        return "the lp_model core lacks: {}".format(", ".join(row["core_lacking"][:3]))
    unversioned = row.get("reports_no_version")
    support = package_support(row) if unversioned else row.get("release_support")
    # On a release reporting a validated version the shared suites fail
    # rather than skip, so this mostly catches one that reports none.
    if support == "validated" and row.get("lacking"):
        return "declared validated, lacks members"
    if support == "partial" and not row.get("lacking") and not row["failed"]:
        return "declared partial, passes in full: move it into validated_versions"
    if support == "untested" and unversioned and not row["failed"]:
        return "unclaimed"
    return None


def status_cell(row, baseline=0):
    status = row["status"]
    if status in ("ok", "failing"):
        # Always the tests that passed, out of every test the run accounted
        # for; the marker, not the number, carries the verdict.
        if row["failed"] or claim_mismatch(row):
            mark = FAIL_MARK
        elif row.get("lacking"):
            mark = PARTIAL_MARK
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
        if row.get("missing_symbols"):
            return "{} will not load ({} symbols missing)".format(
                FAIL_MARK, len(row["missing_symbols"])
            )
        return "{} will not load".format(FAIL_MARK)
    if status == "unavailable":
        if LICENCE_ERROR.search(row["unavailable_reason"]):
            return "{} not tested (licence)".format(WARN_MARK)
        return "{} not tested".format(WARN_MARK)
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


def without_directories(text):
    """`text` with each absolute path cut down to its last component.

    Loader errors and skip reasons quote the library they loaded, and where it
    was extracted belongs to the machine that ran the matrix, not the release.
    A path outside the checkout with a space in it keeps its head.
    """
    text = text.replace(str(ROOT), "")
    return re.sub(r"(?<![\w./:*])/(?:[^\s'\"/:]+/)+", "", text)


def lacked_member(reason):
    """The clause of a lacking skip that names what the release lacks.

    The rest of the message, the library and the release it reports, the row
    already gives.
    """
    return re.sub(r"^mippp: ", "", reason).split(", ")[0]


def row_notes(row, source, baseline=0):
    notes = []
    span = package_span(row)
    if span:
        notes.append("{} {}".format(source["package"], span))
    if row.get("unresolved"):
        notes.append("needs {}".format(", ".join(row["unresolved"])))
    if row.get("unpinned"):
        notes.append("not pinned to its build: {}".format(", ".join(row["unpinned"])))
    if row["status"] == "unavailable":
        notes.append(row["unavailable_reason"])
    # Only when the load failed: a library that ran the suite resolved every
    # symbol by definition, whatever the static probe thinks.
    elif row["status"] == "load-error" and row.get("missing_symbols"):
        notes.append("missing: {}".format(", ".join(row["missing_symbols"][:4])))
    elif row["status"] == "load-error" and not row.get("unresolved"):
        notes.append(row.get("load_error") or "")
    elif row["status"] == "failing" and row.get("failed_tests"):
        # The version test failing alone means everything else passed on a
        # release outside both claims: the claim is the story, not a failure.
        if row["failed"] == 1 and VERSION_TEST.search(row["failed_tests"][0]):
            notes.append("unclaimed")
        else:
            notes.append("failing: {}".format(", ".join(row["failed_tests"][:3])))
    if row["status"] in ("ok", "failing"):
        notes.append(claim_mismatch(row))
    if row.get("lacking"):
        notes.append(
            "lacks: {} ({} test{})".format(
                " / ".join(dict.fromkeys(map(lacked_member, row["lacking_reasons"]))),
                row["lacking"],
                "" if row["lacking"] == 1 else "s",
            )
        )
    skipped = other_skips(row)
    if skipped and row["status"] != "unavailable":
        note = "{} {}skipped".format(skipped, "more " if row.get("lacking") else "")
        # Why, but only when the skips are the story: one capability test
        # bowing out explains itself, a whole suite doing so does not. A
        # backend that refused to run accounts for nearly every skip, also on
        # a row that failed its version test.
        reason = row.get("unavailable_reason") or row.get("skip_reason")
        if reason and (row["status"] == "no-tests" or unusual_skips(row, baseline)):
            note += ": {}".format(reason)
        notes.append(note)
    if row.get("detail"):
        notes.append(row["detail"])
    return without_directories("; ".join(n for n in notes if n)) or "-"


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
        "<!-- Generated by misc/tools/compat_matrix.py -- do not edit by hand. -->",
        "",
        "MIP++ drives each solver through one implementation that adapts at",
        "runtime to a range of releases, so the question this table answers",
        "is: **which released versions does that implementation drive, and",
        "how far?** Its rows are the evidence behind the two claims a backend",
        "declares: `validated_versions`, the releases every test passes on,",
        "and the wider `supported_versions`, those the model core passes on",
        "while the members a release lacks throw",
        "`mippp::feature_unavailable_error`. A backend without partially",
        "supported releases declares the first alone. The",
        "`loaded_release_is_a_supported_one` test fails on a row whose",
        "release lies outside both, and the table checks the rest of the",
        "claim.",
        "",
        "Each row is a published library, downloaded and pointed at through",
        "`MIPPP_<solver>_LIBRARY`, with the backend's test suites run against",
        "it on Linux/x86-64. The result column counts the tests that passed",
        "out of every test accounted for:",
        "",
        "- {} every test passed".format(PASS_MARK),
        "- {} partially supported: the model core passed, and the tests of "
        "the members the release lacks skipped -- the notes say which".format(
            PARTIAL_MARK
        ),
        "- {} some test failed, or the row contradicts the claim: a release "
        "outside both lists (unclaimed), a member the model core lacks, or a "
        "tier the run disagrees with -- the notes say which".format(FAIL_MARK),
        "- {} passed, but an unusual number of tests skipped, or nothing ran "
        "at all -- the notes say which".format(WARN_MARK),
        "",
        "Totals differ between backends, since each runs only the suites its",
        "model types support; a capability checked at runtime skips, and is",
        "counted in the total. The fuzz seed is pinned so that rows differ only",
        "by the library under test. A library that reports no version, such as",
        "SoPlex, is held against the claim by its package version.",
        "",
    ]
    for name in sorted(by_solver):
        config = SOLVERS[name]
        source = config["source"]
        lines.append("## {}".format(config["title"]))
        lines.append("")
        lines.append(
            "Implementation `{}`, library from `{}` ({}).".format(
                config["wrapper"], source["package"], source["kind"]
            )
        )
        if config.get("note"):
            lines.append("")
            lines.append("!!! warning")
            lines.append("    {}{}.".format(config["note"][:1].upper(), config["note"][1:]))
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


def merge(earlier, rows):
    """`rows` over the rows of earlier runs.

    So re-running one solver keeps the rest of the table instead of silently
    emptying it. A row stands for every package shipping its library, and the
    newest of those moves on between runs: a package this run tested retires
    the earlier row it named, whichever package now names the new one.
    """
    refreshed = {
        (row["solver"], version)
        for row in rows
        for version in [row["package_version"], *row.get("also_packages", [])]
    }
    return [
        row
        for row in earlier
        if (row["solver"], row["package_version"]) not in refreshed
    ] + rows


def save(rows, results_file):
    results_file.parent.mkdir(parents=True, exist_ok=True)
    results_file.write_text(json.dumps(rows, indent=2))


def load(results_file):
    """The rows of earlier runs, in the vocabulary this script writes.

    Rows written before supported_versions existed give a run with a failing
    test the status "partial", the word now kept for partial support, which is
    no status: status_cell() would print the bare word instead of a cross.
    """
    rows = json.loads(results_file.read_text())
    for row in rows:
        if row.get("status") == "partial":
            row["status"] = "failing"
    return rows


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
        render(load(results_file), args.output)
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
    # Fatal, and before any download: a solver skipped here would lose its
    # section of the table behind a run that looks successful.
    absent = [config["name"] for config in configs if not compiled_in(args.binary, config)]
    if absent:
        sys.exit(
            "{} not in {}: its gtest_filter selects no "
            "*_api.loaded_release_is_a_supported_one test (named "
            "loaded_release_is_a_validated_one in older binaries). Either the "
            "build left it out -- TEST_SOURCE is sticky in the CMake cache until "
            "the build directory is removed -- or the filter in {} is wrong; "
            "--solvers runs a subset.".format(
                ", ".join(absent), args.binary.name, MANIFEST.name
            )
        )

    earlier = load(results_file) if results_file.exists() else []
    rows = []
    broken = []
    for config in configs:
        print("{}:".format(config["name"]), flush=True)
        try:
            for row in process(config, select_versions(config["source"], args.limit), args):
                rows.append(row)
                # After every row, so that an interrupted run keeps what it tested.
                save(merge(earlier, rows), results_file)
        except Exception:  # an index or a dependency that would not download
            traceback.print_exc()
            broken.append(config["name"])
    merged = merge(earlier, rows)
    save(merged, results_file)
    print("wrote {} ({} rows)".format(results_file, len(merged)))
    render(merged, args.output)
    if broken:
        sys.exit(
            "{} broke off, see above: the table keeps their rows from earlier "
            "runs".format(", ".join(broken))
        )


if __name__ == "__main__":
    main()
