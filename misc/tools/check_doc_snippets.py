#!/usr/bin/env python3
"""Fails unless every documentation snippet section has one start and one end.

A page includes a section of test/doc_snippets/<page>.cpp by name, and the
snippets extension copies the lines between `--8<-- [start:<name>]` and
`--8<-- [end:<name>]`. A start without its end does not fail the docs build: the
page silently shows the rest of the file, test code included. Nor does a
second start of the same name, which cuts the section short.
"""
import argparse
import pathlib
import re
import sys

MARKER_RE = re.compile(r"--8<--\s*\[(start|end):([^\]]+)\]")


def check(path):
    errors, open_at, closed = [], {}, set()
    for number, line in enumerate(path.read_text().splitlines(), start=1):
        for kind, name in MARKER_RE.findall(line):
            where = f"{path}:{number}"
            if kind == "start":
                if name in open_at or name in closed:
                    errors.append(f"{where}: second start of section '{name}'")
                else:
                    open_at[name] = where
            elif name in open_at:
                del open_at[name]
                closed.add(name)
            else:
                errors.append(f"{where}: end of section '{name}' without a start")
    errors += [
        f"{where}: section '{name}' has no end" for name, where in open_at.items()
    ]
    return errors


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("directory", type=pathlib.Path)
    args = parser.parse_args()
    errors = []
    for path in sorted(args.directory.glob("*.cpp")):
        errors += check(path)
    for error in errors:
        print(error)
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
