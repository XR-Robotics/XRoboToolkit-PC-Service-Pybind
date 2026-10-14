#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11"
# dependencies = []
# ///
"""Upload the release files in dist/ to PyPI (or TestPyPI) with `uv publish`.

    uv run scripts/publish.py             # upload to PyPI
    uv run scripts/publish.py --test      # upload to TestPyPI
    uv run scripts/publish.py --dry-run   # run every check, upload nothing

Authentication: set UV_PUBLISH_TOKEN to an API token from
https://pypi.org/manage/account/token/ (https://test.pypi.org/... for --test).
The first upload creates the project, so it needs an "Entire account" token;
afterwards a token scoped to the project is enough.

Before uploading, the script checks that dist/ holds only files PyPI accepts
for the version in pyproject.toml, that the metadata renders (twine check) and
that the wheels stay within the stable ABI (abi3audit). Files already on the
index are skipped, so re-running after a partial upload is safe.
"""

from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# name -> (upload endpoint, simple index used to skip files already uploaded)
INDEXES = {
    "PyPI": ("https://upload.pypi.org/legacy/", "https://pypi.org/simple/"),
    "TestPyPI": ("https://test.pypi.org/legacy/", "https://test.pypi.org/simple/"),
}

# One abi3 wheel per supported platform (see vendor/README.md).
EXPECTED_PLATFORMS = ("win_amd64", "x86_64", "aarch64")


def run(*cmd: str | Path) -> None:
    print("+", " ".join(str(c) for c in cmd), flush=True)
    if subprocess.run([str(c) for c in cmd]).returncode != 0:
        sys.exit(f"error: {cmd[0]} {cmd[1]} failed")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--test", action="store_true", help="upload to TestPyPI instead of PyPI")
    parser.add_argument("--dry-run", action="store_true", help="run every check, upload nothing")
    parser.add_argument("--yes", action="store_true", help="do not ask for confirmation")
    parser.add_argument("--dist", type=Path, default=ROOT / "dist", help="directory holding the files (default: dist/)")
    args = parser.parse_args()

    project = tomllib.loads((ROOT / "pyproject.toml").read_text(encoding="utf-8"))["project"]
    stem = f"{project['name'].replace('-', '_')}-{project['version']}"
    sdist_name = f"{stem}.tar.gz"
    wheel_re = re.compile(rf"{re.escape(stem)}-[^-]+-(?P<abi>[^-]+)-(?P<platform>[^-]+)\.whl")

    files = sorted(p for p in args.dist.glob("*") if p.suffix == ".whl" or p.name.endswith(".tar.gz"))
    if not files:
        sys.exit(f"error: no wheels or sdists in {args.dist}")

    problems, wheels, platforms = [], [], []
    for f in files:
        if f.name == sdist_name:
            continue
        match = wheel_re.fullmatch(f.name)
        if match is None:
            problems.append(f"{f.name}: not a {stem} file (left over from another version?)")
        elif match["platform"].startswith("linux_"):
            problems.append(f"{f.name}: PyPI rejects linux_* wheels; upload the auditwheel-repaired manylinux one")
        elif match["abi"] != "abi3":
            problems.append(f"{f.name}: not an abi3 wheel")
        else:
            wheels.append(f)
            platforms.append(match["platform"])
    if problems:
        print(f"error: remove or rebuild these files in {args.dist}:", *problems, sep="\n  ", file=sys.stderr)
        return 1

    missing = [p for p in EXPECTED_PLATFORMS if not any(t.endswith(p) for t in platforms)]
    if sdist_name not in (f.name for f in files):
        missing.append("sdist")
    print(f"{stem}: {len(files)} files")
    for f in files:
        print(f"  {f.name}  ({f.stat().st_size / 1e6:.1f} MB)")
    if missing:
        print(f"warning: no file for {', '.join(missing)}; they can still be added to the release later")

    run("uvx", "twine", "check", "--strict", *files)
    if wheels:
        run("uvx", "abi3audit", "--strict", *wheels)

    target = "TestPyPI" if args.test else "PyPI"
    upload_url, check_url = INDEXES[target]
    publish = ["uv", "publish", "--trusted-publishing", "never",
               "--publish-url", upload_url, "--check-url", check_url, *files]
    if args.dry_run:
        run(*publish, "--dry-run")
        print(f"dry run OK: nothing was uploaded to {target}")
        return 0

    if not os.environ.get("UV_PUBLISH_TOKEN"):
        sys.exit(f"error: set UV_PUBLISH_TOKEN to a {target} API token (see the docstring of {Path(__file__).name})")
    if not args.yes:
        try:
            answer = input(f"Upload {len(files)} files to {target}? Released files can never be replaced. [y/N] ")
        except EOFError:  # no interactive stdin
            print()
            answer = ""
        if answer.strip().lower() not in ("y", "yes"):
            sys.exit("aborted: nothing was uploaded (pass --yes to skip this question)")
    run(*publish)
    print(f"uploaded: https://{'test.' if args.test else ''}pypi.org/project/{project['name']}/{project['version']}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
