"""Console output in the same style as Fuse:

    === Stage: align ===
    [align] 5 feature-matching starts ...
      [!] warning text

Everything printed is also appended to any log files currently attached (the
per-scan run.log), so a run's console output is reproducible from disk.
"""

from __future__ import annotations

import sys
import textwrap

_files: list = []
_verbose = False


def set_verbose(verbose: bool):
    global _verbose
    _verbose = verbose


def attach(path):
    fh = open(path, "a", encoding="utf-8")
    _files.append(fh)
    return fh


def detach(fh):
    if fh in _files:
        _files.remove(fh)
    fh.close()


def _emit(text: str, stream=None):
    print(text, file=stream or sys.stdout, flush=True)
    for fh in _files:
        fh.write(text + "\n")
        fh.flush()


def log(text: str = ""):
    _emit(text)


def debug(text: str):
    if _verbose:
        _emit(text)
    else:
        for fh in _files:
            fh.write(text + "\n")


def stage(name: str):
    _emit(f"\n=== Stage: {name} ===")


def warn(text: str):
    body = textwrap.fill(text, width=92, initial_indent="  [!] ", subsequent_indent="      ")
    _emit(body, sys.stderr)


def loud(text: str):
    """A warning that must not scroll past unnoticed (e.g. a units mismatch)."""
    bar = "!" * 78
    body = textwrap.fill(text, width=74, initial_indent="!! ", subsequent_indent="!! ")
    _emit(f"\n{bar}\n{body}\n{bar}\n", sys.stderr)
