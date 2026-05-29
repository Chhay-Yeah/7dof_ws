"""On-disk persistence for the teach pendant.

Saved Jogging targets and Motion tasks live only in the GUI's memory while it
runs; this module gives them a home on disk so they survive an app restart.

State is a single JSON file under the user's config dir
(``$XDG_CONFIG_HOME/7dof_pendant/state.json``, falling back to
``~/.config/7dof_pendant/state.json``). Schema::

    {
      "targets": [{"name": str, "joints": [float*7], "xyz": [x, y, z]|null}],
      "tasks":   [{"name": str, "nodes": [...], "edges": [...]}],
      "target_seq": int,   # running counter for default posN names
      "task_seq":   int,   # running counter for default "Task N" names
    }
"""

from __future__ import annotations

import json
import os
from pathlib import Path


def _config_dir() -> Path:
    base = os.environ.get("XDG_CONFIG_HOME")
    root = Path(base) if base else Path.home() / ".config"
    return root / "7dof_pendant"


def state_path() -> Path:
    return _config_dir() / "state.json"


def load_state() -> dict:
    """Read the saved state. Returns an empty dict if the file is missing or
    unreadable — a corrupt file must never block the GUI from starting."""
    path = state_path()
    try:
        with path.open("r", encoding="utf-8") as fh:
            data = json.load(fh)
        if isinstance(data, dict):
            return data
    except (OSError, ValueError):
        pass
    return {}


def save_state(state: dict) -> None:
    """Write state atomically (temp file + replace) so a crash mid-write can't
    truncate the existing file."""
    path = state_path()
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".json.tmp")
        with tmp.open("w", encoding="utf-8") as fh:
            json.dump(state, fh, indent=2)
        tmp.replace(path)
    except OSError:
        # Persistence is best-effort; never crash the GUI over a write failure.
        pass
