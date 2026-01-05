"""
Merge form responses into trajectory metrics.

The mapping rule is defined by the study setup:
- Form responses are assigned reverse-chronologically to trajectories: last form -> last trajectory.
- The simulation version decides whether answers go to the S (sound) or NS (no sound) entry.
- Answers Q0..Q5 are copied as integers 1..5 into the matched trajectory entry.

Usage:
    python data_merger.py [--csv FORM_CSV] [--metrics RUN_METRICS_JSON]
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Dict, Iterable, List, Tuple


DEFAULT_CSV = "Drone Simulation Acoustic Feedback Form (réponses) - Réponses au formulaire 1.csv"
DEFAULT_METRICS = "run_metrics.json"


def normalize(text: str) -> str:
    """Normalize header text for matching."""
    return " ".join(text.strip().replace("\u2019", "'").split()).lower()


# Expected headers mapped to Q-index order.
QUESTION_HEADERS = [
    "How familiar are you with controling drones/vehicles in simulations/video games?",
    "How aware were you of the position and movement of other drones within the swarm ?",
    "How aware were you of the position of obstacles around the swarm ?",
    "How did you find avoiding obstacles on the course?",
    "How aware were you of the swarm’s alignment with the gaps ?",
    "What was the perceived cognitive load of the simulation",
]

VERSION_HEADER = "Which version of the simulation did you just do"


def resolve_headers(fieldnames: Iterable[str]) -> Tuple[str, List[str]]:
    """Map normalized expected headers to actual CSV headers."""
    normalized = {normalize(name): name for name in fieldnames}

    try:
        version_header = normalized[normalize(VERSION_HEADER)]
    except KeyError as exc:
        raise KeyError("Version column not found in CSV headers") from exc

    question_headers = []
    for header in QUESTION_HEADERS:
        norm = normalize(header)
        if norm not in normalized:
            raise KeyError(f"Question header missing in CSV: {header}")
        question_headers.append(normalized[norm])

    return version_header, question_headers


def read_form_responses(csv_path: Path) -> List[Dict]:
    """Read ordered form responses with user id, version, and Q0..Q5 answers."""
    with csv_path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames is None:
            raise ValueError("CSV appears to be empty or missing headers")

        version_header, question_headers = resolve_headers(reader.fieldnames)

        responses = []
        for idx, row in enumerate(reader, start=1):
            version = row[version_header].strip()
            user_id_raw = row.get("User ID")
            if user_id_raw is None:
                raise KeyError("CSV missing 'User ID' column")
            try:
                user_id = int(str(user_id_raw).strip())
            except ValueError as exc:
                raise ValueError(f"Non-numeric User ID in row {idx}: {user_id_raw}") from exc
            answers = {}
            for q_index, header in enumerate(question_headers):
                raw_value = row.get(header, "").strip()
                if not raw_value:
                    continue
                try:
                    answers[f"Q{q_index}"] = int(raw_value)
                except ValueError as exc:
                    raise ValueError(f"Non-numeric answer in row {idx}, column {header}") from exc
            responses.append({"row_id": idx, "user_id": user_id, "version": version, "answers": answers})
    return responses


def trajectory_order(metrics: Dict[str, Dict]) -> List[str]:
    """Return trajectory ids sorted 0_0, 0_1, ... then reversed for mapping."""
    def key_fn(item: str) -> Tuple[int, int]:
        batch, run = item.split("_")
        return int(batch), int(run)

    return sorted(metrics.keys(), key=key_fn)


def target_key(version_text: str) -> str:
    """Decide whether answers belong to S or NS based on version text."""
    if version_text=="Run with audio feedback":
        return "S"
    if version_text=="Run without audio feedback":
        return "NS"
    raise ValueError(f"Cannot determine target (S/NS) from version text: {version_text}")


def merge_answers(
    metrics: Dict[str, Dict],
    responses: List[Dict],
) -> Dict[str, Dict]:
    """
    Inject Q0..Q5 answers into metrics following reverse mapping rule.

    Mapping rule: highest user id -> last trajectory id, next highest -> previous trajectory id, etc.
    Each user id is expected to have both versions; answers go to S/NS based on version text.
    """
    traj_ids = trajectory_order(metrics)
    if not traj_ids:
        raise ValueError("No trajectory ids found in run_metrics.json")

    # Group responses by user id
    grouped: Dict[int, List[Dict]] = {}
    for resp in responses:
        grouped.setdefault(resp["user_id"], []).append(resp)

    ordered_user_ids = sorted(grouped.keys())
    usable_count = min(len(ordered_user_ids), len(traj_ids))

    user_ids_to_use = ordered_user_ids[-usable_count:]
    traj_ids_to_use = traj_ids[-usable_count:]

    for traj_id, user_id in zip(reversed(traj_ids_to_use), reversed(user_ids_to_use)):
        entry = metrics.get(traj_id)
        if entry is None:
            continue
        for resp in grouped.get(user_id, []):
            dest_key = target_key(resp["version"])
            if dest_key not in entry:
                raise KeyError(f"Missing '{dest_key}' section for trajectory {traj_id}")
            entry[dest_key].update(resp["answers"])
    return metrics


def merge_data():
    """Merge using the default CSV and metrics file paths."""
    csv_path = Path(DEFAULT_CSV)
    metrics_path = Path(DEFAULT_METRICS)

    responses = read_form_responses(csv_path)
    # print(json.dumps(responses, indent=2))
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))

    return merge_answers(metrics, responses)

if __name__ == "__main__":
    merged = merge_data()
