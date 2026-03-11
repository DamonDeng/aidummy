# EXP-008 — Named Sequence Database: CRUD + Persistent Storage

**Date:** 2026-03-07  
**Status:** ✅ Complete  
**Participants:** Damon (human), Squilla 🦐 (AI assistant)

---

## Objective

Build a server-side sequence library so that action sequences can be:
1. Created, updated, and deleted via API — no server restart needed
2. Played by name with optional runtime overrides
3. Persisted to disk — survive server restarts
4. Reused as building blocks for future commands

---

## Background

Previously, every call to `POST /robot/play_sequence` required passing the full
sequence JSON body. This was fine for one-off ad-hoc sequences, but made it
impossible to save, reuse, or iterate on named sequences without writing code.

A server-side database solves this cleanly.

---

## Architecture

### Storage: `sequences.json`

A JSON file on disk next to `robot_server.py`. Loaded into an in-memory dict
at startup, flushed on every write. Human-readable and manually editable.

```
robot-server/
  robot_server.py
  sequences.json   ← new, auto-created on first save
```

### Data Model

```json
{
  "elbow-wave": {
    "name": "elbow-wave",
    "description": "Friendly elbow wave, 3 reps",
    "speed": 20.0,
    "settle_threshold": 0.5,
    "settle_timeout": 15.0,
    "steps": [...],
    "created_at": "2026-03-07T06:38:51Z",
    "updated_at": "2026-03-07T06:38:51Z"
  }
}
```

**Name rules:** lowercase, alphanumeric + hyphens/underscores, max 64 chars.
Valid examples: `wave`, `pick-up`, `home_to_rest`.

---

## API Endpoints

| Method | Endpoint | Description |
|---|---|---|
| GET | `/sequences` | List all sequences (summary — no steps) |
| POST | `/sequences` | Create a new named sequence |
| GET | `/sequences/{name}` | Get full definition including steps |
| PUT | `/sequences/{name}` | Partial update (only provided fields change) |
| DELETE | `/sequences/{name}` | Delete permanently |
| POST | `/sequences/{name}/play` | Play by name with optional overrides |

### Play overrides

`POST /sequences/{name}/play` accepts an optional body to override saved
parameters at runtime without modifying the stored definition:

```json
{"speed": 30, "settle_timeout": 10}
```

---

## Code Changes

**File:** `3.Software/robot-server/robot_server.py`

| Change | Description |
|---|---|
| `import threading, json, re, datetime, pathlib` | New imports |
| `_SEQ_FILE`, `_seq_lock`, `_sequences` | DB globals |
| `_load_sequences()` | Loads JSON from disk at startup |
| `_save_sequences()` | Flushes to disk on every write (inside lock) |
| `_now_iso()` | UTC timestamp helper |
| `CreateSequenceBody` | Pydantic model for POST /sequences |
| `UpdateSequenceBody` | Pydantic model for PUT /sequences/{name} |
| `PlaySequenceOverrideBody` | Pydantic model for play overrides |
| `_run_sequence(body)` | **Extracted** core execution engine (was inline in play_sequence) |
| `play_sequence` | Now delegates to `_run_sequence()` |
| 6 new CRUD + play endpoints | Full sequence library API |
| `_load_sequences()` call | Called at module level after lifespan definition |

### Key refactor: `_run_sequence()`

The core sequence execution logic was extracted from `play_sequence` into a
standalone `_run_sequence(body: PlaySequenceBody) -> dict` function. Both
`play_sequence` and `sequences/{name}/play` now call it. This ensures identical
behaviour for ad-hoc and named sequences.

---

## Test Results

| Test | Result |
|---|---|
| `POST /sequences` (create `elbow-wave`) | Created ✅ |
| `GET /sequences` | Listed with metadata ✅ |
| `PUT /sequences/elbow-wave` (update description) | Updated ✅ |
| `GET /sequences/elbow-wave` (full def) | Steps returned correctly ✅ |
| `POST /sequences/elbow-wave/play` | 3 reps, 6 steps, 27s ✅ |
| Server restart | `elbow-wave` still present ✅ |
| `sequences.json` on disk | Human-readable, correct ✅ |

### Real sequence test: `home-to-rest`

Created and played a 3-step wind-down sequence:

```json
{
  "name": "home-to-rest",
  "description": "Home → Z-Home → Rest: full wind-down sequence",
  "speed": 15,
  "steps": [
    {"move_j": [0, 0, 90, 0, 0, 0],    "comment": "home",   "hold": 1.0},
    {"move_j": [0, -45, 140, 0, 0, 0], "comment": "z-home", "hold": 1.0},
    {"move_j": [0, -73, 180, 0, 0, 0], "comment": "rest",   "hold": 0.5}
  ]
}
```

| Step | Settle time |
|---|---|
| home | 5.14s |
| z-home | 5.14s |
| rest | 4.31s |
| **Total** | **17.47s** |

Video recorded and sent to Damon via Feishu ✅

---

## Lessons Learned

- **JSON file > SQLite for this scale** — no dependencies, human-readable,
  easy to inspect and edit manually, trivial to back up
- **Extract shared execution logic** — `_run_sequence()` is cleaner than
  duplicating the full loop inside every endpoint
- **`threading.Lock()` is enough** — FastAPI thread pool + GIL + simple dict
  operations, no need for async locks here
- **Partial updates via PUT** — returning 404 on unknown name and only updating
  provided fields keeps the API predictable

---

## Sequences in Library (as of EXP-008)

| Name | Description | Steps | Speed |
|---|---|---|---|
| `elbow-wave` | Elbow wave — 3 reps | 1 (repeat block) | 20°/s |
| `home-to-rest` | Home → Z-Home → Rest wind-down | 3 | 15°/s |

---

## Next Ideas

- **Sequence chaining**: a step type `{"run_sequence": "wave"}` to call named
  sequences as sub-steps — building complex behaviours from reusable blocks
- **Tags/categories**: group sequences by type (greeting, demo, utility)
- **Sequence versioning**: keep history of edits
- **Export/import**: bulk import from a JSON file for sharing sequences
