# session_tf

Transformations and utilities built on [session_py](https://pypi.org/project/session-py/).

## Prerequisites

- [uv](https://docs.astral.sh/uv/) package manager
- Python 3.8+ (3.13 recommended)

## Setup

```bash
# Create a virtual environment
uv venv

# Activate the virtual environment
# macOS / Linux:
source .venv/bin/activate
# Windows (Git Bash):
source .venv/Scripts/activate

# Install the project and its dependencies in editable mode
uv pip install -e .
```

## Run the example

```bash
python src/example_box.py
```

## Development

```bash
# Install with dev dependencies
uv pip install -e ".[dev]"

# Run tests
pytest -v
```
