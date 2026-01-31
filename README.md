# Robohack

## Meet the team!
Yifan, Tim, Carlneil, Vai

## Quick Install 

Getting started is easy with `uv`. This will set up your virtual environment and install all dependencies (including local packages) in one go.

```bash
# Install dependencies and setup venv
uv sync
```

To activate the environment:
```bash
source .venv/bin/activate
```

### Prerequisites

**1. Install `uv`** (if you don't have it):
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

**2. Python 3.10**:
`uv` will automatically try to find or download Python 3.10 for you. However, if you are on macOS and want to manage it via Homebrew:
```bash
brew install python@3.10
```

---
*Note: This project uses editable installs for `lerobot` and `solo-cli`. Changes made in those directories will reflect immediately in your environment.*

# ==========================================
# QUICK START COMMANDS
# ==========================================
```bash
uv sync
source .venv/bin/activate
```

