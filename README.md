# AeroPlanner Backend (FastAPI + PyMavlink)

Local backend with REST api on FastAPI utilizing pymavlink library and opencv for the hud

## Prerequisites
- [uv](https://github.com/astral-sh/uv) installed.
- Python 3.11. If you don't have Python 3.11 - you can install it:
```bash
uv python install 3.11
```


## One-time project setup
All dependencies are already installed, you only need to enter this command
```bash
uv sync
```

## Run project
```bash
uv run fastapi dev main.py
```