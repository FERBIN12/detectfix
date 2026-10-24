"""Runtime + secret configuration.

Follows the repo convention (navbot/config.py): everything secret comes from
.env so nothing sensitive lives in code or config.json. Non-secret runtime
settings come from config.json. CLI flags override config.json.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

HERE = Path(__file__).resolve().parent.parent     # the bridge-inspection/ folder

try:
    from dotenv import load_dotenv
    # Load the .env that sits in the bridge-inspection folder explicitly, so it
    # is found regardless of the current working directory. Fall back to the
    # default search too.
    load_dotenv(HERE / ".env")
    load_dotenv()
except ImportError:  # python-dotenv is optional; env may be set by the shell
    pass


# --- Secrets (from .env) ------------------------------------------------------
# The whole point of the pipeline is to use the Claude API for the report and
# dashboard agents instead of the repo's local Qwen/vLLM. The interface stays
# model-agnostic (a local model remains a fallback), but Claude is the default.
ANTHROPIC_API_KEY = os.getenv("ANTHROPIC_API_KEY", "")
CLAUDE_MODEL = os.getenv("CLAUDE_MODEL", "claude-sonnet-5-5")
CLAUDE_BASE_URL = os.getenv("CLAUDE_BASE_URL", "https://api.anthropic.com").rstrip("/")
CLAUDE_TIMEOUT = float(os.getenv("CLAUDE_TIMEOUT", "30"))
CLAUDE_MAX_TOKENS = int(os.getenv("CLAUDE_MAX_TOKENS", "3000"))

# Local-model fallback (same shape as navbot/llm.py's OpenAI-compatible client).
FALLBACK_LLM_URL = os.getenv("FALLBACK_LLM_URL", "http://127.0.0.1:8000/v1").rstrip("/")
FALLBACK_LLM_MODEL = os.getenv("FALLBACK_LLM_MODEL", "")
FALLBACK_LLM_API_KEY = os.getenv("FALLBACK_LLM_API_KEY", "")

# Which agent backend to use: "claude" (default), "local", or "kiro".
AGENT_BACKEND = os.getenv("AGENT_BACKEND", "claude").strip().lower()

# Kiro CLI headless backend (opt-in, AGENT_BACKEND=kiro). Uses the Kiro
# subscription's credits; auth is KIRO_API_KEY in the environment or an
# existing `kiro-cli login` session. No tools are trusted, so the agent can
# only answer from the prompt.
KIRO_CLI = os.getenv("KIRO_CLI", "kiro-cli")
KIRO_MODEL = os.getenv("KIRO_MODEL", "")          # e.g. claude-sonnet-4.6; "" = account default
KIRO_TIMEOUT = float(os.getenv("KIRO_TIMEOUT", "180"))


def load_config(path: str | None = None, mode: str | None = None, port: int | None = None) -> dict:
    """Read config.json (BOM-tolerant, like the dashboard), apply CLI overrides."""
    cfg_path = Path(path) if path else (HERE / "config.json")
    cfg = json.loads(cfg_path.read_text(encoding="utf-8-sig"))
    if mode:
        cfg["mode"] = mode
    if port:
        cfg["port"] = port
    # Resolve relative data paths against the bridge-inspection folder.
    for key in ("mcap_path", "prior_dir", "data_dir"):
        val = (cfg.get("live") or {}).get(key)
        if val and not os.path.isabs(val):
            cfg["live"][key] = str((HERE / val).resolve())
    return cfg


def claude_ready() -> bool:
    return bool(ANTHROPIC_API_KEY)
