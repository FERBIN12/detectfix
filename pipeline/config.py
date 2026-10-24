"""Runtime + secret configuration.

Follows the repo convention (navbot/config.py): everything secret comes from
.env so nothing sensitive lives in code or config.json. Non-secret runtime
settings come from config.json. CLI flags override config.json.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

try:
    from dotenv import load_dotenv
    load_dotenv()
except ImportError:  # python-dotenv is optional; env may be set by the shell
    pass

HERE = Path(__file__).resolve().parent.parent     # the bridge-inspection/ folder


# --- Secrets (from .env) ------------------------------------------------------
# The whole point of the pipeline is to use the Claude API for the report and
# dashboard agents instead of the repo's local Qwen/vLLM. The interface stays
# model-agnostic (a local model remains a fallback), but Claude is the default.
ANTHROPIC_API_KEY = os.getenv("ANTHROPIC_API_KEY", "")
CLAUDE_MODEL = os.getenv("CLAUDE_MODEL", "claude-sonnet-4-5")
CLAUDE_BASE_URL = os.getenv("CLAUDE_BASE_URL", "https://api.anthropic.com").rstrip("/")
CLAUDE_TIMEOUT = float(os.getenv("CLAUDE_TIMEOUT", "30"))
CLAUDE_MAX_TOKENS = int(os.getenv("CLAUDE_MAX_TOKENS", "1500"))

# Local-model fallback (same shape as navbot/llm.py's OpenAI-compatible client).
FALLBACK_LLM_URL = os.getenv("FALLBACK_LLM_URL", "http://127.0.0.1:8000/v1").rstrip("/")
FALLBACK_LLM_MODEL = os.getenv("FALLBACK_LLM_MODEL", "")
FALLBACK_LLM_API_KEY = os.getenv("FALLBACK_LLM_API_KEY", "")

# Which agent backend to use: "claude" (default) or "local".
AGENT_BACKEND = os.getenv("AGENT_BACKEND", "claude").strip().lower()


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
