#!/usr/bin/env python3
"""PreToolUse hook: block a git add/commit that would put a .env file in git.

Kiro pipes the pending tool call to stdin as JSON; every string in it is
scanned. Exit 0 = allow, exit 2 = block (stderr is shown to the agent).
Commands that are not git add/commit/stage exit 0 immediately.
"""
import json
import re
import subprocess
import sys

# .env, .env.local, config/.env.prod ... but not .env.example / .env.sample
SECRET_RE = re.compile(r"(^|[\\/\s\"'])\.env(\.(?!example\b|sample\b|template\b)[\w.-]+)?($|[\s\"'])")
GIT_RE = re.compile(r"\bgit\b[^;&|\n]*\b(add|commit|stage)\b[^;&|\n]*")


def _strings(obj):
    if isinstance(obj, str):
        yield obj
    elif isinstance(obj, dict):
        for v in obj.values():
            yield from _strings(v)
    elif isinstance(obj, list):
        for v in obj:
            yield from _strings(v)


def _git(*args):
    try:
        p = subprocess.run(["git", *args], capture_output=True, text=True, timeout=10)
        return p.returncode, p.stdout
    except (OSError, subprocess.SubprocessError):
        return 1, ""


def _secret(path):
    return bool(SECRET_RE.search(" " + path.strip() + " "))


def main() -> int:
    raw = sys.stdin.read()
    try:
        payload = json.loads(raw) if raw.strip() else {}
    except json.JSONDecodeError:
        payload = raw
    command = "\n".join(_strings(payload))
    if not GIT_RE.search(command):
        return 0
    reasons = []
    for m in GIT_RE.finditer(command):
        seg = m.group(0)
        if _secret(seg):
            reasons.append(f"command names a .env file: {seg.strip()!r}")
        if re.search(r"\badd\b", seg) and re.search(r"(\s-f\b|--force\b)", seg):
            reasons.append(f"forced 'git add' can bypass .gitignore: {seg.strip()!r}")
    code, staged = _git("diff", "--cached", "--name-only")
    if code == 0:
        bad = [p for p in staged.splitlines() if _secret(p)]
        if bad:
            reasons.append(f".env already staged: {', '.join(bad)}")
    code, tracked = _git("ls-files")
    if code == 0:
        bad = [p for p in tracked.splitlines() if _secret(p)]
        if bad:
            reasons.append(f".env is tracked by git: {', '.join(bad)}")
    if reasons:
        print("Blocked by .env guard hook: " + "; ".join(reasons), file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
