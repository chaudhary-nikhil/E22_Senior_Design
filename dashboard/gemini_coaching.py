"""
Server-side Gemini coaching for GoldenForm Insights.
Requires GEMINI_API_KEY in the environment (or dashboard/.env loaded by wifi_session_processor).
Never commit API keys — use .env (gitignored).

Production:
- Rate limiting: coaching_rate_allow() (per authenticated user, in-memory).
- Logging: GEMINI_COACHING_LOG=off|audit|debug — client may send X-GoldenForm-Coaching-NoLog: 1 to skip any logging.
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from typing import Any, Dict, List, Optional, Tuple

_logger = logging.getLogger("goldenform.coaching")

_RL_LOCK = threading.Lock()
# user_id -> monotonic list of request timestamps (seconds) for sliding window
_RL_HITS: Dict[int, List[float]] = {}


def coaching_rate_allow(user_id: int) -> Tuple[bool, Optional[str]]:
    """
    Returns (allowed, error_message). Uses sliding window per user (thread-safe).
    Env: GEMINI_COACHING_RL_WINDOW_SEC (default 900), GEMINI_COACHING_RL_MAX (default 12).
    Set GEMINI_COACHING_RL_MAX=0 to disable rate limiting (not recommended for production).
    """
    try:
        max_req = int(os.environ.get("GEMINI_COACHING_RL_MAX", "12"))
    except ValueError:
        max_req = 12
    if max_req <= 0:
        return True, None
    try:
        window = float(os.environ.get("GEMINI_COACHING_RL_WINDOW_SEC", "900"))
    except ValueError:
        window = 900.0
    if window <= 0:
        window = 900.0

    now = time.time()
    with _RL_LOCK:
        hits = _RL_HITS.setdefault(int(user_id), [])
        cutoff = now - window
        while hits and hits[0] < cutoff:
            hits.pop(0)
        if len(hits) >= max_req:
            oldest = hits[0]
            retry_after = int(max(1.0, oldest + window - now))
            return False, (
                f"Too many AI coaching requests. Limit is {max_req} per {int(window // 60)} minutes. "
                f"Try again in about {max(1, retry_after // 60)} min."
            )
        hits.append(now)
    return True, None


def _coaching_log_mode() -> str:
    """off | audit | debug — default off (no prompt/response logging)."""
    raw = (os.environ.get("GEMINI_COACHING_LOG") or "off").strip().lower()
    if raw in ("debug", "verbose"):
        return "debug"
    if raw in ("audit", "1", "true", "yes"):
        return "audit"
    return "off"


def _ensure_coaching_log_handler() -> None:
    """So audit/debug lines appear without configuring the whole app."""
    if _logger.handlers:
        return
    if _coaching_log_mode() == "off":
        return
    h = logging.StreamHandler()
    h.setFormatter(logging.Formatter("%(levelname)s [goldenform.coaching] %(message)s"))
    _logger.addHandler(h)
    _logger.setLevel(logging.INFO)


def _maybe_log_coaching(
    *,
    privacy_no_log: bool,
    user_id: Optional[int],
    prompt: str,
    ok: bool,
    err: Optional[str],
    response_preview: Optional[str],
) -> None:
    if privacy_no_log:
        return
    mode = _coaching_log_mode()
    if mode == "off":
        return
    _ensure_coaching_log_handler()
    uid = user_id if user_id is not None else -1
    if mode == "audit":
        _logger.info(
            "coaching_insights user_id=%s ok=%s prompt_chars=%s err=%s",
            uid,
            ok,
            len(prompt),
            (err or "")[:200],
        )
        return
    # debug — truncated; never log full responses by default
    _logger.info(
        "coaching_insights user_id=%s ok=%s prompt_snip=%s err=%s out_snip=%s",
        uid,
        ok,
        (prompt[:500] + ("…" if len(prompt) > 500 else "")).replace("\n", " "),
        (err or "")[:300],
        ((response_preview or "")[:400] + ("…" if response_preview and len(response_preview) > 400 else "")).replace("\n", " "),
    )


def _model_name() -> str:
    # Prefer a widely-available default. Allow overrides via GEMINI_MODEL.
    return os.environ.get("GEMINI_MODEL", "gemini-2.0-flash").strip() or "gemini-2.0-flash"


def _refresh_gemini_key_from_dotenv() -> None:
    """If GEMINI_API_KEY is still empty, load dashboard/.env (same folder as this module)."""
    if (os.environ.get("GEMINI_API_KEY") or "").strip():
        return
    env_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env")
    if not os.path.isfile(env_path):
        return
    try:
        with open(env_path, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#") or "=" not in line:
                    continue
                k, _, v = line.partition("=")
                k, v = k.strip(), v.strip().strip('"').strip("'")
                if k == "GEMINI_API_KEY" and v:
                    os.environ["GEMINI_API_KEY"] = v
                    return
    except OSError:
        pass


def _call_gemini(api_key: str, prompt: str) -> str:
    model = _model_name()
    url = (
        f"https://generativelanguage.googleapis.com/v1beta/models/"
        f"{model}:generateContent?key={urllib.parse.quote(api_key, safe='')}"
    )
    payload = {
        "contents": [{"role": "user", "parts": [{"text": prompt}]}],
        "generationConfig": {
            "temperature": 0.35,
            "maxOutputTokens": 1400,
        },
    }
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        url,
        data=data,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urllib.request.urlopen(req, timeout=55) as resp:
        body = json.loads(resp.read().decode("utf-8"))
    candidates = body.get("candidates") or []
    if not candidates:
        raise RuntimeError(body.get("error", {}).get("message") or "No candidates from Gemini")
    parts = (candidates[0].get("content") or {}).get("parts") or []
    if not parts:
        raise RuntimeError("Empty content from Gemini")
    return (parts[0].get("text") or "").strip()


def _call_gemini_with_fallback(api_key: str, prompt: str) -> str:
    """
    Call Gemini; if the configured model 404s (common when model name drifts),
    retry a short fallback list.
    """
    primary = _model_name()
    try_models = [primary]
    # Only add fallbacks if they differ from primary.
    for m in ("gemini-flash-latest", "gemini-2.0-flash", "gemini-2.5-flash"):
        if m not in try_models:
            try_models.append(m)

    last_err: Optional[Exception] = None
    for idx, m in enumerate(try_models):
        if idx == 0:
            # use primary path
            try:
                return _call_gemini(api_key, prompt)
            except urllib.error.HTTPError as e:
                last_err = e
                if e.code != 404:
                    raise
                # retry with explicit model override below
        else:
            url = (
                f"https://generativelanguage.googleapis.com/v1beta/models/"
                f"{m}:generateContent?key={urllib.parse.quote(api_key, safe='')}"
            )
            payload = {
                "contents": [{"role": "user", "parts": [{"text": prompt}]}],
                "generationConfig": {
                    "temperature": 0.35,
                    "maxOutputTokens": 1400,
                },
            }
            data = json.dumps(payload).encode("utf-8")
            req = urllib.request.Request(
                url,
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            try:
                with urllib.request.urlopen(req, timeout=55) as resp:
                    body = json.loads(resp.read().decode("utf-8"))
                candidates = body.get("candidates") or []
                if not candidates:
                    raise RuntimeError(body.get("error", {}).get("message") or "No candidates from Gemini")
                parts = (candidates[0].get("content") or {}).get("parts") or []
                if not parts:
                    raise RuntimeError("Empty content from Gemini")
                return (parts[0].get("text") or "").strip()
            except Exception as e:
                last_err = e
                continue
    if last_err:
        raise last_err
    raise RuntimeError("Gemini call failed")


def _num(x: Any, default: float = 0.0) -> float:
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def _build_prompt(
    metrics: Dict[str, Any],
    strokes: List[Dict[str, Any]],
    ideal_loaded: bool,
) -> str:
    """Structured prompt — swimming biomechanics + session numbers only."""
    sc = int(metrics.get("stroke_count") or 0)
    cons = _num(metrics.get("consistency"))
    rate = _num(metrics.get("stroke_rate"))
    avg_ang = _num(metrics.get("avg_entry_angle"))
    avg_dev = _num(metrics.get("avg_deviation"))
    haptic_n = int(metrics.get("haptic_count") or 0)
    dur = _num(metrics.get("duration"))

    lines = [
        "You are an expert swim coach. The athlete uses a wrist IMU (GoldenForm).",
        "Give concise, actionable feedback (no medical diagnosis).",
        "Use short sections with headings: Summary, What went well, Focus next, Strokes to review.",
        "If deviation vs ideal is high, relate it to catch/pull line and rhythm — not vague motivation.",
        "",
        f"Session: {sc} strokes, {dur:.1f}s, stroke rate ~{rate:.1f}/min, consistency ~{cons:.0f}%.",
        f"Average entry angle ~{avg_ang:.1f}°. Average deviation vs ideal (when applicable): {avg_dev:.3f}.",
        f"Device haptic buzz count (during recording): {haptic_n}.",
        f"Ideal baseline loaded for comparison in app: {'yes' if ideal_loaded else 'no'}.",
        "",
        "Per-stroke (vs ideal when present):",
    ]
    for s in strokes[:40]:
        n = int(s.get("stroke_num") or s.get("number") or 0)
        dev = _num(s.get("deviation_vs_ideal"))
        ang = _num(s.get("entry_angle"))
        dh = bool(s.get("device_haptic"))
        alert = bool(s.get("vs_ideal_alert"))
        lines.append(
            f"  Stroke {n}: entry {ang:.1f}°, dev_vs_ideal {dev:.3f}, "
            f"device_haptic={dh}, over_threshold_vs_ideal={alert}"
        )
    lines.append("")
    lines.append(
        "Explain which strokes likely drove haptic feedback (if any) vs which differ most from the "
        "ideal shape. If no haptics and ideal is set, give technique tips from research anyway."
    )
    return "\n".join(lines)


def generate_coaching_insights(
    client_payload: Dict[str, Any],
    *,
    user_id: Optional[int] = None,
    privacy_no_log: bool = False,
) -> Dict[str, Any]:
    """
    Returns {status, text?} or {status, error?, message?}.
    privacy_no_log: when True, skip any server logging for this request (user opt-out).
    """
    _refresh_gemini_key_from_dotenv()
    key = (os.environ.get("GEMINI_API_KEY") or "").strip()
    if not key:
        return {
            "status": "unconfigured",
            "message": "AI coaching is not configured. Set GEMINI_API_KEY on the server.",
        }

    metrics = client_payload.get("session_metrics")
    if not isinstance(metrics, dict):
        metrics = {}
    strokes_raw = client_payload.get("strokes")
    if not isinstance(strokes_raw, list):
        strokes_raw = []
    ideal_loaded = bool(client_payload.get("ideal_loaded"))

    strokes: List[Dict[str, Any]] = []
    for s in strokes_raw[:48]:
        if not isinstance(s, dict):
            continue
        strokes.append(
            {
                "stroke_num": s.get("stroke_num", s.get("number")),
                "entry_angle": _num(s.get("entry_angle")),
                "deviation_vs_ideal": _num(s.get("deviation_vs_ideal")),
                "device_haptic": bool(s.get("device_haptic")),
                "vs_ideal_alert": bool(s.get("vs_ideal_alert")),
            }
        )

    prompt = _build_prompt(metrics, strokes, ideal_loaded)
    err_out: Optional[str] = None
    text_out: Optional[str] = None
    try:
        text_out = _call_gemini_with_fallback(key, prompt)
    except urllib.error.HTTPError as e:
        err_body = ""
        try:
            err_body = e.read().decode("utf-8", errors="replace")[:800]
        except Exception:
            pass
        err_out = f"Gemini HTTP {e.code}"
        _maybe_log_coaching(
            privacy_no_log=privacy_no_log,
            user_id=user_id,
            prompt=prompt,
            ok=False,
            err=err_out + " " + err_body,
            response_preview=None,
        )
        return {
            "status": "error",
            "error": err_out,
            "detail": err_body,
        }
    except urllib.error.URLError as e:
        # Common when the dashboard machine has no internet/DNS (e.g., still on band hotspot).
        err_out = f"Network error contacting Gemini: {getattr(e, 'reason', e)}"
        _maybe_log_coaching(
            privacy_no_log=privacy_no_log,
            user_id=user_id,
            prompt=prompt,
            ok=False,
            err=err_out,
            response_preview=None,
        )
        return {
            "status": "error",
            "error": "Gemini unreachable from this server",
            "detail": str(getattr(e, "reason", e)),
            "hint": "This computer likely has no internet/DNS (often because it’s connected to the band hotspot). Switch to a network with internet and retry.",
        }
    except Exception as e:
        err_out = str(e)
        _maybe_log_coaching(
            privacy_no_log=privacy_no_log,
            user_id=user_id,
            prompt=prompt,
            ok=False,
            err=err_out,
            response_preview=None,
        )
        return {"status": "error", "error": err_out}

    _maybe_log_coaching(
        privacy_no_log=privacy_no_log,
        user_id=user_id,
        prompt=prompt,
        ok=True,
        err=None,
        response_preview=text_out,
    )
    return {"status": "ok", "text": text_out, "model": _model_name()}
