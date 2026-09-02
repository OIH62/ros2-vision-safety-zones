from __future__ import annotations

from typing import Any


def resolve_device(
    requested: str,
    torch_module: Any,
    allow_cpu_fallback: bool = True,
) -> tuple[str, str | None]:
    """Resolve an Ultralytics device without assuming CUDA is usable."""
    value = str(requested).strip().lower()
    if value in {"", "auto"}:
        if _cuda_available(torch_module):
            return "cuda:0", None
        return "cpu", "CUDA is unavailable; selected CPU"

    if value == "cpu":
        return "cpu", None

    if value.isdigit():
        value = f"cuda:{value}"
    elif value == "cuda":
        value = "cuda:0"

    if value.startswith("cuda") and not _cuda_available(torch_module):
        message = f"Requested {value}, but CUDA is unavailable"
        if allow_cpu_fallback:
            return "cpu", f"{message}; selected CPU"
        raise RuntimeError(message)
    return value, None


def _cuda_available(torch_module: Any) -> bool:
    try:
        return bool(
            torch_module.cuda.is_available()
            and torch_module.cuda.device_count() > 0
        )
    except Exception:
        return False
