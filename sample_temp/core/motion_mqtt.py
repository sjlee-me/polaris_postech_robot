"""
AI 회피/주행: MQTT 명령 페이로드 `{ ts, value }` 생성.
`ts`는 NTP(time_sync)로 맞춘 로봇 기준 시각(초), `value`는 정지/주행.
"""
from __future__ import annotations

import re
from typing import Any, Union


def normalize_motion_token(motion: Union[int, str, bool]) -> tuple[int, str]:
    """
    motion을 (0|1, "STOP"|"RUN")으로 통일.
    허용: 0, 1, "0", "1", "STOP", "RUN", True→RUN, False→STOP (대소문자 무시).
    """
    if isinstance(motion, bool):
        return (1, "RUN") if motion else (0, "STOP")
    if isinstance(motion, int):
        if motion not in (0, 1):
            raise ValueError("motion int는 0 또는 1만 허용됩니다.")
        return (motion, "RUN" if motion else "STOP")
    s = str(motion).strip().upper()
    if s in ("0", "STOP", "HALT"):
        return (0, "STOP")
    if s in ("1", "RUN", "GO", "MOVE"):
        return (1, "RUN")
    raise ValueError(f"알 수 없는 motion 값: {motion!r} (0,1,RUN,STOP 등)")


def build_motion_payload(robot_time_sec: float, motion: Union[int, str, bool]) -> dict[str, Any]:
    """
    로봇 NTP 기준 시각 `robot_time_sec`와 주행 여부를 MQTT JSON 한 건으로 만든다.

    Returns:
        {"ts": float, "value": 0|1} — 0 정지, 1 주행.
    """
    iv, _ = normalize_motion_token(motion)
    return {"ts": float(robot_time_sec), "value": iv}


def format_motion_topic(template: str, robot_id: str) -> str:
    """`{robot_id}` 플레이스홀더가 있으면 치환."""
    return template.format(robot_id=str(robot_id))


def parse_cli_motion(s: str) -> Union[int, str]:
    """CLI 문자열을 normalize_motion_token이 받을 수 있는 형태로."""
    t = s.strip()
    if re.fullmatch(r"[01]", t):
        return int(t)
    return t
