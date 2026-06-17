#!/usr/bin/env python3
"""Run the currently proven Chica exactness checks."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def run(label: str, command: list[str]) -> bool:
    print(f"\n== {label} ==")
    result = subprocess.run(command, cwd=ROOT, text=True)
    if result.returncode != 0:
        print(f"{label}: failed with exit code {result.returncode}")
        return False
    return True


def main() -> int:
    (ROOT / "build").mkdir(exist_ok=True)
    if not run(
        "build-animation-replay",
        [
            "c++",
            "-std=c++17",
            "-O2",
            "-Iapp/src/main/cpp",
            "tools/oracle/animation_replay.cpp",
            "app/src/main/cpp/apk_model.cpp",
            "app/src/main/cpp/pulse_conversion.cpp",
            "-o",
            "build/animation_replay",
        ],
    ):
        return 1

    checks = [
        (
            "gait-oracles",
            [
                sys.executable,
                "tools/oracle/compare_gait_oracle.py",
                "../oracle/api35_walk3_025_050_3_gaittrace.jsonl",
                "../oracle/api35_walk2_025_050_2_gaittrace.jsonl",
                "../oracle/api35_walk1_025_050_1_gaittrace.jsonl",
                "../oracle/api35_walk15_025_050_4_gaittrace.jsonl",
                "../oracle/api35_walk25_025_050_5_gaittrace.jsonl",
                "../oracle/api35_walkwave_025_050_6_gaittrace.jsonl",
                "../oracle/api35_crab_walk3_025_050_3_gaittrace.jsonl",
            ],
        ),
        (
            "quad-runtime",
            [sys.executable, "tools/oracle/compare_quad_runtime.py"],
        ),
        (
            "walk-runtime-replay",
            [
                sys.executable,
                "tools/oracle/compare_walk_runtime_replay.py",
                "../oracle/controltrace_walk_runtime_virtualtouch_original.jsonl",
                "../oracle/controltrace_walkclear_dense_virtualtouch_original.jsonl",
            ],
        ),
        (
            "animation-torque-sit",
            [
                sys.executable,
                "tools/oracle/replay_animation_trace.py",
                "../oracle/controltrace_torque_sit_virtualhw_anim.jsonl",
                "--scenario",
                "auto",
                "--strict",
            ],
        ),
        (
            "animation-quad-14",
            [
                sys.executable,
                "tools/oracle/replay_animation_trace.py",
                "../oracle/controltrace_quad_14_keep_autositoff_logcat_anim.jsonl",
                "--scenario",
                "auto",
                "--strict",
            ],
        ),
        (
            "animation-quad-03",
            [
                sys.executable,
                "tools/oracle/replay_animation_trace.py",
                "../oracle/controltrace_quad_03_verify_logcat_anim.jsonl",
                "--scenario",
                "auto",
                "--strict",
            ],
        ),
        (
            "animation-quad-25",
            [
                sys.executable,
                "tools/oracle/replay_animation_trace.py",
                "../oracle/controltrace_quad_25_verify_logcat_anim.jsonl",
                "--scenario",
                "auto",
                "--strict",
            ],
        ),
        (
            "animation-bounce-jump",
            [
                sys.executable,
                "tools/oracle/replay_animation_trace.py",
                "../oracle/controltrace_bounce_jump_autositoff_virtualhw_anim.jsonl",
                "--scenario",
                "impulse",
                "--strict",
            ],
        ),
        (
            "calibration-virtual-touch",
            [sys.executable, "tools/oracle/compare_calibration_virtualtouch.py"],
        ),
        (
            "ack-stand-ramp",
            [sys.executable, "tools/oracle/compare_ack_stand_ramp.py"],
        ),
        (
            "no-hardware-servo",
            [sys.executable, "tools/device/verify_no_hardware_servo.py"],
        ),
        (
            "servo2040-protocol",
            [sys.executable, "tools/device/verify_servo2040_protocol.py"],
        ),
        (
            "pololu-protocol",
            [sys.executable, "tools/device/verify_pololu_protocol.py"],
        ),
    ]

    ok = True
    for label, command in checks:
        ok = run(label, command) and ok
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
