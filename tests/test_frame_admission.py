"""Run the production frame cadence header against host timestamp sequences."""

from collections.abc import Callable, Sequence
import math
import os
from pathlib import Path
import subprocess

import pytest


ROOT = Path(__file__).resolve().parents[1]
UINT32_MASK = (1 << 32) - 1
Event = tuple[int, float] | None
Replay = Callable[[Sequence[Event]], list[bool]]


@pytest.fixture(scope="session")
def replay_cadence(tmp_path_factory: pytest.TempPathFactory) -> Replay:
    # Invoke pytest with --basetemp on persistent disk, as for other host builds.
    build = tmp_path_factory.mktemp("frame_cadence")
    source = build / "frame_cadence.cpp"
    executable = build / "frame_cadence"
    source.write_text(
        r"""
#include <cstdint>
#include <iostream>

#include "components/esp_video_camera/frame_cadence.h"

int main() {
  esphome::esp_video_camera::FrameCadence cadence;
  char command;
  while (std::cin >> command) {
    if (command == 'r') {
      cadence.reset();
    } else if (command == 'a') {
      std::uint32_t timestamp_us;
      float max_fps;
      if (!(std::cin >> timestamp_us >> max_fps))
        return 2;
      std::cout << (cadence.accept(timestamp_us, max_fps) ? '1' : '0') << '\n';
    } else {
      return 3;
    }
  }
}
"""
    )
    compiled = subprocess.run(
        [
            "g++",
            "-std=c++17",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-pedantic",
            "-I",
            str(ROOT),
            str(source),
            "-o",
            str(executable),
        ],
        capture_output=True,
        text=True,
        timeout=30,
        env={**os.environ, "TMPDIR": str(build)},
    )
    assert compiled.returncode == 0, compiled.stdout + compiled.stderr

    def replay(events: Sequence[Event]) -> list[bool]:
        commands = [
            "r" if event is None else f"a {event[0] & UINT32_MASK} {event[1]}"
            for event in events
        ]
        result = subprocess.run(
            [str(executable)],
            input="\n".join(commands) + "\n",
            capture_output=True,
            text=True,
            check=True,
            timeout=10,
        )
        decisions = result.stdout.splitlines()
        assert len(decisions) == sum(event is not None for event in events)
        assert set(decisions) <= {"0", "1"}, result.stdout
        return [decision == "1" for decision in decisions]

    return replay


def test_zero_timestamp_is_a_valid_first_frame(replay_cadence: Replay) -> None:
    timestamps = [0, 0, 20_000, 100_000, 120_000, 200_000]

    assert replay_cadence([(timestamp, 10.0) for timestamp in timestamps]) == [
        True,
        False,
        False,
        True,
        False,
        True,
    ]


def test_exact_ten_fps_input_does_not_become_five_fps(
    replay_cadence: Replay,
) -> None:
    decisions = replay_cadence([(index * 100_000, 10.0) for index in range(101)])

    assert all(decisions)


@pytest.mark.parametrize("max_fps", [10.0, 7.5, 29.97, 0.1, 60.0])
@pytest.mark.parametrize(
    "steps_us", [(20_000,), (28_000,), (20_000, 28_000)], ids=["20ms", "28ms", "mixed"]
)
def test_quantized_input_keeps_average_phase(
    replay_cadence: Replay, max_fps: float, steps_us: tuple[int, ...]
) -> None:
    timestamps = [0]
    for index in range(9_000):
        timestamp = timestamps[-1] + steps_us[index % len(steps_us)]
        if timestamp > 180_000_000:
            break
        timestamps.append(timestamp)

    decisions = replay_cadence([(timestamp, max_fps) for timestamp in timestamps])

    accepted = 0
    for index, (timestamp, decision) in enumerate(zip(timestamps, decisions)):
        accepted += decision
        # One initial frame, then the requested elapsed phase, limited by input.
        ideal = min(index + 1, 1 + math.floor(timestamp * max_fps / 1_000_000))
        assert abs(accepted - ideal) <= 1, (
            f"{max_fps=}, {steps_us=}, {timestamp=}, {accepted=}, {ideal=}"
        )


@pytest.mark.parametrize("max_fps", [10.0, 7.5, 29.97, 0.1, 60.0])
def test_microsecond_counter_wrap_preserves_decisions(
    replay_cadence: Replay, max_fps: float
) -> None:
    offsets = [index * 28_000 for index in range(1_500)]
    ordinary = replay_cadence([(offset + 123, max_fps) for offset in offsets])
    wrapped = replay_cadence(
        [(UINT32_MASK - 50_000 + offset, max_fps) for offset in offsets]
    )

    assert wrapped == ordinary


def test_long_pause_drops_debt_without_a_catchup_burst(replay_cadence: Replay) -> None:
    before_pause = [(timestamp, 10.0) for timestamp in [0, 20_000, 100_000, 120_000]]
    resumed_at = 10_000_000
    after_pause = [
        (resumed_at + offset, 10.0)
        for offset in [0, 1, 20_000, 40_000, 60_000, 80_000, 100_000]
    ]

    assert replay_cadence(before_pause + after_pause) == [
        True,
        False,
        True,
        False,
        True,
        False,
        False,
        False,
        False,
        False,
        True,
    ]


@pytest.mark.parametrize(
    ("old_fps", "new_fps"),
    [(10.0, 7.5), (7.5, 29.97), (29.97, 0.1), (0.1, 60.0), (60.0, 10.0)],
)
def test_fps_change_starts_a_fresh_phase(
    replay_cadence: Replay, old_fps: float, new_fps: float
) -> None:
    assert replay_cadence([(0, old_fps), (1, old_fps), (2, new_fps), (3, new_fps)]) == [
        True,
        False,
        True,
        False,
    ]


def test_reset_starts_a_fresh_phase_at_the_same_timestamp(
    replay_cadence: Replay,
) -> None:
    assert replay_cadence(
        [
            (0, 10.0),
            (20_000, 10.0),
            None,
            (20_000, 10.0),
            (40_000, 10.0),
            (120_000, 10.0),
        ]
    ) == [True, False, True, False, True]
