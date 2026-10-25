#!/usr/bin/env python3
# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Exercise real Keyboard -> IK -> MuJoCo over WebRTC without a display.

Run with `uv run --locked python scripts/test-keyboard-headless.py`.
Uses the common dataflow with isolated recording directories and an additional
official recorder for input poses. No robot hardware or coordinator is used.
"""

import argparse
import asyncio
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time

import aiohttp
from aiortc import RTCConfiguration, RTCPeerConnection, RTCSessionDescription
import numpy as np
from openarm_control import Kinematics
import pyarrow.parquet as pq
import yaml

from model_config import load_profile


ROOT = Path(__file__).resolve().parents[1]
UI = "http://127.0.0.1:8000"
KEYBOARD = "http://127.0.0.1:8080"
CASES = ("home", "translation", "rotation", "gripper", "released")


def require(condition, message):
    if not condition:
        raise AssertionError(message)


async def wait_until(predicate, timeout=30):
    deadline = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= deadline:
            raise TimeoutError("Timed out waiting for WebRTC data channel")
        await asyncio.sleep(0.05)


async def wait_http(session, url, process):
    for _ in range(120):
        if process.poll() is not None:
            raise RuntimeError(f"Dataflow exited during startup: {process.returncode}")
        try:
            async with session.get(url) as response:
                if response.status == 200:
                    return
        except aiohttp.ClientError:
            pass
        await asyncio.sleep(0.25)
    raise TimeoutError(f"Server did not start: {url}")


async def post(session, path):
    async with session.post(UI + path, allow_redirects=False) as response:
        response.raise_for_status()


async def exercise(process):
    async with aiohttp.ClientSession(
        timeout=aiohttp.ClientTimeout(total=15)
    ) as session:
        await wait_http(session, UI, process)
        await wait_http(session, KEYBOARD, process)
        pc = RTCPeerConnection(RTCConfiguration(iceServers=[]))
        channel = pc.createDataChannel("keys")
        pc.addTransceiver("video", direction="recvonly")
        try:
            await pc.setLocalDescription(await pc.createOffer())
            async with session.post(
                KEYBOARD + "/offer",
                json={"sdp": pc.localDescription.sdp, "type": pc.localDescription.type},
            ) as response:
                response.raise_for_status()
                answer = await response.json()
            await pc.setRemoteDescription(RTCSessionDescription(**answer))
            await wait_until(lambda: channel.readyState == "open")

            async def hold(keys):
                for key in keys:
                    channel.send(json.dumps({"type": "keydown", "key": key}))
                await asyncio.sleep(0.4)
                for key in reversed(keys):
                    channel.send(json.dumps({"type": "keyup", "key": key}))
                await asyncio.sleep(0.15)

            for case in CASES:
                await post(session, "/start")
                await asyncio.sleep(0.1)
                if case in ("home", "released"):
                    await asyncio.sleep(0.5)
                elif case in ("translation", "rotation"):
                    for keys in (("w", "i"), ("r", "y"), ("a", "j")):
                        await hold(("Shift", *keys) if case == "rotation" else keys)
                else:
                    await hold(("x", "m"))
                    await hold(("c", "n"))
                await post(session, "/success")
                await asyncio.sleep(0.15)
        finally:
            await pc.close()
        # Test UI Quit, including timer and cyclic module shutdown. Esc routing
        # is deliberately a separate follow-up task.
        await post(session, "/quit")


def samples(directory, episode, category, side, field):
    path = (
        directory
        / "dataset"
        / "episodes"
        / str(episode)
        / category
        / "arms"
        / side
        / "state.parquet"
    )
    table = pq.read_table(path)
    values = np.asarray(table[field].to_pylist())
    require(len(values) >= 10, f"Too few samples in {path}: {len(values)}")
    require(
        values.shape[1] == 8 and np.isfinite(values).all(), f"Invalid values in {path}"
    )
    return values


def angle_range(poses):
    quaternions = poses[:, 3:7].copy()
    quaternions /= np.linalg.norm(quaternions, axis=1, keepdims=True)
    return float(
        np.max(2 * np.arccos(np.clip(np.abs(quaternions @ quaternions[0]), 0, 1)))
    )


def validate(output, profile):
    kin = Kinematics(profile.setup())
    home = {side: kin.setup.read_ee_pose(side).copy() for side in ("right", "left")}
    reports = [
        json.loads(path.read_text())
        for path in (output / "model-config").glob("*.json")
    ]
    require(
        {r["entrypoint"] for r in reports}
        == {"dora-openarm-keyboard", "dora-openarm-ik", "dora-openarm-mujoco"},
        "Missing node model reports",
    )
    for report in reports:
        require(report["profile"] == profile.name, "Nodes selected different profiles")
        require(
            report["xml"] == str(profile.xml), "Nodes selected different model XMLs"
        )
        require(report["settings"] == profile.settings, "Node frames/keyframes differ")
    results = {}
    for episode, case in enumerate(CASES):
        results[case] = {}
        for side in ("right", "left"):
            targets = samples(output / "targets", episode, "action", side, "pose")
            actions = samples(output / "joints", episode, "action", side, "qpos")
            observations = samples(output / "joints", episode, "obs", side, "qpos")
            indices = np.linspace(
                0, len(observations) - 1, min(100, len(observations)), dtype=int
            )
            actual = np.array([kin.fk(side, observations[i]) for i in indices])
            metrics = {
                "target_samples": len(targets),
                "action_samples": len(actions),
                "observation_samples": len(observations),
                "target_xyz_range_m": np.ptp(targets[:, :3], axis=0).tolist(),
                "target_rotation_rad": angle_range(targets),
                "actual_xyz_range_m": np.ptp(actual[:, :3], axis=0).tolist(),
                "actual_rotation_rad": angle_range(actual),
                "target_gripper_range_rad": float(np.ptp(targets[:, 7])),
                "actual_gripper_range_rad": float(np.ptp(observations[:, 7])),
            }
            if case == "home":
                require(
                    np.linalg.norm(targets[0, :3] - home[side][:3]) < 1e-4,
                    f"{side}: Keyboard home position differs from model home",
                )
                require(
                    angle_range(np.vstack((home[side], targets[0, :7]))) < 1e-3,
                    f"{side}: Keyboard home orientation differs from model home",
                )
            elif case == "translation":
                require(
                    np.all(np.ptp(targets[:, :3], axis=0) > 0.003),
                    f"{side}: missing translation axis",
                )
                require(
                    np.linalg.norm(np.ptp(actual[:, :3], axis=0)) > 0.001,
                    f"{side}: MuJoCo did not translate",
                )
            elif case == "rotation":
                require(
                    metrics["target_rotation_rad"] > 0.05, f"{side}: no target rotation"
                )
                require(
                    metrics["actual_rotation_rad"] > 0.005,
                    f"{side}: MuJoCo did not rotate",
                )
            elif case == "gripper":
                require(
                    metrics["target_gripper_range_rad"] > 0.1,
                    f"{side}: no gripper target change",
                )
                require(
                    metrics["actual_gripper_range_rad"] > 0.01,
                    f"{side}: MuJoCo gripper did not move",
                )
            else:
                require(
                    np.max(np.ptp(targets, axis=0)) < 1e-6,
                    f"{side}: targets advanced after key release",
                )
            results[case][side] = metrics
    (output / "results.json").write_text(json.dumps(results, indent=2) + "\n")
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--model",
        choices=("demo", "pedestal"),
        default=os.environ.get("OPENARM_MODEL", "demo"),
    )
    args = parser.parse_args()
    profile = load_profile(args.model)
    for port in (8000, 8080):
        with socket.socket() as probe:
            probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            probe.bind(("127.0.0.1", port))
    (ROOT / "out").mkdir(exist_ok=True)
    output = Path(
        tempfile.mkdtemp(prefix=f"keyboard-smoke-{profile.name}-", dir=ROOT / "out")
    )
    metadata = yaml.safe_load((ROOT / "metadata_mujoco.yaml").read_text())
    metadata["equipment"]["perceptions"]["cameras"] = {}
    metadata_file = output / "metadata.yaml"
    metadata_file.write_text(yaml.safe_dump(metadata, sort_keys=False))
    flow = yaml.safe_load((ROOT / "dataflow.yaml").read_text())
    for node in flow["nodes"]:
        if node["id"] in ("ui", "recorder"):
            node["env"]["METADATA_FILE"] = str(metadata_file)
        if node["id"] == "recorder":
            node["env"]["DIRECTORY"] = str(output / "joints")
    flow["nodes"].append(
        {
            "id": "target-recorder",
            "path": "dora-openarm-dataset-recorder",
            "env": {
                "METADATA_FILE": str(metadata_file),
                "DIRECTORY": str(output / "targets"),
            },
            "inputs": {
                "command": "ui/command",
                "arm_right_action": "leader/pose_right",
                "arm_left_action": "leader/pose_left",
            },
        }
    )
    fd, filename = tempfile.mkstemp(
        prefix="keyboard-smoke-", suffix=".envsubst.yaml", dir=ROOT
    )
    descriptor = Path(filename)
    with os.fdopen(fd, "w") as stream:
        yaml.safe_dump(flow, stream, sort_keys=False)
    expanded = descriptor.with_name(descriptor.stem + ".envsubst.yaml")
    env = os.environ.copy()
    env.pop("PORT", None)
    env.pop("HOST", None)
    env.pop("DORA_OPENARM_MUJOCO_XML", None)
    env.update(
        LEADER="keyboard",
        ROBOT="mujoco",
        MUJOCO_VIEWER="false",
        MUJOCO_RENDER="false",
        AUTO_OPEN="no",
        OPENARM_MODEL=profile.name,
        OPENARM_MODEL_REPORT_DIR=str(output / "model-config"),
    )
    env["PATH"] = str(Path(sys.executable).parent) + os.pathsep + env["PATH"]
    process = None
    print(f"Artifacts: {output}", flush=True)
    try:
        with (output / "dataflow.log").open("w") as log:
            process = subprocess.Popen(
                [
                    sys.executable,
                    str(ROOT / "scripts/dora-envsubst"),
                    "run",
                    str(descriptor),
                    "--uv",
                    "--stop-after",
                    "90s",
                ],
                cwd=ROOT,
                env=env,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            asyncio.run(exercise(process))
            code = process.wait(timeout=15)
            require(code == 0, f"Dataflow exited with code {code}")
        validate(output, profile)
        print(
            f"PASS ({profile.name}): both arms translated, rotated, gripped, held targets after release; UI Quit stopped the dataflow."
        )
    except BaseException:
        print((output / "dataflow.log").read_text()[-12000:], file=sys.stderr)
        raise
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        descriptor.unlink(missing_ok=True)
        expanded.unlink(missing_ok=True)


if __name__ == "__main__":
    main()
