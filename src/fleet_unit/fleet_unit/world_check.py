from __future__ import annotations

import argparse
import json
import os
import selectors
import subprocess
import sys
from collections.abc import Iterator
from typing import Any

GZ_EXECUTABLE = "gz"
READ_BYTES = 65536
STDERR_TAIL_BYTES = 4096


class WorldCheckError(Exception):
    def __init__(self, operation: str, cause: object) -> None:
        super().__init__(f"world check failed: operation: {operation}; cause: {cause}")


def stats_topic(world: str) -> str:
    return f"/world/{world}/stats"


def scene_topic(world: str) -> str:
    return f"/world/{world}/scene/info"


def deletion_topic(world: str) -> str:
    return f"/world/{world}/scene/deletion"


def pose_topic(world: str) -> str:
    return f"/world/{world}/pose/info"


def run_gz(command: list[str], operation: str) -> str:
    try:
        result = subprocess.run(command, capture_output=True, text=True, check=False)
    except OSError as exc:
        raise WorldCheckError(operation, exc) from exc
    if result.returncode != 0:
        raise WorldCheckError(
            operation, f"exit {result.returncode}: {result.stderr.strip() or result.stdout.strip()}"
        )
    return result.stdout


def wait_for_world(world: str) -> None:
    topic = stats_topic(world)
    run_gz([GZ_EXECUTABLE, "topic", "-e", "-n", "1", "-t", topic], f"receive {topic}")
    print(f"world {world} is running", flush=True)


def present_entity(world: str, entity: str) -> int | None:
    topic = pose_topic(world)
    operation = f"receive {topic}"
    reply = run_gz(
        [GZ_EXECUTABLE, "topic", "-e", "-n", "1", "--json-output", "-t", topic], operation
    )
    try:
        poses = json.loads(reply).get("pose", [])
    except json.JSONDecodeError as exc:
        raise WorldCheckError(operation, f"{exc}: {reply.strip()[:200]}") from exc
    if not poses:
        raise WorldCheckError(operation, "the message lists no entities")
    matches = {int(pose["id"]) for pose in poses if pose.get("name") == entity}
    if len(matches) > 1:
        raise WorldCheckError(operation, f"entity name {entity} is used by ids {sorted(matches)}")
    return matches.pop() if matches else None


def open_stream(topic: str) -> subprocess.Popen[bytes]:
    try:
        return subprocess.Popen(
            [GZ_EXECUTABLE, "topic", "-e", "--json-output", "-t", topic],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except OSError as exc:
        raise WorldCheckError(f"subscribe to {topic}", exc) from exc


def stream_messages(streams: dict[str, subprocess.Popen[bytes]]) -> Iterator[tuple[str, Any]]:
    selector = selectors.DefaultSelector()
    for topic, process in streams.items():
        selector.register(process.stdout, selectors.EVENT_READ, (topic, False))
        selector.register(process.stderr, selectors.EVENT_READ, (topic, True))
    partial = dict.fromkeys(streams, b"")
    errors = dict.fromkeys(streams, b"")
    while True:
        for key, _ in selector.select():
            topic, is_stderr = key.data
            chunk = os.read(key.fd, READ_BYTES)
            if not chunk:
                raise WorldCheckError(
                    f"read {topic}",
                    f"the gz topic stream ended: {errors[topic].decode(errors='replace').strip()}",
                )
            if is_stderr:
                errors[topic] = (errors[topic] + chunk)[-STDERR_TAIL_BYTES:]
                continue
            *lines, partial[topic] = (partial[topic] + chunk).split(b"\n")
            for line in filter(bytes.strip, lines):
                try:
                    yield topic, json.loads(line)
                except json.JSONDecodeError as exc:
                    raise WorldCheckError(
                        f"read {topic}", f"{exc}: {line.decode(errors='replace').strip()}"
                    ) from exc


def watch_entity(world: str, entity: str) -> None:
    operation = f"watch entity {entity} in world {world}"
    topics = (scene_topic(world), deletion_topic(world))
    streams = {topic: open_stream(topic) for topic in topics}
    try:
        entity_id = present_entity(world, entity)
        if entity_id is None:
            print(f"waiting for entity {entity} in world {world}", flush=True)
        else:
            print(f"entity {entity} is present in world {world} as id {entity_id}", flush=True)
        for topic, message in stream_messages(streams):
            if topic == scene_topic(world):
                for model in message.get("model", []):
                    if entity_id is None and model.get("name") == entity:
                        entity_id = int(model["id"])
                        print(
                            f"entity {entity} appeared in world {world} as id {entity_id}",
                            flush=True,
                        )
            else:
                removed = {int(item) for item in message.get("data", [])}
                if entity_id in removed:
                    raise WorldCheckError(operation, f"the world removed entity id {entity_id}")
    finally:
        for process in streams.values():
            process.terminate()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="world_check")
    commands = parser.add_subparsers(dest="command", required=True)
    ready = commands.add_parser("ready", help="exit 0 once the world publishes its statistics")
    watch = commands.add_parser("watch", help="run until the entity is removed from the world")
    for command in (ready, watch):
        command.add_argument("--world", required=True)
    watch.add_argument("--entity", required=True)
    arguments = parser.parse_args(argv)
    try:
        if arguments.command == "ready":
            wait_for_world(arguments.world)
        else:
            watch_entity(arguments.world, arguments.entity)
    except WorldCheckError as exc:
        print(exc, file=sys.stderr, flush=True)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
