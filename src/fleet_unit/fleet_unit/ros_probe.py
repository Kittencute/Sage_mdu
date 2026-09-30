from __future__ import annotations

import rclpy
from composition_interfaces.srv import LoadNode
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rosidl_runtime_py.utilities import get_message

from fleet_unit.errors import UnitError
from fleet_unit.manifest import Manifest

PROBE_NODE_SUFFIX = "_unit_runner_probe"


class RosGraphProbe:
    def __init__(self, manifest: Manifest) -> None:
        self._unit = manifest.unit
        rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
        namespace = f"/{manifest.namespace}" if manifest.namespace else "/"
        self._node: Node = rclpy.create_node(manifest.unit + PROBE_NODE_SUFFIX, namespace=namespace)
        self._load_clients: dict[str, object] = {}
        self._message_subscriptions: dict[str, object] = {}
        self._received_topics: set[str] = set()

    def close(self) -> None:
        self._node.destroy_node()
        rclpy.shutdown()

    def node_exists(self, fqn: str) -> bool:
        for name, namespace in self._node.get_node_names_and_namespaces():
            if f"{namespace.rstrip('/')}/{name}" == fqn:
                return True
        return False

    def topic_has_publisher(self, name: str) -> bool:
        return self._node.count_publishers(name) > 0

    def message_received(self, topic: str) -> bool:
        if topic not in self._message_subscriptions:
            types = dict(self._node.get_topic_names_and_types()).get(topic)
            if not types:
                return False
            try:
                message_type = get_message(types[0])
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                raise UnitError(
                    f"import message type {types[0]} of {topic}", exc, self._unit
                ) from exc
            self._message_subscriptions[topic] = self._node.create_subscription(
                message_type,
                topic,
                lambda _message: self._received_topics.add(topic),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT),
            )
        if topic not in self._received_topics:
            return False
        self._node.destroy_subscription(self._message_subscriptions.pop(topic))
        self._received_topics.discard(topic)
        return True

    def spin(self, timeout_s: float) -> None:
        rclpy.spin_once(self._node, timeout_sec=timeout_s)

    def request_load(
        self,
        container_fqn: str,
        package: str,
        plugin: str,
        node_name: str,
        namespace: str,
        remaps: dict[str, str],
        parameters: dict[str, object],
    ):
        service = f"{container_fqn}/_container/load_node"
        client = self._load_clients.get(service)
        if client is None:
            client = self._node.create_client(LoadNode, service)
            self._load_clients[service] = client
        if not client.service_is_ready():
            return None
        request = LoadNode.Request()
        request.package_name = package
        request.plugin_name = plugin
        request.node_name = node_name
        request.node_namespace = f"/{namespace.strip('/')}" if namespace.strip("/") else "/"
        request.remap_rules = [f"{source}:={target}" for source, target in remaps.items()]
        request.parameters = [
            self._parameter_message(name, value, plugin) for name, value in parameters.items()
        ]
        return client.call_async(request)

    def _parameter_message(self, name: str, value: object, plugin: str):
        try:
            return Parameter(name=name, value=value).to_parameter_msg()
        except (TypeError, ValueError) as exc:
            raise UnitError(
                f"convert parameter {name} for component {plugin}", exc, self._unit
            ) from exc
