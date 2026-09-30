from __future__ import annotations

from typing import Any

from fleet_config.model import SIMULATION, UPLINK_ADDRESS_KEY, Fleet, Instance, site_value
from fleet_config.paths import FLEET_SOCKET_PATH
from fleet_config.unit import BoundaryEntry, UnitContext

ALL_MESSAGES = [
    "put",
    "delete",
    "declare_subscriber",
    "query",
    "reply",
    "declare_queryable",
    "liveliness_token",
    "liveliness_query",
    "declare_liveliness_subscriber",
]
EXPORT_EGRESS_MESSAGES = ["put", "liveliness_token", "reply", "declare_queryable"]
EXPORT_INGRESS_MESSAGES = [
    "declare_subscriber",
    "query",
    "declare_liveliness_subscriber",
    "liveliness_query",
]
IMPORT_INGRESS_MESSAGES = ["put", "liveliness_token"]
IMPORT_EGRESS_MESSAGES = [
    "declare_subscriber",
    "declare_liveliness_subscriber",
    "liveliness_query",
]
IMPORT_HISTORY_EGRESS_MESSAGES = ["query"]
SUBSCRIBER_LIVELINESS_MESSAGES = ["liveliness_token"]
GRAPH_DISCOVERY_MESSAGES = ["declare_liveliness_subscriber", "liveliness_query"]
IMPORT_HISTORY_INGRESS_MESSAGES = ["declare_queryable", "reply"]

BOUNDARY_LINK_PROTOCOL = "unixsock-stream"
LOOPBACK_INTERFACE = "lo"
LOOPBACK_ADDRESS = "127.0.0.1"
SESSION_MODE = "client"
GOSSIP_ENABLED = False
TX_QUEUE_BATCHES = 16
SESSION_WAIT_BEFORE_CLOSE_US = 60000000
TX_QUEUE_PRIORITIES = [
    "control",
    "real_time",
    "interactive_high",
    "interactive_low",
    "data_high",
    "data",
    "data_low",
    "background",
]
INTERNAL_KEYS = ["**", "@/**", "@ros2_lv/**", "**/@adv/**"]


def mangle(path: str) -> str:
    return path.replace("/", "%").replace("*", "$*")


def mangle_namespace(namespace: str) -> str:
    return mangle(namespace if namespace.startswith("/") else f"/{namespace}")


def data_key(domain_id: int, topic: str) -> str:
    return f"{domain_id}/{topic[1:]}/**"


def is_namespaced(namespace: str | None, topic: str) -> bool:
    return bool(namespace) and topic.startswith(f"/{namespace}/")


def without_namespace(namespace: str, topic: str) -> str:
    return topic[len(f"/{namespace}/") :]


def any_namespace_data_key(domain_id: int, namespace: str, topic: str) -> str:
    return f"{domain_id}/*/{without_namespace(namespace, topic)}/**"


def any_namespace_advanced_publisher_key(domain_id: int, namespace: str, topic: str) -> str:
    return f"{domain_id}/*/{without_namespace(namespace, topic)}/*/*/@adv/**"


def advanced_publisher_key(domain_id: int, topic: str) -> str:
    return f"{domain_id}/{topic[1:]}/*/*/@adv/**"


def publisher_liveliness_key(domain_id: int, namespace: str | None, topic: str) -> str:
    scope = "*" if namespace is None else mangle_namespace(namespace)
    return f"@ros2_lv/{domain_id}/*/*/*/MP/*/{scope}/*/{mangle(topic)}/**"


def subscriber_liveliness_key(domain_id: int, topic: str) -> str:
    return f"@ros2_lv/{domain_id}/*/*/*/MS/*/*/*/{mangle(topic)}/**"


def liveliness_domain_key(domain_id: int) -> str:
    return f"@ros2_lv/{domain_id}/**"


def _scouting() -> dict[str, Any]:
    return {"multicast": {"enabled": False}, "gossip": {"enabled": GOSSIP_ENABLED}}


def _common(session: bool) -> dict[str, Any]:
    queue: dict[str, Any] = {
        "size": {priority: TX_QUEUE_BATCHES for priority in TX_QUEUE_PRIORITIES}
    }
    document: dict[str, Any] = {
        "queries_default_timeout": 600000,
        "transport": {
            "unicast": {
                "open_timeout": 60000,
                "accept_timeout": 60000,
                "accept_pending": 10000,
                "max_sessions": 10000,
            },
            "link": {"tx": {"lease": 60000, "keep_alive": 2, "queue": queue}},
            "shared_memory": {
                "enabled": False,
                "transport_optimization": {"pool_size": 50331648, "message_size_threshold": 512},
            },
        },
    }
    if session:
        queue["congestion_control"] = {"block": {"wait_before_close": SESSION_WAIT_BEFORE_CLOSE_US}}
        document["timestamping"] = {"enabled": True}
        document["adminspace"] = {"enabled": True}
    else:
        document["routing"] = {"router": {"peers_failover_brokering": False}}
    return document


def _internal_subjects(unit: UnitContext) -> list[dict[str, Any]]:
    if unit.on_fleet_host:
        return [{"id": "internal", "link_protocols": ["tcp"], "interfaces": [LOOPBACK_INTERFACE]}]
    return [{"id": "internal", "interfaces": [LOOPBACK_INTERFACE]}]


def _boundary_subjects(unit: UnitContext) -> list[dict[str, Any]]:
    if unit.on_fleet_host:
        return [{"id": "boundary", "link_protocols": [BOUNDARY_LINK_PROTOCOL]}]
    assert unit.topology is not None
    interface = unit.topology.uplink_interface
    return [{"id": f"boundary_{interface}", "interfaces": [interface]}]


def _entry_keys(unit: UnitContext, entries: tuple[BoundaryEntry, ...]) -> list[str]:
    return [data_key(unit.domain_id, entry.topic) for entry in entries]


def _liveliness_keys(unit: UnitContext, entries: tuple[BoundaryEntry, ...]) -> list[str]:
    return [
        publisher_liveliness_key(unit.domain_id, entry.publisher_namespace, entry.topic)
        for entry in entries
    ]


def _advanced_keys(unit: UnitContext, entries: tuple[BoundaryEntry, ...]) -> list[str]:
    return [advanced_publisher_key(unit.domain_id, entry.topic) for entry in entries]


def _subscriber_keys(unit: UnitContext, entries: tuple[BoundaryEntry, ...]) -> list[str]:
    return [subscriber_liveliness_key(unit.domain_id, entry.topic) for entry in entries]


def _any_namespace_keys(unit: UnitContext, entries: tuple[BoundaryEntry, ...]) -> list[str]:
    keys: list[str] = []
    for entry in entries:
        if not is_namespaced(entry.publisher_namespace, entry.topic):
            continue
        namespace = entry.publisher_namespace
        assert namespace is not None
        keys.append(any_namespace_data_key(unit.domain_id, namespace, entry.topic))
        keys.append(any_namespace_advanced_publisher_key(unit.domain_id, namespace, entry.topic))
        keys.append(publisher_liveliness_key(unit.domain_id, None, entry.topic))
    return keys


def _rule(rule_id: str, messages: list[str], flows: list[str], keys: list[str]) -> dict[str, Any]:
    distinct: list[str] = []
    for key in keys:
        if key not in distinct:
            distinct.append(key)
    return {
        "id": rule_id,
        "messages": list(messages),
        "flows": list(flows),
        "permission": "allow",
        "key_exprs": distinct,
    }


def _boundary_rules(unit: UnitContext) -> list[dict[str, Any]]:
    exports = unit.boundary.exports
    imports = unit.boundary.imports
    domain = [liveliness_domain_key(unit.domain_id)]
    rules: list[dict[str, Any]] = []
    if exports:
        rules.append(
            _rule(
                "boundary_export_egress",
                EXPORT_EGRESS_MESSAGES,
                ["egress"],
                _entry_keys(unit, exports)
                + _advanced_keys(unit, exports)
                + _liveliness_keys(unit, exports),
            )
        )
        rules.append(
            _rule(
                "boundary_export_ingress",
                EXPORT_INGRESS_MESSAGES,
                ["ingress"],
                _entry_keys(unit, exports)
                + _advanced_keys(unit, exports)
                + _liveliness_keys(unit, exports)
                + _any_namespace_keys(unit, exports)
                + domain,
            )
        )
        rules.append(
            _rule(
                "boundary_export_subscriber_ingress",
                SUBSCRIBER_LIVELINESS_MESSAGES,
                ["ingress"],
                _subscriber_keys(unit, exports),
            )
        )
        rules.append(
            _rule("boundary_export_graph_egress", GRAPH_DISCOVERY_MESSAGES, ["egress"], domain)
        )
    if imports:
        rules.append(
            _rule(
                "boundary_import_ingress",
                IMPORT_INGRESS_MESSAGES,
                ["ingress"],
                _entry_keys(unit, imports)
                + _advanced_keys(unit, imports)
                + _liveliness_keys(unit, imports),
            )
        )
        rules.append(
            _rule(
                "boundary_import_egress",
                IMPORT_EGRESS_MESSAGES,
                ["egress"],
                _entry_keys(unit, imports) + _advanced_keys(unit, imports) + domain,
            )
        )
        rules.append(
            _rule(
                "boundary_import_subscriber_egress",
                SUBSCRIBER_LIVELINESS_MESSAGES,
                ["egress"],
                _subscriber_keys(unit, imports),
            )
        )
        rules.append(
            _rule("boundary_import_graph_ingress", GRAPH_DISCOVERY_MESSAGES, ["ingress"], domain)
        )
        rules.append(
            _rule(
                "boundary_import_history_egress",
                IMPORT_HISTORY_EGRESS_MESSAGES,
                ["egress"],
                _entry_keys(unit, imports) + _advanced_keys(unit, imports),
            )
        )
        rules.append(
            _rule(
                "boundary_import_history_ingress",
                IMPORT_HISTORY_INGRESS_MESSAGES,
                ["ingress"],
                _entry_keys(unit, imports) + _advanced_keys(unit, imports),
            )
        )
    return rules


def _access_control(unit: UnitContext) -> dict[str, Any]:
    internal = _internal_subjects(unit)
    boundary = _boundary_subjects(unit)
    rules = _boundary_rules(unit)
    apply_boundary = bool(boundary) and bool(rules)
    all_rules = [_rule("internal", ALL_MESSAGES, ["egress", "ingress"], INTERNAL_KEYS)]
    subjects = list(internal)
    policies: list[dict[str, Any]] = [
        {
            "id": "internal",
            "rules": ["internal"],
            "subjects": [subject["id"] for subject in internal],
        }
    ]
    if apply_boundary:
        all_rules.extend(rules)
        subjects.extend(boundary)
        policies.append(
            {
                "id": "boundary",
                "rules": [rule["id"] for rule in rules],
                "subjects": [subject["id"] for subject in boundary],
            }
        )
    return {
        "enabled": True,
        "default_permission": "deny",
        "rules": all_rules,
        "subjects": subjects,
        "policies": policies,
    }


def _downsampling(unit: UnitContext) -> list[dict[str, Any]]:
    rules = [
        {"key_expr": data_key(unit.domain_id, entry.topic), "freq": entry.max_hz}
        for entry in unit.boundary.exports
        if entry.max_hz is not None
    ]
    boundary = _boundary_subjects(unit)
    if not rules or not boundary:
        return []
    block: dict[str, Any] = {"id": "boundary_egress"}
    if unit.on_fleet_host:
        block["link_protocols"] = [BOUNDARY_LINK_PROTOCOL]
    else:
        assert unit.topology is not None
        block["interfaces"] = [unit.topology.uplink_interface]
    block["flows"] = ["egress"]
    block["messages"] = ["put"]
    block["rules"] = rules
    return [block]


def unit_router(unit: UnitContext) -> dict[str, Any]:
    if unit.on_fleet_host:
        uplink = f"unixsock-stream/{FLEET_SOCKET_PATH}"
    else:
        assert unit.topology is not None
        uplink = f"tcp/{unit.topology.uplink_address}:{unit.instance.fleet_router_tcp_port}"
    document: dict[str, Any] = {
        "mode": "router",
        "connect": {"endpoints": [uplink]},
        "listen": {"endpoints": [f"tcp/{LOOPBACK_ADDRESS}:{unit.router_tcp_port}"]},
        "scouting": _scouting(),
    }
    document.update(_common(session=False))
    document["access_control"] = _access_control(unit)
    down = _downsampling(unit)
    if down:
        document["downsampling"] = down
    return document


def unit_session(unit: UnitContext) -> dict[str, Any]:
    document: dict[str, Any] = {
        "mode": SESSION_MODE,
        "connect": {"endpoints": [f"tcp/{LOOPBACK_ADDRESS}:{unit.router_tcp_port}"]},
        "scouting": _scouting(),
    }
    document.update(_common(session=True))
    return document


def fleet_router_address(fleet: Fleet) -> str:
    if fleet.mode == SIMULATION:
        return LOOPBACK_ADDRESS
    assert fleet.station is not None
    return site_value(fleet.site, fleet.station.id, UPLINK_ADDRESS_KEY)


def fleet_router(instance: Instance, address: str) -> dict[str, Any]:
    document: dict[str, Any] = {
        "mode": "router",
        "listen": {
            "endpoints": [
                f"tcp/{address}:{instance.fleet_router_tcp_port}",
                f"unixsock-stream/{FLEET_SOCKET_PATH}",
            ]
        },
        "scouting": _scouting(),
    }
    document.update(_common(session=False))
    return document
