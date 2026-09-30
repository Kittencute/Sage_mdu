from __future__ import annotations

import logging
import os
import select
import signal
import socket
import subprocess
import time
from dataclasses import dataclass, field
from graphlib import TopologicalSorter
from pathlib import Path
from typing import IO, TYPE_CHECKING

from fleet_unit import commands, state
from fleet_unit.errors import UnitError
from fleet_unit.manifest import (
    ROUTER_NAME,
    ComponentSpec,
    ExitProbe,
    Manifest,
    MessageProbe,
    NodeProbe,
    Probe,
    Process,
    TcpProbe,
    TopicProbe,
)
from fleet_unit.state import ProcessState, UnitState

if TYPE_CHECKING:
    from rclpy.task import Future

    from fleet_unit.ros_probe import RosGraphProbe

ROUTER_KIND = "router"

SETTLED_STATES = (state.READY, state.WAITING)

UNIT_STARTING = "starting"
UNIT_RUNNING = "running"
UNIT_STOPPING = "stopping"
UNIT_STOPPED = "stopped"
UNIT_FAILED = "failed"


@dataclass
class _Runtime:
    spec: Process
    status: ProcessState
    is_router: bool
    after: tuple[str, ...] = ()
    pending_components: list[ComponentSpec] = field(default_factory=list)
    loading: ComponentSpec | None = None
    load_future: Future | None = None
    popen: subprocess.Popen[bytes] | None = None
    log_stream: IO[bytes] | None = None


def _router_process(manifest: Manifest) -> Process:
    return Process(
        name=ROUTER_NAME,
        kind=ROUTER_KIND,
        ready=TcpProbe("127.0.0.1", manifest.router.tcp_port),
    )


def _describe(probe: Probe) -> str:
    if isinstance(probe, NodeProbe):
        return f"node {probe.node}"
    if isinstance(probe, TopicProbe):
        return f"a publisher on {probe.topic}"
    if isinstance(probe, MessageProbe):
        return f"a message on {probe.message}"
    if isinstance(probe, TcpProbe):
        return f"tcp {probe.host}:{probe.port}"
    return f"exit status {probe.exit}"


class UnitSupervisor:
    def __init__(self, manifest: Manifest) -> None:
        self._manifest = manifest
        self._probe: RosGraphProbe | None = None
        self._log = logging.getLogger("fleet_unit.supervisor")
        self._failure: str | None = None
        self._signal: int | None = None
        self._runtimes: dict[str, _Runtime] = {}
        self._order: list[str] = []
        self._build_runtimes()
        self._wakeup_read = -1
        self._wakeup_write = -1

    def _build_runtimes(self) -> None:
        router = _router_process(self._manifest)
        self._runtimes[ROUTER_NAME] = _Runtime(
            spec=router,
            status=ProcessState(name=ROUTER_NAME, kind=ROUTER_KIND, state=state.PENDING),
            is_router=True,
            after=(),
        )
        for spec in self._manifest.processes:
            for component in spec.load:
                commands.component_parameters(self._manifest, spec, component)
            self._runtimes[spec.name] = _Runtime(
                spec=spec,
                status=ProcessState(name=spec.name, kind=spec.kind, state=state.PENDING),
                is_router=False,
                after=spec.after if ROUTER_NAME in spec.after else (ROUTER_NAME,) + spec.after,
            )
        self._order = list(
            TopologicalSorter(
                {name: runtime.after for name, runtime in self._runtimes.items()}
            ).static_order()
        )

    def _prefix(self, name: str) -> str:
        return f"[{self._manifest.unit}/{name}]"

    def _unit_prefix(self) -> str:
        return f"[{self._manifest.unit}]"

    def run(self) -> int:
        self._install_signal_handlers()
        try:
            self._write_state(UNIT_STARTING)
            self._supervise()
        except Exception as exc:
            self._record_failure("supervise unit", exc)
        finally:
            self._shutdown()
        self._write_state(UNIT_FAILED if self._failure is not None else UNIT_STOPPED)
        return 0 if self._failure is None else 1

    def _supervise(self) -> None:
        while self._signal is None and self._failure is None:
            self._reap()
            if self._failure is None:
                self._advance()
            self._write_state(self._tick_state())
            if self._failure is not None or self._signal is not None:
                return
            self._wait_tick()

    def _shutdown(self) -> None:
        if self._failure is not None:
            self._log.error("%s unit failed: %s", self._unit_prefix(), self._failure)
        else:
            self._log.info(
                "%s unit stopping on signal %s",
                self._unit_prefix(),
                signal.Signals(self._signal).name if self._signal else "none",
            )
        try:
            self._write_state(UNIT_STOPPING)
        except Exception as exc:
            self._record_failure("write unit state", exc)
        try:
            self._stop_all(self._signal if self._signal is not None else signal.SIGTERM)
        except Exception as exc:
            self._record_failure("stop processes", exc)
        try:
            self._close_probe()
        except Exception as exc:
            self._record_failure("close graph probe", exc)
        self._restore_signal_handlers()

    def _record_failure(self, operation: str, exc: Exception) -> None:
        if isinstance(exc, UnitError):
            failure = exc
            self._log.error("%s", failure)
        else:
            failure = UnitError(operation, repr(exc), self._manifest.unit)
            self._log.error("%s", failure, exc_info=exc)
        if self._failure is None:
            self._failure = str(failure)

    def _install_signal_handlers(self) -> None:
        self._wakeup_read, self._wakeup_write = os.pipe()
        os.set_blocking(self._wakeup_write, False)
        os.set_blocking(self._wakeup_read, False)
        signal.set_wakeup_fd(self._wakeup_write)
        signal.signal(signal.SIGTERM, self._on_signal)
        signal.signal(signal.SIGINT, self._on_signal)

    def _restore_signal_handlers(self) -> None:
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        signal.set_wakeup_fd(-1)
        os.close(self._wakeup_write)
        os.close(self._wakeup_read)

    def _on_signal(self, signum: int, _frame: object) -> None:
        if self._signal is None:
            self._signal = signum

    def _wait_tick(self) -> None:
        if self._probe is not None:
            self._probe.spin(self._manifest.timeouts.probe_interval_s)
            timeout_s = 0.0
        else:
            timeout_s = self._manifest.timeouts.probe_interval_s
        readable, _, _ = select.select([self._wakeup_read], [], [], timeout_s)
        if readable:
            try:
                os.read(self._wakeup_read, 4096)
            except BlockingIOError:
                return

    def _tick_state(self) -> str:
        if self._failure is not None:
            return UNIT_FAILED
        return UNIT_RUNNING if self._settled() else UNIT_STARTING

    def _settled(self) -> bool:
        return all(runtime.status.state in SETTLED_STATES for runtime in self._runtimes.values())

    def _reap(self) -> None:
        for name in self._order:
            runtime = self._runtimes[name]
            popen = runtime.popen
            if popen is None:
                continue
            code = popen.poll()
            if code is None:
                continue
            self._close_log(runtime)
            runtime.popen = None
            status = runtime.status
            status.pid = None
            if code < 0:
                status.signal = -code
                status.exit_code = None
                detail = f"signal {signal.Signals(-code).name}"
            else:
                status.signal = None
                status.exit_code = code
                detail = f"status {code}"
            probe = runtime.spec.ready
            if (
                isinstance(probe, ExitProbe)
                and code == probe.exit
                and status.state == state.STARTING
            ):
                self._mark_ready(runtime)
                continue
            self._log.warning("%s exited: %s", self._prefix(name), detail)
            self._handle_failure(runtime, f"exited: {detail}")

    def _advance(self) -> None:
        for name in self._order:
            runtime = self._runtimes[name]
            status = runtime.status
            if status.state == state.PENDING:
                unready = self._unready_dependencies(runtime)
                if unready:
                    status.detail = f"waiting for {', '.join(unready)}"
                else:
                    self._begin(runtime)
            elif status.state == state.WAITING:
                self._await_precondition(runtime)
            elif status.state == state.STARTING:
                self._evaluate(runtime)
            if self._failure is not None:
                return

    def _begin(self, runtime: _Runtime) -> None:
        precondition = runtime.spec.wait_for
        if precondition is None or Path(precondition.path).exists():
            self._start(runtime)
            return
        runtime.status.state = state.WAITING
        runtime.status.ready = False
        runtime.status.detail = f"waiting for {precondition.path}"
        self._log.info("%s waiting for %s", self._prefix(runtime.spec.name), precondition.path)

    def _await_precondition(self, runtime: _Runtime) -> None:
        precondition = runtime.spec.wait_for
        if precondition is None:
            raise UnitError(
                "await start precondition",
                "process is waiting without a wait_for precondition",
                self._manifest.unit,
                runtime.spec.name,
            )
        if Path(precondition.path).exists():
            runtime.status.detail = ""
            self._start(runtime)

    def _unready_dependencies(self, runtime: _Runtime) -> list[str]:
        return [
            dependency
            for dependency in runtime.after
            if self._runtimes[dependency].status.state != state.READY
        ]

    def _start(self, runtime: _Runtime) -> None:
        spec = runtime.spec
        status = runtime.status
        status.detail = ""
        status.exit_code = None
        status.signal = None
        status.ready = False

        if not runtime.is_router:
            commands.stage_files(self._manifest, spec)

        if spec.kind == "components":
            status.state = state.STARTING
            self._load_components(runtime)
            return

        if runtime.is_router:
            argv = commands.router_argv(self._manifest)
            environment = dict(os.environ)
            cwd = None
        else:
            argv = commands.process_argv(self._manifest, spec)
            environment = commands.process_environment(self._manifest, spec)
            cwd = commands.prepare_cwd(self._manifest, spec)

        stream = self._open_log(runtime)
        self._log.info("%s starting %s", self._prefix(spec.name), argv[0])
        try:
            popen = subprocess.Popen(
                argv,
                cwd=cwd,
                env=environment,
                stdin=subprocess.DEVNULL,
                stdout=stream,
                stderr=subprocess.STDOUT if stream is not None else None,
                process_group=0,
            )
        except OSError as exc:
            self._close_log(runtime)
            raise UnitError(
                f"start process {spec.name}", exc, self._manifest.unit, spec.name
            ) from exc
        runtime.popen = popen
        status.pid = popen.pid
        status.state = state.STARTING
        if spec.ready is None:
            self._mark_ready(runtime)

    def _open_log(self, runtime: _Runtime) -> IO[bytes] | None:
        if runtime.spec.output != "log":
            return None
        path = commands.log_path(self._manifest, runtime.spec)
        try:
            path.parent.mkdir(parents=True, exist_ok=True)
            stream = path.open("ab")
        except OSError as exc:
            raise UnitError(
                f"open log file {path}", exc, self._manifest.unit, runtime.spec.name
            ) from exc
        runtime.log_stream = stream
        return stream

    def _close_log(self, runtime: _Runtime) -> None:
        stream = runtime.log_stream
        if stream is None:
            return
        runtime.log_stream = None
        stream.close()

    def _load_components(self, runtime: _Runtime) -> None:
        runtime.pending_components = list(runtime.spec.load)
        runtime.loading = None
        runtime.load_future = None
        self._continue_loading(runtime)

    def _continue_loading(self, runtime: _Runtime) -> bool:
        spec = runtime.spec
        container = self._manifest.process(spec.container or "")
        fqn = commands.container_fqn(self._manifest, container)
        while True:
            future = runtime.load_future
            if future is not None:
                if not future.done():
                    return False
                loaded = runtime.loading
                assert loaded is not None
                response = future.result()
                runtime.load_future = None
                runtime.loading = None
                if not response.success:
                    self._handle_failure(
                        runtime,
                        f"load component {loaded.plugin} into {fqn}: {response.error_message}",
                    )
                    return False
            if not runtime.pending_components:
                return True
            component = runtime.pending_components[0]
            future = self._require_probe(runtime).request_load(
                fqn,
                component.package,
                component.plugin,
                component.node_name,
                component.namespace,
                dict(component.remaps),
                commands.component_parameters(self._manifest, spec, component),
            )
            if future is None:
                runtime.status.detail = f"waiting for service {fqn}/_container/load_node"
                return False
            self._log.info("%s loading %s into %s", self._prefix(spec.name), component.plugin, fqn)
            runtime.status.detail = f"loading {component.plugin} into {fqn}"
            runtime.pending_components.pop(0)
            runtime.loading = component
            runtime.load_future = future

    def _mark_ready(self, runtime: _Runtime) -> None:
        status = runtime.status
        status.state = state.READY
        status.ready = True
        self._log.info("%s ready", self._prefix(runtime.spec.name))
        if runtime.is_router and self._probe is None:
            self._log.info(
                "%s opening the graph probe session on the unit router",
                self._unit_prefix(),
            )
            from fleet_unit.ros_probe import RosGraphProbe

            self._probe = RosGraphProbe(self._manifest)

    def _evaluate(self, runtime: _Runtime) -> None:
        if runtime.spec.kind == "components" and not self._continue_loading(runtime):
            return
        if self._probe_satisfied(runtime):
            runtime.status.detail = ""
            self._mark_ready(runtime)
            return
        probe = runtime.spec.ready
        assert probe is not None
        detail = f"waiting for {_describe(probe)}"
        if runtime.status.detail != detail:
            runtime.status.detail = detail
            self._log.info("%s %s", self._prefix(runtime.spec.name), detail)

    def _require_probe(self, runtime: _Runtime) -> RosGraphProbe:
        if self._probe is None:
            raise UnitError(
                "query the ROS graph",
                "the probe session is not open because the unit router is not ready",
                self._manifest.unit,
                runtime.spec.name,
            )
        return self._probe

    def _close_probe(self) -> None:
        probe = self._probe
        if probe is None:
            return
        self._probe = None
        probe.close()

    def _probe_satisfied(self, runtime: _Runtime) -> bool:
        probe = runtime.spec.ready
        if probe is None:
            return True
        if isinstance(probe, ExitProbe):
            return False
        if isinstance(probe, TcpProbe):
            return self._tcp_open(probe)
        if isinstance(probe, NodeProbe):
            return self._require_probe(runtime).node_exists(probe.node)
        if isinstance(probe, TopicProbe):
            return self._require_probe(runtime).topic_has_publisher(probe.topic)
        return self._require_probe(runtime).message_received(probe.message)

    def _tcp_open(self, probe: TcpProbe) -> bool:
        try:
            with socket.create_connection(
                (probe.host, probe.port), timeout=self._manifest.timeouts.probe_interval_s
            ):
                return True
        except (OSError, ValueError):
            return False

    def _handle_failure(self, runtime: _Runtime, detail: str) -> None:
        status = runtime.status
        status.ready = False
        status.state = state.FAILED
        status.detail = detail
        self._failure = f"{self._prefix(runtime.spec.name)} {detail}"

    def _stop_all(self, signal_number: int) -> None:
        running: list[_Runtime] = []
        for name in reversed(self._order):
            runtime = self._runtimes[name]
            if runtime.status.state != state.FAILED:
                runtime.status.detail = ""
            if runtime.popen is not None:
                running.append(runtime)
            elif runtime.status.state not in (state.READY, state.FAILED):
                runtime.status.state = state.STOPPED
        grace = self._manifest.timeouts.shutdown_grace_s
        deadline = time.monotonic() + grace
        unstopped: list[_Runtime] = []
        for group in (
            [runtime for runtime in running if not runtime.is_router],
            [runtime for runtime in running if runtime.is_router],
        ):
            for runtime in group:
                self._signal_process(runtime, signal_number)
            unstopped += self._unexited(group, deadline)
        for runtime in unstopped:
            self._log.warning("%s did not stop in %.3fs", self._prefix(runtime.spec.name), grace)
            runtime.status.detail = f"killed after not stopping in {grace}s"
            self._signal_process(runtime, signal.SIGKILL)
        unkilled = self._unexited(unstopped, time.monotonic() + grace)
        for runtime in running:
            self._close_log(runtime)
            runtime.popen = None
            runtime.status.pid = None
            runtime.status.ready = False
            runtime.status.state = state.STOPPED
        if unstopped:
            names = ", ".join(runtime.spec.name for runtime in unstopped)
            self._record_failure(
                "stop processes",
                UnitError("stop processes", f"{names} required SIGKILL", self._manifest.unit),
            )
        if unkilled:
            names = ", ".join(runtime.spec.name for runtime in unkilled)
            self._record_failure(
                "stop processes",
                UnitError(
                    "stop processes", f"{names} did not exit after SIGKILL", self._manifest.unit
                ),
            )

    def _unexited(self, runtimes: list[_Runtime], deadline: float) -> list[_Runtime]:
        unexited: list[_Runtime] = []
        for runtime in runtimes:
            assert runtime.popen is not None
            try:
                runtime.popen.wait(timeout=max(0.0, deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                unexited.append(runtime)
        return unexited

    def _signal_process(self, runtime: _Runtime, signal_number: int) -> None:
        assert runtime.popen is not None
        self._log.info(
            "%s sending %s", self._prefix(runtime.spec.name), signal.Signals(signal_number).name
        )
        try:
            self._signal_group(runtime.popen.pid, signal_number)
        except UnitError as exc:
            self._record_failure("signal process group", exc)

    def _signal_group(self, pid: int, signal_number: int) -> None:
        try:
            os.killpg(pid, signal_number)
        except ProcessLookupError:
            return
        except PermissionError as exc:
            raise UnitError(
                f"signal process group {pid} with {signal.Signals(signal_number).name}",
                exc,
                self._manifest.unit,
            ) from exc

    def _write_state(self, unit_state: str) -> None:
        processes = [self._runtimes[name].status for name in self._order]
        document = UnitState(
            unit=self._manifest.unit,
            runner_pid=os.getpid(),
            updated_at=time.time(),
            state=unit_state,
            detail="; ".join(
                f"{process.name} {process.detail}"
                for process in processes
                if process.state != state.READY and process.detail
            ),
            processes=processes,
        )
        state.write_state(self._manifest.state_file, document)
