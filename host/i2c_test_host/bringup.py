import os
import shlex
import signal
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from pathlib import Path

import yaml

from .backend import TcpLineBackend
from .collect import collect_response_block
from .peers import load_peers, resolve_host


class BringupError(RuntimeError):
    pass


@dataclass(frozen=True)
class PeerEndpoint:
    name: str
    dialect_name: str
    host: str
    port: int


@dataclass(frozen=True)
class BringupTarget:
    name: str
    endpoint: PeerEndpoint
    work_dir: Path
    elf_path: Path | None
    server_cmd: tuple[str, ...]
    gdb_cmd: tuple[str, ...]
    ready_command: str
    ready_pattern: str
    cleanup_patterns: tuple[str, ...] = ()


@dataclass(frozen=True)
class BringupConfig:
    manifest: Path
    log_dir: Path
    targets: tuple[BringupTarget, BringupTarget]
    verify_timeout_s: float
    wait_timeout_s: float = 30.0
    clean_stale_processes: bool = True




def _host_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _read_env_bool(name: str, default: bool) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return default
    return raw.strip().lower() not in {"0", "false", "no", "off", ""}


def _resolve_path(value: str | Path, *, base_dir: Path | None = None) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute() and base_dir is not None:
        path = base_dir / path
    return path.resolve()


def _env_value(*names: str) -> str | None:
    for name in names:
        raw = os.getenv(name)
        if raw is not None and raw != "":
            return raw
    return None


def _parse_int(value, *, setting_name: str) -> int:
    try:
        if isinstance(value, int):
            return value
        return int(str(value), 0)
    except (TypeError, ValueError) as exc:
        raise BringupError(f"invalid integer for {setting_name}: {value!r}") from exc


def _parse_bool(value, *, setting_name: str) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() not in {"0", "false", "no", "off", ""}
    raise BringupError(f"invalid boolean for {setting_name}: {value!r}")


def _setting_str(bringup: dict, key: str, env_names: tuple[str, ...], default: str) -> str:
    if key in bringup:
        return str(bringup[key])
    raw = _env_value(*env_names)
    return default if raw is None else raw


def _setting_int(bringup: dict, key: str, env_names: tuple[str, ...], default: int) -> int:
    if key in bringup:
        return _parse_int(bringup[key], setting_name=key)
    raw = _env_value(*env_names)
    return default if raw is None else _parse_int(raw, setting_name=env_names[0])


def _setting_bool(bringup: dict, key: str, env_names: tuple[str, ...], default: bool) -> bool:
    if key in bringup:
        return _parse_bool(bringup[key], setting_name=key)
    raw = _env_value(*env_names)
    return default if raw is None else _parse_bool(raw, setting_name=env_names[0])


def _command_from_env(name: str, default: tuple[str, ...]) -> tuple[str, ...]:
    raw = os.getenv(name, "").strip()
    if not raw:
        return default
    parts = tuple(shlex.split(raw))
    if not parts:
        raise BringupError(f"environment variable {name} resolved to an empty command")
    return parts


def _endpoint_from_peer(peer) -> PeerEndpoint:
    host = getattr(peer.backend, "host", None)
    port = getattr(peer.backend, "port", None)
    if not isinstance(host, str) or port is None:
        raise BringupError(
            f"peer '{peer.name}' does not expose a TCP RTT endpoint for host bring-up"
        )
    return PeerEndpoint(
        name=peer.name,
        dialect_name=peer.dialect_name,
        host=host,
        port=int(port),
    )


def _build_server_command(
    *,
    server_bin: str,
    host_ip: str,
    jlink_remote_port: int,
    device: str,
    jlink_speed: int,
    gdb_port: int,
    swo_port: int,
    telnet_port: int,
    rtt_port: int,
) -> tuple[str, ...]:
    return (
        server_bin,
        "-select",
        f"IP={host_ip}:{jlink_remote_port}",
        "-device",
        device,
        "-if",
        "SWD",
        "-speed",
        str(jlink_speed),
        "-port",
        str(gdb_port),
        "-swoport",
        str(swo_port),
        "-telnetport",
        str(telnet_port),
        "-RTTTelnetPort",
        str(rtt_port),
        "-noreset",
        "-nogui",
    )


def _build_gdb_command(
    *,
    gdb_bin: str,
    elf_path: Path,
    gdb_port: int,
    gdb_load: bool,
) -> tuple[str, ...]:
    command: list[str] = [
        gdb_bin,
        str(elf_path),
        "-ex",
        "set pagination off",
        "-ex",
        f"target remote :{gdb_port}",
    ]
    if gdb_load:
        command.extend(
            [
                "-ex",
                "monitor reset",
                "-ex",
                "monitor halt",
                "-ex",
                "load",
                "-ex",
                "monitor reset",
                "-ex",
                "monitor halt",
                "-ex",
                "tbreak main",
                "-ex",
                "continue",
                "-ex",
                "continue",
            ]
        )
    else:
        command.extend(["-ex", "continue"])
    return tuple(command)


def _build_cleanup_patterns(
    *,
    server_bin: str,
    gdb_bin: str,
    gdb_port: int,
    rtt_port: int,
) -> tuple[str, ...]:
    return (
        f"{Path(gdb_bin).name} .*target remote :{gdb_port}",
        f"{Path(server_bin).name} .* -port {gdb_port} ",
        f"JLinkRTTClient .* -RTTTelnetPort {rtt_port}",
    )


def _default_target_settings(dialect_name: str) -> dict[str, object]:
    readiness = {
        "session_v1": ("INFO", "INFO protocol=session_v1"),
    }
    try:
        ready_command, ready_pattern = readiness[dialect_name]
    except KeyError as exc:
        raise BringupError(f"host bring-up does not support dialect '{dialect_name}'") from exc
    return {
        "jlink_speed": 4000,
        "gdb_load": True,
        "server_bin": "JLinkGDBServerCLExe",
        "gdb_bin": "arm-none-eabi-gdb",
        "ready_command": ready_command,
        "ready_pattern": ready_pattern,
    }


def _required_value(
    bringup: dict,
    key: str,
    env_names: tuple[str, ...],
    *,
    peer_name: str,
):
    if key in bringup:
        return bringup[key]
    raw = _env_value(*env_names)
    if raw is not None:
        return raw
    raise BringupError(f"peer '{peer_name}' missing required bringup.{key}")


def _target_from_entry(
    entry: dict,
    peer,
    *,
    manifest_dir: Path,
) -> BringupTarget:
    defaults = _default_target_settings(peer.dialect_name)
    env_prefix = peer.name.upper()
    if "bringup" not in entry:
        raise BringupError(f"peer '{peer.name}' missing required bringup mapping")
    bringup = entry["bringup"]
    if not isinstance(bringup, dict):
        raise BringupError(f"peer '{peer.name}' bringup config must be a mapping")

    elf_path = _resolve_path(
        _required_value(bringup, "elf", (f"{env_prefix}_ELF",), peer_name=peer.name),
        base_dir=manifest_dir,
    )
    work_dir = elf_path.parent
    host_ip = resolve_host(_required_value(
        bringup, "host_ip", (f"{env_prefix}_HOST_IP", "HOST_IP"), peer_name=peer.name
    ))
    jlink_remote_port = _parse_int(_required_value(
        bringup, "jlink_remote_port", (f"{env_prefix}_JLINK_REMOTE_PORT",), peer_name=peer.name
    ), setting_name="jlink_remote_port")
    gdb_port = _parse_int(_required_value(
        bringup, "gdb_port", (f"{env_prefix}_GDB_PORT",), peer_name=peer.name
    ), setting_name="gdb_port")
    swo_port = gdb_port + 1
    telnet_port = gdb_port + 2
    device = str(_required_value(
        bringup, "device", (f"{env_prefix}_DEVICE",), peer_name=peer.name
    ))

    jlink_speed = _setting_int(
        bringup, "jlink_speed", (f"{env_prefix}_JLINK_SPEED",), int(defaults["jlink_speed"])
    )
    gdb_load = _setting_bool(
        bringup, "gdb_load", (f"{env_prefix}_GDB_LOAD",), bool(defaults["gdb_load"])
    )
    ready_command = _setting_str(
        bringup, "ready_command", (), str(defaults["ready_command"])
    )
    ready_pattern = _setting_str(
        bringup, "ready_pattern", (f"{env_prefix}_READY_PATTERN",), str(defaults["ready_pattern"])
    )
    server_bin = _setting_str(
        bringup, "server_bin", (f"{env_prefix}_SERVER_BIN", "JLINK_GDB_SERVER_BIN"),
        str(defaults["server_bin"]),
    )
    gdb_bin = _setting_str(
        bringup, "gdb_bin", (f"{env_prefix}_GDB_BIN", "GDB_BIN"), str(defaults["gdb_bin"])
    )
    server_cmd_override = _command_from_env(f"{env_prefix}_SERVER_CMD", ())
    gdb_cmd_override = _command_from_env(f"{env_prefix}_GDB_CMD", ())
    endpoint = _endpoint_from_peer(peer)
    server_cmd = server_cmd_override or _build_server_command(
        server_bin=server_bin,
        host_ip=host_ip,
        jlink_remote_port=jlink_remote_port,
        device=device,
        jlink_speed=jlink_speed,
        gdb_port=gdb_port,
        swo_port=swo_port,
        telnet_port=telnet_port,
        rtt_port=endpoint.port,
    )
    gdb_cmd = gdb_cmd_override or _build_gdb_command(
        gdb_bin=gdb_bin,
        elf_path=elf_path,
        gdb_port=gdb_port,
        gdb_load=gdb_load,
    )
    cleanup_patterns = _build_cleanup_patterns(
        server_bin=server_cmd[0],
        gdb_bin=gdb_cmd[0],
        gdb_port=gdb_port,
        rtt_port=endpoint.port,
    )
    return BringupTarget(
        name=peer.name,
        endpoint=endpoint,
        work_dir=work_dir,
        elf_path=None if gdb_cmd_override else elf_path,
        server_cmd=server_cmd,
        gdb_cmd=gdb_cmd,
        ready_command=ready_command,
        ready_pattern=ready_pattern,
        cleanup_patterns=cleanup_patterns,
    )


def load_bringup_config(
    manifest: str | Path,
    *,
    log_dir: str | Path | None = None,
    wait_timeout_s: float | None = None,
    verify_timeout_s: float | None = None,
    clean_stale_processes: bool | None = None,
) -> BringupConfig:
    manifest_path = _resolve_path(manifest)
    data = yaml.safe_load(manifest_path.read_text(encoding="utf-8")) or {}
    peer_entries = data.get("peers") or []
    dut_entries = [entry for entry in peer_entries if entry.get("kind") == "dut"]
    peers = load_peers(manifest_path)

    if len(peers) != 2 or len(dut_entries) != 2:
        raise BringupError("host bring-up requires exactly 2 DUT peers")

    logs_root = _resolve_path(log_dir or (_host_root() / ".pytest_cache" / "bringup"))
    built_targets = tuple(
        _target_from_entry(
            entry,
            peer,
            manifest_dir=manifest_path.parent,
        )
        for entry, peer in zip(dut_entries, peers, strict=True)
    )
    targets = (built_targets[0], built_targets[1])

    return BringupConfig(
        manifest=manifest_path,
        log_dir=logs_root,
        targets=targets,
        wait_timeout_s=float(wait_timeout_s or os.getenv("WAIT_TIMEOUT_S", 30)),
        verify_timeout_s=float(verify_timeout_s or os.getenv("VERIFY_TIMEOUT_S", 8)),
        clean_stale_processes=(
            _read_env_bool("CLEAN_STALE_PROCESSES", True)
            if clean_stale_processes is None
            else bool(clean_stale_processes)
        ),
    )


class BringupStack:
    def __init__(self, config: BringupConfig):
        self.config = config
        self._processes: list[tuple[str, subprocess.Popen[str], Path]] = []
        self._processes_lock = threading.Lock()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc, tb):
        self.stop()
        return False

    def start(self) -> None:
        if self._processes:
            return

        for target in self.config.targets:
            self._require_dir(target.work_dir)
            if target.elf_path is not None:
                self._require_file(target.elf_path)
        self.config.log_dir.mkdir(parents=True, exist_ok=True)

        if self.config.clean_stale_processes:
            self.cleanup_stale_processes()

        try:
            with ThreadPoolExecutor(max_workers=len(self.config.targets)) as pool:
                for future in [
                    pool.submit(self._bring_up_target, target)
                    for target in self.config.targets
                ]:
                    future.result()
        except Exception:
            self.stop()
            raise

    def _bring_up_target(self, target: BringupTarget) -> None:
        self._start_target(
            name=f"{target.name}-server",
            cwd=target.work_dir,
            command=target.server_cmd,
            wait_for="Waiting for GDB connection",
        )
        self._start_target(
            name=f"{target.name}-gdb",
            cwd=target.work_dir,
            command=target.gdb_cmd,
        )
        self._wait_for_target_ready(target)

    def stop(self) -> None:
        for _, process, _ in reversed(self._processes):
            self._terminate_process_group(process)
        self._processes.clear()

        if self.config.clean_stale_processes:
            self.cleanup_stale_processes()

    def cleanup_stale_processes(self) -> None:
        patterns = self._stale_patterns()
        if not patterns:
            return
        if not shutil_which("pgrep") or not shutil_which("pkill"):
            return

        self._kill_patterns("-TERM", patterns)
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if not any(self._pattern_is_running(pattern) for pattern in patterns):
                return
            time.sleep(0.2)

        self._kill_patterns("-KILL", patterns)

    def _stale_patterns(self) -> tuple[str, ...]:
        patterns: list[str] = []
        for target in self.config.targets:
            patterns.extend(target.cleanup_patterns)
        return tuple(dict.fromkeys(patterns))

    def _start_target(
        self,
        *,
        name: str,
        cwd: Path,
        command: tuple[str, ...],
        wait_for: str | None = None,
    ) -> None:
        log_path = self.config.log_dir / f"{name}.log"
        log_path.write_text("", encoding="utf-8")
        log_file = log_path.open("a", encoding="utf-8")
        try:
            process = subprocess.Popen(
                command,
                cwd=str(cwd),
                stdin=subprocess.PIPE,
                stdout=log_file,
                stderr=subprocess.STDOUT,
                text=True,
                start_new_session=True,
            )
        finally:
            log_file.close()

        with self._processes_lock:
            self._processes.append((name, process, log_path))
        if wait_for is not None:
            self._wait_for_log_text(name, process, log_path, wait_for)

    def _wait_for_log_text(
        self,
        name: str,
        process: subprocess.Popen[str],
        log_path: Path,
        needle: str,
    ) -> None:
        deadline = time.monotonic() + self.config.wait_timeout_s
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise BringupError(
                    f"{name} exited early with code {process.returncode}\n{self._tail_log(log_path)}"
                )

            if needle in log_path.read_text(encoding="utf-8", errors="ignore"):
                return
            time.sleep(0.2)

        raise BringupError(
            f"timed out waiting for '{needle}' in {name}\n{self._tail_log(log_path)}"
        )

    def _wait_for_target_ready(self, target: BringupTarget) -> None:
        deadline = time.monotonic() + self.config.wait_timeout_s
        last_raw = ""
        while time.monotonic() < deadline:
            backend = TcpLineBackend(
                host=target.endpoint.host,
                port=target.endpoint.port,
            )
            try:
                backend.open()
                backend.write_line(target.ready_command)
                raw = collect_response_block(
                    backend,
                    idle_timeout_s=0.1,
                    max_total_s=self.config.verify_timeout_s,
                    poll_timeout_s=0.02,
                )
                if target.ready_pattern in raw:
                    return
                last_raw = raw
            except Exception as exc:
                last_raw = str(exc)
            finally:
                backend.close()
            time.sleep(0.2)

        raise BringupError(
            f"{target.name} ({target.endpoint.dialect_name}) RTT did not answer "
            f"{target.ready_command!r} with '{target.ready_pattern}'"
            f"\nlast response:\n{last_raw}"
        )

    @staticmethod
    def _require_dir(path: Path) -> None:
        if not path.is_dir():
            raise BringupError(f"directory not found: {path}")

    @staticmethod
    def _require_file(path: Path) -> None:
        if not path.is_file():
            raise BringupError(f"ELF not found: {path}")

    def _kill_patterns(self, signal_name: str, patterns: tuple[str, ...]) -> None:
        for pattern in patterns:
            subprocess.run(
                ["pkill", signal_name, "-f", pattern],
                check=False,
                capture_output=True,
                text=True,
            )

    @staticmethod
    def _pattern_is_running(pattern: str) -> bool:
        result = subprocess.run(
            ["pgrep", "-f", pattern],
            check=False,
            capture_output=True,
            text=True,
        )
        return result.returncode == 0

    @staticmethod
    def _terminate_process_group(process: subprocess.Popen[str]) -> None:
        if process.poll() is not None:
            return

        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            if process.poll() is not None:
                return
            time.sleep(0.1)

        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            return
        process.wait(timeout=1.0)

    @staticmethod
    def _tail_log(path: Path, lines: int = 40) -> str:
        try:
            content = path.read_text(encoding="utf-8", errors="ignore").splitlines()
        except OSError:
            return f"unable to read log: {path}"
        tail = "\n".join(content[-lines:])
        return f"log: {path}\n{tail}"


def shutil_which(name: str) -> str | None:
    for entry in os.getenv("PATH", "").split(os.pathsep):
        if not entry:
            continue
        candidate = Path(entry) / name
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    return None
