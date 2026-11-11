import time


def collect_response_block(
    backend,
    idle_timeout_s: float,
    max_total_s: float,
    poll_timeout_s: float,
) -> str:
    now = time.monotonic
    end_total = now() + max_total_s
    lines: list[str] = []
    last_activity = now()

    while now() < end_total:
        line = backend.read_line(timeout_s=poll_timeout_s)
        if line is None:
            if lines and (now() - last_activity) >= idle_timeout_s:
                break
            continue
        lines.append(line)
        last_activity = now()

    return "\n".join(lines)
