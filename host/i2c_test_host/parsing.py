import re

HEX_BYTE_RE = re.compile(r"0x([0-9a-fA-F]{2})")
STATUS_RE = re.compile(r"status\s*=\s*([A-Za-z0-9_]+)", re.IGNORECASE)
DATA_SPLIT_RE = re.compile(r"\bdata\s*:\s*", re.IGNORECASE)
ADDR_RE = re.compile(r"\baddr\s*:\s*(0x[0-9a-fA-F]+|\d+)", re.IGNORECASE)


def parse_status(raw: str) -> str | None:
    matches = STATUS_RE.findall(raw)
    if not matches:
        return None
    return matches[-1]


def parse_addr(raw: str) -> int | None:
    matches = ADDR_RE.findall(raw)
    if not matches:
        return None
    return int(matches[-1], 0)


def parse_data(raw: str) -> list[int]:
    lines = raw.splitlines()
    for index in range(len(lines) - 1, -1, -1):
        line = lines[index]
        match = DATA_SPLIT_RE.search(line)
        if not match:
            continue
        return [int(token, 16) for token in HEX_BYTE_RE.findall(line[match.end() :])]

    return []


def parse_fields(raw: str, head: str) -> dict[str, str]:
    """Return the key=value fields of the last line whose first token is head (case-insensitive)."""
    for line in reversed(raw.splitlines()):
        tokens = line.split()
        if tokens and tokens[0].lower() == head.lower():
            return dict(token.split("=", 1) for token in tokens[1:] if "=" in token)
    return {}
