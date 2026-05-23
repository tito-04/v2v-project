from collections import deque
from statistics import mean
from typing import Any


class NetworkMetrics:
    def __init__(self, loss_timeout_seconds: float = 3.0, timeline_limit: int = 80) -> None:
        self.loss_timeout_seconds = loss_timeout_seconds
        self.timeline_limit = timeline_limit
        self._stats: dict[str, dict[str, Any]] = {}
        self._timeline: deque[dict[str, Any]] = deque(maxlen=timeline_limit)

    def record_tx(self, message_type: str, sequence: int, generated_at: float, now: float | None = None) -> None:
        clock = generated_at if now is None else now
        stats = self._ensure(message_type)
        stats["sent"] += 1
        stats["pending"][sequence] = {
            "sequence": sequence,
            "generated_at": generated_at,
            "sent_at": clock,
        }
        self._timeline.append({
            "type": message_type,
            "sequence": sequence,
            "status": "sent",
            "timestamp": clock,
        })

    def record_rx(self, message_type: str, now: float) -> float | None:
        stats = self._ensure(message_type)
        stats["received"] += 1
        match = self._pop_oldest_pending(stats)
        if match is None:
            self._timeline.append({
                "type": message_type,
                "sequence": None,
                "status": "received-unmatched",
                "timestamp": now,
            })
            return None

        delay = max(now - float(match["generated_at"]), 0.0)
        stats["delays"].append(delay)
        self._timeline.append({
            "type": message_type,
            "sequence": match["sequence"],
            "status": "received",
            "timestamp": now,
            "delay_sec": delay,
        })
        return delay

    def sweep(self, now: float) -> None:
        for message_type, stats in self._stats.items():
            expired = [
                sequence for sequence, item in stats["pending"].items()
                if now - float(item["generated_at"]) > self.loss_timeout_seconds
            ]
            for sequence in expired:
                item = stats["pending"].pop(sequence)
                stats["lost"] += 1
                self._timeline.append({
                    "type": message_type,
                    "sequence": sequence,
                    "status": "lost",
                    "timestamp": now,
                    "age_sec": now - float(item["generated_at"]),
                })

    def snapshot(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "timeline": list(self._timeline)[-self.timeline_limit:],
        }
        for message_type, stats in self._stats.items():
            sent = int(stats["sent"])
            received = int(stats["received"])
            lost = int(stats["lost"])
            delays = list(stats["delays"])
            payload[message_type] = {
                "sent": sent,
                "received": received,
                "lost": lost,
                "pending": len(stats["pending"]),
                "loss_percent": round((lost / sent) * 100.0, 1) if sent else 0.0,
                "delay_avg_sec": round(mean(delays), 3) if delays else None,
                "delay_p95_sec": round(self._percentile(delays, 0.95), 3) if delays else None,
                "jitter_sec": round(self._jitter(delays), 3) if len(delays) > 1 else 0.0,
                "last_delay_sec": round(delays[-1], 3) if delays else None,
            }
        for message_type in ("cam", "cpm"):
            payload.setdefault(message_type, {
                "sent": 0,
                "received": 0,
                "lost": 0,
                "pending": 0,
                "loss_percent": 0.0,
                "delay_avg_sec": None,
                "delay_p95_sec": None,
                "jitter_sec": 0.0,
                "last_delay_sec": None,
            })
        return payload

    def _ensure(self, message_type: str) -> dict[str, Any]:
        if message_type not in self._stats:
            self._stats[message_type] = {
                "sent": 0,
                "received": 0,
                "lost": 0,
                "pending": {},
                "delays": deque(maxlen=80),
            }
        return self._stats[message_type]

    @staticmethod
    def _pop_oldest_pending(stats: dict[str, Any]) -> dict[str, Any] | None:
        pending = stats["pending"]
        if not pending:
            return None
        sequence = min(pending, key=lambda seq: pending[seq]["generated_at"])
        return pending.pop(sequence)

    @staticmethod
    def _percentile(values: list[float], percentile: float) -> float:
        ordered = sorted(values)
        index = min(len(ordered) - 1, max(0, int(round((len(ordered) - 1) * percentile))))
        return ordered[index]

    @staticmethod
    def _jitter(values: list[float]) -> float:
        diffs = [abs(values[idx] - values[idx - 1]) for idx in range(1, len(values))]
        return mean(diffs) if diffs else 0.0
