"""Goal queue (D12): FIFO, deduplicated, max 3."""
from __future__ import annotations

from collections import deque


class GoalQueue:
    def __init__(self, max_len: int = 3):
        self.max_len = max_len
        self._q: deque = deque()

    def push(self, marker_id, current=None) -> str:
        """Returns 'queued', 'duplicate', 'current' or 'full'."""
        if marker_id == current:
            return "current"
        if marker_id in self._q:
            return "duplicate"
        if len(self._q) >= self.max_len:
            return "full"
        self._q.append(marker_id)
        return "queued"

    def pop(self):
        return self._q.popleft() if self._q else None

    def clear(self):
        self._q.clear()

    def items(self) -> list:
        return list(self._q)

    def __len__(self):
        return len(self._q)

    def __bool__(self):
        return bool(self._q)
