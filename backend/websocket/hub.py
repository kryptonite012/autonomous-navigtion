"""WebSocket fan-out hub.

Robot threads call ``publish`` from any thread; messages are queued per
client and slow clients drop their oldest messages instead of blocking.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from dataclasses import dataclass, field
from typing import Any, Optional

from fastapi import WebSocket

log = logging.getLogger(__name__)

TOPICS = ("telemetry", "pose", "detections", "navigation", "sorting", "events", "alerts", "map", "state")


@dataclass(eq=False)
class Client:
    ws: WebSocket
    username: str
    role: str
    topics: set[str] = field(default_factory=lambda: set(TOPICS))
    queue: "asyncio.Queue[str]" = field(default_factory=lambda: asyncio.Queue(maxsize=100))
    dropped: int = 0


class Hub:
    def __init__(self):
        self.clients: set[Client] = set()
        self.loop: Optional[asyncio.AbstractEventLoop] = None

    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        self.loop = loop

    def _enqueue(self, topic: str, payload: str) -> None:
        for c in list(self.clients):
            if topic not in c.topics:
                continue
            if c.queue.full():
                try:
                    c.queue.get_nowait()
                    c.dropped += 1
                except asyncio.QueueEmpty:
                    pass
            c.queue.put_nowait(payload)

    def publish(self, topic: str, data: Any) -> None:
        if not self.clients or self.loop is None:
            return
        payload = json.dumps({"topic": topic, "ts": time.time(), "data": data}, default=str)
        try:
            running = asyncio.get_running_loop()
        except RuntimeError:
            running = None
        if running is self.loop:
            self._enqueue(topic, payload)
        else:
            self.loop.call_soon_threadsafe(self._enqueue, topic, payload)

    async def serve(self, client: Client) -> None:
        self.clients.add(client)
        sender = asyncio.create_task(self._sender(client))
        try:
            while True:
                msg = await client.ws.receive_text()
                self._handle_client_message(client, msg)
        except Exception:
            pass
        finally:
            sender.cancel()
            self.clients.discard(client)

    async def _sender(self, client: Client) -> None:
        while True:
            payload = await client.queue.get()
            await client.ws.send_text(payload)

    @staticmethod
    def _handle_client_message(client: Client, msg: str) -> None:
        # clients may narrow their subscriptions: {"subscribe": ["telemetry", "pose"]}
        try:
            data = json.loads(msg)
        except ValueError:
            return
        if isinstance(data, dict) and isinstance(data.get("subscribe"), list):
            client.topics = {t for t in data["subscribe"] if t in TOPICS}
