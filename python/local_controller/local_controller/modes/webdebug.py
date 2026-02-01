"""Web debug mode served with Flask."""
from __future__ import annotations

import logging
import queue
import threading
import time
from typing import Any, Iterator, Mapping, Set

from flask import Flask, Response, jsonify, request
from flask.typing import ResponseReturnValue

from ..serial_client import SerialCommandClient
from .base import ControlMode

logger = logging.getLogger(__name__)

INDEX_HTML = """<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>串口 Web Debug</title>
  <style>
    :root {
      --bg: #0b1220;
      --panel: rgba(255, 255, 255, 0.08);
      --border: rgba(255, 255, 255, 0.15);
      --accent: #4fd1c5;
      --text: #e5ecf4;
      --muted: #9fb2c8;
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      background: radial-gradient(circle at 20% 20%, rgba(79, 209, 197, 0.12), transparent 30%),
                  radial-gradient(circle at 80% 10%, rgba(99, 102, 241, 0.12), transparent 30%),
                  var(--bg);
      color: var(--text);
      font-family: "Space Grotesk", "Segoe UI", system-ui, -apple-system, sans-serif;
      display: flex;
      align-items: stretch;
      justify-content: center;
      padding: 18px;
    }
    .shell {
      width: min(1100px, 100%);
      display: flex;
      flex-direction: column;
      gap: 14px;
    }
    .panel {
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 16px;
      box-shadow: 0 20px 60px rgba(0, 0, 0, 0.35);
      padding: 16px 18px;
      backdrop-filter: blur(6px);
    }
    #log {
      flex: 3;
      min-height: 60vh;
      max-height: 70vh;
      overflow-y: auto;
      font-family: "JetBrains Mono", "SFMono-Regular", Consolas, monospace;
      line-height: 1.4;
      white-space: pre-wrap;
      border: 1px solid rgba(255, 255, 255, 0.05);
    }
    #log .entry {
      padding: 4px 0;
      border-bottom: 1px solid rgba(255, 255, 255, 0.04);
    }
    #log .entry:last-child { border-bottom: none; }
    #log .time { color: var(--muted); margin-right: 8px; }
    form {
      flex: 1;
      display: flex;
      gap: 10px;
      align-items: center;
    }
    input[type="text"] {
      flex: 1;
      padding: 12px 14px;
      border-radius: 12px;
      border: 1px solid var(--border);
      background: rgba(255, 255, 255, 0.06);
      color: var(--text);
      font-size: 16px;
      outline: none;
      transition: border-color 0.2s ease, box-shadow 0.2s ease;
    }
    input[type="text"]:focus {
      border-color: var(--accent);
      box-shadow: 0 0 0 3px rgba(79, 209, 197, 0.25);
    }
    button {
      padding: 12px 18px;
      border-radius: 12px;
      border: none;
      background: linear-gradient(135deg, #22d3ee, #6366f1);
      color: #0b1220;
      font-weight: 700;
      cursor: pointer;
      box-shadow: 0 10px 30px rgba(99, 102, 241, 0.35);
      transition: transform 0.15s ease, box-shadow 0.15s ease;
    }
    button:hover { transform: translateY(-2px); box-shadow: 0 14px 36px rgba(99, 102, 241, 0.45); }
    button:active { transform: translateY(0); }
    .status {
      font-size: 14px;
      color: var(--muted);
      margin-left: 6px;
    }
  </style>
</head>
<body>
  <div class="shell">
    <div class="panel" id="log"></div>
    <form class="panel" id="send-form" autocomplete="off">
      <input type="text" id="message" name="message" placeholder="输入要发送的串口内容..." />
      <button type="submit">发送</button>
      <span class="status" id="status">等待连接...</span>
    </form>
  </div>
  <script>
    const logEl = document.getElementById("log");
    const form = document.getElementById("send-form");
    const input = document.getElementById("message");
    const statusEl = document.getElementById("status");

    function appendLine(text) {
      const row = document.createElement("div");
      row.className = "entry";
      const time = document.createElement("span");
      time.className = "time";
      time.textContent = new Date().toLocaleTimeString();
      const body = document.createElement("span");
      body.textContent = text;
      row.appendChild(time);
      row.appendChild(body);
      logEl.appendChild(row);
      logEl.scrollTop = logEl.scrollHeight;
    }

    const source = new EventSource("/events");
    source.onopen = () => { statusEl.textContent = "已连接"; };
    source.onerror = () => { statusEl.textContent = "连接中断，重试中..."; };
    source.onmessage = (event) => {
      const lines = (event.data || "").split("\\n");
      lines.forEach((line) => {
        if (line.length === 0) return;
        appendLine(line);
      });
    };

    form.addEventListener("submit", async (evt) => {
      evt.preventDefault();
      const message = input.value;
      if (!message.trim()) return;
      statusEl.textContent = "发送中...";
      try {
        const res = await fetch("/send", {
          method: "POST",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ message })
        });
        if (!res.ok) {
          const detail = await res.json().catch(() => ({}));
          throw new Error(detail.error || "发送失败");
        }
        input.value = "";
        statusEl.textContent = "发送完成";
      } catch (err) {
        statusEl.textContent = err.message;
      } finally {
        setTimeout(() => { statusEl.textContent = "已连接"; }, 1200);
      }
    });
  </script>
</body>
</html>
"""


class WebDebugMode(ControlMode):
    """Serve a simple two-panel web UI for serial debugging."""

    def __init__(self, client: SerialCommandClient, host: str, port: int, append_newline: bool) -> None:
        super().__init__(client)
        self._host = host
        self._port = port
        self._append_newline = append_newline
        self._app = Flask(__name__)
        self._stop_event = threading.Event()
        self._listeners: Set[queue.Queue[str]] = set()
        self._reader_thread: threading.Thread | None = None
        self._setup_routes()

    def run(self) -> None:
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()
        logger.info("Web debug UI available at http://%s:%d", self._host, self._port)
        try:
            self._app.run(host=self._host, port=self._port, threaded=True)
        finally:
            self._stop_event.set()
            if self._reader_thread:
                self._reader_thread.join(timeout=1)

    def _setup_routes(self) -> None:
        @self._app.get("/")
        def index() -> ResponseReturnValue:
            return Response(INDEX_HTML, mimetype="text/html")

        @self._app.post("/send")
        def send_message() -> ResponseReturnValue:
            payload = request.get_json(silent=True)
            data: Mapping[str, Any] = payload if isinstance(payload, dict) else request.form
            message = data.get("message", "")
            if not isinstance(message, str):
                return jsonify({"error": "message must be a string"}), 400
            if not message:
                return jsonify({"error": "message cannot be empty"}), 400
            try:
                self._client.send_text(message, append_newline=self._append_newline)
            except Exception as exc:  # pylint: disable=broad-except
                logger.error("Failed to send serial data: %s", exc)
                return jsonify({"error": str(exc)}), 500
            return jsonify({"status": "ok"})

        @self._app.get("/events")
        def events() -> ResponseReturnValue:
            client_queue: queue.Queue[str] = queue.Queue(maxsize=200)
            self._listeners.add(client_queue)

            def stream() -> Iterator[str]:
                try:
                    while not self._stop_event.is_set():
                        try:
                            message = client_queue.get(timeout=1.0)
                        except queue.Empty:
                            yield ": keep-alive\n\n"
                            continue
                        for line in (message.replace("\r", "").splitlines() or [""]):
                            yield f"data: {line}\n\n"
                finally:
                    self._listeners.discard(client_queue)

            return Response(stream(), mimetype="text/event-stream")

    def _reader_loop(self) -> None:
        while not self._stop_event.is_set():
            chunk = self._client.drain_input()
            if chunk:
                text = chunk.decode("utf-8", errors="replace")
                self._broadcast(text)
            else:
                time.sleep(0.05)

    def _broadcast(self, message: str) -> None:
        for listener in list(self._listeners):
            try:
                listener.put_nowait(message)
            except queue.Full:
                # Drop oldest message to make room for the latest payload.
                try:
                    listener.get_nowait()
                    listener.put_nowait(message)
                except queue.Empty:
                    pass
