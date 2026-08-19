from __future__ import annotations

import asyncio
import json
from typing import Any, Dict, List

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from pydantic import BaseModel, Field

from .backend import CarBackend, TASK_ORDER


class StartRequest(BaseModel):
    from_index: int = Field(default=-1, ge=-1, le=len(TASK_ORDER) - 1)


class SpeedRequest(BaseModel):
    speed: float = Field(..., ge=0.05, le=2.0)


class ConnectionManager:
    """管理所有活跃的 WebSocket 连接，广播后端事件。"""

    def __init__(self) -> None:
        self.active_connections: List[WebSocket] = []
        self._lock = asyncio.Lock()

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        async with self._lock:
            self.active_connections.append(ws)

    async def disconnect(self, ws: WebSocket) -> None:
        async with self._lock:
            try:
                self.active_connections.remove(ws)
            except ValueError:
                pass

    async def broadcast(self, event: Dict[str, Any]) -> None:
        text = json.dumps(event, ensure_ascii=False)
        async with self._lock:
            dead: List[WebSocket] = []
            for ws in self.active_connections:
                try:
                    await ws.send_text(text)
                except Exception:
                    dead.append(ws)
            for ws in dead:
                try:
                    self.active_connections.remove(ws)
                except ValueError:
                    pass


def create_app(backend: CarBackend) -> FastAPI:
    """创建 FastAPI 应用，绑定给定的后端实例。

    应用启动时会注册事件回调，把后端 emit 的事件转发到 asyncio 队列，
    再由 queue worker 广播给所有 WS 客户端。
    """
    app = FastAPI(title="rak-hri control backend", version="1.0.0")

    manager = ConnectionManager()
    event_queue: "asyncio.Queue[Dict[str, Any]]" = asyncio.Queue()
    # 主事件循环引用：在 startup 时捕获，供后台线程安全投递事件
    main_loop: List[asyncio.AbstractEventLoop] = []

    # ---------- 后端事件 → asyncio 队列 ----------
    def _on_event(ev: Dict[str, Any]) -> None:
        # 事件可能来自后台线程（odom 轮询/任务线程），不能在此调用
        # asyncio.get_running_loop()（非 asyncio 线程会抛 RuntimeError）。
        # 使用 startup 时缓存的主循环引用，线程安全投递。
        loop = main_loop[0] if main_loop else None
        if loop is not None and loop.is_running():
            loop.call_soon_threadsafe(event_queue.put_nowait, dict(ev))

    backend.add_event_callback(_on_event)

    # ---------- 队列 → WS 广播 worker ----------
    @app.on_event("startup")
    async def _start_worker() -> None:
        main_loop.append(asyncio.get_running_loop())
        async def _worker() -> None:
            while True:
                ev = await event_queue.get()
                try:
                    await manager.broadcast(ev)
                finally:
                    event_queue.task_done()

        asyncio.create_task(_worker())
        backend.start_bg()

    @app.on_event("shutdown")
    async def _shutdown() -> None:
        backend.close()

    # ---------- HTTP 端点 ----------
    @app.get("/api/hello")
    async def api_hello() -> JSONResponse:
        return JSONResponse(backend.hello_snapshot())

    @app.get("/api/tasks")
    async def api_tasks() -> JSONResponse:
        return JSONResponse({"tasks": backend.tasks_snapshot()})

    @app.get("/api/status")
    async def api_status() -> JSONResponse:
        return JSONResponse(backend.status_snapshot())

    @app.post("/api/start")
    async def api_start(req: StartRequest) -> JSONResponse:
        try:
            backend.start(from_index=req.from_index)
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc))
        return JSONResponse({"ok": True})

    @app.post("/api/run/{task}")
    async def api_run_task(task: str) -> JSONResponse:
        if task not in TASK_ORDER:
            raise HTTPException(status_code=404, detail=f"未知任务: {task}")
        try:
            backend.run_task(task)
        except RuntimeError as exc:
            raise HTTPException(status_code=409, detail=str(exc))
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc))
        return JSONResponse({"ok": True})

    @app.post("/api/stop")
    async def api_stop() -> JSONResponse:
        backend.stop()
        return JSONResponse({"ok": True})

    @app.post("/api/skip")
    async def api_skip() -> JSONResponse:
        backend.skip()
        return JSONResponse({"ok": True})

    @app.post("/api/reset")
    async def api_reset() -> JSONResponse:
        backend.reset()
        return JSONResponse({"ok": True})

    @app.post("/api/tasks/{task}/speed")
    async def api_set_task_speed(task: str, req: SpeedRequest) -> JSONResponse:
        if task not in TASK_ORDER:
            raise HTTPException(status_code=404, detail=f"未知任务: {task}")
        backend.set_task_speed(task, req.speed)
        return JSONResponse({"ok": True, "task": task, "speed": req.speed})

    # ---------- WebSocket ----------
    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket) -> None:
        await manager.connect(ws)
        try:
            # 连接后第一帧：hello 快照
            await ws.send_text(json.dumps(backend.hello_snapshot(), ensure_ascii=False))
            while True:
                # 客户端基本不发消息，这里只做心跳接收
                data = await ws.receive_text()
                if not data:
                    continue
                try:
                    msg = json.loads(data)
                except Exception:
                    continue
                if isinstance(msg, dict) and msg.get("type") == "ping":
                    try:
                        await ws.send_text(json.dumps({"type": "pong"}, ensure_ascii=False))
                    except Exception:
                        break
        except WebSocketDisconnect:
            pass
        finally:
            await manager.disconnect(ws)

    return app
