import React, {useEffect, useRef, useState} from "react";
import {createRoot} from "react-dom/client";
import {showFrames} from "./frame.js";
import "./style.css";

const directions = {
  ArrowUp: "forward", KeyW: "forward",
  ArrowLeft: "left", KeyA: "left",
  ArrowRight: "right", KeyD: "right",
};

const buttons = [
  ["forward", "前进", "ArrowUp W", "M12 21V3M6 9l6-6 6 6"],
  ["left", "左转", "ArrowLeft A", "M3 5v5h5M3.4 10a8.5 8.5 0 1 1 2 8"],
  ["stop", "停止", "Escape Space", null],
  ["right", "右转", "ArrowRight D", "M21 5v5h-5m4.6 0a8.5 8.5 0 1 0-2 8"],
];

const defaultView = {azimuth: 90, elevation: -90, zoom: 1, x: 0, y: 0};
const clamp = (value, min, max) => Math.min(max, Math.max(min, value));

function useStatus() {
  const [status, setStatus] = useState({online: false, pose: null});
  useEffect(() => {
    let disposed = false;
    let timer;

    async function poll() {
      try {
        const response = await fetch("/api/status", {signal: AbortSignal.timeout(800)});
        const value = await response.json();
        if (!disposed) setStatus(value);
      } catch {
        if (!disposed) setStatus({online: false, pose: null});
      } finally {
        if (!disposed) timer = setTimeout(poll, 200);
      }
    }

    void poll();
    return () => {
      disposed = true;
      clearTimeout(timer);
    };
  }, []);
  return status;
}

function useDrive(online) {
  const [active, setActive] = useState("stop");
  const state = useRef({held: null, target: "stop", last: "stop", sending: false, online});
  state.current.online = online;

  async function send() {
    const drive = state.current;
    if (drive.sending || (drive.target === "stop" && drive.last === "stop")) return;
    const command = drive.target;
    drive.sending = true;
    try {
      const response = await fetch("/api/drive", {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: JSON.stringify(command),
        signal: AbortSignal.timeout(200),
        keepalive: command === "stop",
      });
      if (!response.ok) throw new Error("Drive command rejected");
    } catch {
      drive.target = "stop";
      drive.held = null;
      setActive("stop");
    } finally {
      drive.last = command;
      drive.sending = false;
    }
    if (drive.target === "stop" && command !== "stop") void send();
  }

  function release(source) {
    const drive = state.current;
    if (source !== undefined && drive.held !== source) return;
    drive.target = "stop";
    drive.held = null;
    setActive("stop");
    void send();
  }

  function hold(command, source) {
    const drive = state.current;
    if (!drive.online) return;
    drive.held = source;
    drive.target = command;
    setActive(command);
    void send();
  }

  useEffect(() => {
    if (!online) release();
  }, [online]);

  useEffect(() => {
    const timer = setInterval(send, 85);
    function keydown(event) {
      if (event.code === "ArrowDown") {
        event.preventDefault();
        return;
      }
      if (event.metaKey || event.altKey || event.ctrlKey) return;
      const command = directions[event.code] ||
        (["Enter", "Space"].includes(event.code) && event.target.dataset?.command);
      if (event.code === "Escape" || (event.code === "Space" && !command)) {
        event.preventDefault();
        release();
      } else if (command) {
        event.preventDefault();
        if (event.repeat) return;
        if (command === "stop") release();
        else hold(command, event.code);
      }
    }
    const keyup = (event) => release(event.code);
    const cancel = () => release();
    const visibility = () => { if (document.hidden) release(); };
    window.addEventListener("keydown", keydown);
    window.addEventListener("keyup", keyup);
    window.addEventListener("blur", cancel);
    window.addEventListener("pagehide", cancel);
    document.addEventListener("visibilitychange", visibility);
    return () => {
      clearInterval(timer);
      cancel();
      window.removeEventListener("keydown", keydown);
      window.removeEventListener("keyup", keyup);
      window.removeEventListener("blur", cancel);
      window.removeEventListener("pagehide", cancel);
      document.removeEventListener("visibilitychange", visibility);
    };
  }, []);

  return {active, hold, release};
}

function Pose({value}) {
  return <section className="pose" aria-labelledby="pose-heading">
    <h3 id="pose-heading">实时位姿</h3>
    <dl>{[["x", "X", " m", 2], ["y", "Y", " m", 2], ["yaw", "航向", "°", 1]].map(
      ([key, label, unit, precision]) => <div key={key}>
        <dt>{label}</dt>
        <dd id={`pose-${key}`}>{value ? (Number(value[key].toFixed(precision)) || 0).toFixed(precision) + unit : "—"}</dd>
      </div>
    )}</dl>
  </section>;
}

function App() {
  const camera = useRef(null);
  const [frameMessage, setFrameMessage] = useState("等待本机 MuJoCo 画面");
  const status = useStatus();
  const online = status.online && !frameMessage;
  const readyOnce = useRef(false);
  if (online) readyOnce.current = true;
  const drive = useDrive(online);
  useEffect(() => {
    const canvas = camera.current;
    const viewport = canvas.parentElement;
    const listeners = new AbortController();
    const {signal} = listeners;
    let view = {...defaultView};
    let drag;
    const stopFrames = showFrames(canvas, setFrameMessage, () =>
      [view.azimuth, view.elevation, view.zoom, view.x, view.y].map(value => value.toFixed(4)).join(",")
    );

    function zoomScene(event) {
      if (event.metaKey || event.deltaY === 0) return;
      event.preventDefault();
      const unit = event.deltaMode === 1 ? 16 : event.deltaMode === 2 ? viewport.clientHeight : 1;
      const sensitivity = event.ctrlKey ? 0.012 : 0.002;
      view.zoom = clamp(view.zoom * Math.exp(-event.deltaY * unit * sensitivity), 1, 2.5);
    }

    function startDrag(event) {
      if (drag || !event.isPrimary || ![0, 2].includes(event.button)) return;
      event.preventDefault();
      viewport.setPointerCapture(event.pointerId);
      drag = {id: event.pointerId, x: event.clientX, y: event.clientY, pan: event.button === 2};
      viewport.dataset.dragging = "true";
    }

    function moveCamera(event) {
      if (drag?.id !== event.pointerId) return;
      const dx = event.clientX - drag.x;
      const dy = event.clientY - drag.y;
      drag.x = event.clientX;
      drag.y = event.clientY;
      if (drag.pan || event.shiftKey) {
        const angle = view.azimuth * Math.PI / 180;
        const sin = Math.sin(angle);
        const cos = Math.cos(angle);
        const metresPerPixel = 4.4 / (view.zoom * Math.min(viewport.clientHeight, viewport.clientWidth * 0.7));
        view.x = clamp(view.x + (dy * cos - dx * sin) * metresPerPixel, -2.5, 2.5);
        view.y = clamp(view.y + (dy * sin + dx * cos) * metresPerPixel, -1.8, 1.8);
      } else {
        view.azimuth = (view.azimuth - dx * 0.28) % 360;
        view.elevation = clamp(view.elevation + dy * 0.28, -90, -20);
      }
    }

    function stopDrag(event) {
      if (drag?.id !== event.pointerId) return;
      drag = undefined;
      delete viewport.dataset.dragging;
    }

    viewport.addEventListener("wheel", zoomScene, {passive: false, signal});
    viewport.addEventListener("pointerdown", startDrag, {signal});
    viewport.addEventListener("pointermove", moveCamera, {signal});
    for (const event of ["pointerup", "pointercancel", "lostpointercapture"]) {
      viewport.addEventListener(event, stopDrag, {signal});
    }
    viewport.addEventListener("contextmenu", event => event.preventDefault(), {signal});
    viewport.addEventListener("dblclick", () => { view = {...defaultView}; }, {signal});
    return () => {
      listeners.abort();
      stopFrames();
    };
  }, []);

  return <>
    <header className="masthead">
      <div className="brand">
        <div className="brand-icon" aria-hidden="true">
          <svg viewBox="0 0 40 40" fill="none" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round">
            <circle cx="10" cy="29" r="4.5"/><circle cx="29" cy="29" r="4.5"/>
            <path d="M5 28H3V19h6l4-3h22v12h-1.5M14.5 29h10M24 16V9m-4 0h8V5h-8z"/>
          </svg>
        </div>
        <div><h1>Tracer 2.0</h1><p>MuJoCo · ROS 2 Jazzy</p></div>
      </div>
      <div className="connection" data-state={online ? "online" : readyOnce.current ? "offline" : "starting"} role="status">
        <i/><span>{online ? "仿真已连接" : readyOnce.current ? "仿真连接中断" : "正在连接仿真"}</span>
      </div>
    </header>

    <main className="workspace">
      <section className="panel scene" aria-labelledby="scene-heading">
        <div className="scene-header"><h2 id="scene-heading">试验场景</h2><span>实时画面</span></div>
        <div className="camera" title="拖动旋转，Shift + 拖动平移，滚轮缩放，双击恢复俯视">
          <canvas ref={camera} role="img" aria-label="MuJoCo 围栏试验场画面"/>
          {frameMessage && <p>{frameMessage}</p>}
        </div>
      </section>

      <section className="panel controls" aria-labelledby="drive-heading">
        <h2 id="drive-heading">手动控制</h2>
        <p className="instructions">按住移动，松开停止</p>
        <div className="drive-pad" role="group" aria-label="方向控制">
          {buttons.map(([command, label, shortcut, path]) =>
            <button key={command} type="button" data-command={command} data-active={drive.active === command && command !== "stop"}
              aria-keyshortcuts={shortcut} disabled={!online && command !== "stop"}
              onPointerDown={(event) => {
                if (event.button !== 0 || !event.isPrimary) return;
                if (command === "stop") return drive.release();
                event.currentTarget.setPointerCapture(event.pointerId);
                drive.hold(command, event.pointerId);
              }}
              onPointerUp={(event) => drive.release(event.pointerId)}
              onPointerCancel={(event) => drive.release(event.pointerId)}
              onLostPointerCapture={(event) => drive.release(event.pointerId)}
              onClick={command === "stop" ? () => drive.release() : undefined}>
              <svg viewBox="0 0 24 24" aria-hidden="true">{path ? <path d={path}/> :
                <rect x="5.5" y="5.5" width="13" height="13" rx="1" fill="currentColor" stroke="none"/>}</svg>
              {label}
            </button>
          )}
        </div>
        <p className="keyboard">↑ / ← / → 或 W / A / D</p>
        <Pose value={status.pose}/>
      </section>
    </main>
  </>;
}

createRoot(document.getElementById("app")).render(<App/>);
