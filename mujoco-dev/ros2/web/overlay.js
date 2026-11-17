import {frameOrigin} from "./frame.js";

export function loadEliminationOverlay(source, onFeedback) {
  const cancel = new AbortController();
  let timer;
  let lastText;
  let applied = false;
  let tracking = Boolean(source.handle);
  const report = message => { if (!cancel.signal.aborted) onFeedback(message); };

  async function reload() {
    try {
      const file = source.handle ? await source.handle.getFile() : source.file;
      if (file.size > 1024 * 1024) {
        lastText = undefined;
        report("请选择 1 MB 以内的 JSON 文件。");
        return;
      }
      const text = await file.text();
      if (text === lastText || cancel.signal.aborted) return;
      const response = await fetch(`${frameOrigin}/overlay`, {
        method: "POST",
        headers: {"Content-Type": "application/json"},
        body: text,
        signal: AbortSignal.any([cancel.signal, AbortSignal.timeout(3000)]),
      });
      if (response.ok) {
        lastText = text;
        report(tracking ? (applied ? "消除区已更新 · 自动跟踪" : "消除区已加载 · 自动跟踪") : "消除区已加载");
        applied = true;
      } else if (response.status === 400) {
        lastText = text;
        report(`${await response.text()}${tracking ? " 保存有效文件后会继续更新。" : ""}`);
      } else {
        report("无法加载消除区，请检查仿真连接。");
      }
    } catch (error) {
      if (error.name === "NotAllowedError") {
        tracking = false;
        report("文件读取权限已失效，请重新选择。");
      } else if (error.name === "NotFoundError" || error.name === "NotReadableError") {
        lastText = undefined;
        report("暂时无法读取文件，正在自动重试。");
      } else {
        report("无法加载消除区，请检查仿真连接。");
      }
    } finally {
      if (tracking && !cancel.signal.aborted) timer = setTimeout(reload, 800);
    }
  }

  report("正在加载…");
  void reload();
  return () => {
    cancel.abort();
    clearTimeout(timer);
  };
}
