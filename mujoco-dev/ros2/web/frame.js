// Only the newest available native framebuffer is transferred and uploaded.
export function showFrames(canvas, report) {
  const gl = canvas.getContext("webgl2", {alpha: false, depth: false, antialias: false});
  if (!gl) {
    report("浏览器无法启用 WebGL 2");
    return () => {};
  }

  function shader(type, source) {
    const value = gl.createShader(type);
    gl.shaderSource(value, source);
    gl.compileShader(value);
    if (!gl.getShaderParameter(value, gl.COMPILE_STATUS)) {
      throw new Error(gl.getShaderInfoLog(value));
    }
    return value;
  }

  const vertex = shader(gl.VERTEX_SHADER, `#version 300 es
    out vec2 uv;
    void main() {
      vec2 point = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
      uv = vec2(point.x, 1.0 - point.y);
      gl_Position = vec4(point * 2.0 - 1.0, 0.0, 1.0);
    }`);
  const fragment = shader(gl.FRAGMENT_SHADER, `#version 300 es
    precision mediump float;
    uniform sampler2D image;
    in vec2 uv;
    out vec4 color;
    void main() { color = texture(image, uv); }`);
  const program = gl.createProgram();
  gl.attachShader(program, vertex);
  gl.attachShader(program, fragment);
  gl.linkProgram(program);
  gl.deleteShader(vertex);
  gl.deleteShader(fragment);
  if (!gl.getProgramParameter(program, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(program));
  gl.useProgram(program);

  const texture = gl.createTexture();
  gl.bindTexture(gl.TEXTURE_2D, texture);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
  gl.pixelStorei(gl.UNPACK_ALIGNMENT, 1);

  let stopped = false;
  let request;
  let timer;
  let sequence = "";
  let allocated = false;
  const origin = `${window.location.protocol}//${window.location.hostname}:8767`;

  async function next() {
    if (!document.hidden && !gl.isContextLost()) {
      request = new AbortController();
      const deadline = setTimeout(() => request.abort(), 800);
      try {
        const response = await fetch(`${origin}/frame?after=${sequence}`, {signal: request.signal});
        if (!response.ok) throw new Error("Frame service unavailable");
        if (response.status === 200) {
          const pixels = new Uint8Array(await response.arrayBuffer());
          if (stopped || gl.isContextLost()) return;
          const width = Number(response.headers.get("X-Width"));
          const height = Number(response.headers.get("X-Height"));
          if (pixels.length !== width * height * 3) throw new Error("Invalid RGB buffer");
          if (!allocated || canvas.width !== width || canvas.height !== height) {
            canvas.width = width;
            canvas.height = height;
            gl.viewport(0, 0, width, height);
            gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB8, width, height, 0, gl.RGB, gl.UNSIGNED_BYTE, pixels);
            allocated = true;
          } else {
            gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, width, height, gl.RGB, gl.UNSIGNED_BYTE, pixels);
          }
          gl.drawArrays(gl.TRIANGLES, 0, 3);
          sequence = response.headers.get("X-Frame");
        }
        if (allocated) report("");
      } catch {
        if (!stopped) report("等待本机 MuJoCo 画面");
        // Force an upload after the native process reconnects.
        sequence = "";
      } finally {
        clearTimeout(deadline);
      }
    }
    if (!stopped) timer = setTimeout(next, document.hidden ? 350 : 45);
  }

  function onLost(event) {
    event.preventDefault();
    report("WebGL 画面中断，正在重连");
  }

  function onRestored() {
    dispose();
    cleanup = showFrames(canvas, report);
  }

  function dispose() {
    stopped = true;
    request?.abort();
    clearTimeout(timer);
    canvas.removeEventListener("webglcontextlost", onLost);
    canvas.removeEventListener("webglcontextrestored", onRestored);
    gl.deleteTexture(texture);
    gl.deleteProgram(program);
  }

  let cleanup = dispose;
  canvas.addEventListener("webglcontextlost", onLost);
  canvas.addEventListener("webglcontextrestored", onRestored);
  void next();
  return () => cleanup();
}
