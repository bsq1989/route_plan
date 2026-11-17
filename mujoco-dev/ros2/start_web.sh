#!/usr/bin/env bash
set -euo pipefail

project="$(cd "$(dirname "$0")/.." && pwd -P)"
container=route-plan-mujoco-ros2
image=route-plan-mujoco:jazzy
job=com.route-plan.mujoco-native
page=http://127.0.0.1:8765

fail() {
  printf '%s\n' "$*" >&2
  exit 1
}

[[ "$(uname -s)" == Darwin ]] || fail "网页仿真需要在 Mac 的桌面会话中启动。"
for command in docker npm uv; do
  command -v "$command" >/dev/null || fail "缺少命令 $command。"
done
[[ -f "$project/model/scene_tracer.xml" ]] || fail "缺少 Tracer 底盘模型，请先按 mujoco-dev/README.md 生成 model/。"
docker info --format '{{.ServerVersion}}' >/dev/null 2>&1 || fail "Docker 未就绪，请先启动 Docker Desktop。"

existing=false
if docker container inspect "$container" >/dev/null 2>&1; then
  binding="$(docker inspect -f '{{range index .HostConfig.PortBindings "8080/tcp"}}{{.HostIp}}:{{.HostPort}}{{end}}' "$container")"
  mount="$(docker inspect -f '{{range .Mounts}}{{if eq .Destination "/workspace/mujoco-dev"}}{{.Source}}{{end}}{{end}}' "$container")"
  mode="$(docker inspect -f '{{range .Config.Cmd}}{{println .}}{{end}}' "$container")"
  if [[ "$binding" != 127.0.0.1:8765 || "$mount" != "$project" ]] || ! grep -qx 'web:=true' <<< "$mode"; then
    fail "同名容器使用了其他启动配置。请先停止并移除 $container 后重试。"
  fi
  existing=true
fi

uv sync --project "$project" --locked --inexact
npm ci --prefix "$project/ros2/web" --no-audit --no-fund
npm run build --prefix "$project/ros2/web"

frame_endpoint=http://127.0.0.1:8767/frame
frame_service_running() {
  curl -fsS --max-time 2 -H "Origin: $page" -D - -o /dev/null "$frame_endpoint" 2>/dev/null |
    grep -qi '^X-Frame:'
}

native_running=false
if frame_service_running; then
  native_running=true
fi

if [[ "$existing" == false ]]; then
  if ! docker image inspect "$image" >/dev/null 2>&1; then
    docker build -t "$image" "$project/ros2"
  fi
  docker run -d --init --name "$container" \
    -p 127.0.0.1:8765:8080 \
    --mount "type=bind,source=$project,target=/workspace/mujoco-dev,readonly" \
    "$image" ros2 launch /workspace/mujoco-dev/ros2/tracer.launch.py web:=true
elif [[ "$(docker inspect -f '{{.State.Running}}' "$container")" != true ]]; then
  docker start "$container"
elif [[ "$native_running" == false ]]; then
  # The new native process starts its simulation clock at zero.
  docker restart -t 3 "$container"
fi

if [[ "$native_running" == false ]]; then
  if launchctl print "gui/$(id -u)/$job" >/dev/null 2>&1; then
    launchctl remove "$job"
  fi
  launchctl submit -l "$job" \
    -o "$project/output/native-arena.log" \
    -e "$project/output/native-arena-error.log" \
    -- "$project/.venv/bin/mjpython" "$project/ros2/native_arena.py"
fi

for ((attempt = 0; attempt < 25; attempt++)); do
  if curl -fsS --max-time 1 "$page/api/status" 2>/dev/null | grep -Eq '"online":[[:space:]]*true' &&
      [[ "$(curl -s --max-time 2 -H "Origin: $page" -o /dev/null -w '%{http_code}' "$frame_endpoint")" == 200 ]]; then
    printf '仿真已就绪 %s/\n' "$page"
    exit 0
  fi
  sleep 1
done

fail "仿真启动超时。请查看 docker logs $container 和 $project/output/native-arena-error.log。"
