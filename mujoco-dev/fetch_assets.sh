#!/usr/bin/env bash
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
staging_dir="$(mktemp -d "$project_dir/.source.XXXXXX")"
trap 'rm -rf -- "$staging_dir"' EXIT
host="${TRACER_SSH_HOST:-dgx@192.168.112.108}"
container="${TRACER_CONTAINER:-d252837c84bb}"
tracer_revision="27633a956c845903ee630538afeb17fe70afdd84"
tracer_url="https://raw.githubusercontent.com/agilexrobotics/ugv_gazebo_sim/$tracer_revision/tracer/tracer2_description"

if [[ ! "$container" =~ ^[[:alnum:]][[:alnum:]_.-]*$ ]]; then
    echo "TRACER_CONTAINER must be a Docker container name or ID." >&2
    exit 2
fi

ssh -o BatchMode=yes -o ConnectTimeout=10 -- "$host" \
    "docker exec '$container' tar -C /workspace/tracer_piper -czf - \
        START_HERE.txt mobile_dynamics_config.json build_mobile_dynamics.py \
        mobile_manipulator2_description/urdf mobile_manipulator2_description/meshes \
        mobile_manipulator2_description/README_collision_models.md official_piper_x \
        official_inspect/piper_x_gripper_d435_physics.usda \
        official_inspect/piper_x_gripper_d435.usda" \
    | tar -xzf - -C "$staging_dir"

tracer_dir="$staging_dir/tracer2_description"
mkdir -p "$tracer_dir/urdf" "$tracer_dir/meshes"
tracer_downloads=()
for relative_path in urdf/tracer2_description.urdf meshes/{base_link,left_Link,right_link,left_front_link,left_rear_link,right_front_link,right_rear_link}.STL; do
    tracer_downloads+=(--output "$tracer_dir/$relative_path" "$tracer_url/$relative_path")
done
curl --fail --fail-early --location --silent --show-error --parallel --max-time 45 --retry 2 "${tracer_downloads[@]}"

mkdir -p "$project_dir/source"
cp -R "$staging_dir/." "$project_dir/source/"
