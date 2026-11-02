#!/bin/bash
# Sync all maps (PCD + PGM/YAML) from system/map to gateway database
# Scans all subdirectories, registers any new maps found

SOURCE_DIR="/home/unitree/go2_nav/lite_cog/system/map"
GATEWAY_DIR="/home/unitree/go2_nav/ros_web_gui_app/gateway"
MAPS_DIR="$GATEWAY_DIR/data/maps"
DB="$GATEWAY_DIR/data/nav_web.sqlite"

mkdir -p "$MAPS_DIR"

for d in "$SOURCE_DIR"/*/; do
    name=$(basename "$d")
    # Skip non-map directories
    case "$name" in active|__pycache__|scans|.claude) continue ;; esac

    yaml=$(ls "$d"*.yaml 2>/dev/null | head -1)
    pgm=$(ls "$d"*.pgm 2>/dev/null | head -1)
    pcd=$(ls "$d"*.pcd 2>/dev/null | head -1)

    [ -z "$yaml" ] || [ -z "$pgm" ] && continue

    # Register/update in database with absolute paths
    exists=$(sqlite3 "$DB" "SELECT id FROM maps WHERE name='$name';" 2>/dev/null)
    if [ -z "$exists" ]; then
        id=$(cat /proc/sys/kernel/random/uuid 2>/dev/null || echo "$(date +%s)-$RANDOM")
        sqlite3 "$DB" "INSERT INTO maps (id, name, yaml_path, pcd_path, created_at, active) VALUES ('$id', '$name', '$yaml', '${pcd:-}', datetime('now'), 0);" 2>/dev/null
        echo "[sync_maps] Registered: $name (yaml=$yaml pcd=${pcd:-无})"
    else
        sqlite3 "$DB" "UPDATE maps SET yaml_path='$yaml', pcd_path='${pcd:-}' WHERE name='$name';" 2>/dev/null
        echo "[sync_maps] Updated: $name"
    fi
done

# Also scan scans_*.yaml in the root map dir (legacy flat layout)
for yaml_file in "$SOURCE_DIR"/scans_*.yaml; do
    [ -f "$yaml_file" ] || continue
    base=$(basename "$yaml_file" .yaml)
    # Skip if already handled via directory scan above
    exists=$(sqlite3 "$DB" "SELECT id FROM maps WHERE name='$base';" 2>/dev/null)
    [ -n "$exists" ] && continue

    pgm_file="$SOURCE_DIR/${base}.pgm"
    pcd_file="$SOURCE_DIR/${base}.pcd"
    [ -f "$pgm_file" ] || continue

    pcd_path=""
    [ -f "$pcd_file" ] && pcd_path="$pcd_file"

    id=$(cat /proc/sys/kernel/random/uuid 2>/dev/null || echo "$(date +%s)-$RANDOM")
    sqlite3 "$DB" "INSERT INTO maps (id, name, yaml_path, pcd_path, created_at, active) VALUES ('$id', '$base', '$yaml_file', '$pcd_path', datetime('now'), 0);" 2>/dev/null
    echo "[sync_maps] Registered (flat): $base"
done

echo "[sync_maps] Sync complete"
sqlite3 "$DB" "SELECT name, yaml_path, active FROM maps ORDER BY created_at DESC;" 2>/dev/null

