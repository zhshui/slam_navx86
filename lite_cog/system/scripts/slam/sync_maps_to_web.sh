#!/bin/bash
# Sync SLAM-generated maps (PCD + PGM/YAML) to nav_web gateway

SOURCE_DIR="/home/robot/go2_nav/lite_cog/system/map"
GATEWAY_DIR="/home/robot/go2_nav/nav_web/gateway"
MAPS_DIR="$GATEWAY_DIR/data/maps"
DB="$GATEWAY_DIR/data/nav_web.sqlite"

mkdir -p "$MAPS_DIR"

for yaml_file in "$SOURCE_DIR"/scans_*.yaml; do
    [ -f "$yaml_file" ] || continue
    base=$(basename "$yaml_file" .yaml)
    pgm_file="$SOURCE_DIR/${base}.pgm"
    pcd_file="$SOURCE_DIR/${base}.pcd"
    [ -f "$pgm_file" ] || continue

    # Copy 2D grid map to gateway data directory
    cp "$yaml_file" "$MAPS_DIR/${base}.yaml"
    cp "$pgm_file" "$MAPS_DIR/${base}.pgm"
    sed -i "s|^image:.*|image: ${base}.pgm|" "$MAPS_DIR/${base}.yaml"

    # Determine PCD path
    pcd_rel=""
    if [ -f "$pcd_file" ]; then
        cp "$pcd_file" "$MAPS_DIR/${base}.pcd"
        pcd_rel="data/maps/${base}.pcd"
    fi

    # Register/update in database
    exists=$(sqlite3 "$DB" "SELECT id FROM maps WHERE name='$base';" 2>/dev/null)
    if [ -z "$exists" ]; then
        id=$(cat /proc/sys/kernel/random/uuid 2>/dev/null || echo "$(date +%s)-$RANDOM")
        sqlite3 "$DB" "INSERT INTO maps (id, name, yaml_path, pcd_path, created_at, active) VALUES ('$id', '$base', 'data/maps/${base}.yaml', '$pcd_rel', datetime('now'), 0);" 2>/dev/null
        echo "Registered: $base (pcd=$pcd_rel)"
    else
        sqlite3 "$DB" "UPDATE maps SET yaml_path='data/maps/${base}.yaml', pcd_path='$pcd_rel' WHERE id='$exists';" 2>/dev/null
        echo "Updated: $base (pcd=$pcd_rel)"
    fi
done

echo "Sync complete"
sqlite3 "$DB" "SELECT name, yaml_path, pcd_path, active FROM maps ORDER BY created_at DESC;" 2>/dev/null

