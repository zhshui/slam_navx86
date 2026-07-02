/**
 * Publish the edited occupancy grid directly to /map topic via rosbridge.
 */
import type { RosbridgeConnection } from '../utils/RosbridgeConnection';

export function publishGridToMap(
  connection: RosbridgeConnection,
  grid: {
    header?: { frame_id?: string };
    info: {
      width: number;
      height: number;
      resolution: number;
      origin: {
        position: { x: number; y: number; z: number };
        orientation?: { x: number; y: number; z: number; w: number };
      };
    };
    data: number[] | Int8Array;
  }
): void {
  const now = Date.now();
  const msg = {
    header: {
      stamp: {
        sec: Math.floor(now / 1000),
        nsec: (now % 1000) * 1000000,
      },
      frame_id: grid.header?.frame_id ?? 'map',
    },
    info: {
      map_load_time: {
        sec: Math.floor(now / 1000),
        nsec: (now % 1000) * 1000000,
      },
      resolution: grid.info.resolution,
      width: grid.info.width,
      height: grid.info.height,
      origin: {
        position: {
          x: grid.info.origin.position.x,
          y: grid.info.origin.position.y,
          z: grid.info.origin.position.z,
        },
        orientation: grid.info.origin.orientation ?? { x: 0, y: 0, z: 0, w: 1 },
      },
    },
    data: Array.isArray(grid.data) ? [...grid.data] : Array.from(grid.data),
  };

  connection.publish('/map', 'nav_msgs/OccupancyGrid', msg);
}
