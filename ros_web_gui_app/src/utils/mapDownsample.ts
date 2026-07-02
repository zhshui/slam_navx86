/**
 * Map texture downsampling for mobile GPU compatibility.
 *
 * Mobile browsers have a max WebGL texture size (typically 2048 or 4096).
 * When an occupancy grid map exceeds this limit, the texture creation fails
 * silently and the map renders blank/black.
 *
 * This utility detects the actual GPU limit and downsamples maps that exceed it.
 */

/** Cached max texture size from a temporary WebGL context */
let cachedMaxTextureSize: number | null = null;

/**
 * Query the device's actual WebGL MAX_TEXTURE_SIZE.
 * Uses a temporary canvas → WebGL context to avoid depending on
 * a running Three.js renderer.
 */
export function getMaxTextureSize(): number {
  if (cachedMaxTextureSize !== null) return cachedMaxTextureSize;

  try {
    const canvas = document.createElement('canvas');
    const gl =
      canvas.getContext('webgl2') || canvas.getContext('webgl');
    if (gl) {
      cachedMaxTextureSize = gl.getParameter(gl.MAX_TEXTURE_SIZE) as number;
    }
  } catch {
    // Fallback: conservative mobile-safe limit
  }

  if (cachedMaxTextureSize === null || cachedMaxTextureSize <= 0) {
    // Conservative fallback — most mobile GPUs support at least 2048
    cachedMaxTextureSize = 2048;
  }

  return cachedMaxTextureSize!;
}

/**
 * Downsample occupancy grid data so neither dimension exceeds maxSize.
 *
 * Uses max-pooling (takes the maximum occupancy value in each block)
 * to be conservative — we don't want to lose obstacle cells.
 *
 * @returns downsampled { width, height, data }
 */
export function downsampleOccupancyGrid(
  width: number,
  height: number,
  data: number[] | Int8Array,
  maxSize: number
): { width: number; height: number; data: Int8Array } {
  if (width <= maxSize && height <= maxSize) {
    // No downsampling needed — but ensure Int8Array output
    return {
      width,
      height,
      data: data instanceof Int8Array ? data : new Int8Array(data),
    };
  }

  // Compute integer downsampling factor (at least 2)
  const factor = Math.max(
    Math.ceil(width / maxSize),
    Math.ceil(height / maxSize)
  );

  const newWidth = Math.floor(width / factor);
  const newHeight = Math.floor(height / factor);
  const newData = new Int8Array(newWidth * newHeight);

  for (let ny = 0; ny < newHeight; ny++) {
    for (let nx = 0; nx < newWidth; nx++) {
      // Source block boundaries
      const sxStart = nx * factor;
      const syStart = ny * factor;
      const sxEnd = Math.min(sxStart + factor, width);
      const syEnd = Math.min(syStart + factor, height);

      let maxVal = -1;
      for (let sy = syStart; sy < syEnd; sy++) {
        for (let sx = sxStart; sx < sxEnd; sx++) {
          const val = data[sy * width + sx]!;
          if (val > maxVal) maxVal = val;
        }
      }
      newData[ny * newWidth + nx] = maxVal;
    }
  }

  return { width: newWidth, height: newHeight, data: newData };
}

/**
 * Determine the safe texture size for map rendering.
 * Returns a value ≤ the GPU's MAX_TEXTURE_SIZE, with a safety margin.
 */
export function getSafeMaxTextureSize(): number {
  // Use 90% of actual max to leave headroom for other textures
  return Math.floor(getMaxTextureSize() * 0.9);
}
