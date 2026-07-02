import { useEffect } from 'react';
import { DEFAULT_LAYER_CONFIGS } from '../constants/layerConfigs';
import { loadLayerConfigs, loadImagePositions } from '../utils/layerConfigStorage';

export function useInitialization(
  initialposeTopicRef: React.MutableRefObject<string>,
  imagePositionsRef: React.MutableRefObject<Map<string, { x: number; y: number; scale: number }>>
) {
  useEffect(() => {
    const saved = loadLayerConfigs();
    if (saved) {
      const initialposeConfig = Object.values(saved).find(config => config.id === 'initialpose');
      if (initialposeConfig && initialposeConfig.topic) {
        initialposeTopicRef.current = initialposeConfig.topic as string;
      }
    }
    const defaultInitialposeConfig = DEFAULT_LAYER_CONFIGS.initialpose;
    if (defaultInitialposeConfig && defaultInitialposeConfig.topic) {
      initialposeTopicRef.current = defaultInitialposeConfig.topic as string;
    }
  }, [initialposeTopicRef]);

  useEffect(() => {
    const saved = loadImagePositions();
    if (saved) {
      const map = new Map<string, { x: number; y: number; scale: number }>();
      for (const [layerId, position] of Object.entries(saved)) {
        map.set(layerId, position);
      }
      imagePositionsRef.current = map;
    }
  }, [imagePositionsRef]);
}

