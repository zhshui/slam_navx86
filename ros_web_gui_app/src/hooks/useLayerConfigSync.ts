import { useEffect } from 'react';
import type { LayerConfigMap } from '../types/LayerConfig';
import type { LayerManager } from '../components/layers/LayerManager';
import type { RosbridgeConnection } from '../utils/RosbridgeConnection';

export function useLayerConfigSync(
  layerConfigs: LayerConfigMap,
  layerConfigsRef: React.MutableRefObject<LayerConfigMap>,
  layerManagerRef: React.MutableRefObject<LayerManager | null>,
  connection: RosbridgeConnection | null,
  initialposeTopicRef: React.MutableRefObject<string>
) {
  useEffect(() => {
    layerConfigsRef.current = layerConfigs;
    const initialposeConfig = Object.values(layerConfigs).find(config => config.id === 'initialpose');
    if (initialposeConfig && initialposeConfig.topic) {
      initialposeTopicRef.current = initialposeConfig.topic as string;
    }
  }, [layerConfigs, layerConfigsRef, initialposeTopicRef]);

  useEffect(() => {
    console.log('[useLayerConfigSync] 同步配置:', {
      hasLayerManager: !!layerManagerRef.current,
      isConnected: connection?.isConnected(),
      configKeys: Object.keys(layerConfigs)
    });
    if (layerManagerRef.current && connection?.isConnected()) {
      layerManagerRef.current.setLayerConfigs(layerConfigs);
    } else {
      console.log('[useLayerConfigSync] 跳过:', { hasLM: !!layerManagerRef.current, connected: connection?.isConnected() });
    }
  }, [layerConfigs, connection, layerManagerRef]);
}

