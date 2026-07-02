import { useEffect, useRef } from 'react';
import type { RosbridgeConnection } from '../utils/RosbridgeConnection';
import { TF2JS } from '../utils/tf2js';
import { MapManager } from '../utils/MapManager';
import type { LayerManager } from '../components/layers/LayerManager';
import type { LayerConfigMap } from '../types/LayerConfig';

export function useConnectionInit(
  connection: RosbridgeConnection | null,
  layerManagerRef: React.MutableRefObject<LayerManager | null>,
  layerConfigs: LayerConfigMap
) {
  const initializedRef = useRef(false)

  // 轮询连接状态，一旦连上就初始化图层
  useEffect(() => {
    if (!connection) return

    let initStarted = false

    const tryInit = async () => {
      if (initializedRef.current) return
      if (!connection.isConnected() || !layerManagerRef.current) return
      if (initStarted) return // 防止并发重入

      initStarted = true
      console.log('[useConnectionInit] rosbridge connected, syncing layers...')

      try {
        await connection.initializeMessageReaders()
      } catch (error) {
        console.warn('Failed to initialize message readers, using default configs:', error)
      }

      const mapManager = MapManager.getInstance()
      mapManager.initialize(connection)
      TF2JS.getInstance().initialize(connection)
      layerManagerRef.current.setLayerConfigs(layerConfigs)
      initializedRef.current = true // 成功后标记，防止重复初始化
      console.log('[useConnectionInit] layers synced successfully')
    }

    tryInit()
    const timer = setInterval(tryInit, 500)

    return () => {
      clearInterval(timer)
      initializedRef.current = false
      MapManager.getInstance().disconnect()
    }
  }, [connection, layerConfigs, layerManagerRef])
}

