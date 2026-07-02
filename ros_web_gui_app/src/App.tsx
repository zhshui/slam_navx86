import { useCallback, useEffect, useState } from 'react';
import { ToastContainer } from 'react-toastify';
import 'react-toastify/dist/ReactToastify.css';
import { MapView } from './components/MapView';
import { ConnectionPage } from './components/ConnectionPage';
import { RosbridgeConnection } from './utils/RosbridgeConnection';
import { TF2JS } from './utils/tf2js';
import { GatewayProvider } from './components/gateway/GatewayProvider';
import './App.css';

function App() {
  const [connection, setConnection] = useState<RosbridgeConnection | null>(null);
  const [connected, setConnected] = useState(false);
  const [gatewayToken, setGatewayToken] = useState<string | null>(
    () => localStorage.getItem('gatewayToken')
  );

  const updateUrl = (wsUrl: string) => {
    window.history.pushState({ wsUrl }, '', '/#' + encodeURIComponent(wsUrl));
  };

  const getWsUrlFromUrl = (): string | null => {
    const hash = window.location.hash;
    if (hash && hash.length > 1) {
      try {
        const decoded = decodeURIComponent(hash.substring(1));
        if (decoded.startsWith('ws://') || decoded.startsWith('wss://')) {
          return decoded;
        }
      } catch (e) {
        console.error('Failed to decode URL:', e);
      }
    }
    return null;
  };

  const handleConnect = useCallback(async (url: string): Promise<boolean> => {
    console.log('[App] handleConnect 开始:', url);
    const conn = new RosbridgeConnection();
    const success = await conn.connect(url);
    console.log('[App] handleConnect 结果:', { url, success });
    if (success) {
      setConnection(conn);
      setConnected(true);
      updateUrl(url);
      return true;
    }
    return false;
  }, []);

  const handleDisconnect = useCallback(() => {
    if (connection) {
      connection.disconnect();
      TF2JS.getInstance().disconnect();
      setConnection(null);
      setConnected(false);
    }
  }, [connection]);

  useEffect(() => {
    const wsUrl = getWsUrlFromUrl();
    console.log('[App] 初始连接检查:', { pathname: window.location.pathname, wsUrl, connected });
    if (wsUrl && !connected) {
      console.log('[App] 开始连接 rosbridge:', wsUrl);
      void handleConnect(wsUrl);
    }

    const handlePopState = () => {
      const wsUrl = getWsUrlFromUrl();
      if (wsUrl) {
        if (!connected) {
          void handleConnect(wsUrl);
        }
      } else {
        if (connected && connection) {
          connection.disconnect();
          TF2JS.getInstance().disconnect();
          setConnection(null);
          setConnected(false);
        }
      }
    };

    window.addEventListener('popstate', handlePopState);
    return () => {
      window.removeEventListener('popstate', handlePopState);
    };
  }, [connected, handleConnect, connection]);

  return (
    <>
      <GatewayProvider
        token={gatewayToken}
        onAuthError={() => {
          // 当 token 无效时（过期、密钥变更等）自动清除并返回登录页
          localStorage.removeItem('gatewayToken');
          setGatewayToken(null);
        }}
      >
        {!connected ? (
          <ConnectionPage
            onConnect={handleConnect}
            onGatewayMode={() => {
              // 不使用 rosbridge 直连，只通过网关
              setConnected(true);  // 绕过 rosbridge，直接进入主界面
            }}
          />
        ) : (
          <div style={{ position: 'fixed', top: 0, left: 0, right: 0, bottom: 0, margin: 0, padding: 0, overflow: 'hidden' }}>
            <MapView
              connection={connection}
              gatewayToken={gatewayToken}
              onGatewayLogin={(token: string) => {
                localStorage.setItem('gatewayToken', token);
                setGatewayToken(token);
              }}
              onGatewayLogout={() => {
                localStorage.removeItem('gatewayToken');
                setGatewayToken(null);
              }}
            />
          </div>
        )}
      </GatewayProvider>
      <ToastContainer position="top-center" />
    </>
  );
}

export default App;
