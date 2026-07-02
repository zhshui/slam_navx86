import { createContext, useContext, type ReactNode } from 'react'
import { useGateway, type UseGatewayReturn } from '../../hooks/useGateway'

const GatewayContext = createContext<UseGatewayReturn | null>(null)

export function useGatewayContext(): UseGatewayReturn {
  const ctx = useContext(GatewayContext)
  if (!ctx) throw new Error('useGatewayContext must be used within GatewayProvider')
  return ctx
}

export function GatewayProvider({ token, onAuthError, children }: { token: string | null; onAuthError?: () => void; children: ReactNode }) {
  const gw = useGateway(token, onAuthError)
  return <GatewayContext.Provider value={gw}>{children}</GatewayContext.Provider>
}
