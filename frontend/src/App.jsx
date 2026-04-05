import { useEffect, useRef, useState } from 'react'

const WS_URL = 'ws://localhost:9002'

export default function App() {
  const [telemetry, setTelemetry] = useState(null)
  const [connected, setConnected] = useState(false)
  const maxAltRef = useRef(1) // avoid divide-by-zero at start
  const [maxAlt, setMaxAlt] = useState(1)

  useEffect(() => {
    let ws
    let reconnectTimer

    const connect = () => {
      ws = new WebSocket(WS_URL)

      ws.onopen = () => setConnected(true)
      ws.onclose = () => {
        setConnected(false)
        reconnectTimer = setTimeout(connect, 1000)
      }
      ws.onerror = () => ws.close()
      ws.onmessage = (e) => {
        try {
          const data = JSON.parse(e.data)
          setTelemetry(data)
          if (data.altitude > maxAltRef.current) {
            maxAltRef.current = data.altitude
            setMaxAlt(data.altitude)
          }
        } catch (err) {
          console.error('bad telemetry:', err)
        }
      }
    }

    connect()
    return () => {
      clearTimeout(reconnectTimer)
      if (ws) ws.close()
    }
  }, [])

  // --- Rocket visualization ---
  // Camera follows rocket. Viewport covers viewRangeFeet of altitude.
  // When rocket is low, camera is floored at 0 so ground sits at bottom.
  // As rocket climbs, camera scrolls up so new tick marks appear from above.
  const viewW = 400
  const viewH = 600
  const plotTop = 40
  const plotBottom = viewH - 20
  const plotHeight = plotBottom - plotTop

  const viewRangeFeet = 500        // ft of altitude shown in viewport
  const marginBelowFeet = 120      // keep this much ground/padding below rocket

  const alt = telemetry?.altitude ?? 0
  const cameraBottomFeet = Math.max(0, alt - marginBelowFeet)
  const cameraTopFeet = cameraBottomFeet + viewRangeFeet
  const feetToY = (f) => plotBottom - ((f - cameraBottomFeet) / viewRangeFeet) * plotHeight

  const rocketY = feetToY(alt)
  const groundY = feetToY(0)       // y-coord of altitude 0 (may be off-bottom)
  const groundVisible = groundY <= plotBottom + 1

  const throttle = telemetry?.throttle ?? 0
  const flameLen = throttle * 100
  const flameWidth = 4 + throttle * 10

  // Procedurally generate ticks at fixed feet intervals within the camera window.
  const tickStep = niceStep(viewRangeFeet / 10)
  const firstTick = Math.ceil(cameraBottomFeet / tickStep) * tickStep
  const ticks = []
  for (let v = firstTick; v <= cameraTopFeet; v += tickStep) ticks.push(v)

  const gauge = (label, value, unit = '') => (
    <div style={{
      padding: '10px 14px',
      background: '#151b2e',
      borderRadius: 6,
      border: '1px solid #252d45',
      minWidth: 180
    }}>
      <div style={{ fontSize: 11, color: '#8b98b8', textTransform: 'uppercase', letterSpacing: 1 }}>{label}</div>
      <div style={{ fontSize: 22, fontWeight: 600, fontVariantNumeric: 'tabular-nums' }}>
        {value}{unit && <span style={{ fontSize: 14, color: '#8b98b8', marginLeft: 4 }}>{unit}</span>}
      </div>
    </div>
  )

  const fmt = (v, digits = 1) =>
    v === undefined || v === null || Number.isNaN(v) ? '—' : Number(v).toFixed(digits)

  function niceStep(raw) {
    if (raw <= 0) return 1
    const mag = Math.pow(10, Math.floor(Math.log10(raw)))
    const n = raw / mag
    const pick = n < 1.5 ? 1 : n < 3 ? 2 : n < 7 ? 5 : 10
    return pick * mag
  }

  return (
    <div style={{ padding: 24, maxWidth: 1100, margin: '0 auto' }}>
      <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: 16 }}>
        <h1 style={{ margin: 0, fontSize: 24 }}>Flight Sim Telemetry</h1>
        <div style={{ display: 'flex', alignItems: 'center', gap: 8 }}>
          <span style={{
            width: 10, height: 10, borderRadius: '50%',
            background: connected ? '#3fb950' : '#f85149'
          }} />
          <span style={{ fontSize: 13, color: '#8b98b8' }}>
            {connected ? 'Connected' : 'Disconnected'}
          </span>
        </div>
      </div>

      <div style={{ display: 'flex', gap: 24, alignItems: 'flex-start' }}>
        {/* Rocket view */}
        <div style={{
          background: 'linear-gradient(to bottom, #0d1530 0%, #1a2540 70%, #2a3b5f 100%)',
          borderRadius: 8,
          border: '1px solid #252d45',
          padding: 12
        }}>
          <svg width={viewW} height={viewH} viewBox={`0 0 ${viewW} ${viewH}`}>
            <defs>
              <clipPath id="plotArea">
                <rect x={0} y={plotTop} width={viewW} height={plotHeight} />
              </clipPath>
              <clipPath id="aboveGround">
                <rect x={0} y={plotTop} width={viewW} height={Math.max(0, groundY - plotTop)} />
              </clipPath>
            </defs>

            {/* Procedurally generated altitude tick marks (scroll with camera) */}
            {ticks.map((v) => {
              const y = feetToY(v)
              const labelY = y > plotBottom - 8 ? y - 4 : y + 4
              return (
                <g key={v}>
                  <line x1={40} y1={y} x2={60} y2={y} stroke="#3a4566" strokeWidth={1} />
                  <text x={10} y={labelY} fill="#8b98b8" fontSize={10}>
                    {v}
                  </text>
                </g>
              )
            })}
            <line x1={60} y1={plotTop} x2={60} y2={plotBottom} stroke="#3a4566" strokeWidth={1} />

            {/* Rocket (clipped so flame doesn't render below ground when visible) */}
            <g clipPath={groundVisible ? 'url(#aboveGround)' : 'url(#plotArea)'}>
              <g transform={`translate(200 ${rocketY})`}>
                {/* Flame — grows with throttle */}
                {throttle > 0 && (
                  <>
                    <polygon
                      points={`${-flameWidth},0 ${flameWidth},0 0,${flameLen}`}
                      fill="#ff5722"
                      opacity={0.9}
                    />
                    <polygon
                      points={`${-flameWidth * 0.55},0 ${flameWidth * 0.55},0 0,${flameLen * 0.7}`}
                      fill="#ffc107"
                    />
                    <polygon
                      points={`${-flameWidth * 0.22},0 ${flameWidth * 0.22},0 0,${flameLen * 0.4}`}
                      fill="#ffffff"
                    />
                  </>
                )}
                {/* Fins (extend down to base at y=0) */}
                <polygon points="-10,-10 -10,0 -18,0" fill="#8b98b8" />
                <polygon points="10,-10 10,0 18,0" fill="#8b98b8" />
                {/* Body (base at y=0, top at y=-60) */}
                <rect x={-10} y={-60} width={20} height={60} fill="#d8dce5" stroke="#8b98b8" strokeWidth={1} />
                {/* Nose cone */}
                <polygon points="-10,-60 10,-60 0,-80" fill="#e8ebf0" stroke="#8b98b8" strokeWidth={1} />
                {/* Window */}
                <circle cx={0} cy={-40} r={3} fill="#5ca0ff" />
              </g>
            </g>

            {/* Ground — only drawn when the 0-altitude line is within the viewport */}
            {groundVisible && (
              <>
                <rect x={0} y={groundY} width={viewW} height={plotBottom - groundY + 1} fill="#3d2e1f" />
                <line x1={0} y1={groundY} x2={viewW} y2={groundY} stroke="#6b4e2e" strokeWidth={2} />
              </>
            )}
          </svg>
        </div>

        {/* Gauges */}
        <div style={{ display: 'flex', flexDirection: 'column', gap: 12, flex: 1 }}>
          {gauge('State', telemetry?.state ?? '—')}
          {gauge('Altitude', fmt(telemetry?.altitude), 'ft')}
          {gauge('Velocity', fmt(telemetry?.velocity), 'ft/s')}
          {gauge('Throttle', fmt((telemetry?.throttle ?? 0) * 100), '%')}
          {gauge('Mass', fmt(telemetry?.mass, 0), 'kg')}
          {gauge('Max Altitude', fmt(maxAlt), 'ft')}
          {gauge('Cycle', telemetry?.cycle ?? '—')}
        </div>
      </div>
    </div>
  )
}
