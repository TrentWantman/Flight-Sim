import { useEffect, useRef, useState } from 'react'

const WS_URL = 'ws://localhost:9002'

export default function App() {
  const [telemetry, setTelemetry] = useState(null)
  const [connected, setConnected] = useState(false)
  const maxAltRef = useRef(1)
  const [maxAlt, setMaxAlt] = useState(1)
  const rocketAngleRef = useRef(0)

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
          if (data.posZ > maxAltRef.current) {
            maxAltRef.current = data.posZ
            setMaxAlt(data.posZ)
          }
          // True orientation from the rocket's forward direction vector
          if (data.orientX !== undefined && data.orientZ !== undefined) {
            rocketAngleRef.current = Math.atan2(data.orientX, data.orientZ) * 180 / Math.PI
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

  // --- Layout constants ---
  const viewW = 600
  const viewH = 600
  const plotTop = 30
  const plotBottom = viewH - 20
  const plotHeight = plotBottom - plotTop  // pixel height of plot
  const tickMargin = 60                   // left space for altitude labels
  const plotLeft = tickMargin
  const plotRight = viewW - 10
  const plotWidth = plotRight - plotLeft

  // --- Camera: unified scale, follows rocket in X and Z ---
  const viewRangeFeet = 500
  const scale = plotHeight / viewRangeFeet   // px per foot (same for both axes)
  const hViewRangeFeet = plotWidth / scale   // horizontal feet visible

  const posX = telemetry?.posX ?? 0
  const posZ = telemetry?.posZ ?? 0
  const velX = telemetry?.velX ?? 0
  const velZ = telemetry?.velZ ?? 0
  const throttle = telemetry?.throttle ?? 0
  const speed = Math.sqrt(velX ** 2 + velZ ** 2)

  // Vertical camera: floor at 0 when low
  const cameraBottomZ = Math.max(0, posZ - viewRangeFeet * 0.25)
  const cameraTopZ = cameraBottomZ + viewRangeFeet

  // Horizontal camera: keep launchpad (x=0) visible when close, pan when far
  const cameraLeftX = Math.max(-hViewRangeFeet * 0.4, posX - hViewRangeFeet * 0.5)
  const cameraRightX = cameraLeftX + hViewRangeFeet

  const feetToSvgY = (z) => plotBottom - ((z - cameraBottomZ) * scale)
  const feetToSvgX = (x) => plotLeft + ((x - cameraLeftX) * scale)

  const rocketSvgX = feetToSvgX(posX)
  const rocketSvgY = feetToSvgY(posZ)
  const groundSvgY = feetToSvgY(0)
  const groundVisible = groundSvgY <= plotBottom + 1
  const padSvgX = feetToSvgX(0)

  // Rocket orientation (degrees clockwise from up)
  const rocketAngleDeg = rocketAngleRef.current

  // Flame
  const flameLen = throttle * 100
  const flameWidth = 4 + throttle * 10

  // Velocity vector arrow (capped at 80px)
  const velArrowMaxPx = 80
  const velArrowLen = Math.min(speed * 0.4, velArrowMaxPx)
  const velNx = speed > 0.5 ? velX / speed : 0
  const velNz = speed > 0.5 ? velZ / speed : 0
  // SVG direction: velX → +X, velZ → -Y
  const velArrowDx = velNx * velArrowLen
  const velArrowDy = -velNz * velArrowLen

  // Altitude ticks (vertical, left side)
  const vTickStep = niceStep(viewRangeFeet / 10)
  const vFirstTick = Math.ceil(cameraBottomZ / vTickStep) * vTickStep
  const vTicks = []
  for (let v = vFirstTick; v <= cameraTopZ; v += vTickStep) vTicks.push(v)

  // Horizontal ticks (bottom edge)
  const hTickStep = niceStep(hViewRangeFeet / 8)
  const hFirstTick = Math.ceil(cameraLeftX / hTickStep) * hTickStep
  const hTicks = []
  for (let v = hFirstTick; v <= cameraRightX; v += hTickStep) hTicks.push(v)

  function niceStep(raw) {
    if (raw <= 0) return 1
    const mag = Math.pow(10, Math.floor(Math.log10(raw)))
    const n = raw / mag
    const pick = n < 1.5 ? 1 : n < 3 ? 2 : n < 7 ? 5 : 10
    return pick * mag
  }

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

  return (
    <div style={{ padding: 24, maxWidth: 1200, margin: '0 auto' }}>
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
                <rect x={plotLeft} y={plotTop} width={plotWidth} height={plotHeight} />
              </clipPath>
              <clipPath id="aboveGround">
                <rect x={plotLeft} y={plotTop} width={plotWidth} height={Math.max(0, groundSvgY - plotTop)} />
              </clipPath>
              <marker id="velArrowHead" markerWidth="6" markerHeight="6" refX="5" refY="3" orient="auto">
                <path d="M0,0 L6,3 L0,6 Z" fill="#3fb950" />
              </marker>
            </defs>

            {/* Altitude ticks (left axis) */}
            {vTicks.map((v) => {
              const y = feetToSvgY(v)
              if (y < plotTop - 5 || y > plotBottom + 5) return null
              const labelY = y > plotBottom - 8 ? y - 4 : y + 4
              return (
                <g key={'vt' + v}>
                  <line x1={plotLeft - 15} y1={y} x2={plotLeft} y2={y} stroke="#3a4566" strokeWidth={1} />
                  <text x={5} y={labelY} fill="#8b98b8" fontSize={10}>{v}</text>
                </g>
              )
            })}
            <line x1={plotLeft} y1={plotTop} x2={plotLeft} y2={plotBottom} stroke="#3a4566" strokeWidth={1} />

            {/* Horizontal ticks (bottom axis) */}
            {hTicks.map((v) => {
              const x = feetToSvgX(v)
              if (x < plotLeft - 5 || x > plotRight + 5) return null
              return (
                <g key={'ht' + v}>
                  <line x1={x} y1={plotBottom} x2={x} y2={plotBottom + 10} stroke="#3a4566" strokeWidth={1} />
                  <text x={x} y={plotBottom + 18} fill="#8b98b8" fontSize={9} textAnchor="middle">{v}</text>
                </g>
              )
            })}
            <line x1={plotLeft} y1={plotBottom} x2={plotRight} y2={plotBottom} stroke="#3a4566" strokeWidth={1} />

            {/* Axis labels */}
            <text x={5} y={plotTop - 8} fill="#8b98b8" fontSize={10}>Alt (m)</text>
            <text x={plotRight - 30} y={plotBottom + 18} fill="#8b98b8" fontSize={9}>X (m)</text>

            <g clipPath="url(#plotArea)">
              {/* Ground strip (at altitude 0) */}
              {groundVisible && (
                <>
                  <rect x={plotLeft} y={groundSvgY} width={plotWidth} height={plotBottom - groundSvgY + 1} fill="#3d2e1f" />
                  <line x1={plotLeft} y1={groundSvgY} x2={plotRight} y2={groundSvgY} stroke="#6b4e2e" strokeWidth={2} />
                  {/* Launch pad marker */}
                  {padSvgX >= plotLeft && padSvgX <= plotRight && (
                    <rect x={padSvgX - 12} y={groundSvgY - 3} width={24} height={3} fill="#555" rx={1} />
                  )}
                </>
              )}

              {/* Velocity vector arrow */}
              {speed > 1 && (
                <line
                  x1={rocketSvgX} y1={rocketSvgY - 30}
                  x2={rocketSvgX + velArrowDx} y2={rocketSvgY - 30 + velArrowDy}
                  stroke="#3fb950" strokeWidth={2}
                  markerEnd="url(#velArrowHead)"
                  opacity={0.8}
                />
              )}

              {/* Rocket — rotated by orientation */}
              <g transform={`translate(${rocketSvgX} ${rocketSvgY}) rotate(${rocketAngleDeg})`}>
                {/* Flame */}
                {throttle > 0 && (
                  <>
                    <polygon
                      points={`${-flameWidth},0 ${flameWidth},0 0,${flameLen}`}
                      fill="#ff5722" opacity={0.9}
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
                {/* Fins */}
                <polygon points="-10,-10 -10,0 -18,0" fill="#8b98b8" />
                <polygon points="10,-10 10,0 18,0" fill="#8b98b8" />
                {/* Body (base at 0, top at -60) */}
                <rect x={-10} y={-60} width={20} height={60} fill="#d8dce5" stroke="#8b98b8" strokeWidth={1} />
                {/* Nose cone */}
                <polygon points="-10,-60 10,-60 0,-80" fill="#e8ebf0" stroke="#8b98b8" strokeWidth={1} />
                {/* Window */}
                <circle cx={0} cy={-40} r={3} fill="#5ca0ff" />
              </g>
            </g>
          </svg>
        </div>

        {/* Gauges */}
        <div style={{ display: 'flex', flexDirection: 'column', gap: 10, flex: 1 }}>
          {gauge('State', telemetry?.state ?? '—')}
          {gauge('Altitude (Z)', fmt(telemetry?.posZ), 'm')}
          {gauge('Horizontal (X)', fmt(telemetry?.posX), 'm')}
          {gauge('Vel Z', fmt(telemetry?.velZ), 'm/s')}
          {gauge('Vel X', fmt(telemetry?.velX), 'm/s')}
          {gauge('Speed', fmt(speed), 'm/s')}
          {gauge('Throttle', fmt(throttle * 100), '%')}
          {gauge('Mass', fmt(telemetry?.mass, 0), 'kg')}
          {gauge('Delta-V', fmt(telemetry?.deltaV, 0), 'm/s')}
          {gauge('Gravity Loss', fmt(telemetry?.gravityLoss, 1), 'm/s')}
          {gauge('Max Altitude', fmt(maxAlt), 'm')}
          {gauge('Cycle', telemetry?.cycle ?? '—')}
        </div>
      </div>
    </div>
  )
}
