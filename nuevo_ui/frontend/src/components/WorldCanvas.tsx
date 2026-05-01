/**
 * WorldCanvas — live 2D world-frame map.
 *
 * Card shell and toggle legend styled to match OdometryCard (SensorSection).
 * Canvas style (translucent dark, subtle grid) is the original WorldCanvas design.
 */
import { useRef, useEffect, useState, useCallback } from 'react'
import { useRobotStore, LIDAR_WINDOW_FRAMES } from '../store/robotStore'

// Venue: 5×6 grid, 610 mm cells.
// Origin (0,0) = centre of bottom border of the bottom-left cell.
// → venue spans x: -305..2745, y: 0..3660
const GRID_MM        = 610
const VENUE_COLS     = 5
const VENUE_ROWS     = 6
const VENUE_LEFT_MM  = -GRID_MM / 2                          // -305
const VENUE_RIGHT_MM =  VENUE_LEFT_MM + VENUE_COLS * GRID_MM // 2745
const VENUE_TOP_MM   =  VENUE_ROWS * GRID_MM                 // 3660
const VENUE_PAD_MM   =  GRID_MM / 2                          // 305

const VENUE_EXT = {
  minX: VENUE_LEFT_MM  - VENUE_PAD_MM,
  maxX: VENUE_RIGHT_MM + VENUE_PAD_MM,
  minY:                - VENUE_PAD_MM,
  maxY: VENUE_TOP_MM   + VENUE_PAD_MM,
}

interface Trails { odom: boolean; gps: boolean; fused: boolean; lidar: boolean }

const SERIES = [
  { key: 'odom'  as const, label: 'Odometry', color: '#4ade80' },
  { key: 'fused' as const, label: 'Fused',    color: '#60a5fa' },
  { key: 'gps'   as const, label: 'GPS',      color: '#facc15' },
  { key: 'lidar' as const, label: 'Lidar',    color: '#f87171' },
]

export function WorldCanvas() {
  const canvasRef = useRef<HTMLCanvasElement>(null)

  const fusedPose   = useRobotStore((s) => s.fusedPose)
  const fusedTrail  = useRobotStore((s) => s.fusedPoseTrail)
  const odomTrail   = useRobotStore((s) => s.odometryTrail)
  const gpsStatus     = useRobotStore((s) => s.gpsStatus)
  const tagDetections = useRobotStore((s) => s.tagDetections)
  const lidarPoints = useRobotStore((s) => s.lidarPoints)

  const [trails, setTrails] = useState<Trails>({ odom: true, gps: true, fused: true, lidar: true })

  const toggle = useCallback((key: keyof Trails) => {
    setTrails((t) => ({ ...t, [key]: !t[key] }))
  }, [])

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const dpr = window.devicePixelRatio || 1
    const W   = canvas.clientWidth
    const H   = canvas.clientHeight
    if (!W || !H) return
    canvas.width  = W * dpr
    canvas.height = H * dpr
    const ctx = canvas.getContext('2d')!
    ctx.scale(dpr, dpr)

    const allPts: [number, number][] = []
    if (trails.fused)  allPts.push(...fusedTrail)
    if (trails.odom)   allPts.push(...odomTrail)
    if (fusedPose)     allPts.push([fusedPose.x, fusedPose.y])
    if (trails.gps) for (const t of tagDetections) allPts.push([t.x, t.y])
    if (trails.lidar) {
      for (const frame of lidarPoints)
        for (let i = 0; i < frame.xs.length; i++)
          allPts.push([frame.xs[i], frame.ys[i]])
    }

    // Start from venue bounds; expand only if data falls outside.
    let ext = { ...VENUE_EXT }
    if (allPts.length > 0) {
      const xs = allPts.map((p) => p[0])
      const ys = allPts.map((p) => p[1])
      ext = {
        minX: Math.min(ext.minX, Math.min(...xs) - VENUE_PAD_MM),
        maxX: Math.max(ext.maxX, Math.max(...xs) + VENUE_PAD_MM),
        minY: Math.min(ext.minY, Math.min(...ys) - VENUE_PAD_MM),
        maxY: Math.max(ext.maxY, Math.max(...ys) + VENUE_PAD_MM),
      }
    }
    const rangeX = Math.max(ext.maxX - ext.minX, 1)
    const rangeY = Math.max(ext.maxY - ext.minY, 1)
    const scale  = Math.min(W / rangeX, H / rangeY)

    // Centre the content so padding is equal on all sides regardless of aspect ratio.
    const ox = (W - rangeX * scale) / 2
    const oy = (H - rangeY * scale) / 2

    const toC = (wx: number, wy: number): [number, number] => [
      ox + (wx - ext.minX) * scale,
      oy + rangeY * scale - (wy - ext.minY) * scale,
    ]

    // Translucent dark background (original style)
    ctx.fillStyle = 'rgba(0,0,0,0.55)'
    ctx.fillRect(0, 0, W, H)

    // Venue boundary rectangle
    {
      const [vx0, vy0] = toC(VENUE_LEFT_MM, 0)
      const [vx1, vy1] = toC(VENUE_RIGHT_MM, VENUE_TOP_MM)
      ctx.strokeStyle = 'rgba(255,255,255,0.18)'
      ctx.lineWidth   = 1
      ctx.strokeRect(vx0, vy1, vx1 - vx0, vy0 - vy1)
    }

    // Venue grid lines at 610 mm
    ctx.strokeStyle = 'rgba(255,255,255,0.08)'
    ctx.lineWidth   = 0.5
    for (let col = 0; col <= VENUE_COLS; col++) {
      const [cx] = toC(VENUE_LEFT_MM + col * GRID_MM, 0)
      ctx.beginPath(); ctx.moveTo(cx, 0); ctx.lineTo(cx, H); ctx.stroke()
    }
    for (let row = 0; row <= VENUE_ROWS; row++) {
      const [, cy] = toC(0, row * GRID_MM)
      ctx.beginPath(); ctx.moveTo(0, cy); ctx.lineTo(W, cy); ctx.stroke()
    }

    // Origin cross
    const [originX, originY] = toC(0, 0)
    if (originX >= 0 && originX <= W && originY >= 0 && originY <= H) {
      ctx.strokeStyle = 'rgba(255,255,255,0.20)'
      ctx.lineWidth = 1
      ctx.beginPath(); ctx.moveTo(originX - 6, originY); ctx.lineTo(originX + 6, originY); ctx.stroke()
      ctx.beginPath(); ctx.moveTo(originX, originY - 6); ctx.lineTo(originX, originY + 6); ctx.stroke()
    }

    // Scale label
    ctx.fillStyle = 'rgba(255,255,255,0.40)'
    ctx.font = '9px monospace'
    ctx.textAlign = 'right'
    ctx.textBaseline = 'bottom'
    ctx.fillText(`grid: ${GRID_MM} mm`, W - 4, H - 3)

    // Odometry trail (green)
    if (trails.odom && odomTrail.length > 1) {
      ctx.beginPath()
      ctx.strokeStyle = 'rgba(74,222,128,0.55)'
      ctx.lineWidth = 1.5
      ctx.lineJoin = 'round'
      odomTrail.forEach(([wx, wy], i) => {
        const [cx, cy] = toC(wx, wy)
        i === 0 ? ctx.moveTo(cx, cy) : ctx.lineTo(cx, cy)
      })
      ctx.stroke()
    }

    // Fused trail (blue)
    if (trails.fused && fusedTrail.length > 1) {
      ctx.beginPath()
      ctx.strokeStyle = 'rgba(96,165,250,0.70)'
      ctx.lineWidth = 1.5
      ctx.lineJoin = 'round'
      fusedTrail.forEach(([wx, wy], i) => {
        const [cx, cy] = toC(wx, wy)
        i === 0 ? ctx.moveTo(cx, cy) : ctx.lineTo(cx, cy)
      })
      ctx.stroke()
    }

    // GPS tags — yellow cross + ID label for every currently detected tag
    if (trails.gps && tagDetections.length > 0) {
      const arm = 6
      ctx.strokeStyle = 'rgba(250,204,21,0.90)'
      ctx.lineWidth = 2
      ctx.fillStyle = 'rgba(250,204,21,0.90)'
      ctx.font = '10px monospace'
      ctx.textAlign = 'left'
      ctx.textBaseline = 'bottom'
      for (const tag of tagDetections) {
        const [cx, cy] = toC(tag.x, tag.y)
        ctx.beginPath(); ctx.moveTo(cx - arm, cy); ctx.lineTo(cx + arm, cy); ctx.stroke()
        ctx.beginPath(); ctx.moveTo(cx, cy - arm); ctx.lineTo(cx, cy + arm); ctx.stroke()
        ctx.fillText(`#${tag.tag_id}`, cx + arm + 2, cy)
      }
    }

    // Lidar cloud — all frames in the rolling window, oldest most transparent
    if (trails.lidar && lidarPoints.length > 0) {
      for (let f = 0; f < lidarPoints.length; f++) {
        const alpha = 0.25 + 0.45 * (f / (LIDAR_WINDOW_FRAMES - 1 || 1))  // 0.25 → 0.70
        ctx.fillStyle = `rgba(248,113,113,${alpha.toFixed(2)})`
        const frame = lidarPoints[f]
        for (let i = 0; i < frame.xs.length; i++) {
          const [cx, cy] = toC(frame.xs[i], frame.ys[i])
          ctx.beginPath(); ctx.arc(cx, cy, 1.5, 0, Math.PI * 2); ctx.fill()
        }
      }
    }

    // Robot at fused pose — matches OdometryCard style (blue dot + heading arrow)
    if (fusedPose) {
      const [rx, ry] = toC(fusedPose.x, fusedPose.y)
      const headingCanvas = -fusedPose.theta  // world CCW → canvas CW
      const arrowLen = Math.max(12, Math.min(28, Math.min(rangeX, rangeY) * scale * 0.06))
      const nx = Math.cos(headingCanvas), ny = Math.sin(headingCanvas)
      const tipX = rx + nx * arrowLen, tipY = ry + ny * arrowLen
      const hw = 4

      // Body
      ctx.beginPath()
      ctx.arc(rx, ry, 7, 0, Math.PI * 2)
      ctx.fillStyle   = 'rgba(96,165,250,0.25)'
      ctx.strokeStyle = '#60a5fa'
      ctx.lineWidth   = 1.5
      ctx.fill()
      ctx.stroke()

      // Heading arrow shaft
      ctx.strokeStyle = '#60a5fa'
      ctx.lineWidth   = 2
      ctx.beginPath(); ctx.moveTo(rx, ry); ctx.lineTo(tipX, tipY); ctx.stroke()

      // Arrowhead
      ctx.fillStyle = '#60a5fa'
      ctx.beginPath()
      ctx.moveTo(tipX, tipY)
      ctx.lineTo(tipX - nx * 8 + ny * hw, tipY - ny * 8 - nx * hw)
      ctx.lineTo(tipX - nx * 8 - ny * hw, tipY - ny * 8 + nx * hw)
      ctx.closePath()
      ctx.fill()
    }
  }, [fusedPose, fusedTrail, odomTrail, gpsStatus, tagDetections, lidarPoints, trails])

  return (
    <div className="relative rounded-2xl p-4 backdrop-blur-2xl bg-white/10 border border-white/20 shadow-xl">
      <div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/50 to-transparent rounded-t-2xl" />
      <div className="absolute inset-0 rounded-2xl bg-gradient-to-br from-white/5 to-transparent opacity-50" />

      <div className="relative">
        <h4 className="text-sm font-semibold text-white mb-3">World Map</h4>

        {/* Series toggles — DCPlot legend style */}
        <div className="flex flex-wrap gap-x-3 gap-y-1 mb-2">
          {SERIES.map(({ key, label, color }) => (
            <button
              key={key}
              onClick={() => toggle(key)}
              className="flex items-center gap-1.5 cursor-pointer rounded px-1 py-0.5 transition-opacity hover:bg-white/10"
              style={{ opacity: trails[key] ? 1 : 0.35 }}
            >
              <span style={{ display: 'inline-block', width: 12, height: 3, borderRadius: 2, background: color }} />
              <span className="text-xs font-medium text-white/80">{label}</span>
            </button>
          ))}
        </div>

        <canvas
          ref={canvasRef}
          className="w-full rounded-xl"
          style={{ height: '420px' }}
        />
      </div>
    </div>
  )
}
