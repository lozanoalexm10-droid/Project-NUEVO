/**
 * LidarWidget — live top-down LiDAR scan image.
 *
 * Polls /lidar/latest.png every 3 seconds while expanded.
 * The lidar_renderer ROS2 node must be running to produce the image.
 */
import { useState, useEffect } from 'react'
import { Radar } from 'lucide-react'

const REFRESH_MS = 3000
const STATUS_MS  = 4000
const STALE_S    = 5

export function LidarWidget() {
  const [visible, setVisible]     = useState(false)
  const [tick, setTick]           = useState(0)
  const [hasLoaded, setHasLoaded] = useState(false)
  const [live, setLive]           = useState(false)

  useEffect(() => {
    if (!visible) return
    const id = setInterval(() => setTick((t) => t + 1), REFRESH_MS)
    return () => clearInterval(id)
  }, [visible])

  useEffect(() => {
    if (!visible) return
    const check = async () => {
      try {
        const r = await fetch('/lidar/status')
        if (r.ok) {
          const d = await r.json()
          setLive(d.available && d.age_s < STALE_S)
        } else {
          setLive(false)
        }
      } catch {
        setLive(false)
      }
    }
    check()
    const id = setInterval(check, STATUS_MS)
    return () => clearInterval(id)
  }, [visible])

  const showFallback = !hasLoaded && !live

  return (
    <div className="rounded-xl backdrop-blur-xl bg-white/5 border border-white/10 overflow-hidden">
      <button
        className="w-full flex items-center justify-between px-3 py-2 hover:bg-white/5 transition-colors"
        onClick={() => setVisible((v) => !v)}
      >
        <span className="flex items-center gap-1.5">
          <span className="text-xs font-semibold text-white/70">LiDAR Scan</span>
          {visible && (
            <span
              className={`text-[10px] px-1.5 py-0.5 rounded-full font-medium transition-colors ${
                live
                  ? 'bg-emerald-500/20 text-emerald-400'
                  : 'bg-white/10 text-white/30'
              }`}
            >
              {live ? 'live' : 'stale'}
            </span>
          )}
        </span>
        <span className="flex items-center gap-1.5">
          <Radar className={`size-3.5 ${visible ? 'text-emerald-400' : 'text-white/40'}`} />
          <span className="text-xs text-white/40">{visible ? 'on' : 'off'}</span>
        </span>
      </button>

      {visible && (
        <div className="px-2 pb-2">
          {showFallback ? (
            <div className="flex flex-col items-center justify-center aspect-square min-h-48 rounded-lg bg-black/30 gap-1">
              <span className="text-xs text-white/30">No LiDAR signal</span>
              <span className="text-[10px] text-white/20">
                Start: ros2 run robot lidar_renderer
              </span>
            </div>
          ) : (
            <img
              src={`/lidar/latest.png?t=${tick}`}
              alt="LiDAR scan"
              className="w-full rounded-lg object-contain bg-black/40"
              onLoad={() => setHasLoaded(true)}
            />
          )}
        </div>
      )}
    </div>
  )
}
