/**
 * VisionFeedWidget — live annotated camera feed from the Pi vision node.
 *
 * Polls /vision/latest.jpg every 250 ms while expanded. The vision node must
 * be running with debug_save_enabled=true and debug_save_latest=true for the
 * image to exist (it is written to /runtime_output/vision/latest.jpg on the Pi).
 *
 * A "live" badge turns green when the saved image is less than 3 s old
 * (checked via /vision/status every 2 s).
 */
import { useState, useEffect } from 'react'
import { Video, VideoOff } from 'lucide-react'

const REFRESH_MS = 250   // how often to reload the image
const STATUS_MS  = 2000  // how often to poll /vision/status
const STALE_S    = 3     // image older than this → "stale"

export function VisionFeedWidget() {
  const [visible, setVisible] = useState(false)
  const [tick, setTick]       = useState(0)
  const [imgError, setImgError] = useState(false)
  const [live, setLive]       = useState(false)

  // Refresh image src by bumping tick
  useEffect(() => {
    if (!visible) return
    const id = setInterval(() => setTick((t) => t + 1), REFRESH_MS)
    return () => clearInterval(id)
  }, [visible])

  // Poll /vision/status for live badge
  useEffect(() => {
    if (!visible) return
    const check = async () => {
      try {
        const r = await fetch('/vision/status')
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

  const toggle = () => {
    setVisible((v) => !v)
    setImgError(false)
  }

  return (
    <div className="rounded-xl backdrop-blur-xl bg-white/5 border border-white/10 overflow-hidden">
      {/* Header / toggle */}
      <button
        className="w-full flex items-center justify-between px-3 py-2 hover:bg-white/5 transition-colors"
        onClick={toggle}
      >
        <span className="flex items-center gap-1.5">
          <span className="text-xs font-semibold text-white/70">Vision Feed</span>
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
          {visible ? (
            <Video className="size-3.5 text-emerald-400" />
          ) : (
            <VideoOff className="size-3.5 text-white/40" />
          )}
          <span className="text-xs text-white/40">{visible ? 'on' : 'off'}</span>
        </span>
      </button>

      {/* Feed */}
      {visible && (
        <div className="px-2 pb-2">
          {imgError ? (
            <div className="flex flex-col items-center justify-center h-28 rounded-lg bg-black/30 gap-1">
              <span className="text-xs text-white/30">No vision signal</span>
              <span className="text-[10px] text-white/20">
                Start vision node with debug_save_enabled:=true
              </span>
            </div>
          ) : (
            <img
              key={tick}
              src={`/vision/latest.jpg?t=${tick}`}
              alt="Vision detections"
              className="w-full rounded-lg object-contain bg-black/40"
              onError={() => setImgError(true)}
              onLoad={() => setImgError(false)}
            />
          )}
        </div>
      )}
    </div>
  )
}
