/**
 * CameraWidget — toggleable MJPEG preview stream from the Jetson GPS camera.
 * Admin-only recalibrate button triggers a fresh world-frame calibration.
 */
import { useState } from 'react'
import { Video, VideoOff, RefreshCw } from 'lucide-react'
import { useAuthStore } from '../store/authStore'

const JETSON_BASE = 'http://192.168.8.120:7778'

export function CameraWidget() {
  const role = useAuthStore((s) => s.role)

  const [visible, setVisible] = useState(false)
  const [streamError, setStreamError] = useState(false)
  const [recalState, setRecalState] = useState<'idle' | 'pending' | 'ok' | 'err'>('idle')

  const handleRecalibrate = async () => {
    setRecalState('pending')
    try {
      const res = await fetch(`${JETSON_BASE}/recalibrate`, { method: 'POST' })
      setRecalState(res.ok ? 'ok' : 'err')
    } catch {
      setRecalState('err')
    }
    setTimeout(() => setRecalState('idle'), 3000)
  }

  return (
    <div className="rounded-xl backdrop-blur-xl bg-white/5 border border-white/10 overflow-hidden">
      {/* Header / toggle */}
      <button
        className="w-full flex items-center justify-between px-3 py-2 hover:bg-white/5 transition-colors"
        onClick={() => { setVisible((v) => !v); setStreamError(false) }}
      >
        <span className="text-xs font-semibold text-white/70">Camera Preview</span>
        <span className="flex items-center gap-1.5">
          {visible
            ? <Video className="size-3.5 text-emerald-400" />
            : <VideoOff className="size-3.5 text-white/40" />}
          <span className="text-xs text-white/40">{visible ? 'on' : 'off'}</span>
        </span>
      </button>

      {/* Stream + controls */}
      {visible && (
        <div className="px-2 pb-2 space-y-2">
          {streamError ? (
            <div className="flex items-center justify-center h-28 rounded-lg bg-black/30 text-xs text-white/30">
              No signal
            </div>
          ) : (
            <img
              src={`${JETSON_BASE}/stream`}
              alt="Camera stream"
              className="w-full rounded-lg object-contain bg-black/40"
              onError={() => setStreamError(true)}
              crossOrigin="anonymous"
            />
          )}

          {role === 'admin' && (
            <button
              onClick={handleRecalibrate}
              disabled={recalState === 'pending'}
              className={`w-full flex items-center justify-center gap-1.5 px-2 py-1 rounded-lg text-xs font-medium transition-all
                ${recalState === 'ok'  ? 'bg-emerald-500/20 text-emerald-400 border border-emerald-500/30' :
                  recalState === 'err' ? 'bg-rose-500/20 text-rose-400 border border-rose-500/30' :
                  'bg-white/10 text-white/60 border border-white/10 hover:bg-white/20 hover:text-white disabled:opacity-40'}`}
            >
              <RefreshCw className={`size-3 ${recalState === 'pending' ? 'animate-spin' : ''}`} />
              {recalState === 'pending' ? 'Recalibrating…'  :
               recalState === 'ok'      ? 'Calibrated ✓'    :
               recalState === 'err'     ? 'Failed — retry?' :
               'Recalibrate'}
            </button>
          )}
        </div>
      )}
    </div>
  )
}
