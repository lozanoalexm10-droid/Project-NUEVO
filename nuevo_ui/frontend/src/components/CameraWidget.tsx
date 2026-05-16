/**
 * CameraWidget — toggleable MJPEG preview stream from the Jetson camera.
 */
import { useState } from 'react'
import { Video, VideoOff } from 'lucide-react'

interface Props {
  streamUrl?: string
}

export function CameraWidget({ streamUrl = 'http://192.168.8.120:7778/stream' }: Props) {
  const [visible, setVisible] = useState(false)
  const [error, setError] = useState(false)

  return (
    <div className="rounded-xl backdrop-blur-xl bg-white/5 border border-white/10 overflow-hidden">
      {/* Header / toggle button */}
      <button
        className="w-full flex items-center justify-between px-3 py-2 hover:bg-white/5 transition-colors"
        onClick={() => {
          setVisible((v) => !v)
          setError(false)
        }}
      >
        <span className="text-xs font-semibold text-white/70">Camera Preview</span>
        <span className="flex items-center gap-1.5">
          {visible ? (
            <Video className="size-3.5 text-emerald-400" />
          ) : (
            <VideoOff className="size-3.5 text-white/40" />
          )}
          <span className="text-xs text-white/40">{visible ? 'on' : 'off'}</span>
        </span>
      </button>

      {/* Stream panel */}
      {visible && (
        <div className="px-2 pb-2">
          {error ? (
            <div className="flex items-center justify-center h-28 rounded-lg bg-black/30 text-xs text-white/30">
              No signal
            </div>
          ) : (
            <img
              src={streamUrl}
              alt="Camera stream"
              className="w-full rounded-lg object-contain bg-black/40"
              onError={() => setError(true)}
              crossOrigin="anonymous"
            />
          )}
        </div>
      )}
    </div>
  )
}
