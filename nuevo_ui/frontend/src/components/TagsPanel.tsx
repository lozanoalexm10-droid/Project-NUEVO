/**
 * TagsPanel — lists all currently detected ArUco tags from the global GPS camera.
 */
import { useRobotStore } from '../store/robotStore'

export function TagsPanel() {
  const tags = useRobotStore((s) => s.tagDetections)

  return (
    <div className="rounded-xl backdrop-blur-xl bg-white/5 border border-white/10 p-3 space-y-2">
      <div className="flex items-center justify-between">
        <span className="text-xs font-semibold text-white/70">Tracked Tags</span>
        <span className={`text-xs font-bold px-2 py-0.5 rounded-full ${
          tags.length > 0
            ? 'bg-emerald-500/20 text-emerald-400'
            : 'bg-white/10 text-white/30'
        }`}>
          {tags.length} detected
        </span>
      </div>

      {tags.length === 0 ? (
        <p className="text-xs text-white/30 italic">No tags visible</p>
      ) : (
        <div className="space-y-1">
          {/* Header row */}
          <div className="grid grid-cols-3 text-[10px] text-white/40 font-medium uppercase tracking-wider pb-0.5 border-b border-white/10">
            <span>Tag ID</span>
            <span className="text-right">X (m)</span>
            <span className="text-right">Y (m)</span>
          </div>
          {/* Data rows */}
          {tags.map((tag) => (
            <div
              key={tag.tag_id}
              className="grid grid-cols-3 text-xs font-mono"
            >
              <span className="text-amber-300 font-semibold">{tag.tag_id}</span>
              <span className="text-right text-white/80">{(tag.x / 1000).toFixed(3)}</span>
              <span className="text-right text-white/80">{(tag.y / 1000).toFixed(3)}</span>
            </div>
          ))}
        </div>
      )}
    </div>
  )
}
