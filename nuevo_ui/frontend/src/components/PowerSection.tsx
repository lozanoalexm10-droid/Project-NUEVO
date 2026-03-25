import { AlertTriangle, BatteryWarning } from 'lucide-react';
import { useRobotStore } from '../store/robotStore';

const VBAT_MIN_PRESENT = 2.0; // matches firmware VBAT_MIN_PRESENT_V

// Keyed by BATTERY_TYPE constant from firmware config.h
// 0 = not yet reported; shown as "—"
const BATTERY_INFO: Record<number, { label: string; min: number; max: number }> = {
  1:  { label: '8-Cell NiMH',  min: 8.0,  max: 12.5 },
  2:  { label: '10-Cell NiMH', min: 10.0, max: 15.5 },
  3:  { label: '2S LiPo',      min: 6.6,  max: 8.6  },
  4:  { label: '3S LiPo',      min: 9.9,  max: 12.9 },
  5:  { label: '4S LiPo',      min: 13.2, max: 17.2 },
  6:  { label: '5S LiPo',      min: 16.5, max: 21.5 },
  7:  { label: '6S LiPo',      min: 19.8, max: 25.8 },
  99: { label: 'Custom',        min: 6.0,  max: 24.0 },
};
const DEFAULT_RANGE = { min: 8.0, max: 15.5 };

export function PowerSection() {
  const voltage = useRobotStore((s) => s.voltage);

  const batteryVoltage = voltage ? voltage.batteryMv / 1000 : 0;
  const servoVoltage   = voltage ? voltage.servoRailMv / 1000 : 0;

  const batteryInfo  = BATTERY_INFO[voltage?.batteryType ?? 0];
  const batteryLabel = batteryInfo?.label ?? '—';
  const batteryRange = batteryInfo ?? DEFAULT_RANGE;
  const servoRange = { min: 5, max: 10 };

  const getBatteryWarning = () => {
    if (!voltage) return { show: false, type: '' };
    if (batteryVoltage < VBAT_MIN_PRESENT) return { show: true, type: 'none' };
    if (batteryVoltage < batteryRange.min) return { show: true, type: 'low' };
    if (batteryVoltage > batteryRange.max) return { show: true, type: 'high' };
    return { show: false, type: '' };
  };

  const getServoWarning = () => {
    if (!voltage) return { show: false, type: '' };
    if (servoVoltage < 5.5) return { show: true, type: 'low' };
    if (servoVoltage > 9) return { show: true, type: 'high' };
    return { show: false, type: '' };
  };

  const batteryWarning = getBatteryWarning();
  const servoWarning = getServoWarning();

  const batteryPct = Math.max(0, Math.min(100,
    ((batteryVoltage - batteryRange.min) / (batteryRange.max - batteryRange.min)) * 100
  ));
  const servoPct = Math.max(0, Math.min(100,
    ((servoVoltage - servoRange.min) / (servoRange.max - servoRange.min)) * 100
  ));

  return (
    <div className="relative rounded-2xl p-4 backdrop-blur-2xl bg-white/10 border border-white/20 shadow-xl h-full">
      <div className="absolute inset-x-0 top-0 h-px bg-gradient-to-r from-transparent via-white/50 to-transparent"></div>
      <div className="absolute inset-0 rounded-2xl bg-gradient-to-br from-white/5 to-transparent opacity-50"></div>

      <div className="relative">
        <div className="flex items-center justify-between mb-3">
          <h3 className="text-lg font-semibold text-white">Power</h3>
          <span className="px-3 py-1 rounded-xl bg-white/5 border border-white/15 text-sm text-white/60 font-mono select-none">
            {batteryLabel}
          </span>
        </div>

        {/* Voltage Section */}
        <div className="flex gap-4">

          {/* Battery Voltage */}
          <div className="flex-[5]">
            <div className="flex items-center justify-between mb-2">
              <span className="text-sm text-white/70">Battery</span>
              <div className="flex items-center gap-2">
                {batteryWarning.type === 'none' ? (
                  <>
                    <span className="text-xs font-semibold text-rose-400">No battery</span>
                    <BatteryWarning className="size-4 text-rose-400" />
                  </>
                ) : (
                  <>
                    <span className="text-sm font-mono text-white font-semibold">
                      {voltage ? batteryVoltage.toFixed(1) : '--'}V
                    </span>
                    {batteryWarning.show && (
                      <AlertTriangle className="size-4 text-amber-400" />
                    )}
                  </>
                )}
              </div>
            </div>

            <div className="h-3 rounded-full backdrop-blur-xl bg-white/10 border border-white/20 overflow-hidden">
              <div
                className={`h-full transition-all duration-300 ${
                  batteryWarning.type === 'none' ? 'bg-rose-500/60' :
                  batteryWarning.show ? 'bg-amber-400' : 'bg-emerald-400'
                }`}
                style={{
                  width: batteryWarning.type === 'none' ? '100%' : `${batteryPct}%`,
                  boxShadow: batteryWarning.type === 'none'
                    ? '0 0 12px rgba(244, 63, 94, 0.4)'
                    : batteryWarning.show
                      ? '0 0 12px rgba(251, 191, 36, 0.6)'
                      : '0 0 12px rgba(52, 211, 153, 0.6)',
                }}
              />
            </div>

            <div className="flex justify-between mt-1">
              <span className="text-xs text-white/50">{batteryRange.min}V</span>
              <span className="text-xs text-white/50">{batteryRange.max}V</span>
            </div>
          </div>

          {/* Servo Voltage */}
          <div className="flex-[2]">
            <div className="flex items-center justify-between mb-2">
              <span className="text-sm text-white/70">Servo</span>
              <div className="flex items-center gap-2">
                <span className="text-sm font-mono text-white font-semibold">
                  {voltage ? servoVoltage.toFixed(1) : '--'}V
                </span>
                {servoWarning.show && (
                  <AlertTriangle className="size-4 text-amber-400" />
                )}
              </div>
            </div>

            <div className="h-3 rounded-full backdrop-blur-xl bg-white/10 border border-white/20 overflow-hidden">
              <div
                className={`h-full transition-all duration-300 ${
                  servoWarning.show ? 'bg-amber-400' : 'bg-cyan-400'
                }`}
                style={{
                  width: `${servoPct}%`,
                  boxShadow: servoWarning.show
                    ? '0 0 12px rgba(251, 191, 36, 0.6)'
                    : '0 0 12px rgba(34, 211, 238, 0.6)',
                }}
              />
            </div>

            <div className="flex justify-between mt-1">
              <span className="text-xs text-white/50">{servoRange.min}V</span>
              <span className="text-xs text-white/50">{servoRange.max}V</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
}
