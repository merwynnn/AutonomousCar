import type { DebugLayers, StateMessage } from './types';

function deg(value: number): number {
  return (value * 180) / Math.PI;
}

function profilePath(state: StateMessage, width: number, height: number): string {
  const profile = state.gap.profile;
  if (!profile.length) return '';
  const limit = Math.max(
    1,
    ...profile.map(([, clearance]) => clearance),
    state.gap.clearance_m,
  );
  return profile
    .map(([angle, clearance], index) => {
      const x = ((deg(angle) + deg(state.gap.front_sector_rad)) /
        Math.max(1e-6, deg(state.gap.front_sector_rad) * 2)) * width;
      const y = height - (clearance / limit) * (height - 8) - 4;
      return `${index === 0 ? 'M' : 'L'}${x.toFixed(1)},${y.toFixed(1)}`;
    })
    .join(' ');
}

export function GapDebugPanel({
  state,
  layers,
}: {
  state: StateMessage | null;
  layers: DebugLayers;
}) {
  if (!state || state.auto_controller !== 'gap_follow' || !layers.gap) return null;
  const gap = state.gap;
  const width = 260;
  const height = 92;
  const path = profilePath(state, width, height);
  const limit = Math.max(1, ...gap.profile.map(([, clearance]) => clearance), gap.clearance_m);
  const sector = Math.max(1e-6, deg(gap.front_sector_rad) * 2);
  const xFor = (angleRad: number) =>
    ((deg(angleRad) + deg(gap.front_sector_rad)) / sector) * width;
  const left = Math.max(0, Math.min(width, xFor(gap.gap_left_rad)));
  const right = Math.max(0, Math.min(width, xFor(gap.gap_right_rad)));
  const chosen = Math.max(0, Math.min(width, xFor(gap.angle_rad)));
  const yFor = (clearance: number) => height - (clearance / limit) * (height - 8) - 4;

  return (
    <div className="gap-debug">
      <div className="gap-debug-title">
        <span>Gap solver</span>
        <span className={gap.valid ? 'gap-live' : 'gap-idle'}>
          {gap.valid ? 'TRACKING' : 'NO GAP'}
        </span>
      </div>
      <svg viewBox={`0 0 ${width} ${height}`} className="gap-chart" role="img" aria-label="Gap clearance profile">
        {[0.25, 0.5, 0.75].map((fraction) => (
          <line
            key={fraction}
            x1="0"
            y1={height * fraction}
            x2={width}
            y2={height * fraction}
            stroke="#2a2f36"
            strokeWidth="1"
          />
        ))}
        <rect
          x={Math.min(left, right)}
          y="0"
          width={Math.max(1, Math.abs(right - left))}
          height={height}
          fill="rgba(209, 163, 75, 0.08)"
        />
        {path ? (
          <>
            <path
              d={`${path} L${width},${height} L0,${height} Z`}
              fill="rgba(81, 143, 189, 0.12)"
              stroke="none"
            />
            <path d={path} fill="none" stroke="#6ea8d0" strokeWidth="1.6" />
          </>
        ) : null}
        <line
          x1="0"
          y1={yFor(gap.clearance_m)}
          x2={width}
          y2={yFor(gap.clearance_m)}
          stroke="rgba(209, 163, 75, 0.32)"
          strokeDasharray="3 3"
        />
        <line x1={chosen} y1="0" x2={chosen} y2={height} stroke="#d7a84b" strokeWidth="1.5" />
        <circle cx={chosen} cy={yFor(gap.clearance_m)} r="2.6" fill="#d7a84b" />
        <text x="4" y="10" className="gap-axis-label">clearance</text>
        <text x={width - 4} y={height - 3} textAnchor="end" className="gap-axis-label">
          ±{deg(gap.front_sector_rad).toFixed(0)}°
        </text>
      </svg>
      <div className="gap-readout-grid">
        <span>Bearing</span>
        <strong>{gap.valid ? `${deg(gap.angle_rad).toFixed(1)}°` : '—'}</strong>
        <span>Steering</span>
        <strong>{`${deg(gap.steering_rad).toFixed(1)}°`}</strong>
        <span>Clearance</span>
        <strong>{gap.valid ? `${gap.clearance_m.toFixed(2)} m` : '—'}</strong>
        <span>Opening</span>
        <strong>{gap.valid ? `${deg(gap.width_rad).toFixed(1)}°` : '—'}</strong>
        <span>Throttle</span>
        <strong>{(gap.throttle * 100).toFixed(0)} %</strong>
      </div>
    </div>
  );
}
