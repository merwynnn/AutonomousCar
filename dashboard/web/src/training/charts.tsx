export interface Series {
  label: string;
  color: string;
  points: Array<[number, number | null]>;
  dashed?: boolean;
}

function niceTicks(min: number, max: number, count = 4): number[] {
  if (!Number.isFinite(min) || !Number.isFinite(max) || min === max) {
    return [min];
  }
  const span = max - min;
  const step0 = span / count;
  const mag = 10 ** Math.floor(Math.log10(step0));
  const norm = step0 / mag;
  const step = (norm >= 5 ? 10 : norm >= 2 ? 5 : norm >= 1 ? 2 : 1) * mag;
  const start = Math.ceil(min / step) * step;
  const ticks: number[] = [];
  for (let value = start; value <= max + step * 1e-9; value += step) {
    ticks.push(value);
  }
  return ticks;
}

export function LineChart({
  title,
  series,
  yLabel,
  width = 360,
  height = 150,
}: {
  title: string;
  series: Series[];
  yLabel: string;
  width?: number;
  height?: number;
}) {
  const padL = 44;
  const padR = 10;
  const padT = 8;
  const padB = 20;
  const iw = width - padL - padR;
  const ih = height - padT - padB;

  const xs = series.flatMap((s) => s.points.map((p) => p[0]));
  const ys = series.flatMap((s) =>
    s.points.map((p) => p[1]).filter((v): v is number => typeof v === 'number' && Number.isFinite(v)),
  );
  const xMin = xs.length ? Math.min(...xs) : 0;
  const xMax = xs.length ? Math.max(...xs) : 1;
  const yMin = ys.length ? Math.min(...ys) : 0;
  const yMax = ys.length ? Math.max(...ys) : 1;
  const xSpan = xMax - xMin || 1;
  const yPad = (yMax - yMin) * 0.08 || 0.5;
  const yLo = yMin - yPad;
  const yHi = yMax + yPad;
  const ySpan = yHi - yLo || 1;

  const sx = (x: number) => padL + ((x - xMin) / xSpan) * iw;
  const sy = (y: number) => padT + ih - ((y - yLo) / ySpan) * ih;

  const ticks = niceTicks(yLo, yHi, 4);
  const formatTick = (v: number) =>
    Math.abs(v) >= 100 ? v.toFixed(0) : Math.abs(v) >= 10 ? v.toFixed(1) : v.toFixed(2);

  return (
    <figure className="chart-card">
      <figcaption>
        <span>{title}</span>
        <span className="chart-ylabel">{yLabel}</span>
      </figcaption>
      <svg viewBox={`0 0 ${width} ${height}`} role="img" aria-label={title}>
        {ticks.map((tick) => (
          <g key={tick}>
            <line
              x1={padL}
              x2={width - padR}
              y1={sy(tick)}
              y2={sy(tick)}
              className="chart-grid"
            />
            <text x={padL - 6} y={sy(tick) + 3} className="chart-tick" textAnchor="end">
              {formatTick(tick)}
            </text>
          </g>
        ))}
        <text x={padL} y={height - 6} className="chart-tick" textAnchor="middle">
          {xMin}
        </text>
        <text x={width - padR} y={height - 6} className="chart-tick" textAnchor="middle">
          {xMax}
        </text>
        {series.map((s) => {
          const path = s.points
            .filter((p): p is [number, number] => typeof p[1] === 'number' && Number.isFinite(p[1]))
            .map((p, i) => `${i === 0 ? 'M' : 'L'}${sx(p[0]).toFixed(1)},${sy(p[1]).toFixed(1)}`)
            .join(' ');
          return path ? (
            <path
              key={s.label}
              d={path}
              fill="none"
              stroke={s.color}
              strokeWidth={1.6}
              strokeDasharray={s.dashed ? '4 3' : undefined}
            />
          ) : null;
        })}
      </svg>
      {series.length > 1 ? (
        <div className="chart-legend">
          {series.map((s) => (
            <span key={s.label}>
              <i style={{ background: s.color, ...(s.dashed ? { height: 0, borderTop: `2px dashed ${s.color}` } : {}) }} />
              {s.label}
            </span>
          ))}
        </div>
      ) : null}
    </figure>
  );
}

export function StatCard({ label, value, unit, hint }: { label: string; value: string; unit?: string; hint?: string }) {
  return (
    <div className="stat-card" title={hint}>
      <span className="stat-label">{label}</span>
      <span className="stat-value">
        {value}
        {unit ? <em>{unit}</em> : null}
      </span>
    </div>
  );
}
