import { useEffect, useMemo, useState } from 'react';
import { CUSTOM_DEBUG_RENDERERS } from './DebugRenderers';
import type {
  DebugBand,
  DebugColumn,
  DebugFrame,
  DebugLayers,
  DebugMarker,
  DebugPlot,
  DebugScalar,
  DebugSeries,
  DebugTable,
  StateMessage,
} from './types';

const WIDTH = 322;
const HEIGHT = 112;
const PAD_TOP = 4;
const PAD_BOTTOM = 14;
const PAD_LEFT = 4;
const PAD_RIGHT = 4;

const ROLE_COLORS: Record<string, string> = {
  primary: '#d7a84b',
  secondary: '#6ea8d0',
  muted: '#6b7480',
  accent: '#ff6fae',
  warning: '#c96b5a',
  candidate: '#8ec7ef',
};

function colorFor(role: string): string {
  return ROLE_COLORS[role] ?? ROLE_COLORS.secondary;
}

function formatValue(
  value: DebugScalar,
  precision: number,
  unit: string,
): string {
  if (value === null || value === undefined) return '—';
  if (typeof value === 'boolean') return value ? 'yes' : 'no';
  if (typeof value !== 'number') return `${value}${unit ? ` ${unit}` : ''}`;
  const text = value.toFixed(Math.max(0, precision));
  if (!unit) return text;
  if (unit === '%') return `${text}%`;
  if (unit === 'deg') return `${text}°`;
  return `${text} ${unit}`;
}

function domain(values: number[], fallbackMin: number, fallbackMax: number): [number, number] {
  const finite = values.filter((value) => Number.isFinite(value));
  if (!finite.length) return [fallbackMin, fallbackMax];
  let low = Math.min(...finite);
  let high = Math.max(...finite);
  if (Math.abs(high - low) < 1e-6) {
    low -= 1;
    high += 1;
  }
  const pad = (high - low) * 0.04;
  return [low - pad, high + pad];
}

function seriesPath(
  series: DebugSeries,
  xFor: (value: number) => number,
  yFor: (value: number) => number,
): string {
  return series.points
    .map(([x, y], index) => {
      const prefix = index === 0 ? 'M' : 'L';
      return `${prefix}${xFor(x).toFixed(2)},${yFor(y).toFixed(2)}`;
    })
    .join(' ');
}

function BandRect({
  band,
  xFor,
}: {
  band: DebugBand;
  xFor: (value: number) => number;
}) {
  const left = Math.min(xFor(band.low), xFor(band.high));
  const right = Math.max(xFor(band.low), xFor(band.high));
  return (
    <rect
      x={left}
      y={PAD_TOP}
      width={Math.max(1, right - left)}
      height={HEIGHT - PAD_TOP - PAD_BOTTOM}
      fill={colorFor(band.role)}
      opacity="0.08"
    />
  );
}

function MarkerLine({
  marker,
  xFor,
  yFor,
}: {
  marker: DebugMarker;
  xFor: (value: number) => number;
  yFor: (value: number) => number;
}) {
  const dashed = marker.style === 'dashed';
  if (marker.axis === 'x') {
    const x = xFor(marker.value);
    return (
      <line
        x1={x}
        y1={PAD_TOP}
        x2={x}
        y2={HEIGHT - PAD_BOTTOM}
        stroke={colorFor('primary')}
        strokeWidth="1.2"
        strokeDasharray={dashed ? '3 3' : undefined}
        opacity="0.75"
      />
    );
  }
  const y = yFor(marker.value);
  return (
    <line
      x1={PAD_LEFT}
      y1={y}
      x2={WIDTH - PAD_RIGHT}
      y2={y}
      stroke={colorFor('secondary')}
      strokeWidth="1"
      strokeDasharray={dashed ? '3 3' : undefined}
      opacity="0.55"
    />
  );
}

function SeriesChart({ plot }: { plot: DebugPlot }) {
  const geometry = useMemo(() => {
    const xs: number[] = [];
    const ys: number[] = [];
    for (const series of plot.series) {
      for (const [x, y] of series.points) {
        xs.push(x);
        ys.push(y);
      }
    }
    const [xMin, xMax] = plot.x_range ?? domain(xs, -1, 1);
    const [yMin, yMax] = plot.y_range ?? domain(ys, 0, 1);
    const xFor = (value: number) =>
      PAD_LEFT + ((value - xMin) / Math.max(1e-9, xMax - xMin)) * (WIDTH - PAD_LEFT - PAD_RIGHT);
    const yFor = (value: number) =>
      HEIGHT - PAD_BOTTOM - ((value - yMin) / Math.max(1e-9, yMax - yMin)) * (HEIGHT - PAD_TOP - PAD_BOTTOM);
    return { xMin, xMax, yMin, yMax, xFor, yFor };
  }, [plot]);

  if (!plot.series.length) return null;

  return (
    <div className="debug-chart-block">
      <svg
        viewBox={`0 0 ${WIDTH} ${HEIGHT}`}
        className="debug-chart"
        role="img"
        aria-label={plot.title}
      >
        {plot.bands.map((band, index) => (
          <BandRect key={`${band.label}-${index}`} band={band} xFor={geometry.xFor} />
        ))}
        {[0.25, 0.5, 0.75].map((fraction) => {
          const y = PAD_TOP + (HEIGHT - PAD_TOP - PAD_BOTTOM) * fraction;
          return (
            <line
              key={fraction}
              x1={PAD_LEFT}
              y1={y}
              x2={WIDTH - PAD_RIGHT}
              y2={y}
              stroke="#2a2f36"
              strokeWidth="1"
            />
          );
        })}
        {plot.series.map((series) =>
          series.kind === 'scatter' ? (
            series.points.map(([x, y], index) => (
              <circle
                key={`${series.id}-${index}`}
                cx={geometry.xFor(x)}
                cy={geometry.yFor(y)}
                r="1.7"
                fill={colorFor(series.role)}
              />
            ))
          ) : (
            <path
              key={series.id}
              d={seriesPath(series, geometry.xFor, geometry.yFor)}
              fill="none"
              stroke={colorFor(series.role)}
              strokeWidth={series.role === 'primary' || series.role === 'accent' ? '1.8' : '1.2'}
              opacity={series.role === 'muted' ? '0.65' : '1'}
            />
          ),
        )}
        {plot.markers.map((marker, index) => (
          <MarkerLine
            key={`${marker.axis}-${index}`}
            marker={marker}
            xFor={geometry.xFor}
            yFor={geometry.yFor}
          />
        ))}
        <text x={PAD_LEFT} y="10" className="debug-axis-label">
          {geometry.yMax.toFixed(1)} {plot.y_unit}
        </text>
        <text x={PAD_LEFT} y={HEIGHT - 3} className="debug-axis-label">
          {geometry.xMin.toFixed(0)} {plot.x_unit}
        </text>
        <text
          x={WIDTH - PAD_RIGHT}
          y={HEIGHT - 3}
          textAnchor="end"
          className="debug-axis-label"
        >
          {geometry.xMax.toFixed(0)} {plot.x_unit}
        </text>
      </svg>
      <div className="debug-legend">
        {plot.series.map((series) => (
          <span key={series.id}>
            <i style={{ background: colorFor(series.role) }} />
            {series.label}
          </span>
        ))}
      </div>
    </div>
  );
}

function DebugDataTable({ table }: { table: DebugTable }) {
  return (
    <div className="debug-table-wrap">
      <table className="debug-table">
        <thead>
          <tr>
            {table.columns.map((column: DebugColumn) => (
              <th key={column.key}>{column.label}</th>
            ))}
          </tr>
        </thead>
        <tbody>
          {table.rows.map((row, rowIndex) => (
            <tr
              key={`${table.id}-${rowIndex}`}
              className={rowIndex === table.selected_row ? 'selected' : ''}
            >
              {row.map((value, columnIndex) => {
                const column = table.columns[columnIndex];
                if (!column) return null;
                return (
                  <td key={column.key}>
                    {formatValue(value, column.precision, column.unit)}
                  </td>
                );
              })}
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  );
}

type DebugTab =
  | { key: string; label: string; kind: 'plot'; plot: DebugPlot }
  | { key: string; label: string; kind: 'table'; table: DebugTable };

function frameTabs(frame: DebugFrame): DebugTab[] {
  return [
    ...frame.plots.map(
      (plot): DebugTab => ({ key: `plot-${plot.id}`, label: plot.title, kind: 'plot', plot }),
    ),
    ...frame.tables.map(
      (table): DebugTab => ({
        key: `table-${table.id}`,
        label: table.title,
        kind: 'table',
        table,
      }),
    ),
  ];
}

export function DebugDock({
  state,
  layers,
}: {
  state: StateMessage | null;
  layers: DebugLayers;
}) {
  const frame = state?.debug ?? null;
  const tabs = useMemo(() => (frame ? frameTabs(frame) : []), [frame]);
  const [tabIndex, setTabIndex] = useState(0);

  useEffect(() => {
    setTabIndex(0);
  }, [frame?.controller, tabs.length]);

  if (!frame || !layers.debug) return null;

  const CustomRenderer = CUSTOM_DEBUG_RENDERERS[frame.controller];
  if (CustomRenderer) {
    return (
      <div className="debug-dock">
        <CustomRenderer frame={frame} />
      </div>
    );
  }

  const activeIndex = tabs.length ? Math.min(tabIndex, tabs.length - 1) : 0;
  const active = tabs[activeIndex];
  const detail = typeof frame.meta.detail === 'string' ? frame.meta.detail : '';
  const statusTone = !frame.valid
    ? 'idle'
    : frame.status === 'TRACKING'
      ? 'live'
      : 'warn';

  return (
    <div className="debug-dock" title={detail || undefined}>
      <div className="debug-head">
        <span>{frame.title || frame.controller}</span>
        <span className={`debug-status debug-${statusTone}`}>
          {frame.status || (frame.valid ? 'LIVE' : 'IDLE')}
        </span>
      </div>
      {tabs.length > 0 ? (
        <>
          <div className="debug-tabs" role="tablist" aria-label="Controller debug views">
            {tabs.map((tab, index) => (
              <button
                key={tab.key}
                type="button"
                role="tab"
                aria-selected={index === activeIndex}
                className={index === activeIndex ? 'active' : ''}
                onClick={() => setTabIndex(index)}
              >
                {tab.label}
              </button>
            ))}
          </div>
          <div className="debug-body">
            {active?.kind === 'plot' ? <SeriesChart plot={active.plot} /> : null}
            {active?.kind === 'table' ? <DebugDataTable table={active.table} /> : null}
          </div>
        </>
      ) : null}
      {frame.readouts.length > 0 ? (
        <div className="debug-readouts">
          {frame.readouts.map((readout) => (
            <div key={readout.key}>
              <span>{readout.label}</span>
              <strong>
                {formatValue(readout.value, readout.precision, readout.unit)}
              </strong>
            </div>
          ))}
        </div>
      ) : null}
    </div>
  );
}
