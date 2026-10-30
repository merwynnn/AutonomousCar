import { useEffect, useRef, useState } from 'react';
import { TrackScene } from './scene/TrackScene';
import type { HelloMessage, ScanMessage, StateMessage } from './types';
import { useTelemetry, type ConnectionStatus, type SpeedSample } from './useTelemetry';

function formatTime(seconds: number | null | undefined): string {
  if (seconds === null || seconds === undefined || !Number.isFinite(seconds)) return '—';
  const minutes = Math.floor(seconds / 60);
  const rest = seconds - minutes * 60;
  return minutes > 0 ? `${minutes}:${rest.toFixed(3).padStart(6, '0')}` : `${rest.toFixed(3)} s`;
}

function statusLabel(status: ConnectionStatus): string {
  switch (status) {
    case 'open':
      return 'Live';
    case 'connecting':
      return 'Connecting';
    case 'error':
      return 'Error';
    default:
      return 'Offline';
  }
}

function Metric({ label, value, hint }: { label: string; value: string; hint?: string }) {
  return (
    <div className="metric">
      <span className="metric-label">{label}</span>
      <span className="metric-value">{value}</span>
      {hint ? <span className="metric-hint">{hint}</span> : null}
    </div>
  );
}

function SpeedTrace({ history }: { history: SpeedSample[] }) {
  const width = 520;
  const height = 92;
  const values = history.map((sample) => sample.speed);
  const max = Math.max(4, ...values, 1);
  const points = history.length
    ? history
        .map((sample, index) => {
          const x = history.length === 1 ? 0 : (index / (history.length - 1)) * width;
          const y = height - (sample.speed / max) * (height - 12) - 6;
          return `${x.toFixed(1)},${y.toFixed(1)}`;
        })
        .join(' ')
    : '';
  return (
    <div className="chart-card">
      <div className="card-heading">
        <span>Speed</span>
        <span className="muted">last {history.length} samples</span>
      </div>
      <svg viewBox={`0 0 ${width} ${height}`} className="speed-chart" role="img" aria-label="Speed trace">
        <defs>
          <linearGradient id="speed-fill" x1="0" y1="0" x2="0" y2="1">
            <stop offset="0%" stopColor="#2f81f7" stopOpacity="0.45" />
            <stop offset="100%" stopColor="#2f81f7" stopOpacity="0" />
          </linearGradient>
        </defs>
        {[0.25, 0.5, 0.75].map((fraction) => (
          <line
            key={fraction}
            x1="0"
            y1={height * fraction}
            x2={width}
            y2={height * fraction}
            stroke="#1d2a36"
            strokeWidth="1"
          />
        ))}
        {points ? (
          <>
            <polygon
              points={`0,${height} ${points} ${width},${height}`}
              fill="url(#speed-fill)"
            />
            <polyline
              points={points}
              fill="none"
              stroke="#58a6ff"
              strokeWidth="2.2"
              strokeLinejoin="round"
              strokeLinecap="round"
            />
          </>
        ) : null}
      </svg>
      <div className="chart-footer">
        <span>0</span>
        <span>{max.toFixed(1)} m/s</span>
      </div>
    </div>
  );
}

function TrackCanvas({
  hello,
  state,
  scan,
  cameraMode,
}: {
  hello: HelloMessage | null;
  state: StateMessage | null;
  scan: ScanMessage | null;
  cameraMode: 'orbit' | 'top';
}) {
  const containerRef = useRef<HTMLDivElement>(null);
  const sceneRef = useRef<TrackScene | null>(null);

  useEffect(() => {
    if (!containerRef.current) return;
    const scene = new TrackScene(containerRef.current);
    sceneRef.current = scene;
    return () => {
      scene.dispose();
      sceneRef.current = null;
    };
  }, []);

  useEffect(() => {
    if (hello) sceneRef.current?.setTrack(hello.track);
  }, [hello]);

  useEffect(() => {
    if (state) sceneRef.current?.updateState(state);
  }, [state]);

  useEffect(() => {
    if (state && scan) sceneRef.current?.updateScan(scan, state);
  }, [scan, state]);

  useEffect(() => {
    sceneRef.current?.setCamera(cameraMode);
  }, [cameraMode]);

  return <div ref={containerRef} className="canvas-host" />;
}

export default function App() {
  const { status, hello, state, scan, history, send } = useTelemetry();
  const [speed, setSpeed] = useState(2.0);
  const [cameraMode, setCameraMode] = useState<'orbit' | 'top'>('orbit');
  const slamReady = hello?.slam_cpp ?? null;

  useEffect(() => {
    const onKey = (event: KeyboardEvent) => {
      const target = event.target as HTMLElement | null;
      if (target && ['INPUT', 'TEXTAREA', 'SELECT'].includes(target.tagName)) return;
      if (event.code === 'Space') {
        event.preventDefault();
        send({ type: state?.paused === false ? 'run.pause' : 'run.start' });
      } else if (event.key.toLowerCase() === 'r') {
        send({ type: 'run.reset' });
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [send, state?.paused]);

  const race = state?.race;
  const finitePoints = scan?.ranges.filter((range) => range !== null).length ?? 0;
  const scanning = scan?.ranges.length ?? 0;
  const running = state ? !state.paused && !state.finished : false;
  const warnings = hello?.warnings ?? [];

  return (
    <div className="app">
      <aside className="sidebar">
        <div className="brand">
          <div className="brand-mark">A</div>
          <div>
            <div className="brand-name">AWAKE 2027</div>
            <div className="brand-sub">Strategy simulator</div>
          </div>
        </div>

        <div className={`status-pill status-${status}`}>
          <span className="status-dot" />
          {statusLabel(status)}
        </div>

        <div className="sidebar-section">
          <div className="section-label">Scenario</div>
          <div className="scenario-name">{hello?.scenario.name ?? 'Loading…'}</div>
          <div className="scenario-meta">
            {hello ? `${hello.track.width_m.toFixed(2)} m lane · ${hello.race.laps} laps` : '—'}
          </div>
        </div>

        <div className="sidebar-section">
          <div className="section-label">Run control</div>
          <div className="button-row">
            <button
              type="button"
              className="primary"
              onClick={() => send({ type: running ? 'run.pause' : 'run.start' })}
            >
              {running ? 'Pause' : 'Start'}
            </button>
            <button type="button" className="ghost" onClick={() => send({ type: 'run.reset' })}>
              Reset
            </button>
          </div>
          <button
            type="button"
            className="ghost wide"
            onClick={() => send({ type: 'run.step', dt: 1 / 30 })}
          >
            Step 33 ms
          </button>
        </div>

        <div className="sidebar-section">
          <div className="section-label">
            Target speed <span className="muted">{speed.toFixed(1)} m/s</span>
          </div>
          <input
            type="range"
            min="0.5"
            max="6"
            step="0.1"
            value={speed}
            onChange={(event) => {
              const value = Number(event.target.value);
              setSpeed(value);
              send({ type: 'run.set_speed', speed: value });
            }}
          />
        </div>

        <div className="sidebar-section">
          <div className="section-label">Camera</div>
          <div className="segmented">
            <button
              type="button"
              className={cameraMode === 'orbit' ? 'active' : ''}
              onClick={() => setCameraMode('orbit')}
            >
              Orbit
            </button>
            <button
              type="button"
              className={cameraMode === 'top' ? 'active' : ''}
              onClick={() => setCameraMode('top')}
            >
              Top
            </button>
          </div>
        </div>

        <div className="sidebar-section">
          <div className="section-label">Realism</div>
          <div className={`realism-badge realism-${state?.sensors.realism ?? 'realistic'}`}>
            {state?.sensors.realism ?? 'realistic'}
          </div>
          <p className="small-note">
            Lidar noise, dropouts and latency are simulated. Ideal mode is debug-only.
          </p>
        </div>

        <div className="sidebar-footer">
          <div className="small-note">
            M0 bootstrap · centreline demo
            <br />
            Docs: AGENTS.md · STATUS.md
          </div>
        </div>
      </aside>

      <main className="main">
        <header className="topbar">
          <div>
            <h1>Test track</h1>
            <p>PDF-like peanut · 16 cm pipe walls · indoor 15 × 8 m</p>
          </div>
          <div className="topbar-stats">
            <div>
              <span className="topbar-label">Lap</span>
              <strong>{race ? `${race.lap} / ${race.laps}` : '—'}</strong>
            </div>
            <div>
              <span className="topbar-label">Lap time</span>
              <strong>{formatTime(race?.lap_time)}</strong>
            </div>
            <div>
              <span className="topbar-label">Best</span>
              <strong>{formatTime(race?.best_lap)}</strong>
            </div>
            <div>
              <span className="topbar-label">Total</span>
              <strong>{formatTime(race?.total_time)}</strong>
            </div>
          </div>
        </header>

        <section className="viewport">
          <TrackCanvas hello={hello} state={state} scan={scan} cameraMode={cameraMode} />
          <div className="viewport-badge">
            <span className="live-dot" /> {scanning ? 'Lidar live' : 'Waiting for scan'}
          </div>
          <div className="viewport-note">
            Green → blue points are lidar returns. Orange is the travelled path.
          </div>
          {warnings.length > 0 ? (
            <div className="warning-banner">{warnings.join(' · ')}</div>
          ) : null}
        </section>

        <section className="bottom">
          <div className="metrics-row">
            <Metric label="Speed" value={`${(state?.car.speed ?? 0).toFixed(2)} m/s`} hint="body speed" />
            <Metric label="Lidar returns" value={`${finitePoints} / ${scanning || '—'}`} hint="valid beams" />
            <Metric label="Penalties" value={`${race?.penalties ?? 0}`} hint="competition-style" />
            <Metric
              label="SLAM core"
              value={slamReady === null ? '—' : slamReady ? 'C++ ready' : 'Not built'}
              hint="pipeline in M3"
            />
          </div>
          <SpeedTrace history={history} />
        </section>
      </main>

      <aside className="rightbar">
        <div className="panel">
          <div className="card-heading">
            <span>Race</span>
            <span className="muted">{state?.finished ? 'Finished' : state?.paused ? 'Paused' : 'Running'}</span>
          </div>
          <div className="race-grid">
            <Metric label="Lap" value={race ? `${race.lap} / ${race.laps}` : '—'} />
            <Metric label="Last lap" value={formatTime(race?.last_lap)} />
            <Metric label="Best lap" value={formatTime(race?.best_lap)} />
            <Metric label="Total" value={formatTime(race?.total_time)} />
          </div>
        </div>

        <div className="panel">
          <div className="card-heading">
            <span>Sensors</span>
            <span className={`mini-badge realism-${state?.sensors.realism ?? 'realistic'}`}>
              {state?.sensors.realism ?? 'realistic'}
            </span>
          </div>
          <div className="sensor-row">
            <span>Lidar</span>
            <strong>{state?.sensors.lidar_beams ?? hello?.sensors.lidar.beams ?? '—'} beams</strong>
          </div>
          <div className="sensor-row">
            <span>Rate</span>
            <strong>{state?.sensors.lidar_rate_hz ?? hello?.sensors.lidar.rate_hz ?? '—'} Hz</strong>
          </div>
          <div className="sensor-row">
            <span>Returns</span>
            <strong>{finitePoints} valid</strong>
          </div>
          <div className="sensor-row">
            <span>Ultrasonic</span>
            <strong>{hello?.sensors.ultrasonic.length ?? 3} sensors</strong>
          </div>
          <div className="sensor-row">
            <span>IMU</span>
            <strong>{hello?.sensors.imu_rate_hz ?? '—'} Hz</strong>
          </div>
          <div className="sensor-row">
            <span>Mount height</span>
            <strong>
              {hello?.sensors.lidar.mount.position_m?.[2]?.toFixed(2) ?? '—'} m
            </strong>
          </div>
        </div>

        <div className="panel">
          <div className="card-heading">
            <span>SLAM</span>
            <span className="muted">M3</span>
          </div>
          <p className="small-note">
            From-scratch 2D lidar SLAM: correlative matching, occupancy grid, pose graph and
            particle-filter localization. Live map and ATE diagnostics will appear here.
          </p>
          <div className="disabled-bar">
            <span>{slamReady ? 'C++ core compiled' : 'C++ core not built'}</span>
          </div>
        </div>

        <div className="panel">
          <div className="card-heading">
            <span>Milestone</span>
            <span className="mini-badge">M0</span>
          </div>
          <p className="small-note">
            Bootstrap in progress. Next: PyBullet car, realistic sensor bus, manual driving and
            lap timing. The dashboard shell is live now.
          </p>
          <div className="progress">
            <div className="progress-bar" style={{ width: '18%' }} />
          </div>
          <div className="small-note">Docs: docs/02-architecture.md · docs/04-sensors.md</div>
        </div>
      </aside>
    </div>
  );
}
