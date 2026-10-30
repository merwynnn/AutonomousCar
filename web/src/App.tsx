import { useEffect, useRef, useState } from 'react';
import { TrackScene } from './scene/TrackScene';
import type {
  DebugLayers,
  HelloMessage,
  ScanMessage,
  SlamMessage,
  StateMessage,
} from './types';
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
  slam,
  cameraMode,
  layers,
}: {
  hello: HelloMessage | null;
  state: StateMessage | null;
  scan: ScanMessage | null;
  slam: SlamMessage | null;
  cameraMode: 'orbit' | 'top';
  layers: DebugLayers;
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
    if (slam) sceneRef.current?.updateSlam(slam);
  }, [slam]);

  useEffect(() => {
    sceneRef.current?.setCamera(cameraMode);
  }, [cameraMode]);

  useEffect(() => {
    sceneRef.current?.setLayerVisibility(layers);
  }, [layers]);

  return <div ref={containerRef} className="canvas-host" />;
}

export default function App() {
  const { status, hello, state, scan, slam, history, send } = useTelemetry();
  const [speed, setSpeed] = useState(2.4);
  const [cameraMode, setCameraMode] = useState<'orbit' | 'top'>('orbit');
  const [controlMode, setControlMode] = useState<'auto' | 'manual'>('auto');
  const [autoController, setAutoController] = useState<'wall_follow' | 'pure_pursuit'>(
    'wall_follow',
  );
  const [mclEnabled, setMclEnabled] = useState(false);
  const [loopClosureEnabled, setLoopClosureEnabled] = useState(false);
  const [layers, setLayers] = useState<DebugLayers>({
    map: true,
    slam: true,
    mcl: true,
    particles: true,
    odom: false,
    gtTrail: true,
  });
  const keysRef = useRef({ w: false, a: false, s: false, d: false });
  const slamReady = hello?.slam_cpp ?? null;

  useEffect(() => {
    send({
      type: 'slam.set_options',
      mcl_enabled: mclEnabled,
      loop_closure_enabled: loopClosureEnabled,
    });
  }, [mclEnabled, loopClosureEnabled, send]);

  useEffect(() => {
    send({ type: 'control.mode', mode: controlMode });
  }, [controlMode, send]);

  useEffect(() => {
    send({ type: 'control.controller', controller: autoController });
  }, [autoController, send]);

  useEffect(() => {
    const onDown = (event: KeyboardEvent) => {
      const target = event.target as HTMLElement | null;
      if (target && ['INPUT', 'TEXTAREA', 'SELECT'].includes(target.tagName)) return;
      const key = event.key.toLowerCase();
      if (key in keysRef.current) {
        event.preventDefault();
        keysRef.current[key as keyof typeof keysRef.current] = true;
        if (state?.paused) send({ type: 'run.start' });
      }
    };
    const onUp = (event: KeyboardEvent) => {
      const key = event.key.toLowerCase();
      if (key in keysRef.current) {
        event.preventDefault();
        keysRef.current[key as keyof typeof keysRef.current] = false;
      }
    };
    window.addEventListener('keydown', onDown);
    window.addEventListener('keyup', onUp);
    return () => {
      window.removeEventListener('keydown', onDown);
      window.removeEventListener('keyup', onUp);
    };
  }, [send, state?.paused]);

  useEffect(() => {
    if (controlMode !== 'manual') return;
    const timer = window.setInterval(() => {
      const keys = keysRef.current;
      const throttle = (keys.w ? 1 : 0) - (keys.s ? 0.7 : 0);
      const steering = (keys.a ? 0.45 : 0) - (keys.d ? 0.45 : 0);
      send({ type: 'drive.command', throttle, steering });
    }, 50);
    return () => window.clearInterval(timer);
  }, [controlMode, send]);

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
          <div className="section-label">Driving mode</div>
          <div className="segmented">
            <button
              type="button"
              className={controlMode === 'auto' ? 'active' : ''}
              onClick={() => setControlMode('auto')}
            >
              Auto
            </button>
            <button
              type="button"
              className={controlMode === 'manual' ? 'active' : ''}
              onClick={() => setControlMode('manual')}
            >
              Manual
            </button>
          </div>
          {controlMode === 'auto' ? (
            <div className="segmented" style={{ marginTop: 6 }}>
              <button
                type="button"
                className={autoController === 'wall_follow' ? 'active' : ''}
                onClick={() => setAutoController('wall_follow')}
              >
                Wall-follow
              </button>
              <button
                type="button"
                className={autoController === 'pure_pursuit' ? 'active' : ''}
                onClick={() => setAutoController('pure_pursuit')}
              >
                Pursuit
              </button>
            </div>
          ) : null}
          <p className="small-note">
            {controlMode === 'auto'
              ? autoController === 'wall_follow'
                ? 'Lidar wall-follow, no ground truth.'
                : 'Pure pursuit + racing line. Uses GT pose (upper bound).'
              : 'W/S throttle · A/D steer. Keep the page focused.'}
          </p>
        </div>

        <div className="sidebar-section">
          <div className="section-label">Localization</div>
          <div className="segmented">
            <button
              type="button"
              className={mclEnabled ? '' : 'active'}
              onClick={() => setMclEnabled(false)}
            >
              MCL off
            </button>
            <button
              type="button"
              className={mclEnabled ? 'active' : ''}
              onClick={() => setMclEnabled(true)}
            >
              MCL on
            </button>
          </div>
          <div className="segmented" style={{ marginTop: 6 }}>
            <button
              type="button"
              className={loopClosureEnabled ? '' : 'active'}
              onClick={() => setLoopClosureEnabled(false)}
            >
              Loop off
            </button>
            <button
              type="button"
              className={loopClosureEnabled ? 'active' : ''}
              onClick={() => setLoopClosureEnabled(true)}
            >
              Loop on
            </button>
          </div>
          <button
            type="button"
            className="ghost wide"
            style={{ marginTop: 8 }}
            onClick={() => send({ type: 'slam.reset_mcl' })}
          >
            Reset MCL
          </button>
          <p className="small-note">
            {loopClosureEnabled
              ? 'Loop closure is experimental: the pipe walls are repetitive and false matches can corrupt the map.'
              : 'Loop closure is off. Enable it only for experiments.'}
          </p>
        </div>

        <div className="sidebar-section">
          <div className="section-label">Debug layers</div>
          <div className="debug-grid">
            {(
              [
                ['map', 'Map'],
                ['slam', 'SLAM'],
                ['mcl', 'MCL'],
                ['particles', 'Particles'],
                ['odom', 'Odometry'],
                ['gtTrail', 'GT trail'],
              ] as Array<[keyof DebugLayers, string]>
            ).map(([key, label]) => (
              <label className="debug-toggle" key={key}>
                <input
                  type="checkbox"
                  checked={layers[key]}
                  onChange={(event) =>
                    setLayers((current) => ({ ...current, [key]: event.target.checked }))
                  }
                />
                {label}
              </label>
            ))}
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
            Auto target speed <span className="muted">{speed.toFixed(1)} m/s</span>
          </div>
          <input
            type="range"
            min="0.5"
            max="6"
            step="0.1"
            value={speed}
            disabled={controlMode === 'manual'}
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
            M3 · PyBullet + C++ SLAM front-end
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
          <TrackCanvas
            hello={hello}
            state={state}
            scan={scan}
            slam={slam}
            cameraMode={cameraMode}
            layers={layers}
          />
          <div className="viewport-badge">
            <span className="live-dot" /> {controlMode.toUpperCase()} ·{' '}
            {controlMode === 'auto'
              ? autoController === 'pure_pursuit'
                ? 'PURSUIT'
                : 'WALL'
              : 'MANUAL'}{' '}
            · {scanning ? 'Lidar live' : 'Waiting for scan'}
          </div>
          <div className="viewport-note">
            Green → blue points are lidar returns. Orange is the travelled path.
            {controlMode === 'manual' ? ' Driving: W/S throttle, A/D steer.' : ''}
          </div>
          {warnings.length > 0 ? (
            <div className="warning-banner">{warnings.join(' · ')}</div>
          ) : null}
        </section>

        <section className="bottom">
          <div className="metrics-row">
            <Metric label="Speed" value={`${(state?.car.speed ?? 0).toFixed(2)} m/s`} hint="body speed" />
            <Metric
              label="Steering"
              value={`${(((state?.car.steering ?? 0) * 180) / Math.PI).toFixed(1)}°`}
              hint="actual front angle"
            />
            <Metric label="Lidar returns" value={`${finitePoints} / ${scanning || '—'}`} hint="valid beams" />
            <Metric
              label="SLAM ATE"
              value={
                slam?.available ? `${(slam.ate_m * 100).toFixed(1)} cm` : slamReady ? 'Ready' : '—'
              }
              hint={slam?.available ? `${slam.scan_ms.toFixed(1)} ms/scan` : 'pipeline in M3'}
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
          <div className="progress">
            <div
              className="progress-bar"
              style={{
                width: `${Math.min(
                  100,
                  Math.max(
                    0,
                    ((race?.progress_m ?? 0) /
                      (hello?.track.track_length_m ?? 30)) *
                      100,
                  ),
                )}%`,
              }}
            />
          </div>
          <div className="sensor-row" style={{ marginTop: 8 }}>
            <span>Progress</span>
            <strong>
              {(race?.progress_m ?? 0).toFixed(1)} m · lateral{' '}
              {(race?.lateral_m ?? 0).toFixed(2)} m
            </strong>
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
            <span>Yaw rate</span>
            <strong>{((state?.car.yaw_rate ?? 0) * 180 / Math.PI).toFixed(1)} °/s</strong>
          </div>
          <div className="sensor-row">
            <span>Mount height</span>
            <strong>
              {hello?.sensors.lidar.mount.position_m?.[2]?.toFixed(2) ?? '—'} m
            </strong>
          </div>
          {(state?.sensors.sonars ?? []).map((sonar) => (
            <div className="sensor-row" key={sonar.name}>
              <span>{sonar.name.replaceAll('_', ' ')}</span>
              <strong>{sonar.valid && sonar.range_m !== null ? `${sonar.range_m.toFixed(2)} m` : 'no echo'}</strong>
            </div>
          ))}
        </div>

        <div className="panel">
          <div className="card-heading">
            <span>SLAM</span>
            <span className={`mini-badge ${slam?.available ? 'realism-realistic' : ''}`}>
              {slam?.available ? 'ONLINE' : 'M3'}
            </span>
          </div>
          {slam?.available ? (
            <>
              <div className="sensor-row">
                <span>ATE vs GT</span>
                <strong>{(slam.ate_m * 100).toFixed(1)} cm</strong>
              </div>
              <div className="sensor-row">
                <span>Scan time</span>
                <strong>{slam.scan_ms.toFixed(1)} ms</strong>
              </div>
              <div className="sensor-row">
                <span>Match / inliers</span>
                <strong>
                  {slam.match_score.toFixed(2)} / {slam.inliers}
                </strong>
              </div>
              <div className="sensor-row">
                <span>Scans</span>
                <strong>{slam.scan_count}</strong>
              </div>
              <div className="sensor-row">
                <span>Odom drift</span>
                <strong>{(slam.odom_drift_m * 100).toFixed(0)} cm</strong>
              </div>
              <div className="sensor-row">
                <span>Loop closures</span>
                <strong>{slam.loop_closures}</strong>
              </div>
              <div className="sensor-row">
                <span>Graph nodes / edges</span>
                <strong>
                  {slam.graph_nodes} / {slam.graph_edges}
                </strong>
              </div>
              {mclEnabled ? (
                <>
                  <div className="sensor-row">
                    <span>MCL ATE</span>
                    <strong>
                      {slam.mcl?.initialized
                        ? `${(slam.mcl.ate_m * 100).toFixed(1)} cm`
                        : 'initializing…'}
                    </strong>
                  </div>
                  <div className="sensor-row">
                    <span>MCL ESS / spread</span>
                    <strong>
                      {slam.mcl?.initialized
                        ? `${slam.mcl.ess.toFixed(0)} / ${slam.mcl.spread_m.toFixed(2)} m`
                        : '—'}
                    </strong>
                  </div>
                </>
              ) : null}
              <p className="small-note">
                Blue = SLAM, green = MCL, grey = odometry, amber floor = map.
                {loopClosureEnabled
                  ? ' Loop closure is experimental and may accept false matches.'
                  : ' Loop closure is off; enable it in Localization to experiment.'}
              </p>
            </>
          ) : (
            <>
              <p className="small-note">
                From-scratch 2D lidar SLAM: correlative matching, occupancy grid and
                diagnostics. The C++ core is {slamReady ? 'compiled' : 'not built'}.
              </p>
              <div className="disabled-bar">
                <span>{slamReady ? 'Waiting for scans' : 'Build the C++ core'}</span>
              </div>
            </>
          )}
        </div>

        <div className="panel">
          <div className="card-heading">
            <span>Strategy benchmarks</span>
            <span className="muted">5 laps · test track</span>
          </div>
          <div className="sensor-row">
            <span>Wall-follow (lidar)</span>
            <strong>148.34 s</strong>
          </div>
          <div className="sensor-row">
            <span>Pure pursuit (GT)</span>
            <strong>46.12 s</strong>
          </div>
          <p className="small-note">
            Pure pursuit runs on the ground-truth pose and is an upper bound until
            SLAM/MCL localization lands in M3.
          </p>
        </div>

        <div className="panel">
          <div className="card-heading">
            <span>Milestone</span>
            <span className="mini-badge">M3</span>
          </div>
          <p className="small-note">
            SLAM front-end + MCL + debug layers are live. MCL and experimental loop
            closure can be toggled independently. Next: reliable place recognition and a
            deployable pure-pursuit result.
          </p>
          <div className="progress">
            <div className="progress-bar" style={{ width: '78%' }} />
          </div>
          <div className="small-note">Docs: docs/02-architecture.md · docs/04-sensors.md</div>
        </div>
      </aside>
    </div>
  );
}
