import { useCallback, useEffect, useMemo, useState } from 'react';
import {
  fetchHistory,
  fetchResults,
  fetchStatus,
  startRun,
  stopRun,
  watchBest,
  watchResult,
  type HistoryRecord,
  type StartParams,
  type TrainingResult,
  type TrainingStatus,
} from './api';
import { LineChart, StatCard } from './charts';

const STAGES = ['b1', 'b2a', 'b2b'] as const;

const STAGE_INFO: Record<string, string> = {
  b1: 'B1 — structured gap controller: 23 parameters (gap threshold, clearance window, scoring weights, steering gain/rate, speed schedule). Robustness-first, safe fallback.',
  b2a: 'B2a — linear beam scorer: 36 downsampled beams × 12 features with a linear score and target-speed head (45 params), warm-started from B1. Fastest policy family.',
  b2b: 'B2b — B2a expanded with a 16-unit residual hidden layer (605 params), evolved with diagonal CMA-ES. Numerically identical to B2a at initialisation.',
};

function formatDuration(seconds: number | null | undefined): string {
  if (seconds === null || seconds === undefined || !Number.isFinite(seconds)) return '--:--';
  const total = Math.round(seconds);
  const h = Math.floor(total / 3600);
  const m = Math.floor((total % 3600) / 60);
  const s = total % 60;
  return h > 0 ? `${h}:${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}` : `${String(m).padStart(2, '0')}:${String(s).padStart(2, '0')}`;
}

function formatLap(lap: number | null | undefined): string {
  return lap === null || lap === undefined ? 'DNF' : lap.toFixed(2);
}

export default function TrainingApp() {
  const [stage, setStage] = useState<string>('b1');
  const [status, setStatus] = useState<TrainingStatus | null>(null);
  const [records, setRecords] = useState<HistoryRecord[]>([]);
  const [results, setResults] = useState<TrainingResult[]>([]);
  const [startFrom, setStartFrom] = useState<string>('');
  const [message, setMessage] = useState<string | null>(null);
  const [form, setForm] = useState<Omit<StartParams, 'stage'>>({
    generations: 100,
    popsize: 20,
    workers: 6,
    seed: 42,
    budget_seconds: 0,
    random_track: true,
  });

  const refresh = useCallback(async (selected: string) => {
    try {
      const [nextStatus, history, library] = await Promise.all([
        fetchStatus(selected),
        fetchHistory(selected),
        fetchResults(),
      ]);
      setStatus(nextStatus);
      setRecords(history.records);
      setResults(library.results);
    } catch {
      // Backend may be restarting; the next poll recovers.
    }
  }, []);

  useEffect(() => {
    void refresh(stage);
    const timer = window.setInterval(() => void refresh(stage), 1000);
    return () => window.clearInterval(timer);
  }, [stage, refresh]);

  const running = status?.process_running ?? false;

  const onStart = async () => {
    setMessage(null);
    try {
      await startRun({ ...form, stage, init_result: startFrom || null });
      setMessage(
        startFrom
          ? `Started ${stage} training warm-started from "${startFrom}".`
          : `Started ${stage} training run from scratch.`,
      );
    } catch (error) {
      setMessage(error instanceof Error ? error.message : String(error));
    }
  };

  const onWatchResult = async (name: string) => {
    setMessage(null);
    try {
      const result = await watchResult(name);
      setMessage(result.message);
    } catch (error) {
      setMessage(error instanceof Error ? error.message : String(error));
    }
  };

  const onStop = async () => {
    setMessage(null);
    try {
      const result = await stopRun(stage);
      setMessage(result.stopped ? `Stopped ${stage} run.` : `${stage} was not running.`);
    } catch (error) {
      setMessage(error instanceof Error ? error.message : String(error));
    }
  };

  const onWatchBest = async () => {
    setMessage(null);
    try {
      const result = await watchBest(stage);
      setMessage(result.message);
    } catch (error) {
      setMessage(error instanceof Error ? error.message : String(error));
    }
  };

  const genRecords = useMemo(
    () => records.filter((r) => r.phase === 'running' && r.generation >= 0),
    [records],
  );

  const charts = useMemo(() => {
    return {
      lap: genRecords.map((r) => [r.generation, r.lap_s ?? null] as [number, number | null]),
      bestLap: genRecords.map((r) => [r.generation, r.best_lap_s ?? null] as [number, number | null]),
      score: genRecords.map((r) => [r.generation, r.score ?? null] as [number, number | null]),
      bestScore: genRecords.map((r) => [r.generation, r.best_score ?? null] as [number, number | null]),
      sigma: genRecords.map((r) => [r.generation, r.sigma ?? null] as [number, number | null]),
      seconds: genRecords.map((r) => [r.generation, r.generation_seconds ?? null] as [number, number | null]),
      contacts: genRecords.map((r) => [r.generation, r.contacts ?? null] as [number, number | null]),
    };
  }, [genRecords]);

  const percent = status?.percent ?? 0;

  return (
    <div className="training-shell">
      <header className="training-header">
        <div className="training-title">
          <span className="wordmark-mark">A</span>
          <strong>Training</strong>
          <em>CMA-ES POLICY SEARCH</em>
        </div>
        <label className="training-type-select" title={STAGE_INFO[stage]}>
          <span>Training type</span>
          <select
            value={stage}
            onChange={(e) => {
              setStage(e.target.value);
              setStartFrom('');
            }}
          >
            {STAGES.map((name) => (
              <option key={name} value={name}>
                {name}
              </option>
            ))}
          </select>
        </label>
        <span className="training-stage-info" title={STAGE_INFO[stage]}>
          {STAGE_INFO[stage]}
        </span>
        <span className={`training-phase phase-${status?.phase ?? 'idle'}`}>
          {running ? 'RUNNING' : (status?.phase ?? 'idle').toUpperCase()}
        </span>
      </header>

      <section className="training-controls panel">
        <div className="control-grid">
          <label className="start-from-select">
            <span>Start from</span>
            <select
              value={startFrom}
              disabled={running}
              onChange={(e) => setStartFrom(e.target.value)}
            >
              <option value="">From scratch</option>
              {results
                .filter((r) => (stage === 'b1' ? r.policy_type === 'b1' : true))
                .map((r) => (
                  <option key={r.name} value={r.name}>
                    {r.name} — {r.policy_type ?? r.stage}, {formatLap(r.best_lap_s)}s
                  </option>
                ))}
            </select>
          </label>
          <label>
            <span>Generations</span>
            <input
              type="number"
              min={1}
              max={2000}
              value={form.generations}
              disabled={running}
              onChange={(e) => setForm({ ...form, generations: Number(e.target.value) })}
            />
          </label>
          <label>
            <span>Popsize</span>
            <input
              type="number"
              min={4}
              max={64}
              value={form.popsize}
              disabled={running}
              onChange={(e) => setForm({ ...form, popsize: Number(e.target.value) })}
            />
          </label>
          <label>
            <span>Workers</span>
            <input
              type="number"
              min={1}
              max={32}
              value={form.workers}
              disabled={running}
              onChange={(e) => setForm({ ...form, workers: Number(e.target.value) })}
            />
          </label>
          <label>
            <span>Seed</span>
            <input
              type="number"
              min={0}
              value={form.seed}
              disabled={running}
              onChange={(e) => setForm({ ...form, seed: Number(e.target.value) })}
            />
          </label>
          <label>
            <span>Budget (s, 0 = none)</span>
            <input
              type="number"
              min={0}
              value={form.budget_seconds}
              disabled={running}
              onChange={(e) => setForm({ ...form, budget_seconds: Number(e.target.value) })}
            />
          </label>
          <label className="checkbox-label">
            <input
              type="checkbox"
              checked={form.random_track}
              disabled={running}
              onChange={(e) => setForm({ ...form, random_track: e.target.checked })}
            />
            <span>Random tracks</span>
          </label>
        </div>
        <div className="control-actions">
          <button type="button" className="primary" onClick={onStart} disabled={running}>
            Start training
          </button>
          <button type="button" onClick={onStop} disabled={!running}>
            Stop
          </button>
          <button
            type="button"
            onClick={onWatchBest}
            disabled={!status?.best_available}
            title={`Load the best ${stage} policy of the current run (data/training/${stage}_best.npz) into the Track view simulator`}
          >
            View best {stage} car in dashboard
          </button>
        </div>
      </section>

      <section className="training-results panel">
        <h3>Saved results</h3>
        {results.length === 0 ? (
          <p className="training-results-empty">
            No saved runs yet — every finished training run is kept here under its own
            name (data/training/results/). Warm-start a new run from any of them or
            watch one in the simulator.
          </p>
        ) : (
          <table className="training-results-table">
            <thead>
              <tr>
                <th>Run</th>
                <th>Type</th>
                <th>Best lap</th>
                <th>Contacts</th>
                <th>Gens</th>
                <th>Wall time</th>
                <th />
              </tr>
            </thead>
            <tbody>
              {results.map((r) => (
                <tr key={r.name}>
                  <td title={r.init ? `warm start: ${r.init}` : undefined}>{r.name}</td>
                  <td>{r.policy_type ?? r.stage ?? '—'}</td>
                  <td>{formatLap(r.best_lap_s)} s</td>
                  <td>{r.best_contacts === null || r.best_contacts === undefined ? '—' : String(r.best_contacts)}</td>
                  <td>{r.generations ?? '—'}</td>
                  <td>{formatDuration(r.wall_time_s)}</td>
                  <td>
                    <button type="button" onClick={() => void onWatchResult(r.name)}>
                      Watch
                    </button>
                  </td>
                </tr>
              ))}
            </tbody>
          </table>
        )}
      </section>

      <section className="training-progress panel">
        <div className="progress-bar">
          <i style={{ width: `${Math.min(100, percent)}%` }} />
        </div>
        <div className="progress-meta">
          <span>{percent.toFixed(1)}%</span>
          <span>
            gen {(status?.generation ?? -1) + 1}/{status?.generations ?? '?'}
          </span>
          <span>elapsed {formatDuration(status?.elapsed_s)}</span>
          <span>ETA {status?.eta_text ?? formatDuration(status?.eta_s)}</span>
          <span>avg gen {formatDuration(status?.average_generation_s)}</span>
          {status?.track ? <span>track {status.track}</span> : null}
        </div>
      </section>

      <section className="training-stats">
        <StatCard label="Best lap" value={formatLap(status?.best_lap_s)} unit="s" hint={`generation ${status?.best_generation ?? '—'}`} />
        <StatCard label="Best contacts" value={status?.best_contacts === null || status?.best_contacts === undefined ? '—' : String(status.best_contacts)} />
        <StatCard label="Best score" value={status?.best_score === null || status?.best_score === undefined ? '—' : status.best_score.toFixed(1)} />
        <StatCard label="Best progress" value={status?.best_progress_m === undefined ? '—' : status.best_progress_m.toFixed(1)} unit="m" />
        <StatCard label="Sigma" value={status?.sigma === undefined ? '—' : status.sigma.toFixed(4)} />
        <StatCard label="Gen best lap" value={formatLap(status?.generation_lap_s)} unit="s" />
      </section>

      <section className="training-charts">
        <p className="training-about">
          <strong>About these graphs</strong> — x-axis is the CMA-ES generation.{' '}
          <em>Lap time</em>: clean reference-track lap of each generation&apos;s best
          candidate (solid) vs the all-time best, re-validated clean at the reference
          start (dashed). <em>Score</em>: the candidate score the optimizer minimises
          (lower is better). <em>Contacts</em>: wall/obstacle hits at physics
          resolution. <em>Sigma</em>: CMA-ES search step size — shrinking means the
          search is converging. <em>Wall time</em>: seconds per generation.
        </p>
        <LineChart
          title="Lap time per generation"
          yLabel="s"
          series={[
            { label: 'best of generation', color: '#d1a34b', points: charts.lap },
            { label: 'all-time best', color: '#74aed6', points: charts.bestLap, dashed: true },
          ]}
        />
        <LineChart
          title="Score per generation"
          yLabel="score"
          series={[
            { label: 'generation best', color: '#d1a34b', points: charts.score },
            { label: 'all-time best', color: '#74aed6', points: charts.bestScore, dashed: true },
          ]}
        />
        <LineChart
          title="Contacts per generation"
          yLabel="count"
          series={[{ label: 'generation best', color: '#bd5b5b', points: charts.contacts }]}
        />
        <LineChart
          title="Search sigma"
          yLabel="sigma"
          series={[{ label: 'sigma', color: '#7ba873', points: charts.sigma }]}
        />
        <LineChart
          title="Wall time per generation"
          yLabel="s"
          series={[{ label: 'generation seconds', color: '#8b939c', points: charts.seconds }]}
        />
        <div className="chart-card chart-empty">
          <span>{genRecords.length === 0 ? 'No generations yet — start a run to see progress.' : `${genRecords.length} generations recorded for stage ${stage}.`}</span>
          {status?.log ? <span className="chart-log">log {status.log}</span> : null}
        </div>
      </section>

      {message ? <footer className="training-message">{message}</footer> : null}
    </div>
  );
}
