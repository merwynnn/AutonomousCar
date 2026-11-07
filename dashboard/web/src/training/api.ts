export interface TrainingStatus {
  stage: string;
  phase: string;
  reason?: string;
  process_running: boolean;
  best_available: boolean;
  best_path: string | null;
  generation?: number;
  generations?: number;
  percent?: number;
  elapsed_s?: number;
  eta_s?: number | null;
  eta_text?: string;
  average_generation_s?: number;
  generation_seconds?: number;
  best_lap_s?: number | null;
  best_contacts?: number | null;
  best_generation?: number | null;
  best_progress_m?: number;
  best_score?: number | null;
  generation_lap_s?: number | null;
  generation_contacts?: number;
  generation_score?: number;
  sigma?: number;
  popsize?: number;
  workers?: number;
  seed?: number;
  track?: string;
  out?: string;
  pid?: number;
  log?: string;
  updated_at?: number;
}

export interface HistoryRecord {
  generation: number;
  phase: string;
  lap_s?: number | null;
  contacts?: number;
  progress_m?: number;
  score?: number;
  fitness?: number;
  best_lap_s?: number | null;
  best_contacts?: number | null;
  best_generation?: number | null;
  best_score?: number | null;
  sigma?: number;
  generation_seconds?: number;
  track?: string | null;
  updated_at?: number;
}

export interface StartParams {
  stage: string;
  generations: number;
  popsize: number;
  workers: number;
  seed: number;
  budget_seconds: number;
  random_track: boolean;
  init_result?: string | null;
  run_name?: string | null;
  tracks_per_generation?: number;
  fitness_aggregation?: string;
  vary_sensor_seed?: boolean;
  stop_after_stall?: number;
}

export interface TrainingResult {
  name: string;
  stage: string | null;
  policy_type: string | null;
  best_lap_s: number | null;
  best_contacts: number | null;
  generations: number | null;
  wall_time_s: number | null;
  aggregation: string | null;
  init: string | null;
  created: number;
  path: string;
}

async function request<T>(path: string, options?: RequestInit): Promise<T> {
  const response = await fetch(path, options);
  const body = await response.json().catch(() => ({}));
  if (!response.ok) {
    throw new Error(body.detail ?? `request failed (${response.status})`);
  }
  return body as T;
}

export function fetchRuns(): Promise<{ runs: TrainingStatus[] }> {
  return request('/api/training/runs');
}

export function fetchStatus(stage: string): Promise<TrainingStatus> {
  return request(`/api/training/status?stage=${encodeURIComponent(stage)}`);
}

export function fetchHistory(stage: string): Promise<{ stage: string; records: HistoryRecord[] }> {
  return request(`/api/training/history?stage=${encodeURIComponent(stage)}&limit=5000`);
}

export function startRun(params: StartParams): Promise<{ ok: boolean; stage: string; pid?: number }> {
  return request('/api/training/start', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(params),
  });
}

export function stopRun(stage: string): Promise<{ ok: boolean; stopped: boolean }> {
  return request('/api/training/stop', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ stage }),
  });
}

export function watchBest(stage: string): Promise<{ ok: boolean; message: string }> {
  return request('/api/training/watch-best', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ stage }),
  });
}

export function fetchResults(): Promise<{ results: TrainingResult[] }> {
  return request('/api/training/results');
}

export function fetchResultHistory(
  name: string,
): Promise<{ name: string; records: HistoryRecord[] }> {
  return request(`/api/training/result-history?name=${encodeURIComponent(name)}&limit=20000`);
}

export function watchResult(name: string): Promise<{ ok: boolean; message: string }> {
  return request('/api/training/watch-result', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ name }),
  });
}
