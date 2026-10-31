import { useCallback, useEffect, useMemo, useState } from 'react';
import type {
  ConfigPayload,
  ConfigResultMessage,
  HelloMessage,
} from './types';

type Tab = 'vehicle' | 'sensors';

interface TuningPanelProps {
  open: boolean;
  initialTab: Tab;
  hello: HelloMessage | null;
  configResult: ConfigResultMessage | null;
  onClose: () => void;
  onSend: (message: object) => void;
}

const ENUMS: Record<string, string[]> = {
  shape: ['box', 'cylinder', 'sphere'],
  drive: ['4wd', '2wd'],
  scan_direction: ['clockwise', 'counterclockwise'],
  realism: ['ideal', 'realistic', 'harsh'],
};

const UNITS: Record<string, string> = {
  _m2: 'm²',
  _m_s2: 'm/s²',
  _deg_s: '°/s',
  _rad_s: 'rad/s',
  _mps: 'm/s',
  _mah: 'mAh',
  _ohm: 'Ω',
  _rpm: 'rpm',
  _nm: 'N·m',
  _kg: 'kg',
  _deg: '°',
  _hz: 'Hz',
  _ms: 'ms',
  _m: 'm',
  _v: 'V',
  _a: 'A',
  _s: 's',
  _rad: 'rad',
};

function clone<T>(value: T): T {
  return JSON.parse(JSON.stringify(value)) as T;
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null && !Array.isArray(value);
}

function isNumberArray(value: unknown): value is number[] {
  return Array.isArray(value) && value.every((item) => typeof item === 'number');
}

function updateAtPath(root: ConfigPayload, path: Array<string | number>, value: unknown): ConfigPayload {
  const walk = (node: unknown, index: number): unknown => {
    if (index >= path.length) return value;
    const key = path[index];
    if (Array.isArray(node)) {
      const copy = [...node];
      copy[Number(key)] = walk(copy[Number(key)], index + 1);
      return copy;
    }
    if (isRecord(node)) {
      return { ...node, [String(key)]: walk(node[String(key)], index + 1) };
    }
    return node;
  };
  return walk(root, 0) as ConfigPayload;
}

function humanize(key: string): string {
  const unit = Object.keys(UNITS).find((suffix) => key.endsWith(suffix));
  const base = unit ? key.slice(0, -unit.length) : key;
  return base
    .replaceAll('_', ' ')
    .replace(/\b\w/g, (char) => char.toUpperCase());
}

function unitFor(key: string): string | null {
  const suffix = Object.keys(UNITS).find((item) => key.endsWith(item));
  return suffix ? UNITS[suffix] : null;
}

function stepFor(key: string): string {
  if (
    [
      'beams',
      'rays',
      'cells',
      'ticks_per_rev',
      'seed',
      'max_rpm',
      'capacity_mah',
    ].includes(key)
  ) {
    return '1';
  }
  return 'any';
}

function PrimitiveField({
  fieldKey,
  value,
  onChange,
}: {
  fieldKey: string;
  value: unknown;
  onChange: (value: unknown) => void;
}) {
  const label = humanize(fieldKey);
  const unit = unitFor(fieldKey);

  if (typeof value === 'boolean') {
    return (
      <label className="tune-check">
        <input type="checkbox" checked={value} onChange={(event) => onChange(event.target.checked)} />
        <span>{label}</span>
      </label>
    );
  }

  if (typeof value === 'number') {
    return (
      <label className="tune-field">
        <span>{label}</span>
        <div className="tune-input-row">
          <input
            type="number"
            step={stepFor(fieldKey)}
            value={value}
            onChange={(event) => {
              const next = event.target.value === '' ? 0 : Number(event.target.value);
              if (Number.isFinite(next)) onChange(next);
            }}
          />
          {unit ? <em>{unit}</em> : null}
        </div>
      </label>
    );
  }

  if (typeof value === 'string') {
    if (ENUMS[fieldKey]) {
      return (
        <label className="tune-field">
          <span>{label}</span>
          <select value={value} onChange={(event) => onChange(event.target.value)}>
            {ENUMS[fieldKey].map((option) => (
              <option key={option} value={option}>
                {option}
              </option>
            ))}
          </select>
        </label>
      );
    }
    if (fieldKey === 'color' && /^#[0-9a-fA-F]{6}$/.test(value)) {
      return (
        <label className="tune-field">
          <span>{label}</span>
          <div className="tune-color-row">
            <input type="color" value={value} onChange={(event) => onChange(event.target.value)} />
            <input
              type="text"
              value={value}
              onChange={(event) => onChange(event.target.value)}
            />
          </div>
        </label>
      );
    }
    if (fieldKey === 'description') {
      return (
        <label className="tune-field tune-field-wide">
          <span>{label}</span>
          <textarea
            rows={2}
            value={value}
            onChange={(event) => onChange(event.target.value)}
          />
        </label>
      );
    }
    return (
      <label className="tune-field">
        <span>{label}</span>
        <input type="text" value={value} onChange={(event) => onChange(event.target.value)} />
      </label>
    );
  }

  return null;
}

function NumberVector({
  fieldKey,
  values,
  onChange,
}: {
  fieldKey: string;
  values: number[];
  onChange: (values: number[]) => void;
}) {
  return (
    <div className="tune-field">
      <span>{humanize(fieldKey)}</span>
      <div className="tune-vector">
        {values.map((value, index) => (
          <label key={index}>
            <span>{values.length === 3 ? ['x', 'y', 'z'][index] : index}</span>
            <input
              type="number"
              step="any"
              value={value}
              onChange={(event) => {
                const next = [...values];
                next[index] = Number(event.target.value);
                onChange(next);
              }}
            />
          </label>
        ))}
        <em>{unitFor(fieldKey)}</em>
      </div>
    </div>
  );
}

function FieldTree({
  path,
  value,
  depth,
  onChange,
}: {
  path: Array<string | number>;
  value: unknown;
  depth: number;
  onChange: (path: Array<string | number>, value: unknown) => void;
}) {
  if (isNumberArray(value)) {
    return (
      <NumberVector
        fieldKey={String(path[path.length - 1] ?? '')}
        values={value}
        onChange={(next) => onChange(path, next)}
      />
    );
  }

  if (Array.isArray(value)) {
    if (value.length === 0) {
      return <div className="tune-empty">No entries</div>;
    }
    if (value.every(isRecord)) {
      return (
        <div className="tune-array">
          {value.map((item, index) => {
            const name = typeof item.name === 'string' ? item.name : `item ${index + 1}`;
            return (
              <details className="tune-group nested" key={`${path.join('.')}.${index}`} open={depth < 2}>
                <summary>
                  <span>{name}</span>
                  <span className="tune-index">{index}</span>
                </summary>
                <div className="tune-fields">
                  {Object.entries(item).map(([key, child]) => (
                    <FieldTree
                      key={key}
                      path={[...path, index, key]}
                      value={child}
                      depth={depth + 1}
                      onChange={onChange}
                    />
                  ))}
                </div>
              </details>
            );
          })}
        </div>
      );
    }
    return (
      <div className="tune-field">
        <span>{humanize(String(path[path.length - 1] ?? ''))}</span>
        <span className="tune-readonly">{value.length} entries</span>
      </div>
    );
  }

  if (isRecord(value)) {
    return (
      <details className="tune-group" open={depth < 1}>
        <summary>
          <span>{humanize(String(path[path.length - 1] ?? ''))}</span>
          <span className="tune-path">{path.join('.')}</span>
        </summary>
        <div className="tune-fields">
          {Object.entries(value).map(([key, child]) => (
            <FieldTree
              key={key}
              path={[...path, key]}
              value={child}
              depth={depth + 1}
              onChange={onChange}
            />
          ))}
        </div>
      </details>
    );
  }

  return (
    <PrimitiveField
      fieldKey={String(path[path.length - 1] ?? '')}
      value={value}
      onChange={(next) => onChange(path, next)}
    />
  );
}

function setSteering(draft: ConfigPayload, key: string, value: unknown): ConfigPayload {
  return updateAtPath(draft, ['vehicle', 'steering', key], value);
}

export function TuningPanel({
  open,
  initialTab,
  hello,
  configResult,
  onClose,
  onSend,
}: TuningPanelProps) {
  const [tab, setTab] = useState<Tab>(initialTab);
  const [draft, setDraft] = useState<ConfigPayload | null>(null);
  const [baseline, setBaseline] = useState<ConfigPayload | null>(null);
  const [history, setHistory] = useState<ConfigPayload[]>([]);
  const [redo, setRedo] = useState<ConfigPayload[]>([]);
  const [status, setStatus] = useState('');

  useEffect(() => {
    if (!open) {
      setDraft(null);
      setBaseline(null);
      setHistory([]);
      setRedo([]);
      setStatus('');
      return;
    }
    setTab(initialTab);
    if (hello) {
      setDraft((current) => current ?? clone(hello.config));
      setBaseline((current) => current ?? clone(hello.config));
    }
  }, [open, initialTab, hello?.config]);

  useEffect(() => {
    if (configResult && open) {
      setStatus(configResult.message);
      if (configResult.ok) {
        setBaseline(draft ? clone(draft) : null);
      }
    }
  }, [configResult]);

  const dirty = useMemo(
    () => Boolean(draft && baseline && JSON.stringify(draft) !== JSON.stringify(baseline)),
    [draft, baseline],
  );

  const edit = useCallback(
    (path: Array<string | number>, value: unknown) => {
      if (!draft) return;
      const next = updateAtPath(draft, path, value);
      setHistory((current) => [...current.slice(-79), draft]);
      setRedo([]);
      setDraft(next);
    },
    [draft],
  );

  const undo = useCallback(() => {
    if (!draft || history.length === 0) return;
    const previous = history[history.length - 1];
    setHistory(history.slice(0, -1));
    setRedo([draft, ...redo].slice(0, 80));
    setDraft(previous);
  }, [draft, history, redo]);

  const redoEdit = useCallback(() => {
    if (!draft || redo.length === 0) return;
    const next = redo[0];
    setRedo(redo.slice(1));
    setHistory([...history.slice(-79), draft]);
    setDraft(next);
  }, [draft, history, redo]);

  const close = useCallback(() => {
    if (dirty && !window.confirm('Discard unsaved tuning changes?')) return;
    onClose();
  }, [dirty, onClose]);

  useEffect(() => {
    if (!open) return;
    const onKey = (event: KeyboardEvent) => {
      const target = event.target as HTMLElement | null;
      const typing =
        target &&
        (['INPUT', 'TEXTAREA', 'SELECT'].includes(target.tagName) || target.isContentEditable);
      if (!(event.ctrlKey || event.metaKey)) {
        if (event.key === 'Escape') {
          event.preventDefault();
          close();
        }
        return;
      }
      const key = event.key.toLowerCase();
      if (key === 'z') {
        event.preventDefault();
        if (event.shiftKey) redoEdit();
        else undo();
      } else if (key === 'y') {
        event.preventDefault();
        redoEdit();
      } else if (typing) {
        return;
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [open, undo, redoEdit, close]);

  if (!open || !draft || !hello) return null;

  const steering = draft.vehicle.steering;
  const steeringObject = isRecord(steering) ? steering : null;
  const sourcePath = tab === 'vehicle' ? hello.paths.vehicle : hello.paths.sensors;

  const apply = (save: boolean) => {
    setStatus(save ? 'Saving and rebuilding…' : 'Applying live…');
    onSend({
      type: 'config.apply',
      save,
      vehicle: draft.vehicle,
      sensors: draft.sensors,
    });
  };

  return (
    <div className="tuning-backdrop" role="dialog" aria-modal="true" aria-label="Car tuning console">
      <div className="tuning-panel">
        <header className="tuning-header">
          <div>
            <h2>Car tuning console</h2>
            <p>
              Edit the same config that generates URDF/MJCF, physics, sensors and the car studio.
            </p>
          </div>
          <div className="tuning-header-actions">
            <button type="button" className="tuning-icon" title="Undo (Ctrl+Z)" disabled={!history.length} onClick={undo}>
              ↶
            </button>
            <button type="button" className="tuning-icon" title="Redo (Ctrl+Y)" disabled={!redo.length} onClick={redoEdit}>
              ↷
            </button>
            <button type="button" className="tuning-close" onClick={close} title="Close (Esc)">
              ✕
            </button>
          </div>
        </header>

        <div className="tuning-tabs">
          <button type="button" className={tab === 'vehicle' ? 'active' : ''} onClick={() => setTab('vehicle')}>
            Vehicle
          </button>
          <button type="button" className={tab === 'sensors' ? 'active' : ''} onClick={() => setTab('sensors')}>
            Sensors
          </button>
          <span className="tuning-source">{sourcePath}</span>
        </div>

        <div className="tuning-quick">
          <span className="tuning-quick-title">Quick tune — steering</span>
          {steeringObject ? (
            <>
              {(
                [
                  ['rate_deg_s', 'Steering speed', '°/s'],
                  ['latency_ms', 'Servo latency', 'ms'],
                  ['max_angle_deg', 'Max angle', '°'],
                  ['deadband_deg', 'Deadband', '°'],
                ] as Array<[string, string, string]>
              ).map(([key, label, unit]) => (
                <label key={key}>
                  <span>{label}</span>
                  <input
                    type="number"
                    step="any"
                    value={Number(steeringObject[key] ?? 0)}
                    onChange={(event) => {
                      const next = Number(event.target.value);
                      if (Number.isFinite(next)) {
                        setHistory([...history.slice(-79), draft]);
                        setRedo([]);
                        setDraft(setSteering(draft, key, next));
                      }
                    }}
                  />
                  <em>{unit}</em>
                </label>
              ))}
            </>
          ) : null}
          <span className="tuning-quick-note">
            Lower the steering speed if the car oscillates or hits walls in the gap benchmark.
          </span>
        </div>

        <div className="tuning-body">
          <FieldTree
            path={[tab]}
            value={draft[tab]}
            depth={0}
            onChange={(path, value) => edit(path, value)}
          />
        </div>

        <footer className="tuning-footer">
          <div className={`tuning-status ${configResult && !configResult.ok ? 'error' : ''}`}>
            {dirty ? 'Unsaved changes · ' : ''}
            {status || (hello ? 'Ctrl+Z undo · Ctrl+Y redo' : '')}
          </div>
          <button type="button" className="tuning-button" onClick={() => {
            if (baseline) {
              setDraft(clone(baseline));
              setHistory([]);
              setRedo([]);
            }
          }} disabled={!dirty}>
            Revert
          </button>
          <button type="button" className="tuning-button" onClick={() => apply(false)}>
            Apply live
          </button>
          <button type="button" className="tuning-button primary" onClick={() => apply(true)}>
            Apply & save YAML
          </button>
        </footer>
      </div>
    </div>
  );
}
