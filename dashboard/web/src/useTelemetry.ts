import { useCallback, useEffect, useRef, useState } from 'react';
import type {
  ConfigResultMessage,
  ControllerResultMessage,
  HelloMessage,
  ScanMessage,
  ServerMessage,
  StateMessage,
  TrackResultMessage,
} from './types';

export type ConnectionStatus = 'connecting' | 'open' | 'closed' | 'error';

export interface SpeedSample {
  t: number;
  speed: number;
  steering: number;
}

export interface Telemetry {
  status: ConnectionStatus;
  hello: HelloMessage | null;
  state: StateMessage | null;
  scan: ScanMessage | null;
  history: SpeedSample[];
  configResult: ConfigResultMessage | null;
  trackResult: TrackResultMessage | null;
  controllerResult: ControllerResultMessage | null;
  send: (message: object) => void;
}

const MAX_HISTORY = 240;

export function useTelemetry(): Telemetry {
  const [status, setStatus] = useState<ConnectionStatus>('connecting');
  const [hello, setHello] = useState<HelloMessage | null>(null);
  const [state, setState] = useState<StateMessage | null>(null);
  const [scan, setScan] = useState<ScanMessage | null>(null);
  const [history, setHistory] = useState<SpeedSample[]>([]);
  const [configResult, setConfigResult] = useState<ConfigResultMessage | null>(null);
  const [trackResult, setTrackResult] = useState<TrackResultMessage | null>(null);
  const [controllerResult, setControllerResult] =
    useState<ControllerResultMessage | null>(null);

  const socketRef = useRef<WebSocket | null>(null);
  const reconnectRef = useRef<number | null>(null);

  useEffect(() => {
    let disposed = false;

    const connect = () => {
      if (disposed) return;
      setStatus('connecting');
      const protocol = window.location.protocol === 'https:' ? 'wss' : 'ws';
      const socket = new WebSocket(`${protocol}://${window.location.host}/ws/telemetry`);
      socketRef.current = socket;

      socket.onopen = () => {
        if (!disposed) setStatus('open');
      };

      socket.onmessage = (event) => {
        if (disposed) return;
        try {
          const message = JSON.parse(event.data) as ServerMessage;
          switch (message.type) {
            case 'hello':
              setHello(message);
              break;
            case 'state':
              setState(message);
              setHistory((previous) => {
                const next = [
                  ...previous,
                  { t: message.t, speed: message.car.speed, steering: message.car.steering },
                ];
                return next.length > MAX_HISTORY ? next.slice(-MAX_HISTORY) : next;
              });
              break;
            case 'scan':
              setScan(message);
              break;
            case 'config.result':
              setConfigResult(message);
              break;
            case 'track.result':
              setTrackResult(message);
              break;
            case 'controller.result':
              setControllerResult(message);
              break;
          }
        } catch {
          // Ignore malformed frames; the next frame will arrive shortly.
        }
      };

      socket.onerror = () => {
        if (!disposed) setStatus('error');
      };

      socket.onclose = () => {
        if (disposed) return;
        setStatus('closed');
        reconnectRef.current = window.setTimeout(connect, 2000);
      };
    };

    connect();

    return () => {
      disposed = true;
      if (reconnectRef.current !== null) {
        window.clearTimeout(reconnectRef.current);
      }
      socketRef.current?.close();
    };
  }, []);

  const send = useCallback((message: object) => {
    const socket = socketRef.current;
    if (socket && socket.readyState === WebSocket.OPEN) {
      socket.send(JSON.stringify(message));
    }
  }, []);

  return {
    status,
    hello,
    state,
    scan,
    history,
    configResult,
    trackResult,
    controllerResult,
    send,
  };
}
