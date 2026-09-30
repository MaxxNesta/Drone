"use client";
import { useCallback, useEffect, useRef, useState } from "react";
import {
  Ack,
  decode,
  emptyStream,
  Envelope,
  FleetConfig,
  Outcome,
  receive,
  Status,
} from "./model";
export async function api<T>(
  path: string,
  body?: unknown,
  method?: string,
): Promise<T> {
  const response = await fetch("/api/" + path, {
    method: method || (body === undefined ? "GET" : "POST"),
    headers: { "Content-Type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
    cache: "no-store",
  });
  const data = await response.json();
  if (response.status === 401)
    window.dispatchEvent(new Event("skyview:session-expired"));
  if (!response.ok)
    throw new Error(
      typeof data.detail === "string"
        ? data.detail
        : `Request rejected (${response.status})`,
    );
  return data;
}
export function useMission(enabled: boolean) {
  const [stream, setStream] = useState(emptyStream),
    [status, setStatus] = useState<Status | null>(null),
    [config, setConfig] = useState<FleetConfig | null>(null);
  const [connection, setConnection] = useState("connecting"),
    [error, setError] = useState(""),
    [transportError, setTransportError] = useState(""),
    [outcomes, setOutcomes] = useState<Outcome[]>([]),
    [acks, setAcks] = useState<Ack[]>([]);
  const cursor = useRef<Envelope | null>(null),
    lastReceived = useRef(0);
  const [reload, setReload] = useState(0);
  const refresh = useCallback(() => setReload((n) => n + 1), []);
  useEffect(() => {
    if (!enabled) return;
    let live = true;
    let pending = false;
    async function poll() {
      if (pending) return;
      pending = true;
      try {
        const s = await api<Status>("status");
        if (!live) return;
        setStatus((previous) =>
          previous?.epoch === s.epoch && previous.sequence > s.sequence
            ? previous
            : s,
        );
        setTransportError("");
        if (!s.run_id) setConnection("ready");
        const report = await api<{ run_id: string; outcomes: Outcome[] }>(
          "commands",
        );
        if (live && report.run_id === s.run_id) setOutcomes(report.outcomes);
        if (lastReceived.current && Date.now() - lastReceived.current > 15000)
          setConnection("stale");
      } catch (e) {
        if (live) {
          setConnection("disconnected");
          setTransportError((e as Error).message);
        }
      } finally {
        pending = false;
      }
    }
    void poll();
    const interval = setInterval(poll, 1500);
    return () => {
      live = false;
      clearInterval(interval);
    };
  }, [enabled, reload]);
  useEffect(() => {
    if (!enabled || !status?.run_id) {
      setConfig(null);
      return;
    }
    let live = true;
    setConfig(null);
    void api<{ run_id: string; config: FleetConfig | null }>("plan")
      .then((p) => {
        if (live && p.run_id === status.run_id) setConfig(p.config);
      })
      .catch((e) => setError(e.message));
    return () => {
      live = false;
    };
  }, [enabled, status?.run_id, reload]);
  useEffect(() => {
    setAcks([]);
    setOutcomes([]);
  }, [status?.run_id]);
  useEffect(() => {
    if (!enabled || !status?.run_id) return;
    let stopped = false;
    let socket: WebSocket;
    let timer: ReturnType<typeof setTimeout>;
    let attempt = 0;
    const open = () => {
      if (stopped) return;
      setConnection(attempt ? "reconnecting" : "connecting");
      const query = cursor.current
        ? `?epoch=${encodeURIComponent(cursor.current.epoch)}&after=${cursor.current.sequence}`
        : "";
      socket = new WebSocket(`ws://${location.host}/telemetry${query}`);
      socket.onmessage = (event) => {
        try {
          const value = JSON.parse(event.data);
          lastReceived.current = Date.now();
          if (value.type === "heartbeat") {
            setStatus(value);
            setConnection("live");
            return;
          }
          const next = decode(value);
          const prior = cursor.current;
          if (
            prior?.epoch === next.epoch &&
            prior.run_id === next.run_id &&
            next.sequence < prior.sequence
          )
            return;
          cursor.current = next;
          setStream((s) => receive(s, next));
          setStatus(next.status);
          setConnection(
            next.delivery === "resync" ? "resynchronizing" : "live",
          );
          setTransportError("");
          attempt = 0;
        } catch (e) {
          setTransportError((e as Error).message);
          socket.close();
        }
      };
      socket.onclose = () => {
        if (stopped) return;
        setConnection("disconnected");
        timer = setTimeout(
          open,
          Math.min(8000, 500 * 2 ** Math.min(attempt++, 4)),
        );
      };
      socket.onerror = () => socket.close();
    };
    open();
    return () => {
      stopped = true;
      clearTimeout(timer);
      socket?.close();
    };
  }, [enabled, status?.run_id, reload]);
  const playback = async (
    paused: boolean,
    rate = status?.playback_speed || 1,
  ) => {
    if (!status?.run_id) return;
    try {
      setStatus(
        await api<Status>("playback", {
          run_id: status.run_id,
          paused,
          speed: rate,
        }),
      );
      setError("");
    } catch (e) {
      setError((e as Error).message);
    }
  };
  const command = async (name: string, vehicle_id?: string) => {
    if (!status?.run_id) return;
    try {
      const request_id = crypto.randomUUID();
      const ack = await api<{ target_tick: number }>("commands", {
        run_id: status.run_id,
        request_id,
        command: name,
        ...(vehicle_id ? { vehicle_id } : {}),
      });
      setAcks((a) =>
        [
          ...a,
          {
            request_id,
            target_tick: ack.target_tick,
            command: name,
            vehicle_id,
            run_id: status.run_id!,
          },
        ].slice(-20),
      );
      setError("");
    } catch (e) {
      setError((e as Error).message);
    }
  };
  return {
    stream,
    status,
    config,
    connection,
    error: error || transportError,
    setError: (message: string) => {
      setError(message);
      setTransportError("");
    },
    outcomes,
    acks,
    refresh,
    playback,
    command,
  };
}
