import type { ComponentType } from 'react';
import type { DebugFrame } from './types';

export type CustomDebugRenderer = ComponentType<{ frame: DebugFrame }>;

/**
 * Optional per-controller debug panels.
 *
 * The generic `DebugDock` covers readouts, plots, tables and spatial overlays.
 * Register a component here only when a controller needs a layout that the
 * generic primitives cannot express. Adding a controller does not require
 * touching the server or this file unless it needs a fully custom panel.
 */
export const CUSTOM_DEBUG_RENDERERS: Record<string, CustomDebugRenderer> = {};
