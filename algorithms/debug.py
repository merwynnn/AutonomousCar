"""Serialisable debug snapshots produced by driving algorithms.

The dashboard is a consumer of these snapshots, never an owner of the data
structures. A controller can return a :class:`DebugFrame` from an optional
``debug_frame()`` method and the dashboard renders the readouts, plots, tables
and spatial overlays without importing controller-specific classes.

The module is part of the portable algorithms package: it imports only the
standard library and NumPy, so it can be copied to the Raspberry Pi unchanged.
Debug frames are optional and are not part of :class:`onboard.api.RobotAPI`.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from typing import Any, Literal, TypeAlias

import numpy as np

Scalar: TypeAlias = float | int | bool | str | None
Point: TypeAlias = tuple[float, float]
Frame: TypeAlias = Literal["car", "world"]
Role: TypeAlias = Literal[
    "primary",
    "secondary",
    "muted",
    "accent",
    "warning",
    "candidate",
]


@dataclass(frozen=True)
class Readout:
    """One compact labelled value in the debug panel."""

    key: str
    label: str
    value: Scalar
    unit: str = ""
    precision: int = 2


@dataclass(frozen=True)
class Series:
    """A line or scatter series for an XY plot."""

    id: str
    label: str
    points: Sequence[Point] | np.ndarray
    kind: Literal["line", "scatter"] = "line"
    role: Role = "secondary"


@dataclass(frozen=True)
class Marker:
    """A reference line drawn on a plot."""

    axis: Literal["x", "y"]
    value: float
    label: str = ""
    style: Literal["solid", "dashed"] = "dashed"


@dataclass(frozen=True)
class Band:
    """A shaded interval on the X axis, for example a detected opening."""

    low: float
    high: float
    label: str = ""
    role: Role = "muted"


@dataclass(frozen=True)
class Plot:
    """A concise XY plot. All series share the same Y unit."""

    id: str
    title: str
    series: tuple[Series, ...]
    x_label: str = ""
    y_label: str = ""
    x_unit: str = ""
    y_unit: str = ""
    x_range: tuple[float, float] | None = None
    y_range: tuple[float, float] | None = None
    markers: tuple[Marker, ...] = ()
    bands: tuple[Band, ...] = ()


@dataclass(frozen=True)
class Column:
    """A table column description."""

    key: str
    label: str
    unit: str = ""
    precision: int = 2


@dataclass(frozen=True)
class Table:
    """A compact candidate/ranking table."""

    id: str
    title: str
    columns: tuple[Column, ...]
    rows: tuple[tuple[Scalar, ...], ...]
    selected_row: int | None = None


@dataclass(frozen=True)
class PointMarker:
    """One labelled point in a spatial overlay."""

    x: float
    y: float
    label: str = ""
    role: Role = "candidate"


@dataclass(frozen=True)
class PointsOverlay:
    """Spatial points, optionally with short labels."""

    points: tuple[PointMarker, ...]
    frame: Frame = "car"
    size: float = 0.04
    kind: Literal["points"] = field(default="points", init=False)


@dataclass(frozen=True)
class PolylineOverlay:
    """A polyline in the car or estimated world frame."""

    points: tuple[Point, ...]
    frame: Frame = "car"
    role: Role = "secondary"
    width: float = 1.5
    closed: bool = False
    kind: Literal["polyline"] = field(default="polyline", init=False)


@dataclass(frozen=True)
class RayOverlay:
    """A straight ray from an origin, in the car or world frame."""

    angle_rad: float
    range_m: float
    origin: Point = (0.0, 0.0)
    frame: Frame = "car"
    role: Role = "primary"
    width: float = 1.5
    kind: Literal["ray"] = field(default="ray", init=False)


@dataclass(frozen=True)
class SectorOverlay:
    """A filled angular sector, useful for an opening or sensor field of view."""

    start_angle_rad: float
    end_angle_rad: float
    range_m: float
    center: Point = (0.0, 0.0)
    frame: Frame = "car"
    role: Role = "secondary"
    opacity: float = 0.10
    kind: Literal["sector"] = field(default="sector", init=False)


@dataclass(frozen=True)
class LabelOverlay:
    """A short floating label."""

    position: Point
    text: str
    frame: Frame = "car"
    role: Role = "muted"
    kind: Literal["label"] = field(default="label", init=False)


Overlay: TypeAlias = PointsOverlay | PolylineOverlay | RayOverlay | SectorOverlay | LabelOverlay


@dataclass(frozen=True)
class DebugFrame:
    """One controller debug snapshot for the dashboard."""

    controller: str
    t: float
    title: str = ""
    status: str = ""
    valid: bool = True
    readouts: tuple[Readout, ...] = ()
    plots: tuple[Plot, ...] = ()
    tables: tuple[Table, ...] = ()
    overlays: tuple[Overlay, ...] = ()
    meta: Mapping[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        """Return a JSON-serialisable dashboard payload."""

        return {
            "controller": self.controller,
            "title": self.title,
            "t": round(float(self.t), 4),
            "status": self.status,
            "valid": self.valid,
            "readouts": [
                {
                    "key": readout.key,
                    "label": readout.label,
                    "value": _jsonable(readout.value),
                    "unit": readout.unit,
                    "precision": readout.precision,
                }
                for readout in self.readouts
            ],
            "plots": [_plot_payload(plot) for plot in self.plots],
            "tables": [_table_payload(table) for table in self.tables],
            "overlays": [_overlay_payload(overlay) for overlay in self.overlays],
            "meta": _jsonable(dict(self.meta)),
        }


def _coord(value: Any) -> float:
    number = float(value)
    return round(number, 4) if math.isfinite(number) else 0.0


def _point(value: Sequence[Any]) -> list[float]:
    return [_coord(value[0]), _coord(value[1])]


def _points(values: Sequence[Point] | np.ndarray) -> list[list[float]]:
    return [_point(value) for value in values]


def _jsonable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return _jsonable(value.tolist())
    if isinstance(value, np.generic):
        return _jsonable(value.item())
    if isinstance(value, Mapping):
        return {str(key): _jsonable(item) for key, item in value.items()}
    if isinstance(value, list | tuple):
        return [_jsonable(item) for item in value]
    if value is None or isinstance(value, bool | str):
        return value
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return round(value, 4) if math.isfinite(value) else None
    return str(value)


def _plot_payload(plot: Plot) -> dict[str, Any]:
    return {
        "id": plot.id,
        "title": plot.title,
        "x_label": plot.x_label,
        "y_label": plot.y_label,
        "x_unit": plot.x_unit,
        "y_unit": plot.y_unit,
        "x_range": _jsonable(plot.x_range),
        "y_range": _jsonable(plot.y_range),
        "series": [
            {
                "id": series.id,
                "label": series.label,
                "kind": series.kind,
                "role": series.role,
                "points": _points(series.points),
            }
            for series in plot.series
        ],
        "markers": [
            {
                "axis": marker.axis,
                "value": _coord(marker.value),
                "label": marker.label,
                "style": marker.style,
            }
            for marker in plot.markers
        ],
        "bands": [
            {
                "low": _coord(band.low),
                "high": _coord(band.high),
                "label": band.label,
                "role": band.role,
            }
            for band in plot.bands
        ],
    }


def _table_payload(table: Table) -> dict[str, Any]:
    return {
        "id": table.id,
        "title": table.title,
        "columns": [
            {
                "key": column.key,
                "label": column.label,
                "unit": column.unit,
                "precision": column.precision,
            }
            for column in table.columns
        ],
        "rows": [[_jsonable(value) for value in row] for row in table.rows],
        "selected_row": table.selected_row,
    }


def _overlay_payload(overlay: Overlay) -> dict[str, Any]:
    if isinstance(overlay, PointsOverlay):
        return {
            "kind": overlay.kind,
            "frame": overlay.frame,
            "size": _coord(overlay.size),
            "points": [
                {
                    "x": _coord(point.x),
                    "y": _coord(point.y),
                    "label": point.label,
                    "role": point.role,
                }
                for point in overlay.points
            ],
        }
    if isinstance(overlay, PolylineOverlay):
        return {
            "kind": overlay.kind,
            "frame": overlay.frame,
            "role": overlay.role,
            "width": _coord(overlay.width),
            "closed": overlay.closed,
            "points": _points(overlay.points),
        }
    if isinstance(overlay, RayOverlay):
        return {
            "kind": overlay.kind,
            "frame": overlay.frame,
            "role": overlay.role,
            "width": _coord(overlay.width),
            "origin": _point(overlay.origin),
            "angle_rad": _coord(overlay.angle_rad),
            "range_m": _coord(overlay.range_m),
        }
    if isinstance(overlay, SectorOverlay):
        return {
            "kind": overlay.kind,
            "frame": overlay.frame,
            "role": overlay.role,
            "opacity": _coord(overlay.opacity),
            "center": _point(overlay.center),
            "start_angle_rad": _coord(overlay.start_angle_rad),
            "end_angle_rad": _coord(overlay.end_angle_rad),
            "range_m": _coord(overlay.range_m),
        }
    return {
        "kind": overlay.kind,
        "frame": overlay.frame,
        "role": overlay.role,
        "position": _point(overlay.position),
        "text": overlay.text,
    }


__all__ = [
    "Band",
    "Column",
    "DebugFrame",
    "Frame",
    "LabelOverlay",
    "Marker",
    "Overlay",
    "Plot",
    "PointMarker",
    "PointsOverlay",
    "PolylineOverlay",
    "RayOverlay",
    "Readout",
    "Role",
    "Scalar",
    "SectorOverlay",
    "Series",
    "Table",
]
