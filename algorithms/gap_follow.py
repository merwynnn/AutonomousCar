"""Contrôleur « trou le plus dégagé » pour le dashboard.

Ce module est une **façade** : la logique de conduite vit dans
``algorithms/gap_policy.py`` (``StructuredGapPolicy``, la version B1
entraînable par CMA-ES). Ici, on se contente de :

1. construire la politique B1 avec ses paramètres par défaut (``B1_DEFAULT``) ;
2. adapter sa signature ``act(observation)`` au contrat du registre
   ``update(observation, speed_limit)`` — le ``speed_limit`` du tableau de
   bord est volontairement ignoré, comme pour ``LearnedGapController`` ;
3. traduire les sorties de la politique en ``GapDebug`` riches pour les
   panneaux de débogage du dashboard (profils, scores, candidats, overlays).

Ainsi il n'existe qu'une seule implémentation de l'algorithme : toute
amélioration de ``StructuredGapPolicy`` profite automatiquement au contrôleur
de démonstration, au training CMA-ES et au warm start B2.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from onboard.api import Actuation, Observation

from algorithms.debug import (
    Band,
    Column,
    DebugFrame,
    Marker,
    Overlay,
    Plot,
    PointMarker,
    PointsOverlay,
    PolylineOverlay,
    RayOverlay,
    Readout,
    Role,
    SectorOverlay,
    Series,
    Table,
)
from algorithms.gap_policy import StructuredGapPolicy


@dataclass(frozen=True)
class GapCandidate:
    """Un faisceau candidat scoré, exposé uniquement pour le débogage."""

    rank: int
    index: int
    angle_rad: float
    raw_range_m: float
    smoothed_range_m: float
    clearance_m: float
    angle_change_penalty_m: float
    free_fraction: float
    score: float
    in_gap: bool
    selected: bool


@dataclass
class GapDebug:
    """État de débogage du suivi de trou, visualisé dans le dashboard.

    Ces données décrivent ce que la politique « voit » (scan, dégagement,
    scores) et pourquoi elle choisit une direction et une vitesse.
    """

    angle_rad: float = 0.0  # Direction visée, avant lissage (radians)
    steering_rad: float = 0.0  # Direction réelle après lissage (radians)
    range_m: float = 0.0  # Distance lissée au faisceau choisi (mètres)
    clearance_m: float = 0.0  # Dégagement minimal dans la fenêtre (mètres)
    gap_width_rad: float = 0.0  # Largeur angulaire du trou (radians)
    gap_left_rad: float = 0.0
    gap_right_rad: float = 0.0
    front_sector_rad: float = 0.0
    throttle: float = 0.0
    min_clearance_m: float = 0.0
    valid: bool = False  # Indique que la commande a pu être calculée
    profile: np.ndarray = field(
        default_factory=lambda: np.zeros((0, 2), dtype=float)
    )
    # --- Données de décision pour le panneau de débogage générique ---------
    status: str = "IDLE"
    detail: str = ""
    best_bearing_rad: float = 0.0
    target_bearing_rad: float = 0.0
    threshold_m: float = 0.0
    window_beams: int = 0
    beam_count: int = 0
    scan_t: float = 0.0
    target_speed_mps: float = 0.0
    gap_detected: bool = False
    angles_rad: np.ndarray = field(
        default_factory=lambda: np.zeros(0, dtype=float)
    )
    raw_ranges_m: np.ndarray = field(
        default_factory=lambda: np.zeros(0, dtype=float)
    )
    smoothed_ranges_m: np.ndarray = field(
        default_factory=lambda: np.zeros(0, dtype=float)
    )
    clearance_profile_m: np.ndarray = field(
        default_factory=lambda: np.zeros(0, dtype=float)
    )
    scores: np.ndarray = field(
        default_factory=lambda: np.zeros(0, dtype=float)
    )
    penalty_m: np.ndarray = field(
        default_factory=lambda: np.zeros(0, dtype=float)
    )
    free_mask: np.ndarray = field(
        default_factory=lambda: np.zeros(0, dtype=bool)
    )
    chosen_index: int = -1
    candidates: tuple[GapCandidate, ...] = ()


def _penalty_profile(
    policy: StructuredGapPolicy,
    angles: np.ndarray,
) -> np.ndarray:
    """Termes de pénalité « doux » du score, par faisceau.

    Le score B1 est ``récompenses - pénalités`` ; les panneaux affichent la
    part de pénalité ( changement de direction + biais vers l'avant ) séparément
    pour expliquer pourquoi un faisceau dégagé peut ne pas être choisi.
    """

    p = policy.params
    return (
        p.angle_change_weight * np.abs(angles - policy.last_angle)
        + p.center_bias_weight * np.abs(angles)
    )


def _speed_reason(policy: StructuredGapPolicy, clearance_m: float) -> str:
    """Étiquette courte expliquant le palier de vitesse choisi."""

    p = policy.params
    if clearance_m < p.clearance_slow_1_m:
        return "tight clearance"
    if clearance_m < p.clearance_slow_2_m:
        return "cautious clearance"
    return "open"


def _sample_front(
    angles: np.ndarray,
    front: np.ndarray,
    arrays: dict[str, np.ndarray],
    max_points: int = 90,
) -> dict[str, np.ndarray]:
    """Réduit le secteur avant à ~``max_points`` points pour les graphes.

    Les calculs (scoring, candidats) restent en pleine résolution ; seul le
    rendu est décimé pour garder les panneaux lisibles et légers.
    """

    front_indices = np.flatnonzero(front)
    step = max(1, front_indices.size // max_points)
    sampled = front_indices[::step]
    return {name: np.asarray(values)[sampled] for name, values in arrays.items()}


def _build_candidates(
    angles: np.ndarray,
    ranges: np.ndarray,
    work: np.ndarray,
    clearance: np.ndarray,
    free: np.ndarray,
    free_fraction: np.ndarray,
    scores: np.ndarray,
    penalty: np.ndarray,
    best: int,
    limit: int = 8,
) -> tuple[GapCandidate, ...]:
    """Classe les meilleurs faisceaux du secteur avant par score décroissant."""

    front_indices = np.flatnonzero(np.isfinite(scores))
    ranked = front_indices[np.argsort(scores[front_indices])[::-1]]
    candidates = []
    for rank, index in enumerate(ranked[:limit]):
        candidates.append(
            GapCandidate(
                rank=rank + 1,
                index=int(index),
                angle_rad=float(angles[index]),
                raw_range_m=float(ranges[index]),
                smoothed_range_m=float(work[index]),
                clearance_m=float(clearance[index]),
                angle_change_penalty_m=float(penalty[index]),
                free_fraction=float(free_fraction[index]),
                score=float(scores[index]),
                in_gap=bool(free[index]),
                selected=bool(index == best),
            )
        )
    return tuple(candidates)


class FurthestGapController:
    """Contrôleur de suivi de trou : façade de ``StructuredGapPolicy``.

    L'algorithme (résumé) :

    1. **Filtrage** : seules les mesures du secteur avant (±``front_sector_deg``)
       sont conservées.
    2. **Lissage** : moyenne mobile sur ``smoothing_beams`` faisceaux pour
       réduire le bruit du lidar.
    3. **Dégagement** : pour chaque faisceau, distance minimale dans une
       fenêtre angulaire — une estimation « pessimiste » de l'espace libre,
       qui ne rate pas un obstacle juste à côté de la direction visée.
    4. **Scoring** : chaque direction est notée par une somme pondérée
       (dégagement, portée, largeur de trou) moins des pénalités
       (changement de direction, écart à l'avant).
    5. **Commandes** : le faisceau gagnant donne la direction (lissée par un
       filtre passe-bas) ; la vitesse suit un palier qui ralentit quand le
       dégagement diminue, et se courbe selon le virage en cours.

    Le comportement par défaut est le preset « mur » de B1 (``B1_DEFAULT``) :
    c'est la même politique que celle optimisée par CMA-ES.
    """

    def __init__(self, policy: StructuredGapPolicy | None = None) -> None:
        # Politique unique : par défaut, paramètres B1 hand-tunés.
        self.policy = policy if policy is not None else StructuredGapPolicy()
        self.debug = GapDebug(front_sector_rad=np.radians(
            self.policy.params.front_sector_deg
        ))

    def reset(self) -> None:
        """Remet à zéro l'état interne (direction, mémoire, débogage)."""

        self.policy.reset()
        self.debug = GapDebug(front_sector_rad=np.radians(
            self.policy.params.front_sector_deg
        ))

    def update(self, observation: Observation, speed_limit: float = 1.5) -> Actuation:
        """Un pas de contrôle : lit les capteurs, renvoie les commandes.

        Args:
            observation: L'état des capteurs (seul le lidar est utilisé).
            speed_limit: Ignoré. La politique B1 possède son propre plan de
                vitesse (paliers de dégagement + limite en virage), cohérent
                avec ``LearnedGapController`` ; le curseur du dashboard n'est
                pas un plafond dur.

        Returns:
            Actuation: ``throttle`` entre 0 et 1, ``steering`` en radians
            (positif = gauche).
        """

        scan = observation.lidar_current
        if scan is None:
            self.debug = GapDebug(
                valid=False,
                status="NO SCAN",
                detail="lidar unavailable",
                front_sector_rad=np.radians(self.policy.params.front_sector_deg),
            )
            return self.policy.act(observation)

        # 1. Surface de score AVANT d'agir : elle reflète exactement ce que la
        #    politique va voir (la pénalité de changement de direction utilise
        #    encore la direction précédente).
        surface = self.policy.score_surface(observation)
        penalty = None
        if surface is not None:
            penalty = _penalty_profile(
                self.policy, np.asarray(surface["angles"], dtype=float)
            )

        # 2. La politique unique décide : direction lissée + plan de vitesse.
        command = self.policy.act(observation)

        # 3. Traduction en débogage pour le dashboard.
        self.debug = self._build_debug(observation, command, surface, scan, penalty)
        return command

    def _build_debug(
        self,
        observation: Observation,
        command: Actuation,
        surface: dict | None,
        scan,
        penalty: np.ndarray | None,
    ) -> GapDebug:
        """Construit le ``GapDebug`` à partir de la surface de score B1."""

        front_sector_rad = np.radians(self.policy.params.front_sector_deg)
        if surface is None or not self.policy.last_debug.get("valid", 0.0):
            return GapDebug(
                valid=False,
                status="NO FRONT",
                detail=(
                    "no valid lidar returns within "
                    f"±{np.degrees(front_sector_rad):.0f}°"
                ),
                front_sector_rad=float(front_sector_rad),
                beam_count=scan.beam_count,
                scan_t=float(scan.t_measure),
            )

        # Champs calculés par la politique au dernier tick.
        info = self.policy.last_debug
        best = int(np.argmax(np.asarray(surface["scores"], dtype=float)))
        angles = np.asarray(surface["angles"], dtype=float)
        front = np.asarray(surface["front"], dtype=bool)
        work = np.asarray(surface["work"], dtype=float)
        clearance = np.asarray(surface["clearance"], dtype=float)
        free = np.asarray(surface["free"], dtype=bool)
        scores = np.asarray(surface["scores"], dtype=float)
        threshold = float(surface["threshold"])
        window = int(surface["window"])
        ranges = np.asarray(scan.ranges_m, dtype=float)
        assert penalty is not None

        # Limites du trou contigu autour du faisceau choisi.
        lo = best
        hi = best
        while lo > 0 and free[lo - 1]:
            lo -= 1
        while hi < free.size - 1 and free[hi + 1]:
            hi += 1
        gap_left = float(min(angles[lo], angles[hi]))
        gap_right = float(max(angles[lo], angles[hi]))
        gap_width = float(abs(angles[hi] - angles[lo]))

        # Séries décimées du secteur avant pour les graphes.
        sampled = _sample_front(
            angles,
            front,
            {
                "angles": angles,
                "raw": ranges,
                "smoothed": work,
                "clearance": clearance,
                "scores": scores,
                "penalty": penalty,
                "free": free,
            },
        )
        profile = np.column_stack([sampled["angles"], sampled["clearance"]])
        gap_detected = bool(free[best])
        best_clearance = float(clearance[best])

        return GapDebug(
            angle_rad=float(info.get("target_bearing_rad", 0.0)),
            steering_rad=float(command.steering),
            range_m=float(work[best]),
            clearance_m=best_clearance,
            gap_width_rad=gap_width,
            gap_left_rad=gap_left,
            gap_right_rad=gap_right,
            front_sector_rad=float(front_sector_rad),
            throttle=float(command.throttle),
            min_clearance_m=float(clearance[front].min()),
            valid=True,
            profile=profile,
            status="TRACKING" if gap_detected else "FALLBACK",
            detail=(
                f"{np.degrees(gap_width):.0f}° opening, "
                f"speed: {_speed_reason(self.policy, best_clearance)}"
                if gap_detected
                else "no contiguous free opening; width estimated from window"
            ),
            best_bearing_rad=float(info.get("best_bearing_rad", 0.0)),
            target_bearing_rad=float(info.get("target_bearing_rad", 0.0)),
            threshold_m=threshold,
            window_beams=window,
            beam_count=scan.beam_count,
            scan_t=float(scan.t_measure),
            target_speed_mps=float(info.get("target_speed_mps", 0.0)),
            gap_detected=gap_detected,
            angles_rad=sampled["angles"],
            raw_ranges_m=sampled["raw"],
            smoothed_ranges_m=sampled["smoothed"],
            clearance_profile_m=sampled["clearance"],
            scores=sampled["scores"],
            penalty_m=sampled["penalty"],
            free_mask=sampled["free"],
            chosen_index=best,
            candidates=_build_candidates(
                angles,
                ranges,
                work,
                clearance,
                free,
                np.asarray(surface["free_fraction"], dtype=float),
                scores,
                penalty,
                best,
            ),
        )

    def debug_frame(self) -> DebugFrame:
        """Construit l'instantané de débogage pour le dashboard."""

        debug = self.debug
        if not debug.valid:
            return DebugFrame(
                controller="gap_follow",
                title="Gap solver",
                t=debug.scan_t,
                status=debug.status,
                valid=False,
                readouts=(
                    Readout(
                        "front",
                        "Front sector",
                        float(np.degrees(debug.front_sector_rad)),
                        "deg",
                        0,
                    ),
                ),
                meta={"detail": debug.detail},
            )

        angles_deg = np.degrees(debug.angles_rad)
        center_deg = float(np.degrees(debug.best_bearing_rad))
        target_deg = float(np.degrees(debug.target_bearing_rad))
        left_deg = float(np.degrees(debug.gap_left_rad))
        right_deg = float(np.degrees(debug.gap_right_rad))
        profile_series = (
            Series(
                "raw",
                "raw",
                _profile_points(angles_deg, debug.raw_ranges_m),
                role="muted",
            ),
            Series(
                "smoothed",
                "smoothed",
                _profile_points(angles_deg, debug.smoothed_ranges_m),
                role="secondary",
            ),
            Series(
                "clearance",
                "clearance",
                _profile_points(angles_deg, debug.clearance_profile_m),
                role="primary",
            ),
        )
        bands = (Band(left_deg, right_deg, "opening", "secondary"),)
        selected_row = next(
            (
                index
                for index, candidate in enumerate(debug.candidates)
                if candidate.selected
            ),
            None,
        )
        candidate_rows = tuple(
            (
                candidate.rank,
                float(np.degrees(candidate.angle_rad)),
                candidate.raw_range_m,
                candidate.clearance_m,
                candidate.score,
                "yes" if candidate.in_gap else "no",
            )
            for candidate in debug.candidates[:6]
        )
        gap_range = min(max(debug.range_m, 0.45), 2.0)
        sector_deg = float(np.degrees(debug.front_sector_rad))
        arc = _arc_points(debug.gap_left_rad, debug.gap_right_rad, gap_range)
        overlay_role: Role = "secondary" if debug.gap_detected else "muted"
        overlays: list[Overlay] = [
            SectorOverlay(
                start_angle_rad=debug.gap_left_rad,
                end_angle_rad=debug.gap_right_rad,
                range_m=gap_range,
                role=overlay_role,
                opacity=0.08,
            ),
            PolylineOverlay(
                points=((0.0, 0.0), *arc, (0.0, 0.0)),
                role=overlay_role,
                width=1.2,
            ),
        ]
        if abs(debug.best_bearing_rad - debug.target_bearing_rad) > 0.01:
            overlays.append(
                RayOverlay(
                    angle_rad=debug.best_bearing_rad,
                    range_m=gap_range,
                    role="warning",
                    width=1.1,
                )
            )
        overlays.append(
            RayOverlay(
                angle_rad=debug.target_bearing_rad,
                range_m=gap_range,
                role="accent",
                width=1.8,
            )
        )
        for candidate in _spread_candidates(debug.candidates):
            overlays.append(
                PointsOverlay(
                    points=(
                        PointMarker(
                            x=float(np.cos(candidate.angle_rad) * gap_range),
                            y=float(np.sin(candidate.angle_rad) * gap_range),
                            label=f"#{candidate.rank}",
                            role="accent" if candidate.selected else "candidate",
                        ),
                    ),
                    size=0.05,
                )
            )

        return DebugFrame(
            controller="gap_follow",
            title="Gap solver",
            t=debug.scan_t,
            status=debug.status,
            valid=True,
            readouts=(
                Readout("best", "Best", center_deg, "deg", 1),
                Readout("target", "Target", target_deg, "deg", 1),
                Readout(
                    "steering",
                    "Steer",
                    float(np.degrees(debug.steering_rad)),
                    "deg",
                    1,
                ),
                Readout("opening", "Opening", float(np.degrees(debug.gap_width_rad)), "deg", 1),
                Readout("threshold", "Free min", debug.threshold_m, "m", 2),
                Readout("speed", "Speed", debug.target_speed_mps, "m/s", 2),
                Readout("throttle", "Throttle", debug.throttle * 100.0, "%", 0),
                Readout("window", "Window", debug.window_beams, "beams", 0),
            ),
            plots=(
                Plot(
                    id="profile",
                    title="Range profile",
                    series=profile_series,
                    x_label="bearing",
                    y_label="distance",
                    x_unit="deg",
                    y_unit="m",
                    x_range=(-sector_deg, sector_deg),
                    markers=(
                        Marker("x", center_deg, "best", "solid"),
                        Marker("y", debug.threshold_m, "free", "dashed"),
                    ),
                    bands=bands,
                ),
                Plot(
                    id="score",
                    title="Score",
                    series=(
                        Series(
                            "score",
                            "score",
                            _profile_points(angles_deg, debug.scores),
                            role="primary",
                        ),
                        Series(
                            "penalty",
                            "penalty",
                            _profile_points(
                                angles_deg,
                                debug.penalty_m,
                            ),
                            role="warning",
                        ),
                    ),
                    x_label="bearing",
                    y_label="score",
                    x_unit="deg",
                    y_unit="m",
                    x_range=(-sector_deg, sector_deg),
                    markers=(Marker("x", center_deg, "best", "solid"),),
                    bands=bands,
                ),
            ),
            tables=(
                Table(
                    id="candidates",
                    title="Candidates",
                    columns=(
                        Column("rank", "#", "", 0),
                        Column("bearing", "Brg", "deg", 1),
                        Column("range", "Range", "m", 2),
                        Column("clearance", "Clr", "m", 2),
                        Column("score", "Score", "m", 2),
                        Column("gap", "Gap", "", 0),
                    ),
                    rows=candidate_rows,
                    selected_row=selected_row,
                ),
            ),
            overlays=tuple(overlays),
            meta={
                "detail": debug.detail,
                "gap_detected": debug.gap_detected,
                "beam_count": debug.beam_count,
                "chosen_index": debug.chosen_index,
            },
        )


def _profile_points(
    angles_deg: np.ndarray,
    values: np.ndarray,
) -> tuple[tuple[float, float], ...]:
    return tuple(
        (float(angle), float(value))
        for angle, value in zip(angles_deg, values, strict=True)
    )


def _arc_points(
    start_rad: float,
    end_rad: float,
    radius: float,
    steps: int = 20,
) -> tuple[tuple[float, float], ...]:
    angles = np.linspace(start_rad, end_rad, steps)
    return tuple(
        (float(np.cos(angle) * radius), float(np.sin(angle) * radius))
        for angle in angles
    )


def _spread_candidates(
    candidates: tuple[GapCandidate, ...],
    limit: int = 3,
    min_separation_rad: float = 0.16,
) -> tuple[GapCandidate, ...]:
    """Garde quelques candidats bien séparés pour garder la vue 3D lisible."""

    selected: list[GapCandidate] = []
    for candidate in candidates:
        if all(
            abs(candidate.angle_rad - other.angle_rad) >= min_separation_rad
            for other in selected
        ):
            selected.append(candidate)
        if len(selected) >= limit:
            break
    return tuple(selected)
