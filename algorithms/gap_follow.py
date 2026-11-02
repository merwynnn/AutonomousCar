"""Lidar-only furthest-gap controller for the dashboard demo.

This is a normal algorithm: it sees only ``Observation`` and returns
``Actuation``. It uses no simulator internals and no ground-truth pose.
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


@dataclass(frozen=True)
class GapCandidate:
    """One scored beam candidate, exposed only for debugging."""

    rank: int
    index: int
    angle_rad: float
    raw_range_m: float
    smoothed_range_m: float
    clearance_m: float
    angle_change_penalty_m: float
    score: float
    in_gap: bool
    selected: bool


@dataclass
class GapDebug:
    """Informations de débogage pour l'algorithme de suivi de trou, visualisées dans le dashboard.

    Ces données permettent de comprendre ce que l'algorithme "voit" et pourquoi il prend
    certaines décisions de direction et de vitesse.
    """
    angle_rad: float = 0.0  # Angle de direction souhaité (dans le repère du véhicule, radians)
    steering_rad: float = 0.0  # Commande de direction réelle après lissage (radians)
    range_m: float = 0.0  # Distance mesurée au centre du trou choisi (mètres)
    clearance_m: float = 0.0  # Distance minimale dans la fenêtre de dégagement (mètres)
    gap_width_rad: float = 0.0  # Largeur angulaire du trou détecté (radians)
    gap_left_rad: float = 0.0
    gap_right_rad: float = 0.0
    front_sector_rad: float = 0.0
    throttle: float = 0.0
    min_clearance_m: float = 0.0
    valid: bool = False  # Indique que la commande a pu être calculée
    profile: np.ndarray = field(
        default_factory=lambda: np.zeros((0, 2), dtype=float)
    )
    # --- richer decision data for the generic debug panel -----------------
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


class FurthestGapController:
    """Contrôleur d'évitement d'obstacles basé sur le lidar qui dirige vers le trou le plus large et dégagé.

    L'idée principale : le lidar mesure les distances autour du véhicule. L'algorithme cherche
    la direction avec le plus d'espace libre (un "trou" entre les obstacles) et tourne vers cette
    direction tout en ajustant la vitesse selon la proximité des obstacles.

    L'algorithme fonctionne en plusieurs étapes :
    1. Filtrage : on ne garde que les mesures du lidar dans le secteur avant (±front_sector_deg)
    2. Lissage : on applique une moyenne mobile sur les distances pour réduire le bruit
    3. Dégagement : pour chaque faisceau, on calcule la distance minimale dans une fenêtre angulaire
    4. Scoring : on note chaque direction en fonction du dégagement, avec une légère préférence
       pour la direction précédente (pour éviter de zigzaguer)
    5. Largeur du trou : on mesure la largeur angulaire de l'espace libre autour de la meilleure direction
    6. Commandes : on lisse la direction et on réduit la vitesse si les obstacles sont proches
    """

    def __init__(
        self,
        max_steering: float = 0.50,
        front_sector_deg: float = 100.0,
        clearance_window_deg: float = 60.0,
        min_gap_m: float = 0.60,
        max_speed_for_throttle: float = 5.5,
        angle_change_weight: float = 0.15,
    ) -> None:
        """Initialise le contrôleur de suivi de trou.

        Args:
            max_steering: Angle de direction maximum en radians (la commande sera limitée à cette valeur).
                          Par exemple 0.50 rad ≈ 28.6°.
            front_sector_deg: Secteur angulaire devant le véhicule (en degrés, ±) à considérer pour
                              la détection de trous. Par exemple 100° signifie de -100° à +100°.
            clearance_window_deg: Taille de la fenêtre (en degrés) utilisée pour calculer le dégagement
                                  minimal. Plus cette valeur est grande, plus l'algorithme est prudent.
            min_gap_m: Distance minimale (en mètres) qu'un faisceau doit dégager pour être
                       considéré comme libre. C'est un seuil de portée, pas une largeur
                       métrique de trou.
            max_speed_for_throttle: Vitesse (en m/s) à laquelle l'accélérateur atteint 100%. Au-delà,
                                    la vitesse est limitée par ce paramètre.
            angle_change_weight: Poids de la pénalité de changement d'angle, en mètres par radian.
                                 Plus il est grand, plus l'algorithme préfère garder la direction précédente.
        """
        self.max_steering = max_steering
        self.front_sector = np.radians(front_sector_deg)
        self.clearance_window_deg = clearance_window_deg
        self.min_gap_m = min_gap_m
        self.max_speed_for_throttle = max_speed_for_throttle
        self.angle_change_weight = angle_change_weight
        self.steering = 0.0
        self.last_angle = 0.0
        self.debug = GapDebug()

    def reset(self) -> None:
        """Réinitialise l'état interne du contrôleur pour un nouveau run.

        Cette méthode est appelée au début de chaque simulation pour effacer la mémoire
        de l'algorithme (direction précédente, informations de débogage). Cela garantit
        que chaque run commence dans le même état.
        """
        self.steering = 0.0
        self.last_angle = 0.0
        self.debug = GapDebug()

    def update(self, observation: Observation, speed_limit: float = 1.5) -> Actuation:
        """Calcule les commandes (accélérateur et direction) à partir des observations des capteurs.

        Cette méthode est appelée à chaque pas de temps de la simulation (60 fois par seconde).
        Elle analyse les données du lidar pour décider vers où tourner et à quelle vitesse avancer.

        Args:
            observation: Contient toutes les lectures des capteurs (lidar, odométrie, etc.).
                        Seul le lidar est utilisé par cet algorithme.
            speed_limit: Vitesse maximale autorisée en m/s (définie dans la configuration du scénario).
                         L'algorithme peut choisir d'aller moins vite si les obstacles sont proches.

        Returns:
            Actuation: Un objet contenant deux commandes :
                       - throttle: valeur entre 0 et 1 (0 = arrêt, 1 = accélération maximale)
                       - steering: angle de direction en radians (positif = gauche, négatif = droite)
        """
        # Récupération du scan dans le repère lidar courant. `lidar_current`
        # utilise la compensation d'odométrie quand elle est disponible et
        # retombe sur le scan brut sinon.
        scan = observation.lidar_current
        if scan is None:
            # Si pas de lidar, on s'arrête
            self.debug = GapDebug(
                valid=False,
                status="NO SCAN",
                detail="lidar unavailable",
                front_sector_rad=float(self.front_sector),
            )
            return Actuation(throttle=0.0, steering=self.steering)

        # dt est le temps écoulé depuis le dernier appel (en secondes)
        # On utilise une valeur minimale pour éviter les divisions par zéro
        dt = max(observation.dt, 1e-4)

        # Conversion des données du lidar en tableaux NumPy pour faciliter les calculs
        angles = np.asarray(scan.angles_rad, dtype=float)  # Angles des faisceaux (radians)
        ranges = np.asarray(scan.ranges_m, dtype=float)   # Distances mesurées (mètres)



        # Filtrage des mesures valides :
        # - scan.valid : le capteur a renvoyé une mesure
        # - np.isfinite : la distance n'est pas infinie ou NaN
        # - ranges > 0.05 : on ignore les mesures trop proches (bruit du capteur)
        valid = scan.valid & np.isfinite(ranges) & (ranges > 0.05)

        # On ne garde que les mesures dans le secteur avant du véhicule
        # np.abs(angles) donne la valeur absolue de l'angle
        # self.front_sector est la limite angulaire (par exemple 0.87 rad pour 50°)
        front = valid & (np.abs(angles) <= self.front_sector)

        # Si aucune mesure valide dans le secteur avant, on s'arrête
        if not np.any(front):
            self.debug = GapDebug(
                valid=False,
                status="NO FRONT",
                detail=(
                    "no valid lidar returns within "
                    f"±{np.degrees(self.front_sector):.0f}°"
                ),
                front_sector_rad=float(self.front_sector),
                beam_count=scan.beam_count,
                scan_t=float(scan.t_measure),
            )
            return Actuation(throttle=0.0, steering=0.0)

        # Remplacement des distances hors secteur par la distance maximale
        # Cela permet d'ignorer les angles morts et de ne pas fausser les calculs
        open_range = float(np.nanmax(ranges[front]))
        work = np.where(front, ranges, open_range)

        # Lissage des distances avec une moyenne mobile sur 7 faisceaux
        # Le lidar a du bruit (mesures qui varient légèrement), donc on moyenne
        # pour obtenir une estimation plus stable de la distance réelle
        kernel = np.ones(7) / 7.0  # Noyau de convolution : 7 valeurs qui s'additionnent à 1
        # On ajoute 3 valeurs de chaque côté pour gérer les bords (padding)
        padded = np.concatenate([work[-3:], work, work[:3]])
        # np.convolve applique le noyau de convolution (moyenne mobile)
        # mode="same" garde la même taille de tableau
        # [3:-3] supprime le padding ajouté
        work = np.convolve(padded, kernel, mode="same")[3:-3]

        # Calcul du dégagement minimal dans une fenêtre glissante
        # L'idée : pour chaque direction, on regarde la distance minimale dans un secteur angulaire
        # Cela donne une estimation "pessimiste" mais sûre de l'espace disponible
        # Si on regarde juste la distance dans une direction, on pourrait manquer un obstacle juste à côté

        # step : écart angulaire moyen entre deux faisceaux consécutifs (en radians)
        # np.diff calcule les différences entre angles consécutifs
        # np.median prend la valeur médiane pour être robuste aux variations
        step = abs(float(np.median(np.diff(angles))))

        # window : nombre de faisceaux dans la fenêtre de dégagement
        # On convertit l'angle (degrés) en radians, puis on divise par l'écart entre faisceaux
        window = max(1, int(round(np.radians(self.clearance_window_deg) / step)))

        # On préfère une fenêtre de taille impaire pour qu'elle soit centrée sur un faisceau
        if window % 2 == 0:
            window += 1

        # On limite la taille de la fenêtre pour ne pas dépasser le nombre de faisceaux disponibles
        window = min(window, angles.size if angles.size % 2 == 1 else angles.size - 1)

        # pad : nombre de faisceaux à ajouter de chaque côté pour gérer les bords
        pad = window // 2

        # On ajoute du padding pour que la fenêtre glissante puisse fonctionner sur les bords
        # On recopie les valeurs du début à la fin et vice versa (padding circulaire)
        padded_work = (
            np.concatenate([work[-pad:], work, work[:pad]]) if pad > 0 else work
        )

        # sliding_window_view crée une vue où chaque ligne est une fenêtre de taille 'window'
        # C'est très efficace car cela ne copie pas les données, cela crée juste une vue
        windows = np.lib.stride_tricks.sliding_window_view(padded_work, window)

        # clearance : pour chaque fenêtre, on prend la distance minimale
        # C'est le "pire cas" dans ce secteur angulaire
        clearance = windows.min(axis=1)

        # Calcul des scores pour chaque direction
        # On veut maximiser le dégagement (clearance) mais on préfère légèrement garder
        # la direction précédente pour éviter que le véhicule zigzague constamment
        # angle_change_weight est un coefficient empirique : plus il est grand, plus on préfère la direction précédente
        angle_change_penalty = self.angle_change_weight * 3* np.abs(angles - self.last_angle)
        scores = clearance - angle_change_penalty

        # On met un score très bas (-inf) pour les directions hors secteur avant
        # Cela garantit qu'elles ne seront jamais choisies
        scores[~front] = -np.inf

        # On choisit la direction avec le score maximum
        best = int(np.argmax(scores))
        best_bearing = float(angles[best])  # Direction du meilleur faisceau
        desired = best_bearing  # Angle souhaité (direction vers laquelle tourner)
        best_clearance = float(clearance[best])  # Dégagement dans cette direction
        best_range = float(work[best])  # Distance brute à cet angle

        # Mise à jour de l'angle précédent pour la pénalité de changement d'angle
        self.last_angle = best_bearing

        # Mesure de la largeur du trou en détectant l'espace libre contigu autour du meilleur faisceau
        # Un "trou" est une zone contiguë où toutes les distances sont supérieures à un seuil
        # Le seuil est le maximum entre : min_gap_m (portée minimum) et 75% du meilleur dégagement
        # Cela permet d'adapter le seuil à la situation : plus de dégagement = seuil plus élevé
        threshold = max(self.min_gap_m, 0.75 * best_clearance)
        free = front & (work >= threshold)
        gap_detected = bool(free[best])

        if free[best]:
            # Si le meilleur faisceau est dans une zone libre, on étend vers la gauche et la droite
            # pour trouver les limites du trou
            lo = best  # Limite gauche (index)
            hi = best  # Limite droite (index)

            # On étend vers la gauche tant qu'il y a de l'espace libre
            while lo > 0 and free[lo - 1]:
                lo -= 1

            # On étend vers la droite tant qu'il y a de l'espace libre
            while hi < free.size - 1 and free[hi + 1]:
                hi += 1

            # La largeur du trou est la différence d'angle entre les limites
            gap_width = float(abs(angles[hi] - angles[lo]))
        else:
            # Si le meilleur faisceau n'est pas dans une zone libre (cas rare),
            # on estime la largeur avec la taille de la fenêtre de dégagement
            gap_width = float(window * abs(step))
            lo = max(0, best - window // 2)
            hi = min(angles.size - 1, best + window // 2)

        # Limitation de l'angle souhaité à la direction maximale du véhicule
        # On ne peut pas tourner plus que max_steering (limite mécanique)
        desired = max(-self.max_steering, min(self.max_steering, desired))

        # Lissage de la direction avec un filtre passe-bas
        # Au lieu de tourner brusquement vers la nouvelle direction, on tourne progressivement
        # smoothing est le coefficient de lissage :
        # - Si dt est petit (simulation rapide), smoothing est petit = changement lent
        # - Si dt est grand, smoothing approche 1 = changement rapide
        # 10.0 est un coefficient empirique qui détermine la rapidité du lissage
        smoothing = min(1.0, 10.0 * dt)

        # Mise à jour de la direction : on se rapproche de 'desired' proportionnellement à 'smoothing'
        # C'est une interpolation linéaire entre l'ancienne et la nouvelle direction
        self.steering += (desired - self.steering) * smoothing

        # On s'assure que la direction finale reste dans les limites
        self.steering = max(-self.max_steering, min(self.max_steering, self.steering))

        # Réduction de la vitesse lorsque le dégagement est faible
        # La vitesse de base est au minimum 0.45 m/s (pour que le véhicule bouge toujours un peu)
        # ou la limite de vitesse configurée, selon laquelle est plus grand
        speed = max(0.45, speed_limit)
        speed_reason = "open"

        # Si les obstacles sont très proches (dégagement < 0.50 m), on ralentit fortement
        if best_clearance < 0.50:
            speed = min(speed, 0.55)  # Vitesse maximale 0.55 m/s (très lent)
            speed_reason = "tight clearance"
        # Si les obstacles sont modérément proches (dégagement < 0.80 m), on ralentit un peu
        elif best_clearance < 0.80:
            speed = min(speed, 1.0)  # Vitesse maximale 1.0 m/s (lent)
            speed_reason = "cautious clearance"

        # Calcul de l'accélérateur (throttle) à partir de la vitesse cible
        # throttle est une valeur entre 0 et 1 qui représente la puissance du moteur
        # On divise la vitesse par max_speed_for_throttle pour normaliser
        # Si speed = max_speed_for_throttle, throttle = 1 (pleine puissance)
        # Si speed est plus petit, throttle est proportionnellement plus petit
        throttle = max(0.0, min(1.0, speed / self.max_speed_for_throttle))

        # Données échantillonnées pour le graphe : on ne garde que le secteur avant
        # et on réduit à ~90 points pour garder le panneau lisible et léger.
        front_indices = np.flatnonzero(front)
        sample_step = max(1, front_indices.size // 90)
        sample_indices = front_indices[::sample_step]
        sampled_angles = angles[sample_indices]
        sampled_raw = ranges[sample_indices]
        sampled_smoothed = work[sample_indices]
        sampled_clearance = clearance[sample_indices]
        sampled_scores = scores[sample_indices]
        sampled_penalty = angle_change_penalty[sample_indices]
        sampled_free = free[sample_indices]
        profile = np.column_stack([sampled_angles, sampled_clearance])

        # Classement des faisceaux candidats. Les candidats sont calculés sur
        # toute la résolution du scan, pas sur le profil décimé du graphe.
        ranked = front_indices[np.argsort(scores[front_indices])[::-1]]
        candidates = tuple(
            GapCandidate(
                rank=rank + 1,
                index=int(index),
                angle_rad=float(angles[index]),
                raw_range_m=float(ranges[index]),
                smoothed_range_m=float(work[index]),
                clearance_m=float(clearance[index]),
                angle_change_penalty_m=float(angle_change_penalty[index]),
                score=float(scores[index]),
                in_gap=bool(free[index]),
                selected=bool(index == best),
            )
            for rank, index in enumerate(ranked[:8])
        )

        # Mise à jour des informations de débogage pour la visualisation dans le dashboard
        # Ces données permettent de voir ce que l'algorithme a calculé
        # (direction, vitesse, trou choisi) et pourquoi.
        self.debug = GapDebug(
            angle_rad=desired,  # Direction souhaitée après limitation
            steering_rad=self.steering,  # Direction réelle après lissage
            range_m=best_range,  # Distance au centre du trou
            clearance_m=best_clearance,  # Dégagement minimal
            gap_width_rad=gap_width,  # Largeur angulaire du trou
            gap_left_rad=float(min(angles[lo], angles[hi])),
            gap_right_rad=float(max(angles[lo], angles[hi])),
            front_sector_rad=float(self.front_sector),
            throttle=throttle,
            min_clearance_m=float(clearance[front].min()),
            valid=True,  # Indique que le calcul a réussi
            profile=profile,
            status="TRACKING" if gap_detected else "FALLBACK",
            detail=(
                f"{np.degrees(gap_width):.0f}° opening, speed: {speed_reason}"
                if gap_detected
                else "no contiguous free opening; width estimated from window"
            ),
            best_bearing_rad=best_bearing,
            target_bearing_rad=desired,
            threshold_m=threshold,
            window_beams=window,
            beam_count=scan.beam_count,
            scan_t=float(scan.t_measure),
            target_speed_mps=speed,
            gap_detected=gap_detected,
            angles_rad=sampled_angles,
            raw_ranges_m=sampled_raw,
            smoothed_ranges_m=sampled_smoothed,
            clearance_profile_m=sampled_clearance,
            scores=sampled_scores,
            penalty_m=sampled_penalty,
            free_mask=sampled_free,
            chosen_index=best,
            candidates=candidates,
        )

        # Retour des commandes pour le véhicule
        return Actuation(throttle=throttle, steering=self.steering)

    def debug_frame(self) -> DebugFrame:
        """Build the dashboard-neutral debug snapshot for this controller."""

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
    """Keep only a few well-separated candidates so the 3D view stays readable."""

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
