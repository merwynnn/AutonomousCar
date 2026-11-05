"""Politique B1 : suivi de trou « structuré », paramétrable et entraînable.

Ce fichier contient l'unique implémentation de l'algorithme de suivi de trou
du projet (``StructuredGapPolicy``). Elle est utilisée telle quelle par :

- le contrôleur de démonstration du dashboard (``algorithms/gap_follow.py``) ;
- l'entraînement CMA-ES (``training/``), qui optimise le vecteur de
  paramètres ``B1Params`` ;
- le warm start B2 (``training/warmstart.py``), qui apprend à imiter la
  surface de score produite ici (``score_surface``) ;
- la sécurité de ``LearnedGapController`` (``algorithms/learned_gap.py``),
  qui bascule sur B1 quand la politique apprise est en difficulté.

Comme tout ce qui tourne sur le Raspberry Pi, ce module n'importe que NumPy
et ``onboard.api`` : aucune dépendance envers le simulateur ou le dashboard.

L'algorithme, en six étapes (voir ``StructuredGapPolicy.act``) :

1. **Lecture du scan** : récupération du lidar courant (compensé du mouvement
   si disponible) et filtrage des mesures invalides.
2. **Filtrage avant** : seuls les faisceaux dans le secteur avant
   (±``front_sector_deg``) participent — on ne conduit pas vers l'arrière.
3. **Lissage** : moyenne mobile circulaire sur ``smoothing_beams`` faisceaux
   pour absorber le bruit et les mesures manquantes du capteur.
4. **Dégagement** : pour chaque faisceau, distance minimale dans une fenêtre
   angulaire (``clearance_window_deg``). C'est une estimation « pessimiste »
   de l'espace disponible : un obstacle juste à côté de la direction visée
   abaisse son score.
5. **Scoring** : chaque direction reçoit un score = récompenses (dégagement,
   portée, largeur de trou) − pénalités (changement de direction, écart à
   l'avant). Le faisceau au score maximal est choisi.
6. **Commandes** : la direction est lissée par un filtre passe-bas ; la
   vitesse suit un palier qui ralentit quand le dégagement diminue, se
   relance avec la largeur du trou et se limite en virage.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from onboard.api import Actuation, Observation

# ---------------------------------------------------------------------------
# 1. Aides partagées pour les paramètres bornés
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class ParamSpec:
    """Bornes et valeur par défaut d'un paramètre physique.

    CMA-ES optimise un vecteur en unités normalisées [0, 1] ; ``ParamSpec``
    fait la conversion vers les unités physiques (degrés, mètres, m/s...).
    """

    name: str
    low: float
    high: float
    default: float


def _coerce_odd(value: float, upper: int | None = None) -> int:
    """Convertit ``value`` en un entier impair (taille de fenêtre centrée)."""

    count = max(1, int(round(value)))
    if count % 2 == 0:
        count += 1
    if upper is not None and count > upper:
        count = upper if upper % 2 == 1 else max(1, upper - 1)
    return count


def _clip(value: float, low: float, high: float) -> float:
    """Borne ``value`` dans [low, high]."""

    return float(min(high, max(low, value)))


def _sigmoid(value: float) -> float:
    """Sigmoïde numériquelement sûre (utilisé par B2, gardé ici pour partage)."""

    value = _clip(value, -60.0, 60.0)
    return float(1.0 / (1.0 + np.exp(-value)))


def _unit_to_params(vector: np.ndarray, specs: tuple[ParamSpec, ...]) -> np.ndarray:
    """Mappe un vecteur normalisé [0, 1] vers les bornes physiques."""

    vector = np.asarray(vector, dtype=float)
    low = np.array([spec.low for spec in specs], dtype=float)
    high = np.array([spec.high for spec in specs], dtype=float)
    return low + np.clip(vector, 0.0, 1.0) * (high - low)


def _params_to_unit(params: np.ndarray, specs: tuple[ParamSpec, ...]) -> np.ndarray:
    """Mappe des valeurs physiques vers le vecteur normalisé [0, 1]."""

    low = np.array([spec.low for spec in specs], dtype=float)
    high = np.array([spec.high for spec in specs], dtype=float)
    return np.clip((np.asarray(params, dtype=float) - low) / (high - low), 0.0, 1.0)


# ---------------------------------------------------------------------------
# 2. Paramètres B1 : le « génome » optimisé par CMA-ES
# ---------------------------------------------------------------------------

# Chaque entrée est une dimension de recherche : (nom, borne basse, borne
# haute, valeur par défaut). Les bornes restreignent CMA-ES à des valeurs
# conduisables (ex. une vitesse de base entre 0,8 et 4 m/s).
B1_SPECS: tuple[ParamSpec, ...] = (
    # --- Perception du scan ------------------------------------------------
    ParamSpec("front_sector_deg", 55.0, 135.0, 100.0),  # demi-secteur avant
    ParamSpec("clearance_window_deg", 20.0, 120.0, 60.0),  # fenêtre de dégagement
    ParamSpec("min_gap_m", 0.20, 1.20, 0.60),  # seuil « faisceau libre »
    ParamSpec("smoothing_beams", 1.0, 11.0, 7.0),  # largeur du lissage
    # --- Scoring des directions ---------------------------------------------
    ParamSpec("angle_change_weight", 0.0, 1.5, 0.45),  # pénalité de zigzag
    ParamSpec("clearance_weight", 0.0, 2.0, 1.0),  # récompense de dégagement
    ParamSpec("range_weight", 0.0, 1.0, 0.0),  # récompense de portée brute
    ParamSpec("gap_width_weight", 0.0, 2.0, 0.50),  # récompense de largeur
    ParamSpec("center_bias_weight", 0.0, 1.0, 0.0),  # préférence pour l'avant
    ParamSpec("aim_blend", 0.0, 1.0, 0.0),  # beam gagnant <-> centre du trou
    # --- Direction -----------------------------------------------------------
    ParamSpec("max_steering_rad", 0.25, 0.49, 0.49),  # butée mécanique
    ParamSpec("steer_smoothing", 2.0, 30.0, 10.0),  # raideur du passe-bas
    ParamSpec("steer_gain", 0.50, 2.0, 1.0),  # gain angle -> volant
    # --- Vitesse -------------------------------------------------------------
    ParamSpec("base_speed_mps", 0.80, 4.0, 2.20),  # vitesse de croisière
    ParamSpec("clearance_slow_1_m", 0.20, 1.0, 0.55),  # seuil « très proche »
    ParamSpec("clearance_slow_1_speed", 0.30, 2.0, 0.50),  # vitesse « très proche »
    ParamSpec("clearance_slow_2_m", 0.50, 2.0, 0.90),  # seuil « proche »
    ParamSpec("clearance_slow_2_speed", 0.80, 3.0, 0.90),  # vitesse « proche »
    ParamSpec("gap_width_speed_gain", 0.0, 4.0, 1.0),  # relance dans les troues
    ParamSpec("curvature_speed_gain", 0.0, 4.0, 0.80),  # limite en virage
    ParamSpec("speed_smoothing", 2.0, 20.0, 8.0),  # lissage de la vitesse
    ParamSpec("throttle_max_speed", 2.0, 5.5, 4.0),  # 100 % d'accélérateur
    ParamSpec("speed_min_mps", 0.20, 1.0, 0.40),  # vitesse plancher
)

B1_DEFAULT = np.array([spec.default for spec in B1_SPECS], dtype=float)
B1_LOW = np.array([spec.low for spec in B1_SPECS], dtype=float)
B1_HIGH = np.array([spec.high for spec in B1_SPECS], dtype=float)


@dataclass
class B1Params:
    """Les 23 paramètres physiques de la politique B1.

    ``from_vector`` convertit un vecteur normalisé (celui de CMA-ES ou du
    fichier de poids) en valeurs physiques ; sans vecteur, on retombe sur
    les valeurs par défaut calées à la main.
    """

    front_sector_deg: float
    clearance_window_deg: float
    min_gap_m: float
    smoothing_beams: float
    angle_change_weight: float
    clearance_weight: float
    range_weight: float
    gap_width_weight: float
    center_bias_weight: float
    aim_blend: float
    max_steering_rad: float
    steer_smoothing: float
    steer_gain: float
    base_speed_mps: float
    clearance_slow_1_m: float
    clearance_slow_1_speed: float
    clearance_slow_2_m: float
    clearance_slow_2_speed: float
    gap_width_speed_gain: float
    curvature_speed_gain: float
    speed_smoothing: float
    throttle_max_speed: float
    speed_min_mps: float

    @classmethod
    def from_vector(cls, vector: np.ndarray | None = None) -> B1Params:
        values = B1_DEFAULT if vector is None else _unit_to_params(vector, B1_SPECS)
        return cls(**{spec.name: float(values[i]) for i, spec in enumerate(B1_SPECS)})


# ---------------------------------------------------------------------------
# 3. Lecture du scan lidar
# ---------------------------------------------------------------------------


def _front_scan(
    observation: Observation,
    front_sector_rad: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray] | None:
    """Extrait le scan lidar courant et masque les mesures invalides.

    Retourne ``(angles, ranges, valid)`` ou ``None`` si aucun lidar n'est
    disponible. Une mesure est valide si le capteur l'a publiée, si elle est
    finie et si elle dépasse 5 cm (en dessous, c'est du bruit de contact).
    """

    scan = observation.lidar_current
    if scan is None:
        return None
    angles = np.asarray(scan.angles_rad, dtype=float)
    ranges = np.asarray(scan.ranges_m, dtype=float)
    valid = np.asarray(scan.valid, dtype=bool) & np.isfinite(ranges) & (ranges > 0.05)
    if angles.size == 0 or ranges.size != angles.size or valid.size != angles.size:
        return None
    front = valid & (np.abs(angles) <= front_sector_rad)
    if not np.any(front):
        return None
    return angles, ranges, valid


# ---------------------------------------------------------------------------
# 4. La politique B1
# ---------------------------------------------------------------------------


class StructuredGapPolicy:
    """B1 : contrôleur de suivi de trou paramétrable.

    Le contrôleur ne voit QUE les capteurs (``Observation``, sans pose réelle)
    et produit une ``Actuation`` normalisée. Il possède un état interne
    minimal — direction courante, direction précédente, vitesse courante —
    qui est remis à zéro par ``reset()`` entre deux runs.
    """

    def __init__(self, vector: np.ndarray | None = None) -> None:
        # ``vector`` : vecteur de poids normalisé [0, 1] (CMA-ES / fichier
        # .npz). ``None`` -> valeurs par défaut calées à la main.
        self.params = B1Params.from_vector(vector)
        self.reset()

    def reset(self) -> None:
        """Remet l'état interne à zéro pour un nouveau run."""

        self.steering = 0.0  # dernière direction commandée (radians)
        self.last_angle = 0.0  # dernière direction choisie (pour l'anti-zigzag)
        self.speed = 0.0  # dernière vitesse cible (m/s)
        self.last_debug: dict[str, float] = {}  # sorties du dernier tick

    def score_surface(
        self, observation: Observation
    ) -> dict[str, np.ndarray | float | int] | None:
        """Calcule la surface de score complète (pleine résolution du scan).

        Exposée pour l'entraînement et le débogage : le warm start B2 fait une
        régression de ses features par faisceau sur cette surface plutôt que
        sur une action isolée, ce qui préserve l'ordre de sûreté appris par B1.

        Retourne ``None`` si aucun faisceau valide n'est disponible.
        """

        p = self.params
        front_sector = np.radians(p.front_sector_deg)
        scan = _front_scan(observation, front_sector)
        if scan is None:
            return None
        angles, ranges, valid = scan
        front = valid & (np.abs(angles) <= front_sector)
        if not np.any(front):
            return None

        # --- Étape 3 : lissage circulaire du profil de portée -----------------
        # Les faisceaux hors secteur sont remplacés par la portée maximale
        # observable, pour ne pas fausser la moyenne mobile aux bords.
        open_range = float(np.nanmax(ranges[front]))
        work = np.where(front, ranges, open_range)

        kernel_count = _coerce_odd(p.smoothing_beams, angles.size)
        if kernel_count > 1:
            kernel = np.ones(kernel_count) / kernel_count
            pad = kernel_count // 2
            padded = np.concatenate([work[-pad:], work, work[:pad]])
            work = np.convolve(padded, kernel, mode="same")[pad:-pad]

        # --- Étape 4 : dégagement minimal par fenêtre angulaire ---------------
        # Fenêtre glissante de ``clearance_window_deg`` convertie en nombre de
        # faisceaux ; padding circulaire pour couvrir tout le scan.
        step = abs(float(np.median(np.diff(angles)))) if angles.size > 1 else 0.01
        window = _coerce_odd(
            np.radians(p.clearance_window_deg) / max(step, 1e-6), angles.size
        )
        pad = window // 2
        padded_work = np.concatenate([work[-pad:], work, work[:pad]]) if pad else work
        windows = np.lib.stride_tricks.sliding_window_view(padded_work, window)
        clearance = windows.min(axis=1)

        # --- Détection du trou : seuil adaptatif ------------------------------
        # Un faisceau est « libre » si sa portée lissée dépasse le seuil
        # (portée minimale ``min_gap_m`` ou 75 % du meilleur dégagement).
        threshold = max(p.min_gap_m, 0.75 * float(np.max(clearance[front])))
        free = front & (work >= threshold)
        # Fraction de faisceaux libres dans la même fenêtre : mesure la
        # « largeur » du trou autour de chaque direction.
        padded_free = (
            np.concatenate([free[-pad:], free, free[:pad]]).astype(float)
            if pad
            else free.astype(float)
        )
        free_windows = np.lib.stride_tricks.sliding_window_view(padded_free, window)
        free_fraction = free_windows.mean(axis=1)

        # --- Étape 5 : score de chaque direction -----------------------------
        # Récompenses : dégagement, portée brute, largeur de trou.
        # Pénalités : écart à la direction précédente (anti-zigzag), écart à
        # l'avant du véhicule (biais central).
        scores = (
            p.clearance_weight * clearance
            + p.range_weight * work
            + p.gap_width_weight * 2.0 * free_fraction
            - p.angle_change_weight * np.abs(angles - self.last_angle)
            - p.center_bias_weight * np.abs(angles)
        )
        # Hors secteur avant : jamais sélectionnable.
        scores = np.where(front, scores, -np.inf)
        return {
            "angles": angles,
            "front": front,
            "work": work,
            "clearance": clearance,
            "free": free,
            "free_fraction": free_fraction,
            "scores": scores,
            "threshold": threshold,
            "window": window,
            "step": step,
            "front_sector": front_sector,
        }

    def act(self, observation: Observation) -> Actuation:
        """Un pas de contrôle : choisir une direction et une vitesse cible.

        Retourne ``Actuation(throttle, steering)`` : ``throttle`` entre 0 et 1
        (fraction de la vitesse maximale), ``steering`` en radians (positif =
        gauche), déjà lissés.
        """

        p = self.params
        # --- Étapes 1 à 5 : surface de score (voir ``score_surface``) ---------
        surface = self.score_surface(observation)
        if surface is None:
            # Pas de lidar ou aucun faisceau valide : on garde la dernière
            # direction et on s'arrête (throttle nul).
            self.last_debug = {"valid": 0.0}
            return Actuation(throttle=0.0, steering=self.steering)
        angles = np.asarray(surface["angles"], dtype=float)
        clearance = np.asarray(surface["clearance"], dtype=float)
        free = np.asarray(surface["free"], dtype=bool)
        scores = np.asarray(surface["scores"], dtype=float)
        window = int(surface["window"])
        step = float(surface["step"])
        dt = max(observation.dt, 1e-4)

        # --- Direction gagnante ------------------------------------------------
        best = int(np.argmax(scores))
        if not np.isfinite(scores[best]):
            return Actuation(throttle=0.0, steering=0.0)

        best_bearing = float(angles[best])
        self.last_angle = best_bearing  # mémorisé pour l'anti-zigzag du tick suivant
        best_clearance = float(clearance[best])

        # --- Largeur du trou autour du faisceau choisi -------------------------
        # On étend le masque « libre » contigu autour du gagnant, vers la
        # gauche et la droite, pour mesurer l'ouverture réelle.
        if free[best]:
            lo = best
            hi = best
            while lo > 0 and free[lo - 1]:
                lo -= 1
            while hi < free.size - 1 and free[hi + 1]:
                hi += 1
            gap_left = float(min(angles[lo], angles[hi]))
            gap_right = float(max(angles[lo], angles[hi]))
            gap_width = float(abs(angles[hi] - angles[lo]))
            gap_center = 0.5 * (gap_left + gap_right)
        else:
            # Pas de trou contigu (cas rare) : on estime la largeur par la
            # fenêtre de dégagement elle-même.
            gap_width = float(window * step)
            gap_left = best_bearing - gap_width / 2.0
            gap_right = best_bearing + gap_width / 2.0
            gap_center = best_bearing

        # --- Étape 6a : direction ---------------------------------------------
        # ``aim_blend`` interpole entre le faisceau gagnant (0.0) et le centre
        # du trou (1.0) ; le gain convertit l'angle visé en angle de volant,
        # borné par la butée mécanique, puis lissé par un filtre passe-bas
        # exponentiel (``steer_smoothing`` fixe la rapidité).
        desired = p.aim_blend * gap_center + (1.0 - p.aim_blend) * best_bearing
        target = _clip(p.steer_gain * desired, -p.max_steering_rad, p.max_steering_rad)
        alpha = min(1.0, p.steer_smoothing * dt)
        self.steering = _clip(
            self.steering + (target - self.steering) * alpha,
            -p.max_steering_rad,
            p.max_steering_rad,
        )

        # --- Étape 6b : vitesse cible -------------------------------------------
        # Palier de dégagement : très proche -> clearance_slow_1_speed,
        # proche -> clearance_slow_2_speed, sinon vitesse de base.
        speed = p.base_speed_mps
        if best_clearance < p.clearance_slow_1_m:
            speed = min(speed, p.clearance_slow_1_speed)
        elif best_clearance < p.clearance_slow_2_m:
            speed = min(speed, p.clearance_slow_2_speed)
        # Relance autorisée par la largeur du trou.
        speed = min(p.base_speed_mps, speed + p.gap_width_speed_gain * gap_width)
        # Limite en virage : plus le véhicule tourne fort (courbure estimée
        # par yaw_rate / vitesse), plus la cible est réduite.
        measured_speed = max(float(observation.state.speed_mps), 0.2)
        curvature = abs(float(observation.state.yaw_rate_rad_s)) / measured_speed
        speed = min(speed, p.base_speed_mps / (1.0 + p.curvature_speed_gain * curvature))
        speed = _clip(speed, p.speed_min_mps, p.base_speed_mps)
        # Lissage passe-bas, puis conversion en throttle normalisé.
        alpha = min(1.0, p.speed_smoothing * dt)
        self.speed += (speed - self.speed) * alpha
        throttle = _clip(self.speed / max(p.throttle_max_speed, 1e-6), 0.0, 1.0)

        # --- Sorties de débogage (dashboard, tests, warm start) ---------------
        self.last_debug = {
            "valid": 1.0,
            "best_bearing_rad": best_bearing,
            "target_bearing_rad": desired,
            "steering_target_rad": target,
            "clearance_m": best_clearance,
            "gap_width_rad": gap_width,
            "target_speed_mps": self.speed,
            "throttle": throttle,
            "steering_rad": self.steering,
            "score": float(scores[best]),
        }
        return Actuation(throttle=throttle, steering=self.steering)
