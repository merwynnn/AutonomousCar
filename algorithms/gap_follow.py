"""Lidar-only furthest-gap controller for the dashboard demo.

This is a normal algorithm: it sees only ``Observation`` and returns
``Actuation``. It uses no simulator internals and no ground-truth pose.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from onboard.api import Actuation, Observation


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
    valid: bool = False  # Indique si un trou valide a été trouvé
    profile: np.ndarray = field(
        default_factory=lambda: np.zeros((0, 2), dtype=float)
    )


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
       pour le centre (pour éviter de zigzaguer)
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
    ) -> None:
        """Initialise le contrôleur de suivi de trou.

        Args:
            max_steering: Angle de direction maximum en radians (la commande sera limitée à cette valeur).
                          Par exemple 0.50 rad ≈ 28.6°.
            front_sector_deg: Secteur angulaire devant le véhicule (en degrés, ±) à considérer pour
                              la détection de trous. Par exemple 100° signifie de -50° à +50°.
            clearance_window_deg: Taille de la fenêtre (en degrés) utilisée pour calculer le dégagement
                                  minimal. Plus cette valeur est grande, plus l'algorithme est prudent.
            min_gap_m: Largeur minimale acceptable d'un trou en mètres. Les trous plus étroits sont
                       ignorés car le véhicule ne pourrait pas passer.
            max_speed_for_throttle: Vitesse (en m/s) à laquelle l'accélérateur atteint 100%. Au-delà,
                                    la vitesse est limitée par ce paramètre.
        """
        self.max_steering = max_steering
        self.front_sector = np.radians(front_sector_deg)
        self.clearance_window_deg = clearance_window_deg
        self.min_gap_m = min_gap_m
        self.max_speed_for_throttle = max_speed_for_throttle
        self.steering = 0.0
        self.debug = GapDebug()

    def reset(self) -> None:
        """Réinitialise l'état interne du contrôleur pour un nouveau run.

        Cette méthode est appelée au début de chaque simulation pour effacer la mémoire
        de l'algorithme (direction précédente, informations de débogage). Cela garantit
        que chaque run commence dans le même état.
        """
        self.steering = 0.0
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
        # Récupération des données du lidar
        scan = observation.lidar
        if scan is None:
            # Si pas de lidar, on s'arrête
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
            self.debug = GapDebug(valid=False)
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
        # On veut maximiser le dégagement (clearance) mais on préfère légèrement le centre
        # pour éviter que le véhicule zigzague constamment entre gauche et droite
        # 0.15 est un coefficient empirique : plus il est grand, plus on préfère le centre
        scores = clearance - 0.15 * np.abs(angles)

        # On met un score très bas (-inf) pour les directions hors secteur avant
        # Cela garantit qu'elles ne seront jamais choisies
        scores[~front] = -np.inf

        # On choisit la direction avec le score maximum
        best = int(np.argmax(scores))
        desired = float(angles[best])  # Angle souhaité (direction vers laquelle tourner)
        best_clearance = float(clearance[best])  # Dégagement dans cette direction
        best_range = float(work[best])  # Distance brute à cet angle

        # Mesure de la largeur du trou en détectant l'espace libre contigu autour du meilleur faisceau
        # Un "trou" est une zone contiguë où toutes les distances sont supérieures à un seuil
        # Le seuil est le maximum entre : min_gap_m (trou minimum) et 75% du meilleur dégagement
        # Cela permet d'adapter le seuil à la situation : plus de dégagement = seuil plus élevé
        free = front & (work >= max(self.min_gap_m, 0.75 * best_clearance))

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

        # Si les obstacles sont très proches (dégagement < 0.50 m), on ralentit fortement
        if best_clearance < 0.50:
            speed = min(speed, 0.55)  # Vitesse maximale 0.55 m/s (très lent)
        # Si les obstacles sont modérément proches (dégagement < 0.80 m), on ralentit un peu
        elif best_clearance < 0.80:
            speed = min(speed, 1.0)  # Vitesse maximale 1.0 m/s (lent)

        # Calcul de l'accélérateur (throttle) à partir de la vitesse cible
        # throttle est une valeur entre 0 et 1 qui représente la puissance du moteur
        # On divise la vitesse par max_speed_for_throttle pour normaliser
        # Si speed = max_speed_for_throttle, throttle = 1 (pleine puissance)
        # Si speed est plus petit, throttle est proportionnellement plus petit
        throttle = max(0.0, min(1.0, speed / self.max_speed_for_throttle))
        sample_step = max(1, angles.size // 90)
        profile = np.column_stack(
            [angles[::sample_step], clearance[::sample_step]]
        )
        # Mise à jour des informations de débogage pour la visualisation dans le dashboard
        # Ces données permettent de voir ce que l'algorithme a calculé (direction, vitesse, trou choisi)
        self.debug = GapDebug(
            angle_rad=desired,  # Direction souhaitée
            steering_rad=self.steering,  # Direction réelle après lissage
            range_m=best_range,  # Distance au centre du trou
            clearance_m=best_clearance,  # Dégagement minimal
            gap_width_rad=gap_width,  # Largeur angulaire du trou
            gap_left_rad=float(min(angles[lo], angles[hi])),
            gap_right_rad=float(max(angles[lo], angles[hi])),
            front_sector_rad=float(self.front_sector),
            throttle=throttle,
            min_clearance_m=float(clearance.min()),
            valid=True,  # Indique que le calcul a réussi
            profile=profile,
        )

        # Retour des commandes pour le véhicule
        return Actuation(throttle=throttle, steering=self.steering)
