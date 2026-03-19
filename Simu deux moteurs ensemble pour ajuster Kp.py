import math
import numpy as np
import matplotlib.pyplot as plt

# Terrain
CELL = 30
GRID_N = 3
GRID_W, GRID_H = GRID_N*CELL, GRID_N*CELL

ZONE_W = 50
BOARD_H = 125
GAP_X = 30
MARGIN_Y = 17.5

X_MIN = -(ZONE_W + GAP_X)
X_MAX = GRID_W + GAP_X + ZONE_W
Y_MIN = -MARGIN_Y
Y_MAX = GRID_H + MARGIN_Y

LEFT_ZONE  = (X_MIN, Y_MIN, ZONE_W, BOARD_H)
RIGHT_ZONE = (GRID_W + GAP_X, Y_MIN, ZONE_W, BOARD_H)

HOME       = np.array([LEFT_ZONE[0] + ZONE_W/2, (Y_MIN + Y_MAX)/2])
HOME_THETA = 0.0

# Robot
R  = 2.1
L  = 24
DT = 0.01

tension_min = 50
tension_max = 200
tau         = 0.02

# Paramètres distincts par moteur
#Gauche
#HIGH
pente_L  = 0.0214;  offset_L = 0.267;  KP_L = 10
#LOW
# pente_L  = 0.0206;  offset_L = 0.252;  KP_L = 11.4

#DROIT
#HIGH
pente_R  = 0.0209;  offset_R = 0.267;  KP_R = 12.8
#LOW
#pente_R  = 0.0223;  offset_R = 0.268;  KP_R = 9

duree_accel_demarrage   = 0.05
duree_plateau_demarrage = 0.05
t_bascule_regulation    = duree_accel_demarrage + duree_plateau_demarrage

vitesse_max          = 1.0
duree_accel_consigne = 0.1

def vitesse_moteur(tension, pente, offset):
    tension_int = round(tension)
    if tension_int < tension_min:
        return pente * tension_min - offset
    return pente * tension_int - offset

def wrap_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def cell_center(col1, row1):
    return np.array([(col1-1)*CELL + CELL/2, (row1-1)*CELL + CELL/2])

# Terrain
def add_zone(ax, rect):
    x, y, w, h = rect
    ax.add_patch(plt.Rectangle((x, y), w, h, fill=True,
                                facecolor='white', edgecolor='black', zorder=0))

def draw_tape_noir(ax):
    ax.plot([-GAP_X, -GAP_X],             [Y_MIN, Y_MAX], 'k-', lw=2, zorder=0)
    ax.plot([GRID_W+GAP_X, GRID_W+GAP_X], [Y_MIN, Y_MAX], 'k-', lw=2, zorder=0)
    ax.plot([-GAP_X, GRID_W+GAP_X],       [Y_MAX, Y_MAX], 'k-', lw=2, zorder=0)
    ax.plot([-GAP_X, GRID_W+GAP_X],       [Y_MIN, Y_MIN], 'k-', lw=2, zorder=0)

def draw_field(ax):
    add_zone(ax, LEFT_ZONE)
    add_zone(ax, RIGHT_ZONE)
    for k in range(GRID_N + 1):
        ax.plot([k*CELL, k*CELL], [0, GRID_H], 'k-', lw=1, zorder=1)
        ax.plot([0, GRID_W], [k*CELL, k*CELL], 'k-', lw=1, zorder=1)
    draw_tape_noir(ax)

# Simulation moteur
def simuler_moteur(dist, kp, pente, offset, sens):
    acceleration  = vitesse_max / duree_accel_consigne
    tours_total   = dist / (2 * math.pi * R)

    consigne_vitesse_fn = lambda t: min(acceleration * t, vitesse_max)

    vitesse_min      = vitesse_moteur(tension_min, pente, offset)
    vitesse_actuelle = 0.0
    tours_effectues  = 0.0
    vitesses_reelles = []
    i_bascule        = None
    consigne_parcourue = 0.0
    i = 0

    while tours_effectues < tours_total:
        t = i * DT

        if t < duree_accel_demarrage:
            vitesse_actuelle = (t / duree_accel_demarrage) * vitesse_min

        elif t < t_bascule_regulation:
            vitesse_cible    = vitesse_min
            vitesse_actuelle = vitesse_actuelle + (DT / tau) * (vitesse_cible - vitesse_actuelle)

        else:
            if i_bascule is None:
                i_bascule = i
                consigne_parcourue = 0.0

            # Avance la consigne
            t_reg = (i - i_bascule) * DT
            v_ref = consigne_vitesse_fn(t_reg)
            consigne_parcourue += v_ref * DT

            erreur  = consigne_parcourue - tours_effectues
            tension = np.clip(tension_min + kp * erreur, tension_min, tension_max)
            vitesse_cible    = vitesse_moteur(tension, pente, offset)
            vitesse_actuelle = vitesse_actuelle + (DT / tau) * (vitesse_cible - vitesse_actuelle)

        tours_effectues += (vitesse_actuelle * DT)
        vitesses_reelles.append(sens*vitesse_actuelle)
        i += 1

    return np.array(vitesses_reelles), len(vitesses_reelles)
# Trajectoire
target_xy = cell_center(3, 2)
dist      = 130

#Paramètre sens pour changer de trajectoire (1,1) pour tout droit, (-1, 1) pour tourner à droite, (1, -1) pour tourner à gauche. (-1, -1) pour reculer
vitesses_L, n_pas_L = simuler_moteur(dist, KP_L, pente_L, offset_L, 1)
vitesses_R, n_pas_R = simuler_moteur(dist, KP_R, pente_R, offset_R, 1)
n_pas = min(n_pas_L, n_pas_R)

x, y, th = HOME[0], HOME[1], HOME_THETA
xs, ys   = [x], [y]

for i in range(n_pas):
    wL = vitesses_L[i]
    wR = vitesses_R[i]
    v  = R * 0.5  * (wR + wL) * 2 * math.pi
    w  = (R / L)  * (wR - wL) * 2 * math.pi
    x += v * math.cos(th) * DT
    y += v * math.sin(th) * DT
    th = wrap_pi(th + w * DT)
    xs.append(x)
    ys.append(y)

# Figure
fig, ax = plt.subplots(figsize=(10, 6))

draw_field(ax)

ax.plot(target_xy[0], target_xy[1], 'rx', markersize=14)
ax.plot(xs, ys, color='C0', lw=1.5,
        label=f'Trajectoire simulée (Kp_L={KP_L}, Kp_R={KP_R})', zorder=5)
ax.plot(HOME[0], HOME[1], 'go', ms=7, zorder=4, label='Départ')

ax.set_aspect('equal', adjustable='box')
ax.set_xlim(X_MIN - 0.1, X_MAX + 0.1)
ax.set_ylim(Y_MIN - 0.1, Y_MAX + 0.1)
#ax.set_xlim(-55.5, -54.25)
#ax.set_ylim(44.8, 45.6)
ax.legend(loc='upper right', fontsize=12)
#ax.set_title("Trajectoire réelle simulée", fontsize=16, fontweight='bold')

plt.savefig("Traj_goright5_zoom.svg")

plt.tight_layout()
plt.show()