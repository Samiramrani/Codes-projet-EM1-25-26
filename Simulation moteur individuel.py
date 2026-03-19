import math
import numpy as np
import matplotlib.pyplot as plt

# Paramètres moteur
rayon_roue = 2.1
pas_temps = 0.01

tension_min = 50
tension_max = 200

pente = 0.0214
offset = 0.267

tau = 0.02

duree_accel_demarrage = 0.05
duree_plateau_demarrage = 0.05
duree_totale = 6.0

nombre_de_pas = int(duree_totale / pas_temps)
liste_temps = np.arange(nombre_de_pas) * pas_temps

def vitesse_moteur(tension):
    tension_int = round(tension)
    if tension_int < tension_min:
        return pente * tension_min - offset
    return pente * tension_int - offset

vitesse_min = vitesse_moteur(tension_min)

vitesse_max = 1.0
duree_accel_consigne = 0.1
acceleration = vitesse_max / duree_accel_consigne

consigne_vitesse = []
for i in range(nombre_de_pas):
    t = i * pas_temps
    if t < duree_accel_consigne:
        consigne_vitesse.append(acceleration * t)
    else:
        consigne_vitesse.append(vitesse_max)

consigne_vitesse = np.array(consigne_vitesse)
consigne_distance = np.cumsum(consigne_vitesse) * pas_temps * 2 * math.pi * rayon_roue

t_bascule_regulation = duree_accel_demarrage + duree_plateau_demarrage

def simuler(kp):
    vitesse_actuelle = 0.0
    tours_effectues = 0.0
    vitesses_reelles = []
    i_bascule = None

    for i in range(nombre_de_pas):
        t = i * pas_temps

        if t < duree_accel_demarrage:
            vitesse_actuelle = (t / duree_accel_demarrage) * vitesse_min

        elif t < t_bascule_regulation:
            vitesse_cible = vitesse_min
            vitesse_actuelle = vitesse_actuelle + (pas_temps / tau) * (vitesse_cible - vitesse_actuelle)

        else:
            if i_bascule is None:
                i_bascule = i
            erreur = consigne_distance[i - i_bascule] - tours_effectues
            tension = np.clip(tension_min + kp * erreur, tension_min, tension_max)
            vitesse_cible = vitesse_moteur(tension)
            vitesse_actuelle = vitesse_actuelle + (pas_temps / tau) * (vitesse_cible - vitesse_actuelle)

        tours_effectues += vitesse_actuelle * pas_temps * 2 * math.pi * rayon_roue
        vitesses_reelles.append(vitesse_actuelle)

    return np.array(vitesses_reelles)

# Les 3 valeurs de Kp à comparer
kp1, kp2, kp3 = 2, 25, 10

vitesses1 = simuler(kp1)
vitesses2 = simuler(kp2)
vitesses3 = simuler(kp3)

fig, ax = plt.subplots(figsize=(10, 5))

ax.plot(liste_temps, consigne_vitesse, 'k--', lw=1.5, label="Consigne")
ax.plot(liste_temps, vitesses1, color='C0', lw=1, label=f"Kp = {kp1}")
ax.plot(liste_temps, vitesses2, color='green', lw=1, label=f"Kp = {kp2}")
ax.plot(liste_temps, vitesses3, color='red', lw=1, label=f"Kp = {kp3}")

ax.set_xlabel("Temps (s)", fontsize=14)
ax.set_ylabel("Vitesse (tours/s)", fontsize=14)
ax.axhline(0, color='black', linewidth=2)
ax.axvline(0, color='black', linewidth=2)
ax.legend()
ax.grid(True)

plt.savefig("Kp_exp3.svg")

plt.tight_layout()
plt.show()



