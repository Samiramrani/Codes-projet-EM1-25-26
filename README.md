# Codes-projet-EM1-25-26
Codes Arduino et simulation sur Python du projet robot jouer de OXO du groupe EM1.

Le code Arduino complet contient toutes les fonctions nécessaires au déroulement d'une partie. Le code de l'évaluation standard contient des instructions spécifiques au tracé de symboles dans les coins.

La simulation sur Python a été faite en deux parties. D'abord, une simulation du comportement du moteur gauche en avant permettant pour estimer un premier gain proportionnel. Ensuite, un deuxième code permet d'ajuster les gains proportionnels du moteur gauche en arrière et l'autre moteur dans les deux directions sur base de trajectoires complètes (ligne droite, rotation). Afin de changer la trajctoire simulée, il faut mettre la variable "sens" en 1 ou -1 en fonction de la direction de rotation souhaitée pour chaque moteur (lignes 137-138). Notons que pour les rotations, il est nécessaire de soit zoomer sur l'image obtenue, ou de redéfinir l'échelle d'affichage (lignes 168-169).
