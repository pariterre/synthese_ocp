# Installation

Ce script assume que l'environnement basé sur `environment.yml` a été installé et est actif.
Sous Linux, avec conda de préinstallé, pour installer l'environnement, dans un terminal à partir du dossier actuel, faites les commandes suivantes:
```bash
conda env create -f environment.yml
conda activate synthese
```

# Utilisation

## Position initiale (*L-sit*) et finale (*handstand*)

Afin de générer la position initiale, il est possible de lancer le script `find_poses.py`. 
Si la variable est `straddle` est mise à `True`, alors la position trouvée sera un *L-sit*, sinon, ce sera un *handstand*.
Il est à noter que pour des raisons de dynamique, la position de *L-sit* a été modifiée pour une position dont l'angle à l'épaule est déjà plus grand que 90 degrés.
Pour afficher la position normale (*L-sit*), il faut affecter la valeur `False` à la variable `modified_straddle`.

Le script en question résout un problème d'optimisation avec les contraintes que 
- le centre de masse soit aligné avec l'origin;
- le centre de masse soit au centre des points de contact des mains;
- les contacts des mains doivent être exactement à la hauteur du sol.
La symmétrie du système est directement implémentée dans le calcul de la cinématique grâce à la fonction `get_q_from_x(x)`
Puisque aucun objectif n'est associé à cette optimisation, dès que les contraintes sont respectées, la solution est acceptée. 
Ceci rend la résolution extrêmement sensible à la solution initiale
Ceci est une bonne chose, car il suffit de placer l'avatar dans un position proche du *L-sit* ou du *handstand*, puis de lancer l'optimisation.
La solution est alors très proche de celle initiale, mais respecte les contraintes.

## Problème d'optimisation

Le problème d'optimisation peut être lancé grâce au script `straddle_to_handstand.py`.

### Différence avec le manuscrit

Le modèle utilisé et le problème résolu est légèrement différent de celui présenté dans le manuscrit.
Voici ces différences en rafale: 
- La rotation autours de l'axe y au bassin a été ajoutée, car autrement la dynamique avec contact, telle que retournée par *RBDL*, était erronnée;
- Un objectif pour minimiser le changement de couples aux articulations a été ajouté comme terme de régularisation, particulièrement renforé au poignet;
- La symétrie a été réalisée en ajoutant des contraintes d'égalité ou d'opposé entre les degrés de liberté et les contrôles correspondants;
- Les couples articulaires sont directement utilisés (sans passer par une activation de couple) et peuvent varier de -200Nm à 200Nm;
- Les conditions initiales du problème pour les états sont une interpolation linéaire entre la position initiale et la position finale, tandis que celles des contrôles sont 0 à tous les noeuds;
- La position de départ n'est pas le *L-sit* car des problèmes de dynamique sont apparus au moment du passage des bras à 90°. 
La position de départ est donc plus élevée pour que l'angle de 90° soit déjà passé;
- Seulement deux conditions expérimentales ont été testées, et les différences de souplesse ont été appliquées à l'abduction plutôt qu'à la fin.
La raison de cela est qu'il était impossible d'avoir une solution initiale adéquate en changeant la flexibilité de la flexion de la hanche.
Ces deux conditions expérimentales sont : la condition souple (72° d'abduction maximale) et la condition non-souple (34° d'abduction maximale). 

### Résultats

Les états et contrôles aux noeuds sont enregistrés dans le dossier de `resultat` sous le format `.bob`. 
Ce format est utilisable avec le *package* *pickle* et un exemple pour charger les données est inclus dans le fichier `straddle_to_handstand.py`.
De plus, des vidéos des ces résultats au format `.mp4`, générées grâce à *biorbd-viz*, en vue de côté et de face pour les deux conditions, ont été produites. 

