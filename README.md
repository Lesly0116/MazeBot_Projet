# MazeBot_Projet - Navigation labyrinthe avec CoppeliaSim

# MazeBot_Projet

## Description

MazeBot simule un robot mobile à roues différentielles (Pioneer P3DX) évoluant dans un labyrinthe 3D construit sous **CoppeliaSim**. Le robot dispose de capteurs de proximité ultrasoniques simulés (avant, gauche, droite) pour détecter les murs et naviguer de manière autonome.

L'algorithme de navigation implémenté est le **Wall-Following** (longe le mur gauche ou droit) avec un système avancé de mémoire spatiale et de détection de répétition de trajectoire. La vitesse des roues est commandée en temps réel selon les lectures des capteurs. Un chronomètre mesure le temps de résolution et un tracé de trajectoire est affiché à l'écran.

## Membres du groupe

| Nom                | Rôle                               |
| ------------------ | ---------------------------------- |
| Jean Lesly JOCELYN | Développement complet du code      |
| ------------------ | Développement complet / Navigation |
| ------------------ | Développement complet / Navigation |
| ------------------ | Développement complet / Navigation |

## Lien GitHub

[https://github.com/Lesly0116/MazeBot_Projet](https://github.com/Lesly0116/MazeBot_Projet)

## Capture / Vidéo de la simulation

![MazeBot - Navigation autonome dans un labyrinthe 3D](videos/MazeBot_Simulation.mp4)

Vidéo montrant le robot Pioneer P3DX naviguant de l'entrée à la sortie du labyrinthe avec l'algorithme Wall-Following

[Télécharger la vidéo](videos/MazeBot_Simulation.mp4) (cliquez pour visionner)

## Composants / Modèles 3D utilisés

| Composant          | Modèle          | Source                                 | Description                                        |
| ------------------ | --------------- | -------------------------------------- | -------------------------------------------------- |
| Robot              | Pioneer P3DX    | `Models/robots/mobile/PioneerP3DX.ttm` | Robot mobile à roues différentielles               |
| Moteurs gauche     | `leftMotor`     | Modèle Pioneer P3DX                    | Commande vitesse roue gauche                       |
| Moteurs droit      | `rightMotor`    | Modèle Pioneer P3DX                    | Commande vitesse roue droite                       |
| Capteur avant      | `sensor_avant`  | `Add/Proximity Sensor/Cone Type`       | Détection obstacles devant                         |
| Capteur gauche     | `sensor_gauche` | `Add/Proximity Sensor/Cone Type`       | Détection obstacles à gauche                       |
| Capteur droite     | `sensor_droite` | `Add/Proximity Sensor/Cone Type`       | Détection obstacles à droite                       |
| Zone d'arrivée     | `Arrivee`       | `Add/Dummy`                            | Point d'arrivée du robot (invisible en simulation) |
| Zone de départ     | `Depart`        | `Add/Dummy`                            | Point de départ du robot (invisible en simulation) |
| Murs du labyrinthe | `Cuboid`        | `Add/Primitive shape/Cuboid`           | Obstacles formant le labyrinthe                    |

## Répartition du travail

| Tâche                                             | Responsable        |
| ------------------------------------------------- | ------------------ |
| Architecture générale du code                     | ------------------ |
| Construction de la scene                          | ------------------ |
| Algorithme de suivi de mur (Wall-Following)       | Jean Lesly JOCELYN |
| Système de mémoire spatiale et zones              | Jean Lesly JOCELYN |
| Détection et correction des trajectoires répétées | Jean Lesly JOCELYN |
| Tests et réglages des paramètres                  | ------------------ |
| Documentation (README)                            | ------------------ |

## Tests réalisés

| Test                  | Objectif                                          | Résultat                          |
| --------------------- | ------------------------------------------------- | --------------------------------- |
| Suivi de mur gauche   | Vérifier que le robot longe correctement les murs | Validé                            |
| Suivi de mur droit    | Vérifier le changement de stratégie               | Validé                            |
| Détection d'impasse   | Robot doit faire demi-tour                        | Reste a ameliorer mais fonctionne |
| Anti-répétition       | Ne pas tourner en boucle dans un couloir          | Validé                            |
| Arrivée à destination | S'arrête quand distance < 0.40m                   | Validé                            |
| Affichage tableau UI  | Mise à jour temps/distance/zone                   | Validé                            |
| Trajectoire rouge     | Tracé continu du parcours                         | Validé                            |

## Installation

### Prérequis

- **CoppeliaSim** (version Edu ) avec support ZMQ Remote API
- **Python 3.7+**
- **Git**

### Cloner le dépôt

```bash
git clone https://github.com/Lesly0116/MazeBot_Projet.git
cd MazeBot_Projet
```
