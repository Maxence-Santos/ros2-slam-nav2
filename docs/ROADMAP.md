# 🗺️ Roadmap du Projet — SLAM & Navigation Autonome

Ce document détaille les différentes phases de développement pour mener à bien le projet de robotique mobile autonome sous ROS 2 Jazzy et Gazebo Harmonic.

---

## Phase 1 — Fondations ROS 2 & Modélisation (Semaines 1-2)

**Objectifs :** Mettre en place l'environnement de travail et créer un robot simulé fonctionnel capable de se déplacer.

### Tâches
- [x] Installer Ubuntu 24.04, ROS 2 Jazzy et Gazebo Harmonic.
- [x] Apprendre les concepts fondamentaux ROS 2 (nodes, topics, services, actions, colcon, launch files).
- [x] Créer la structure du package `slam_robot`.
- [x] Rédiger le modèle URDF/Xacro du robot :
  - Chassis et roues (diff-drive).
  - Capteurs : LiDAR 2D, Caméra RGB-D, IMU.
- [x] Intégrer les plugins Gazebo (`gazebo_ros2_control`, capteurs).
- [x] Créer le monde `warehouse.sdf` dans Gazebo.
- [x] Tester la téléopération avec `teleop_twist_keyboard` via `cmd_vel`.

**Ressources :**
- [Documentation Officielle ROS 2 Jazzy](https://docs.ros.org/en/jazzy/index.html)
- [Gazebo Harmonic Tutorials](https://gazebosim.org/docs/harmonic/tutorials)
- [Articulated Robotics - Building a Mobile Robot](https://articulatedrobotics.xyz/)

**Critères de validation :**
- [x] Le robot apparaît dans Gazebo sans erreurs.
- [x] Les données des capteurs (LiDAR `/scan`, Caméra, IMU) s'affichent correctement dans RViz2.
- [x] Le robot peut être conduit au clavier dans la simulation.

**Pièges courants :**
- Problèmes de TF (Transformations) : Vérifiez toujours que `robot_state_publisher` publie bien les TFs statiques à partir du Xacro.
- Conflits de namespaces ou noms de topics entre Gazebo et ROS 2.

---

## Phase 2 — SLAM (Semaines 3-4)

**Objectifs :** Implémenter la cartographie de l'environnement à l'aide de SLAM Toolbox.

### Tâches
- [x] Comprendre les bases du SLAM (Occupancy Grid, Scan Matching, Graphes de poses).
- [x] Installer et configurer `slam_toolbox`.
- [x] Écrire le fichier `slam_toolbox.yaml` avec les paramètres pour le mode *asynchrone*.
- [x] Créer le launch file `slam.launch.py`.
- [x] Naviguer manuellement dans le warehouse pour générer une carte complète.
- [x] Sauvegarder la carte en utilisant le noeud `map_saver_cli` de `nav2_map_server`.
- [ ] Itérer sur les paramètres de SLAM (résolution, range max, loop closure) pour optimiser la qualité.

**Ressources :**
- [SLAM Toolbox GitHub & Docs](https://github.com/SteveMacenski/slam_toolbox)
- [Nav2 SLAM Tutorial](https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html)

**Critères de validation :**
- [x] Une carte propre et cohérente de l'entrepôt est sauvegardée sous forme de `.yaml` et `.pgm`.
- [x] Les murs sont bien définis (pas de double murs dus à un mauvais odom/TF).

**Pièges courants :**
- Mauvaise odométrie provoquant des sauts dans la carte. Assurez-vous que le `diff_drive_controller` est bien calibré.
- Oublier de configurer le frame `map` -> `odom` fourni par SLAM Toolbox.

---

## Phase 3 — Navigation Autonome (Semaines 5-6)

**Objectifs :** Permettre au robot de se déplacer de manière autonome et sécurisée d'un point A à un point B en évitant les obstacles.

### Tâches
- [x] Configurer `robot_localization` (EKF) pour fusionner l'odométrie des roues et l'IMU (`ekf.yaml`).
- [x] Intégrer et configurer le stack Nav2 (`nav2_params.yaml`) :
  - **Controller Server** : DWB Local Planner.
  - **Planner Server** : NavFn ou Smac Planner.
  - **Costmaps** : Global et Local (Obstacle layer, Inflation layer, Static layer).
- [x] Créer `navigation.launch.py` pour lancer Nav2 et AMCL (si navigation sur carte statique).
- [x] Tester la navigation avec l'outil "2D Goal Pose" dans RViz.
- [x] Développer le script Python `waypoint_follower.py` pour envoyer une séquence d'objectifs via l'action `NavigateToPose`.
- [ ] Ajouter des obstacles dynamiques dans Gazebo pour valider l'évitement.

**Ressources :**
- [Nav2 Documentation](https://navigation.ros.org/)
- [Robot Localization Docs](http://docs.ros.org/en/noetic/api/robot_localization/html/index.html) (Concepts applicables en ROS 2)

**Critères de validation :**
- [x] Le robot navigue de manière fluide vers le but sans percuter les murs.
- [x] L'EKF publie une transformation `odom -> base_link` robuste.
- [x] Le script Python exécute la route de waypoints avec succès.

**Pièges courants :**
- Problèmes de costmap liés à des frames incorrects.
- L'inflation radius trop grand empêchant le robot de passer dans des portes ou allées étroites.
- Incompatibilité des fréquences de publication entre IMU et odométrie pour l'EKF.

---

## Phase 4 — Behavior Trees & Missions (Semaine 7)

**Objectifs :** Rendre la logique de navigation plus intelligente et gérer les échecs via des arbres de comportement (Behavior Trees).

### Tâches
- [ ] Comprendre le fonctionnement des Behavior Trees dans Nav2 (XML files).
- [ ] Créer un BT personnalisé pour une mission de patrouille (ex: navigate to pose, wait, spin, navigate to next).
- [ ] Gérer les cas de blocage avec des *Recovery Behaviors* pertinents (clear costmap, back-up, spin).
- [ ] Intégrer le BT custom dans la configuration Nav2.

**Ressources :**
- [Nav2 Behavior Trees](https://navigation.ros.org/behavior_trees/index.html)
- [BehaviorTree.CPP Documentation](https://www.behaviortree.dev/)

**Critères de validation :**
- [ ] Le robot exécute des comportements complexes programmés via le XML du BT.
- [ ] S'il est bloqué, le robot tente de se dégager seul avant de déclarer un échec.

**Pièges courants :**
- Syntaxe XML invalide dans le fichier BT, empêchant le lancement du `bt_navigator`.
- Boucles infinies dans les noeuds de recovery.

---

## Phase 5 — Polish & Documentation (Semaine 8)

**Objectifs :** Finaliser le projet pour en faire une pièce maîtresse de portfolio.

### Tâches
- [x] Dockeriser complètement l'application (Dockerfile & docker-compose.yml) pour une reproductibilité parfaite.
- [x] Enregistrer une vidéo de démonstration haute qualité (screencast Gazebo + RViz).
- [ ] Collecter et analyser les métriques du système (précision du mapping, temps de calcul, CPU usage).
- [x] Mettre à jour le fichier README.md avec les instructions d'installation et de lancement, la vidéo, et les badges.
- [x] Nettoyer et commenter l'intégralité du code source (Python/C++ et XML/YAML).

**Critères de validation :**
- [x] Le projet peut être cloné et exécuté en 2 commandes (via Docker).
- [x] La documentation est claire, pro et sans fautes.
- [x] Le repository GitHub est prêt à être partagé avec des recruteurs.
