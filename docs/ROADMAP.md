# 🗺️ Roadmap du Projet — SLAM & Navigation Autonome

## Phase 1 — Fondations ROS 2 & Modélisation
- [x] Environnement ROS 2 + Gazebo Harmonic
- [x] Package `slam_robot`, URDF/Xacro, capteurs LiDAR/IMU
- [x] Mondes `warehouse.sdf` / `warehouse_static.sdf`
- [x] Téléopération `cmd_vel`

## Phase 2 — SLAM
- [x] `slam_toolbox` async + launch
- [x] Carte `warehouse_map` (.yaml / .pgm)
- [x] Export via `save_map.py`
- [ ] *(optionnel)* Fine-tuning résolution / loop closure

## Phase 3 — Navigation autonome
- [x] Navigateur unifié A* + inflation 0.45 m
- [x] Évasion dynamique 360° (4 secteurs, goal-aware side selection)
- [x] Escape rear/flank 0.42 m/s, yield-on-re-merge
- [x] Scripts `start_slam_demo.sh` / `start_nav_demo.sh`
- [x] Acteurs dynamiques dans le monde warehouse

## Phase 4 — Behavior Trees Nav2 (optionnel hors scope v1)
- [ ] BT custom de patrouille Nav2
- [ ] Recovery behaviors avancés

> La navigation portfolio s’appuie sur le contrôleur custom (`nav_controller.py`) plutôt que sur un BT Nav2 complet.

## Phase 5 — Polish & documentation
- [x] README + post-mortem technique + Mermaid
- [x] Module pur `navigation_geometry.py` + **18 unit tests**
- [x] Dockerfile (build + tests, sans GUI)
- [x] Tableau de métriques carte / vitesses / seuils
- [x] Guide capture média (`media/README.md`)
- [ ] Fichier `media/demo.gif` ou `demo.mp4` (capture manuelle sur host)
- [x] Roadmap alignée sur le code réel
