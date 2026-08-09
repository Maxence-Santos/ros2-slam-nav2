# Demo media

Add a portfolio-ready capture here:

| File | Purpose |
|------|---------|
| `demo.gif` or `demo.mp4` | 20–40 s clip: SLAM map build **or** nav + pedestrian evasion |

## Recommended capture (host)

```bash
# Terminal 1
./start_nav_demo.sh

# Then record Gazebo + RViz with your preferred tool, e.g.:
# - GNOME Screenshot / SimpleScreenRecorder
# - OBS Studio
# - wf-recorder / peek (GIF)
```

**Shot list (nav demo):**

1. RViz map + green A* path after setting a 2D Goal Pose  
2. Pedestrian approaching → lateral evasion (0.35 m/s)  
3. Optional: rear/flank escape acceleration (0.42 m/s)  
4. Yield stop if re-merge corridor blocked  

Keep resolution ≤ 1280 px wide for GitHub README embeds.
