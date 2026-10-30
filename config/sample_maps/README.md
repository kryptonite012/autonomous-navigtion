# Sample maps

`sim_living_room` was produced by `scripts/sim_explore.py` in the simulated
5 × 4 m room (`config/worlds/living_room.yaml`) using only the simulated
ultrasonic/ToF sensors and odometry, i.e. the **experimental** mapping mode.
Copy it into `data/maps/` to load it from the dashboard:

```bash
mkdir -p data/maps && cp config/sample_maps/sim_living_room.* data/maps/
```
