# Demonstration instructions

## A. Simulation demo (no hardware, ~10 minutes)

```bash
python3 -m backend.main          # with frontend built: cd frontend && npm ci && npm run build
```

Open `http://localhost:8000`, log in (password in `data/initial_admin_password`).

1. **Overview** — point out the SIMULATION badge, simulated device health,
   live camera with the red object detected.
2. **Sensors** — obstacle radar with stop/slow zones.
3. **Navigation → Manual control** — drive with W/A/S/D; drive at the wall
   and show the robot stopping in the stop zone (*Telemetry → Limited by*).
4. **E-STOP** — press it while driving; teleop is refused (409) until
   **Reset**.
5. **Mapping → Start autonomous exploration** — watch the map grow; note the
   EXPERIMENTAL label (no LiDAR). Stop after ~2 min and save as `demo`.
6. **Navigation** — click a free spot on the map; the A* path appears and
   the robot follows it.
7. **Color Sorting → Start** — the robot finds objects, picks them, the bin
   sensor confirms, history fills in. Show the success/failed chart.
8. **Object Detection** — sample HSV thresholds from the camera, show the mask.
9. **Event Logs** — every action above is logged; admin sees the audit log.

Headless alternatives: `scripts/sim_explore.py`, `scripts/sim_sort.py`,
`scripts/benchmark.py --sorting`.

## B. Hardware demo

Preparation (day before):

* Charge both batteries; run `scripts/hardware_check.py`.
* Calibrate colours **in the demo room's lighting** (calibration §5).
* Prepare 8–10 matte objects (red, green, blue, yellow + 1–2 others) of a
  size the gripper holds (~4–5 cm), and 5 labelled bins.
* Clear a 3 × 3 m area; remove reflective/soft obstacles that confuse
  ultrasonics, or use them deliberately to discuss limitations.

Script:

1. Power on; show the e-stop working **without** the dashboard (press →
   wheels lose power even when commanded).
2. Dashboard: HARDWARE badge, all devices `ok`, battery voltage.
3. Manual drive; obstacle stop with a box placed in front.
4. Mapping (with LiDAR: ROS 2 SLAM Toolbox; without: experimental grid).
5. Navigate to two named locations (`home`, `dock`).
6. Sorting run of 10 objects; read out verified successes vs failures from
   the history and compare with what is physically in the bins.
7. Pull a sensor cable during manual drive → forward motion blocked, alert
   shown (sensor disconnection handling).

Record the numbers in `docs/test-report.md` (hardware table) and
`docs/benchmark-report.md` (Run 2).
