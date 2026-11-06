# Can this laptop run Gazebo for the rest of the project?

PC assessment for the warehouse AMR project, 2026-11-06: `main` after Phase 2 part 1 (the
generated warehouse), measured with a separate build outside the repository; nothing in the
repository was changed for it.

Revised after an independent review: one missing run added (three robots with Nav2 at the 1 ms
step with unique IMU names, section 5.3), and the statements on the cause of the low RTF, swap,
heat, the 5 ms result, camera frame loss, ArUco false detections and the bag figures reworded to
match what the data show. The other numbers are unchanged; the reviewer re-derived them from the
raw data and repeated the one-robot run (RTF 1.00, Gazebo 0.68 cores against 0.70 here).

## 1. The answer

**One robot (Phases 2-8): yes. Three robots (Phases 9-12): only with changes, and even then it is tight.**

- **Phases 2-8 (one robot) fit.** Today's stack runs in real time (RTF 1.00) on about 1 of the 12
  threads. The heaviest single-robot setup I could build (Nav2, both side cameras and an ArUco
  detector) still ran at RTF 0.99 on about 6 threads. Phases 2-6 are comfortable; Phases 7-8b work
  with care, because they leave only 2-4 threads spare (section 6). The exception is the
  **Gazebo window**: with it open, even one robot dropped to RTF 0.92, because the integrated GPU
  was 85 % busy. Use RViz for anything heavier than a quick look.
- **Phases 9-12 (three robots) do not fit as the world is set up today.** Three robots with Nav2 ran
  at RTF 0.59-0.68; the plan's gate is 0.7. The cause is the world's 1 ms physics step. I checked
  this with a clean comparison: the same robots (unique IMU names) with only the step changed gave
  RTF 0.68 at 1 ms, 1.00 at 2 ms and 1.00 at 5 ms with Nav2 standing, so the IMU name clash (plan
  risk R2) was not the cause. The step costs CPU in two places. The clock is most likely the larger
  one: Gazebo publishes `/clock` after every step (1000 times a second at 1 ms), every ROS node on
  sim time receives each message, and the three Nav2 stacks, which do no physics, used 5.0 cores at
  1 ms against 2.0 at 5 ms. Gazebo's own physics is the smaller one: 2.25 cores against 0.97. I did
  not test a clock relay, so the clock's share is inferred, not proven.
  - **2 ms step:** three robots with Nav2 navigated reliably at RTF 0.99. Add one robot scanning with
    two cameras and RTF fell to 0.85 (lowest 0.66) with all threads busy. It works, but only just.
  - **5 ms step:** the same showcase-like load held RTF 1.00 for 8 minutes on 6.8 threads. That is a
    lower bound: one robot's navigation kept failing at 5 ms, so its Nav2 was mostly idle. The 5 ms
    step is not validated until that failure is understood.
  - **Verdict: tight.** It needs a bigger, validated physics step, Gazebo without its window (RViz
    instead), and only one scanning robot at a time. All nine cameras at once is not feasible.
- **RAM, the GPU and disk are not the limit, but swap is nearly full.** The project used at most
  2 GiB (PSS) and MemAvailable never fell below 3.9 GiB. Swap, however, rose from 2.4 GiB at the
  start to 3.5-4.0 of 4.0 GiB in the later runs, most likely idle desktop apps being paged out.
  That is not a shortage today, but it leaves no cushion, so close unused apps and browser tabs
  before long runs. The GPU is not a limit in headless runs. Disk has 17 GB free, and about
  11-12 GB more can be freed.
- **The CPU is the limit.** Under load the clocks fall from about 3.5 to 1.9 GHz while the chip
  stays at 84-97 °C whatever the load. I cannot tell whether its power limit or its temperature
  limit sets the clocks (reading the power needs root). On the charger I saw no slowdown over
  8 minutes; the 60-minute soak is untested.

## 2. The machine

| Item | Value |
|---|---|
| Model | HP Pavilion x360 14-ek1xxx |
| CPU | Intel Core i5-1335U (13th gen, 15 W "U" class): 2 performance cores (P, 4 threads, up to 4.6 GHz) + 8 efficiency cores (E, 8 threads, up to 3.4 GHz) = 10 cores, 12 threads |
| RAM | 15.3 GiB. With the usual desktop apps open (Firefox, VS Code, ...): MemAvailable 5.3-7.6 GiB during this session (it varied with the open apps). Swap file 4 GiB: 2.4 GiB in use at the start, 3.5-4.0 GiB (almost full) during the later runs, and 3.9-4.0 GiB at the end of the session |
| GPU | Intel Iris Xe (RPL-U), i915 driver, Mesa 25.2.8, OpenGL 4.6. It has no memory of its own: it shares the system RAM. No NVIDIA GPU |
| Disk | 108 GB system partition, 17 GB free (84 % used). `/tmp` is on the same partition |
| Software | Ubuntu 24.04, kernel 7.0.0-38, ROS 2 Jazzy, Gazebo Harmonic (gz-sim 8.15.0), Nav2 1.3.13, OpenCV 4.6.0 |
| Power during every run | On the charger (AC online; battery 91 % at the start, full by the end), power profile **performance** (`powerprofilesctl get`; energy-performance preference "performance"). The `powersave` governor that `scaling_governor` shows is normal: with the intel_pstate driver it is the dynamic governor, and the profile decides how hard it boosts |
| Temperature | Package 64-71 °C idle; **84-97 °C on average under simulation**, peaks of 100 °C (the chip's limit). The temperature hardly depends on the load (section 4.1, item 5) |
| Power limits | The limits the OS can see (`/sys/class/powercap`: 200 W long term, 39 W short term) are not the ones that apply; the firmware sets the real limit. The actual package power (RAPL energy counter) needs root, so I could not read it |

## 3. How I measured

- **Each scenario** ran in a clean shell (only `/opt/ros/jazzy` and the scratch build), ROS domain 66,
  LOCALHOST discovery, a private Gazebo partition, headless unless stated. After start-up it
  settled for 15-50 s, then a sampler recorded every 2 s for 46-120 s (8 minutes for the
  sustained run). Raw data: `data/*.jsonl` (time series) and `data/*_summary.json`.
- **RTF** (real-time factor): simulated seconds divided by wall-clock seconds over the window,
  from Gazebo's own statistics topic (`/world/warehouse/stats`). 1.00 = real time. The plan's
  Phase 9 gate is RTF ≥ 0.7. "Lowest 2 s" is the worst 2-second interval.
- **Cores**: CPU-seconds per wall-clock second. 1.0 = one of the 12 threads fully busy. Per
  process from `/proc/<pid>/stat` (exact scheduler time). The machine total and the split into
  P- and E-cores come from `/proc/schedstat`. I first used `/proc/stat` for the total, but its
  busy time is sampled on the timer tick, and Gazebo's 1000-per-second loop beat against it:
  the total jumped by ±1.2 cores within seconds. `/proc/schedstat` counts exact nanoseconds.
- **"Stack"** = everything the project itself runs (Gazebo, bridges, robot_state_publisher, EKF,
  Nav2, the ArUco detector). **"Test tools"** = my waypoint driver, goal sender and probes,
  counted separately (they stand in for teleop and the future task executor). The sampler
  itself used 0.03-0.08 cores.
- **Memory**: PSS (a shared library counts once, split between its users) for my processes, and
  the lowest MemAvailable of the window for the whole machine. Gazebo's GPU buffers live in
  system RAM too; MemAvailable includes them.
- **GPU**: busy share of the whole GPU from its idle-state counter (RC6), and render time per
  process from the driver's per-client statistics (DRM fdinfo).
- **Clocks and heat**: average current frequency of the P- and E-cores, package temperature.
- **Driving**: robots drove on fixed routes (robot1 east, then up and down aisle 1 at 0.5 m/s;
  robots 2 and 3 back and forth in their own lanes), with sim-time stamps at 20 Hz as the new
  0.15 s `cmd_vel_timeout` requires. With Nav2, goals in the aisles (a warehouse tour for one
  robot; one aisle per robot for three).
- The ROS 2 engineering skill that my instructions name was not installed in this session, so
  everything here follows standard ROS 2 and Gazebo practice.

## 4. Results

| Scenario | RTF (lowest 2 s) | Busy cores (of 12) | Stack cores | Gazebo server | Nav2 | Other stack | Test tools | My PSS (MiB) | MemAvail min (MiB) | P / E clock (GHz) | Pkg temp mean/max (°C) | GPU busy % |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| S1 idle: desktop apps only | - | 0.1 | 0.00 | 0.00 | 0.00 | 0.00 | 0.00 | 0 | 5343 | 1.7 / 1.2 | 71 / 87 | 0 |
| S2 1 robot standing (today, 1 ms) | 1.00 (1.00) | 1.2 | 1.00 | 0.70 | 0.00 | 0.30 | 0.00 | 548 | 4598 | 3.2 / 3.0 | 96 / 100 | 2 |
| S2 1 robot driving (today, 1 ms) | 1.00 (1.00) | 1.5 | 0.98 | 0.70 | 0.00 | 0.28 | 0.33 | 591 | 4822 | 3.5 / 2.8 | 92 / 98 | 2 |
| S3 3 robots standing (1 ms) | 1.00 (1.00) | 2.7 | 2.43 | 1.51 | 0.00 | 0.92 | 0.00 | 793 | 4493 | 3.3 / 2.6 | 91 / 96 | 4 |
| S3 3 robots driving (1 ms) | 1.00 (0.98) | 3.8 | 2.91 | 1.85 | 0.00 | 1.06 | 0.63 | 836 | 4274 | 3.0 / 2.3 | 92 / 97 | 4 |
| S4 1 robot, cameras idle (lazy) | 1.00 (1.00) | 1.6 | 1.08 | 0.76 | 0.00 | 0.33 | 0.34 | 642 | 4750 | 3.5 / 2.8 | 93 / 98 | 2 |
| S4 1 robot, 2 side cams + ArUco | 1.00 (1.00) | 3.4 | 2.68 | 1.31 | 0.00 | 1.37 | 0.48 | 822 | 4555 | 3.0 / 2.4 | 92 / 96 | 11 |
| S4 3 robots, 9 cameras idle (lazy) | 1.00 (0.98) | 4.5 | 3.47 | 2.03 | 0.00 | 1.44 | 0.68 | 894 | 4468 | 2.8 / 2.1 | 91 / 95 | 4 |
| S4 3 robots, robot1 2 side cams + ArUco | 0.94 (0.86) | 6.8 | 5.55 | 2.73 | 0.00 | 2.82 | 0.82 | 1079 | 4597 | 2.3 / 1.9 | 91 / 94 | 13 |
| S4 3 robots, all 9 cams + ArUco | 0.59 (0.57) | 8.1 | 7.02 | 2.77 | 0.00 | 4.26 | 0.63 | 1343 | 4171 | 2.0 / 1.8 | 93 / 94 | 26 |
| S5 1 robot + Nav2 idle (1 ms) | 1.00 (1.00) | 2.3 | 2.09 | 0.89 | 0.87 | 0.33 | 0.00 | 821 | 5053 | 3.3 / 2.7 | 91 / 95 | 2 |
| S5 1 robot + Nav2 navigating (1 ms) | 1.00 (0.99) | 3.3 | 2.58 | 1.06 | 1.10 | 0.41 | 0.46 | 883 | 5009 | 3.1 / 2.4 | 92 / 97 | 2 |
| S5 1 robot + Nav2 + 2 cams ArUco (1 ms) | 0.99 (0.96) | 5.8 | 4.81 | 1.65 | 1.47 | 1.69 | 0.59 | 1084 | 4906 | 2.7 / 2.0 | 88 / 91 | 12 |
| S5 3 robots + 3 Nav2 idle (1 ms)* | 0.67 (0.59) | 9.5 | 8.88 | 2.31 | 5.12 | 1.45 | 0.00 | 1574 | 4186 | 1.9 / 1.7 | 92 / 94 | 3 |
| S5 3 robots + 3 Nav2 navigating (1 ms)* | 0.59 (0.50) | 9.8 | 8.59 | 2.21 | 5.02 | 1.36 | 0.74 | 1662 | 4186 | 1.9 / 1.6 | 94 / 95 | 3 |
| S5 3 robots + 3 Nav2 idle (1 ms, unique IMU names)† | 0.68 (0.66) | 9.5 | 9.02 | 2.25 | 4.99 | 1.78 | 0.00 | 1670 | 5354 | 2.1 / 1.7 | 86 / 88 | 3 |
| S5 3 robots + 3 Nav2 navigating (1 ms, unique IMU names)† | 0.59 (0.51) | 10.0 | 8.77 | 2.16 | 4.95 | 1.66 | 0.74 | 1755 | 4933 | 2.0 / 1.7 | 86 / 88 | 3 |
| S5 3 robots + 3 Nav2 idle (2 ms) | 1.00 (0.97) | 8.6 | 7.76 | 2.08 | 4.16 | 1.52 | 0.00 | 1668 | 4407 | 2.2 / 1.8 | 89 / 90 | 8 |
| S5 3 robots + 3 Nav2 navigating (2 ms) | 0.99 (0.96) | 9.6 | 8.46 | 2.25 | 4.63 | 1.58 | 0.67 | 1753 | 4464 | 2.0 / 1.7 | 89 / 90 | 5 |
| S5 3 robots + 3 Nav2 idle (5 ms) | 1.00 (1.00) | 4.0 | 3.72 | 0.97 | 2.02 | 0.73 | 0.00 | 1668 | 4332 | 3.1 / 2.5 | 92 / 97 | 5 |
| S5 3 robots + 3 Nav2 navigating (5 ms) | 1.00 (1.00) | 4.8 | 4.19 | 1.06 | 2.35 | 0.77 | 0.29 | 1755 | 4011 | 2.9 / 2.3 | 93 / 97 | 5 |
| S5 3 robots + 3 Nav2 + robot1 2 cams ArUco (2 ms) | 0.85 (0.66) | 10.7 | 9.31 | 2.38 | 4.52 | 2.41 | 0.64 | 1951 | 4844 | 1.9 / 1.6 | 88 / 90 | 13 |
| S6 sustained, 8 min: 3 robots + 3 Nav2 + robot1 2 cams ArUco (5 ms) | 1.00 (0.95) | 6.8 | 5.94 | 1.55 | 2.82 | 1.58 | 0.37 | 1952 | 4067 | 2.6 / 2.0 | 88 / 91 | 16 |
| S7 GUI: 1 robot, Gazebo window + RViz (1 ms) | 0.92 (0.80) | 7.2 | 4.57 | 3.01 | 0.00 | 1.56 | 0.78 | 1079 | 5564 | 2.1 / 1.7 | 84 / 87 | 85 |
| S7 GUI: 3 robots, Gazebo window + RViz (1 ms) | 0.63 (0.54) | 8.7 | 5.99 | 3.85 | 0.00 | 2.14 | 0.73 | 1380 | 5420 | 2.1 / 1.8 | 86 / 87 | 88 |

Columns: "Stack cores" = everything the project runs; "Gazebo server" includes the Gazebo
window's process in the GUI rows; "Other stack" = bridges, EKF, robot_state_publisher, the ArUco
node and, in the GUI rows, RViz; "Test tools" = my driver or goal sender (not part of the stack).
"Busy cores" is the whole machine, the desktop apps included (0.1-0.6 cores in headless runs; in
the GUI runs GNOME Shell and Xwayland used 1.2-1.4 cores to draw the windows).

\* 1 ms, three Nav2 stacks: robot2's IMU clash (plan risk R2, section 5.5) broke its localization
in this run, so its navigation was not representative; the idle line (Nav2 up, no goals) is
not affected by that.

† Added after review: the same 1 ms world with my scratch robot, which gives each robot its own
IMU name (as in the 2 ms and 5 ms runs), so the step is now the only difference between the 1, 2
and 5 ms rows. No clash was logged and all 23 goals succeeded (8, 8 and 7). The scratch robot also
carries the three idle lazy cameras, which is why its bridges cost about 0.35 cores more.

### 4.1 What the numbers say

1. **Today's simulation is light.** One robot in the warehouse, headless, runs at RTF 1.00 on
   about 1.0 core (Gazebo 0.70 of it) and 0.55 GiB. Three robots driving: RTF 1.00 (lowest 2 s:
   0.98) on 2.9 cores and 0.84 GiB.
2. **The CPU is the only real limit.** Memory: the heaviest run used 1.95 GiB (PSS) and
   MemAvailable never fell below 3.9 GiB (4011 MiB), so the 1.5 GiB guard never fired and RAM did not
   run short. **Swap is the weak spot**: it held 2.4 GiB at the start (S1) and rose to 3.5-4.0 of 4.0
   GiB in the later runs (4019 MiB at the end of the GUI run). The kernel paged out up to 19 000
   pages (74 MiB) in one run (S5 5 ms navigating) and paged back in up to 4 752 (S7 three robots with
   windows). What it pages out is most likely idle desktop apps, not the simulation, but a full
   swap leaves no cushion: if more apps are opened during a run, RAM is all that is left. GPU: 2 % busy
   with the lidars, 11-13 % with two cameras rendering, 26 % with nine.
3. **The hidden cost is the clock.** The warehouse world steps its physics every 1 ms, and
   Gazebo publishes `/clock` after every step: 1000 messages per second, delivered to every ROS
   node that runs on sim time. Measured on idle nodes that do nothing else: a C++ node pays 0.045
   cores, a Python node **0.27 cores**; at a 5 ms step 0.012 and 0.068; without sim time zero.
   A Nav2 stack runs more than a dozen nodes in its container, so Nav2 costs 0.87 cores while standing still,
   and my small Python driver cost 0.33 cores, nearly all of it for the clock.
4. **Three robots with three Nav2 stacks do not fit at today's 1 ms step:** RTF 0.67-0.68 standing
   and 0.59 navigating (lowest 2 s: 0.50-0.51), 9.5-10.0 of 12 threads busy. The plan's Phase 9
   gate is 0.7. This holds with and without the IMU name clash: the repository robot (clash) gave
   0.67 and 0.59, my scratch robot (unique names) 0.68 and 0.59 with all 23 goals succeeded. **With
   a 5 ms step the same setup runs at RTF 1.00 on 4.8 threads**; with 2 ms at RTF 0.99 on 9.6
   threads (no room left). Where the step's cost goes, with Nav2 standing at 1 / 2 / 5 ms: Nav2
   5.0 / 4.2 / 2.0 cores, Gazebo 2.25 / 2.08 / 0.97. Nav2 does no physics, so its share is most
   likely the clock; Gazebo's share is its own physics and sensors. Adding one robot scanning with
   two cameras and ArUco: 2 ms gives RTF 0.85 (lowest 0.66) on 10.7 threads, all goals succeeded;
   5 ms gives RTF 1.00 on 6.8 threads, but robot2's navigation failed repeatedly (section 5.3), so
   6.8 is a lower bound.
5. **The laptop slows down as it gets busier.** With 1-2 threads busy the P-cores average about
   3.5 GHz and the E-cores 2.9 GHz; with 4 threads 3.0 / 2.3; with about 7 threads 2.3-2.6 / 1.9-2.0; with 9.5
   threads 1.9 / 1.6. The temperature, by contrast, barely follows the load: the lightest runs were
   the hottest (S2 standing, 1.2 threads: 96 °C on average at 3.2 GHz; the clock probe, 1.0-1.6
   threads: 96-97 °C), and the heaviest ran at 88-94 °C at 1.9 GHz. The chip seems to be held near
   a fixed limit and spends it on high clocks for a few threads or low clocks for many. Whether that
   limit is power or temperature I cannot tell: the package power needs root to read. Either way
   every extra node also slows all the others, and about 9.5-10 busy threads is the practical
   ceiling.
6. **Cameras: idle ones are free, busy ones are expensive.** With the lazy bridges an idle camera
   costs nothing measurable (+0.06 cores, GPU unchanged): plan risk R3's assumption holds. Two
   side cameras with ArUco add about 1.6 cores for one robot (Gazebo +0.55, camera bridge +0.17,
   the ArUco node 0.8) at RTF 1.00. With three robots at 1 ms the same two cameras pull RTF down
   to 0.94 (lowest 0.86). All nine cameras at once: RTF 0.59 even without Nav2.
7. **Camera frames and ArUco time.** The detector received 8-9 frames/s per camera with one robot
   and 7-8 with three, not the configured 10. ArUco (OpenCV 4.6, subpixel corners) took 7-10 ms per
   640x480 frame on average, p95 12.5-18 ms: inside the plan's p95 ≤ 20 ms. The world has no tags
   yet (they come in Phase 7), so these are times for images without real tags, and the p95 grows
   with load. In the 8-minute run the detector nevertheless reported one marker on each side camera
   (ids 182 and 102, in 8 354 frames). These are **false positives**: DICT_5X5_250 with subpixel
   corners found a "tag" in shelf or floor texture. That is rare (0.02 % of frames), but Phase 7
   should check that a single detection cannot create a false inventory entry, for example by
   requiring the same id over several frames or checking it against the expected ids.
8. **Plan risk R2 is real.** With three robots whose IMUs are all named `imu_sensor`, one robot's
   controller manager registered two IMUs ("already existing key imu_sensor/..."). In the 1 ms Nav2
   run robot2 then lost its localization: it believed it stood inside shelf rows B/C and every plan
   failed with "Start occupied" (0 of 106 goals). With a unique IMU name per robot (scratch robot)
   all goals of all three robots succeeded at 1 ms (23 of 23) and at 2 ms (34 of 34). The robot2
   failures at 5 ms happened with unique names, so they have a different cause.
9. **The Gazebo window is expensive on this GPU.** One robot with the Gazebo window and RViz: RTF
   0.92 (lowest 0.80), GPU 85 % busy, 7.2 threads busy (Gazebo server 1.8 instead of 0.70, the
   window 1.2, RViz 0.7, GNOME Shell 1.2 and Xwayland 0.2 to draw it). Three robots: RTF 0.63. The
   GPU shares the chip's power budget with the CPU, so the CPU clocks fell to 2.1 / 1.7 GHz as well.
   RViz alone costs about 0.7 threads and 2-3 % of the GPU.
10. **No slowdown seen over 8 minutes.** In the 8-minute run (three robots, three Nav2 stacks, one
    robot scanning, 5 ms step) RTF stayed at 1.00 in every minute. The package stayed at 87-88 °C on
    average and the clocks at 2.5-3.0 / 2.0-2.1 GHz from the first sampled minute to the last. Two
    limits on that: sampling started after a 2-minute start-up, so I did not see how quickly the
    chip got there, and the chip was already hot from the earlier runs (the temperature depends on
    what ran before). Whether it holds for the 60-minute soak (Phase 5) is untested.

## 5. Details

### 5.1 Where the CPU goes today (1 robot, headless, driving, 1 ms step)

| Process | Cores | RSS (MB) |
|---|---|---|
| Gazebo server (physics, lidar rendering, the ros2_control controller manager inside it) | 0.70 | 514 |
| `/clock` bridge (1000 messages/s) | 0.10 | 50 |
| scan + ground-truth bridge | 0.06 | 53 |
| EKF | 0.08 | 38 |
| robot_state_publisher | 0.04 | 33 |
| **Stack** | **0.98** | PSS 0.55 GiB |
| My Python driver (test tool, stands in for teleop) | 0.33 | 69 |

Gazebo's busiest thread (the simulation loop) used 0.33 cores, so one robot is far from a
single-thread limit; it ran on a P-core only about 30 % of the time. Three robots: Gazebo 1.85,
bridges 0.43, EKFs 0.40, robot_state_publishers 0.24, total 2.9 cores; the loop thread 0.66.

### 5.2 Cameras (PERCEPTION.md §3.2: 640x480 RGB, 10 Hz, lazy bridges, 1 ms step)

| Setup (all robots driving) | RTF (lowest 2 s) | Busy threads | Gazebo | All bridges | ArUco node | GPU busy | Gazebo GPU memory | Frames/s received per camera | ArUco ms mean / p95 |
|---|---|---|---|---|---|---|---|---|---|
| 1 robot, 3 cameras idle | 1.00 (1.00) | 1.6 | 0.76 | 0.20 | - | 2 % | - | - | - |
| 1 robot, 2 side cameras + ArUco | 1.00 (1.00) | 3.4 | 1.31 | 0.36 | 0.80 | 11 % | 243 MiB | 8.2 and 9.1 | 7.6 / 13.8 and 6.9 / 12.5 |
| 1 robot + Nav2 + 2 side cameras + ArUco | 0.99 (0.96) | 5.8 | 1.65 | 0.46 | 0.94 | 12 % | 287 MiB | 7.7 and 8.3 | 8.7 / 14.9 and 7.5 / 14.5 |
| 3 robots, 9 cameras idle | 1.00 (0.98) | 4.5 | 2.03 | 0.75 | - | 4 % | 129 MiB | - | - |
| 3 robots, robot1's 2 side cameras + ArUco | 0.94 (0.86) | 6.8 | 2.73 | 0.96 | 1.00 | 13 % | 247 MiB | 7.0 and 7.7 | 9.6 / 16.8 and 10.0 / 17.9 |
| 3 robots, all 9 cameras + ArUco | 0.59 (0.57) | 8.1 | 2.77 | 0.98 | 2.56 | 26 % | 546 MiB | 4.1-5.8 | 5.2-9.1 / 5.7-17.3 |

- Without cameras the Gazebo server used 0.70 (1 robot) and 1.85 (3 robots): idle cameras add
  0.06-0.2 cores and no GPU time. Each idle lazy camera bridge process still costs about 0.09 cores,
  almost all of it the `/clock` subscription (section 5.4).
- At RTF 0.59 a 10 Hz camera can deliver at most 5.9 frames per wall-clock second, so in the
  9-camera case the frame rate followed the slow simulation. At RTF 1.00 about 10-20 % of the frames
  never reached the detector. At least part of that is lost in transport (921 KB images over DDS):
  the bag recorder in section 5.8 also missed frames and reported "Number of messages lost on the
  transport layer: 16" for a 9-second camera bag. Whether the renderer also skips frames I did not
  check.
- The ArUco node is a Python stand-in (the detection itself is OpenCV's C++ code) in its own
  process, fed by a separate bridge process. The plan's C++ component sharing a container with the
  bridges (plan §2.5) should cost less. The images held shelves but no real tags (two false
  detections in the 8-minute run, section 4.1 item 7).

### 5.3 Nav2 (AMCL, Smac 2D, Rotation Shim + RPP, one composed container per robot)

| Setup | RTF (lowest 2 s) | Busy threads | Nav2 | Gazebo | Goals succeeded | Nav2 PSS per robot |
|---|---|---|---|---|---|---|
| 1 robot, Nav2 up, standing (1 ms) | 1.00 (1.00) | 2.3 | 0.87 | 0.89 | - | 224 MiB |
| 1 robot, warehouse tour (1 ms) | 1.00 (0.99) | 3.3 | 1.10 | 1.06 | 8 of 8 | 231 MiB |
| 3 robots, standing (1 ms) | **0.67 (0.59)** | 9.5 | 5.12 | 2.31 | - | 221 MiB |
| 3 robots, one aisle each (1 ms) | **0.59 (0.50)** | 9.8 | 5.02 | 2.21 | robot1 8/8, robot2 0/106 (R2), robot3 0/0 | 229 MiB |
| 3 robots, standing (1 ms, unique IMU names) | **0.68 (0.66)** | 9.5 | 4.99 | 2.25 | - | 223 MiB |
| 3 robots, one aisle each (1 ms, unique IMU names) | **0.59 (0.51)** | 10.0 | 4.95 | 2.16 | 23 of 23 | 229 MiB |
| 3 robots, standing (2 ms) | 1.00 (0.97) | 8.6 | 4.16 | 2.08 | - | 222 MiB |
| 3 robots, one aisle each (2 ms) | 0.99 (0.96) | 9.6 | 4.63 | 2.25 | 34 of 34 | 228 MiB |
| 3 robots, standing (5 ms) | 1.00 (1.00) | 4.0 | 2.02 | 0.97 | - | 223 MiB |
| 3 robots, one aisle each (5 ms) | 1.00 (1.00) | 4.8 | 2.35 | 1.06 | 29 of 38 (robot2: 6 of 15) | 230 MiB |

- The 2 ms and 5 ms runs and the "unique IMU names" 1 ms rows used my scratch robot with a unique
  IMU name per robot (section 5.5); the first two 1 ms three-robot rows used the repository's
  robot. Comparing the two 1 ms pairs: the clash broke robot2's navigation but did not change the
  load or the RTF (0.67 / 0.68 standing, 0.59 / 0.59 navigating).
- At 1 ms the three stacks would need roughly 14-17 threads to run in real time (busy threads
  divided by RTF: 9.5 / 0.68 = 14 standing, 9.8 / 0.59 = 17 navigating). This is a rough linear
  guess; the clocks fall further as the load rises, so the true figure may be higher. The laptop
  delivers about 9.5-10, so Gazebo slows down until the work fits.
- At 5 ms robot2 aborted 9 of 15 goals after 5 successful ones ("RegulatedPurePursuitController
  detected collision ahead!", then "Controller patience exceeded") in an aisle no other robot used.
  At 2 ms the same routes succeeded 34 of 34. I did not find the cause; treat it as a reason to
  validate the larger step (recommendation 1), not as proof that 5 ms is safe.
- Nav2 shut down with a segmentation fault in its container after SIGINT in every run (a known
  teardown problem); nothing was left running.

### 5.4 What `/clock` costs (1 robot standing, plus four idle probe nodes)

| Idle node | 1 ms step (1000 clock messages/s) | 5 ms step (200/s) |
|---|---|---|
| C++ node, `use_sim_time:=true` | 0.045 cores | 0.012 cores |
| C++ node, `use_sim_time:=false` | 0.000 | 0.000 |
| Python (rclpy) node, `use_sim_time:=true` | **0.27 cores** | 0.068 cores |
| Python (rclpy) node, `use_sim_time:=false` | 0.000 | 0.000 |
| Gazebo server (1 robot standing) | 0.75 | 0.34 |
| EKF / robot_state_publisher / all bridges | 0.09 / 0.05 / 0.15 | 0.04 / 0.02 / 0.07 |

Every node of the project runs on sim time (plan §6), and today's world steps every 1 ms, so each
node pays for 1000 clock messages a second whether it needs them or not.

### 5.5 Plan risk R2 (IMU names), seen in practice

With three robots, one robot's controller manager logged "Tried to insert StateInterface with
already existing key imu_sensor/..." (robot3 in the first run, robot2 in the Nav2 run): it found
two IMUs called `imu_sensor` in the world. In the 1 ms Nav2 run robot2 then localized itself inside
shelf rows B/C. My scratch robot renames the IMU per robot (`<namespace>_imu_sensor` in the gz
sensor and in the ros2_control `<sensor>`) and passes the name to `imu_sensor_broadcaster` through
the spawner (`--controller-ros-args "-r ~/imu:=imu -p sensor_name:=robot2_imu_sensor"`). With that,
no clash was logged and all three robots navigated.

### 5.6 Sustained run: does the laptop slow down as it heats up?

Three robots, three Nav2 stacks, robot1's two side cameras with ArUco, 5 ms step, headless, on the
charger (battery full), performance profile, sampled for 8 minutes after a 2-minute start-up:

| Minute | RTF (lowest 2 s) | Busy threads | P / E clock (GHz) | Package °C mean / max | MemAvailable min (MiB) | GPU busy |
|---|---|---|---|---|---|---|
| 1 | 1.000 (1.000) | 7.0 | 2.55 / 1.98 | 87.8 / 90 | 4164 | 16 % |
| 2 | 1.000 (1.000) | 7.1 | 2.52 / 1.96 | 87.6 / 90 | 4330 | 15 % |
| 3 | 1.000 (0.994) | 7.1 | 2.48 / 1.97 | 87.7 / 90 | 4067 | 16 % |
| 4 | 0.997 (0.946) | 7.3 | 2.48 / 1.97 | 87.5 / 91 | 4315 | 18 % |
| 5 | 1.000 (0.998) | 6.8 | 2.67 / 2.05 | 87.8 / 90 | 4768 | 16 % |
| 6 | 1.000 (1.000) | 6.4 | 2.64 / 2.05 | 87.6 / 91 | 4860 | 15 % |
| 7 | 1.000 (0.994) | 6.5 | 2.86 / 2.10 | 87.3 / 90 | 4683 | 15 % |
| 8 | 1.000 (0.999) | 6.3 | 2.99 / 2.14 | 88.1 / 90 | 4875 | 15 % |

- Only 2 of 228 two-second intervals were below 0.97, both during a burst at 9.9 busy threads in
  minute 4. Stack: Nav2 2.82, Gazebo 1.55, ArUco 0.61, bridges 0.52, EKFs 0.30,
  robot_state_publishers 0.16 threads; 1.95 GiB PSS.
- Goals in this run: robot1 25 of 28, robot3 44 of 44, robot2 17 of 433 (it kept ending up with
  "Start occupied", the 5 ms problem of section 5.3; most of its 416 failed goals were aborted
  almost at once). robot2's Nav2 therefore planned and steered far less than a healthy robot, so
  the 6.8 threads and the Nav2 2.82 above are a lower bound for three working robots. How much
  lower: at 5 ms, three Nav2 stacks used 2.02 cores standing and 2.35 navigating, so a healthy
  robot2 would add something like 0.1-0.3 threads. The frames reached the detector at 7.9 per
  second per camera; ArUco took 9.4-10.2 ms on average, with a p95 of 18.9-19.5 ms, and reported
  the two false detections described in section 4.1 item 7.
- What this run shows about heat: no slowdown over the 8 sampled minutes (the clocks even rose
  slightly as the busy threads fell from 7.3 to 6.3). It does not show how the chip warmed up (the
  first 2 minutes were start-up, not sampled), nor a 60-minute run.
- The same load at 2 ms (2 minutes, not 8): RTF 0.85 (lowest 0.66), 10.7 threads busy, clocks 1.9 /
  1.6 GHz, every goal succeeded (35 of 35), 5.3-5.6 frames/s per camera, ArUco p95 26 ms.

### 5.7 With windows (1 ms step, robots driving, about 1.5 minutes each)

| Setup | RTF (lowest 2 s) | Busy threads | Gazebo server / window | RViz | Desktop drawing (GNOME Shell + Xwayland) | GPU busy | GPU memory (Gazebo window / desktop) |
|---|---|---|---|---|---|---|---|
| 1 robot, `sim.launch.py world:=warehouse` (Gazebo window + RViz, the default) | 0.92 (0.80) | 7.2 | 1.80 / 1.21 | 0.69 | 1.40 | 85 % | 450 / 644 MiB |
| 3 robots, Gazebo window + RViz for robot1 | 0.63 (0.54) | 8.7 | 2.39 / 1.46 | 0.65 | 1.23 | 88 % | 469 / 636 MiB |
| for comparison: 1 robot headless | 1.00 (1.00) | 1.5 | 0.70 / - | - | - | 2 % | - |
| for comparison: 3 robots headless | 1.00 (0.98) | 3.8 | 1.85 / - | - | - | 4 % | - |

The window draws the 3D scene continuously on the integrated GPU. The GPU is then nearly saturated,
and because it shares the chip's power and heat budget with the CPU, everything else slows down too. RViz draws far
less (2-3 % of the GPU). For a teleop session this means the simulation probably ran at
about 0.9 of real time: still smooth to drive, but not real time.

### 5.8 Disk: 17 GB free against what the later phases add

Measured today (`df`, `du`, `docker system df`):

| Where | Size | Note |
|---|---|---|
| Free on the system partition | **17.1 GB** of 108 GB | `/tmp` and Docker are on the same partition |
| Workspace `build/`, `install/`, `log/` | 3.4 MB, 2.0 MB, 2.0 MB | No C++ package yet; symlink installs |
| `~/.ros/log` | 2.3 GB | Logs of every launch so far; safe to delete old ones |
| Docker | images 9.8 GB (3.4 GB reclaimable), build cache 8.3 GB (0.8 GB reclaimable) | Three tags of the 6.5 GB dev image, `python:3.12`, `ros:jazzy-ros-base` |
| `~/.cache` | 9.9 GB | VS Code C++ IntelliSense 5.0 GB, Raspberry Pi imager 3.1 GB, Chrome 1.0 GB, pip 0.3 GB |
| Trash | 2.1 GB | |
| The measuring session's temporary files (`/tmp`) | 0.67 GB | Gone after a reboot |

Bag sizes, measured with two real MCAP bags of robot1 driving (`s8_bags.sh`). I deleted the bags
to save disk, and the sizes and frame counts were printed to the terminal only, so they cannot be
re-derived from the saved files; the recorder logs (`logs/s8_rec_*.log`) are kept.

| Bag profile | Measured | Per minute | Per hour |
|---|---|---|---|
| No images: scan, tf, tf_static, odom, odom/wheel, imu, joint_states, ground_truth/odom, cmd_vel | 7.0 MB in 28.6 s (3 messages lost) | **15 MB** | 0.9 GB |
| The same + robot1's two side cameras (raw 640x480 RGB) | 146 MB in 8.8 s (16 messages lost) | **about 1 GB** | about 60 GB |

The camera figure rests on only 8.8 seconds, so treat it as a rough rate. It is also slightly low:
the recorder got only 8.3 and 9.5 camera frames per second instead of 10 and reported 16 messages
lost on the transport layer. At a full 10 frames/s, two raw cameras write about 1.1 GB per minute
(2 × 10 × 921 KB per second). The losses point to the transport (large images over DDS) as at least
part of the cause of the missing frames in section 5.2; the renderer is not ruled out.

What the later phases add (estimates, except the bag rates):

| Item | Phase | Size |
|---|---|---|
| C++ packages (task executor, order and fleet managers, perception, battery, ...) built with debug info | 4-11 | about 1-2 GB for `build/` + `install/` (typical ament C++ packages are 20-150 MB each) |
| ROS logs | all | grow by every launch; already 2.3 GB |
| Bags without images (benchmarks, localization evaluation) | 2-12 | see the measured rate above |
| Bags with raw camera images (perception regression) | 7-8 | see the measured rate above: keep them seconds long, or compress |
| Maps and filter masks | 2-3 | under 1 MB each (the 636 x 428 map is 0.27 MB) |
| 8b dataset (ML report: v1 about 3,000 images, JPEG) | 8b | 0.1-0.3 GB; ONNX model tens of MB; `onnxruntime-vendor` about 0.1-0.2 GB |
| MoveIt and its setup assistant (apt) | 13 | several hundred MB |
| GIFs (≤ 3 MB each) and narrated MP4s | 1-12 | up to a few hundred MB per video before upload |
| The GUI dev container (`osrf/ros:jazzy-desktop-full`) if it is used | any | several GB; the existing dev image is already 6.5 GB |

Verdict on disk: 17 GB is enough for the code, builds, maps, the 8b dataset and short bags.
It is not enough for careless camera bags, which fill it in minutes. Freeing the 11-12 GB
listed above gives comfortable room.

## 6. Projection per phase

Verdict scale: **comfortable** = RTF 1.00 with about 4 or more threads to spare; **works with
care** = RTF about 0.95-1.00 measured or expected, under the conditions given; **tight** = it runs,
but RTF is between 0.7 and 0.95 or no threads are left, so any extra load drops it; **not
feasible** = RTF below the plan's 0.7 gate as measured. "Measured" means a row of section 4; the
rest is an estimate from those rows (each Python node on sim time: 0.27 threads at 1 ms, 0.07 at
5 ms; each idle C++ node: 0.045 and 0.012).

| Phase | What runs | Basis | Expected load | Verdict | Depends on |
|---|---|---|---|---|---|
| **2** map + AMCL | 1 robot, slam_toolbox once, then AMCL; teleop (Python) | Measured today's stack: 1.0 thread, RTF 1.00. slam_toolbox not measured | 1.5-3 threads | **Comfortable** | The Gazebo window is fine for teleop and mapping (RTF 0.92), but run the 5-minute localization evaluation headless so its timing is real time |
| **3** Nav2, keepout and speed filters, collision monitor, Rev C | 1 robot | Measured 1 robot + Nav2: 2.6 threads, RTF 1.00, 8 of 8 goals | 3-3.5 threads (+4 filter nodes, maybe a second lidar) | **Comfortable** | Headless for the 50-goal benchmark. The optional MPPI benchmark was not measured (MPPI is the CPU-heavy controller, plan D6) |
| **4a-4c** BT executor, orders, KPIs, Flask dashboard | 1 robot + about 3 C++ nodes + 3 Python nodes on sim time + a browser tab | Estimate from the clock probe | 4-5 threads | **Comfortable** | Each Python node on sim time costs 0.27 threads at 1 ms; keep them few |
| **5** docking, battery, diagnostics, 60-minute soak | As 4 + docking server (50 Hz), battery_sim, aggregator | Estimate; the 8-minute run showed no thermal drift (60 minutes untested) | 4.5-5 threads | **Comfortable** | On the charger; vents free for the 60-minute soak |
| **6** hardening, M1 video | As 5 + screen recording | Recording not measured | about 5 threads + the recorder | **Comfortable** | Record RViz rather than the Gazebo window; try hardware video encoding (the Iris Xe has it, but I did not test recording at all) |
| **7-8** cameras, ArUco, patrol | 1 robot + Nav2 + 2 side cameras + detector (front camera only while docking) | Measured: RTF 0.99 (lowest 0.96), 5.8 threads, ArUco p95 ≤ 15 ms, 7.7-8.3 frames/s | 6-6.5 threads with the Phase 4-6 nodes | **Works with care** | Gazebo headless + RViz; cameras lazy, one detector input at a time; the C++ detector component (cheaper than my Python stand-in); check the frame rate in P7 |
| **8b** CPU ONNX detector | As 7-8 + 2-thread inference, 2 Hz per side camera | Not measured; PERCEPTION.md §5.5 estimates 0.3-1.0 threads | 7-7.5 threads, leaving 2-3 threads below the 9.5-10 ceiling; clocks then near 2.3 / 1.9 GHz, so inference sits at its slow end | **Works with care** (the least margin of the one-robot phases) | 2 intra-op threads, no spinning; scan at 0.3 m/s if 3 inferences/s cannot be held; a bigger physics step frees 1-2 threads |
| **9-12** three robots, fleet and traffic managers, dashboard, 15-min showcase | 3 robots, 3 Nav2 stacks, 1 robot scanning, about 10 more C++ nodes, dashboard + browser | Measured, three Nav2 stacks: **1 ms** RTF 0.59-0.68 (with or without the IMU clash); **2 ms** RTF 0.99, with a scanning robot 0.85 (lowest 0.66) on 10.7 threads; **5 ms** RTF 1.00 on 4.8 threads, with a scanning robot 1.00 for 8 min on 6.8 threads (a lower bound: robot2's navigation was failing for most of that run, so its Nav2 did less work than in a healthy fleet) | 2 ms: over 11 threads with the M3 nodes (more than the laptop delivers); 5 ms: 7.5-8.5 threads (the margin covers the lower-bound effect, which was 0.3 threads between idle and navigating Nav2 at 5 ms) | **1 ms: not feasible. 2 ms: tight (RTF about 0.8). 5 ms: works with care, but not validated: navigation must pass the checks of recommendation 1 (robot2 failed at 5 ms)** | The physics step (recommendation 1); Gazebo headless + RViz (the window alone cut 3 robots to RTF 0.63); R2 fixed; one scanning robot at a time, never all 9 cameras (RTF 0.59); dashboard browser on another device or a light tab |
| **13** (stretch) arm + MoveIt | 1 robot + arm joints + move_group | Not measured | As 7-8 + 1-2 threads while planning | **Works with care** with one robot; **tight** with three | As 7-8 |

Two caveats apply to every line. First, the project's own nodes (task executor, managers,
dashboard) are not written yet, so their cost is estimated. Second, the plan's C++ perception
component should be cheaper than the Python stand-in I measured, while tags in the images will
make detection slightly more expensive than the images without real tags measured here.

## 7. Recommendations

1. **Make the physics step bigger before Phase 9; test it while Phase 3 brings Nav2.** It is the
   only change that makes the final phases fit. Use 2 ms or 5 ms: both divide the 10 ms
   controller and IMU period, 4 ms does not. The world generator in `warehouse_layout` writes the
   `<physics>` block. With three robots, three Nav2 stacks and one scanning robot:
   - **2 ms:** RTF 0.85 on 10.7 threads, 35 of 35 goals succeeded.
   - **5 ms:** RTF 1.00 on 6.8 threads, but robot2 kept ending up "Start occupied" (17 of 433
     goals). Because robot2's Nav2 was mostly idle, 6.8 threads is a lower bound for a healthy
     fleet.

   Find out why (compare its ground-truth pose with AMCL's estimate when it fails), then keep the
   largest step that passes these checks:
   - the Phase 1 odometry test (< 1 % straight, < 2 % in a turn) and the IMU σ check;
   - the Nav2 50-goal benchmark;
   - from Phase 5, the docking test (≤ 2 cm).

   If only 2 ms passes, buy back CPU with item 5 and with untested options: a relay that passes
   every 5th `/clock` message to ROS, 5 Hz cameras (PERCEPTION.md §3.5), fewer AMCL particles, or
   another RMW (plan A5).
2. **For more than one robot, run Gazebo without its window and watch in RViz.** Today
   `headless:=true` also turns RViz off; a launch option for "Gazebo headless, RViz on" would make
   this the easy default. Use the Gazebo window for single-robot work and short demos.
3. **Fix plan risk R2 at the start of Phase 9**: one IMU name per robot (`<namespace>_imu_sensor` in
   the gz sensor and the ros2_control `<sensor>`, and `sensor_name` for `imu_sensor_broadcaster`,
   for example through the spawner, as in section 5.5:
   `--controller-ros-args "-r ~/imu:=imu -p sensor_name:=<namespace>_imu_sensor"`). Tested in
   scratch: no clash, all three robots navigated.
4. **Perception**: keep the cameras lazy (verified to cost nothing when idle), let only one robot
   scan at a time, never run all nine cameras, and build `aruco_detector` as a C++ component in one
   container with the camera bridges, as plan §2.5 says. In Phase 7, check the received frame rate
   (7-9 frames/s measured instead of 10; part of the loss is in transport, section 5.8) and the
   detector's p95 under the real load (12.5-18 ms measured on images without real tags). Expect rare
   false detections (2 in 8 354 frames here), so confirm an id over several frames before it counts.
   PERCEPTION.md's 5 Hz fallback (at least 11 views per tag) is there if needed.
5. **Prefer C++ for nodes that always run, and give sim time only to nodes that need it.** At the 1 ms
   step every Python node on sim time costs 0.27 cores just to receive the clock (C++: 0.045). Teleop,
   the dashboard, the order generator, the KPI tracker and the scenario tools are Python. Bridges
   need no sim time at all (an idle camera bridge cost 0.09 cores).
6. **Run on the charger with the performance profile**, as in every measurement here. Keep the air
   vents free (hard surface): the chip sits at 84-97 °C under simulation. On battery or in
   power-saver mode the clocks will be lower (not measured). No slowdown was seen over 8 minutes;
   the 60-minute soak in Phase 5 is the first real test of a long run, so record RTF and the
   clocks during it.
7. **Watch swap and close what is not needed before long runs.** Swap held 2.4 GiB at the
   start of this session and 3.9-4.0 of 4.0 GiB at the end, with up to 7.6 GiB MemAvailable. RAM was
   never short, but with swap full there is no cushion. Close unused browser tabs and apps before
   a soak or a three-robot run, and check with `free -h`.
8. **Disk: free space before Phase 7.** 17 GB is free; about 11-12 GB more can be freed without
   touching the project: `~/.ros/log` (2.3 GB of old ROS logs), the Trash (2.1 GB), the Raspberry Pi
   imager cache (3.1 GB), unused Docker images and build cache (about 4 GB reclaimable; three dev
   image tags and 8.3 GB of build cache in total), and VS Code's C++ cache (5.0 GB; it can be
   capped). Never record raw camera images for long (section 5.8).
9. **Record RTF in every scenario** (plan R3 already asks for it). The sampler from this assessment
   can serve as the tool. Treat RTF below 0.9 as a warning and below 0.7 as a failure (the Phase 9
   gate).

## 8. What I did not measure

- **Nodes that do not exist yet**: slam_toolbox mapping (Phase 2), the keepout and speed filter
  servers and a possible second lidar (Phase 3), the MPPI benchmark, the BT task executor, order
  manager, generator, KPI tracker and Flask dashboard (Phase 4), docking, battery_sim and diagnostics
  (Phase 5), the fleet and traffic managers (Phases 10-11), the arm and MoveIt (Phase 13). Their
  share in section 6 is an estimate from the measured cost of similar nodes (the `/clock` probe and
  Nav2), not a measurement.
- **The real perception stack**: I used a Python ArUco node in its own process, fed by a separate
  bridge process (an upper bound), not the plan's C++ component in one container with the bridges,
  and the world had no tags yet. The 8b ONNX detector does not exist; its numbers come from
  PERCEPTION.md §5.5.
- **Physics fidelity at 2 ms and 5 ms**: I only saw that robots drove and Nav2 goals succeeded (with
  one unexplained robot2 problem at 5 ms, section 5.3). The odometry accuracy test, the IMU sigma
  check and docking precision were not run at the larger steps.
- **Throttling `/clock` without changing the physics step** (for example a small relay that forwards
  every 5th clock message): not tried.
- **Exactly why cameras delivered 7-9 instead of 10 frames/s**: the bag recorder's "lost on the
  transport layer" count shows that the transport loses some frames; whether the renderer also skips
  frames I did not check.
- **The bag figures from the saved files**: the bags were deleted and their sizes printed to the
  terminal only (section 5.8); only the recorder logs are kept.
- **Battery power and the balanced or power-saver profiles**: every run was on the charger with the
  performance profile.
- **Runs longer than 8 minutes** (the Phase 5 60-minute soak, the Phase 12 15-minute showcase), and
  the first 2 minutes of the 8-minute run (start-up, not sampled).
- **Screen recording** for the demo videos (including hardware video encoding), and a browser tab
  showing the dashboard.
- **Without root**: the GPU's per-engine load (`intel_gpu_top`) and the CPU package power (RAPL).
  GPU busy time comes from the idle counter and per-process render time instead. Without the
  package power I cannot tell whether a power limit or a temperature limit sets the clocks.
- **Other load**: before every run I checked for Gazebo or ROS processes that were not
  mine and found none; the desktop apps used 0.1-0.6 cores during the headless runs (Firefox, VS Code,
  GNOME Shell).

## 9. Files

The measuring scripts and raw data were kept in a temporary folder outside the repository and
are not committed: they hard-code that folder and were written for this one assessment. What they
were, so the measurements can be repeated:

| File | What it was |
|---|---|
| `sampler.py` | The measuring tool (CPU per process group, memory, GPU, clocks, temperature, RTF, memory guard). Recommendation 10 suggests turning it into a project tool |
| `lib.sh` | Clean environment, start/stop in own process groups, leftover check, memory guard |
| `s2_one_robot.sh`, `s3_three_robots.sh`, `s4_cameras.sh`, `s5_nav2.sh`, `s7_gui.sh`, `s8_bags.sh`, `s9_clock_probe.sh` | One script per scenario |
| `driver.py`, `goals.py`, `aruco_node.py`, `probe_idle.py` | Test tools: waypoint driver, Nav2 goal sender, ArUco load test, idle probe node |
| `cfg/robot_cam.launch.py`, `cfg/camera_bridge.yaml` | A test robot with the three Rev E cameras (lazy bridges) and unique IMU names |
| `cfg/warehouse_2ms.sdf`, `cfg/warehouse_5ms.sdf`, `cfg/gazebo_file.launch.py` | Copies of the warehouse world with a 2 ms and a 5 ms physics step |
| `make_map.py`, `map/warehouse_raster.*` | Nav2 map rasterised from `layout.yaml` at 0.05 m |
| `make_nav2_params.py`, `cfg/nav2_params_robot*.yaml` | Nav2 parameters (plan D6: Rotation Shim + RPP, Smac 2D, AMCL) |
| `analyze.py`, `data/`, `logs/` | The results table, raw samples, summaries and launch logs |
