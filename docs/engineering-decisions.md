# Engineering decisions

Each entry is a problem I hit, what I chose, and the evidence in the repository.

## 1. Learn the floor from the run instead of hard-coding it
**Problem:** floor color changes with the room, the time of day and the camera's exposure.
**Decision:** the first frames build a 2-D hue-saturation histogram (30 x 32 bins) inside a trapezoid in front of the car; later frames are scored by back-projection. The value channel is left out, so shadows and exposure changes do not break the model, and pixels too dark or too bright for hue to be reliable are counted separately as "unknown" instead of being read as obstacle.
**Evidence:** `vision/ground.py`; the HSV exploration is in the history (`feat(vision): explora o espaço de cor HSV`).

## 2. Derive the threshold from the data
**Problem:** a fixed threshold failed on videos where the floor color varied (lighting, reflections, tile patterns).
**Decision:** the threshold is the 5th percentile of the back-projection inside the region of interest during warm-up (floored at 0.02), so by construction about 95% of the known floor passes.
**Evidence:** `calibrate_thr` in `vision/ground.py`; `python -m vision.coverage_log --thr auto` prints the derived value (34.5 on the sofa video).

## 3. Decide on relative confidence, with an explicit "uncertain" state
**Problem:** the absolute floor probability depends on the video and the light, so one fixed threshold on it is fragile.
**Decision:** `r` = mean central probability / reference, where the reference is the median over the first 0.9 s (the first frame alone cannot be used, because the histogram trivially matches itself). Three states: `FREE`, `UNCERTAIN` (bad light or intermediate `r`: no obstacle is declared) and `OBSTACLE` (very low `r` *and* a well-exposed center). Leaving `OBSTACLE` needs `r >= 0.4`, higher than the entry threshold of 0.2, so the state does not chatter.
**Evidence:** `vision/decision.py`, thresholds in `config.py`.

## 4. Express time in seconds, not frames
**Problem:** the original logic counted frames; the phone and the PC do not run at the same rate.
**Decision:** every persistence and timeout is in seconds, and the reference window is time-based.
**Evidence:** `tests/test_decision.py` feeds the same signal at 20, 10 and 5 fps. Obstacle entry and exit land at 3.1 s / 5.4 s, 3.1 s / 5.4 s and 3.2 s / 5.4 s.

## 5. Add an independent cue where color is weak
**Problem:** a wall painted close to the floor's color produces a high floor probability, so the color model alone under-reacts. Vision cannot fully close this case, which is why the range sensor stays in the loop (decision 6).
**Decision:** the edge probe looks for the baseboard line and feeds the controller a separate "edge is near" flag, so two cues with different failure modes are combined.
**Evidence:** `vision/edge_probe.py`, logs in `assets/simulation_Scar/Reframed_videos/docs/logs/teste_canny_edge/`.

## 6. Let the range sensor overrule vision, and let the firmware not trust Python
**Problem:** a vision-only decision can be wrong, and a phone or Wi-Fi link can stall at any time.
**Decision:** in the controller the ultrasonic reading refutes or confirms a vision obstacle. Independently, the ESP32 stops at 20 cm, stops if no command arrives for 500 ms, and only drives after being armed with the BOOT button held for 3 s. Firmware 0.3 also survives a brownout and reports the reset reason. The soft AP means no router is needed.
**Evidence:** `control/controller.py`, `car_firmware/car_firmware.ino`, `tests/test_controller.py`.

## 7. Hide the hardware behind small interfaces
**Problem:** testing the control loop on the car costs a battery cycle and a lot of floor time.
**Decision:** the controller only sees a range head and a drive link. Mock versions replay a recorded video against a scripted sensor scenario (with optional noise, dropouts and faults), so the same `Runner` is used on the PC, in tests and on the phone.
**Evidence:** `control/hal.py`, `replay/replay_main.py`, `tests/test_main.py`. A replay is open loop: the video does not react to commands, so it validates decisions and timing, not closed-loop behavior.

## 8. Measure before optimizing, and leave cores free
**Problem:** the budget is 100 ms per frame at 10 fps (33 ms at 30 fps) on a 2018 phone.
**Decision:** frames are downscaled to 320 px wide before processing, and OpenCV is limited to one thread. A benchmark per stage showed that segmentation (~5 ms) and the edge probe (~3.5 ms) dominate, while decision and controller cost under 0.05 ms.
**Evidence:** `tools/bench_s9.py`; logs in `docs/logs/bench/`. On the PC, 8.5 ms per frame with 1 thread against 9.6 ms with 12. On the S9, 17.5 to 18.1 ms with 1, 2, 3 or 4 threads, so the extra threads buy nothing and one is used.

## 9. Keep experiments out of the run loop
Optical flow (`vision/flow_probe.py`) was implemented and logged, then deferred: it did not solve the hardest case (the wall whose color is close to the floor's), so it stays in the repository as an experiment, not in the pipeline that drives the car.

