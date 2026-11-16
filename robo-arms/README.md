# Robo arms: Robot Arm AI Challenge simulator

Real-time, browser-only simulation of the IEEE Global Robot Championships Malta 2026 **Robot Arm AI Challenge** (rulebook V0.6, section 5.8). Two SO-101-style arms play every round, scored live against the published tables, with a LeRobot-style panel (observation cameras, joint state, action chunks, policy choice, episode log).

**Run it:** download `index.html` and open it in any browser. No server or install needed.

## What it covers

- Game 1, Pick-and-Place: Colour Sort, Shape Sort, Dynamic Sort (judge moves bins and nudges objects mid-run). Object points 1/2/3/5 (small) and 2/3/4/5 (large), wrong-bin deductions, +10 completion bonus, 0 to 100 clamp, (R1+R2+R3)/3.
- Game 2, Two-Arm Coordination: Pouring (grams in bowl vs spilled), Folding (two-stage fold, alignment bands, 5 s validation), Wrap and Pack (coverage, boxing, flap closure).
- Final, Tower Construction Duel: two teams, blue and red blocks, 30 s whistle command windows, throws only when the own tower is at least 6 blocks, red block recovery, collapses and penalties.

## Controls

- Toolbar or keys `1` to `7` select a round, `Space` pauses, `N` advances. Autoplay runs all rounds in order.
- Policy cards (ACT, Diffusion, SmolVLA) change chunk length, inference latency, placement accuracy and whether operator commands in the final are understood.
- E-STOP halts all motion (rule 5.7.2) and voids the run.
- Toggles: judge perturbations, action chunk overlay, reach radii, camera rigs.

## Simplifications

Planar inverse kinematics with the gripper kept vertical, no contact physics between arms and objects, and a simple collision-avoidance rule between teammate arms. Where the rulebook is inconsistent (four cloths listed in Folding, caps that assume three) the published maximums are enforced.
