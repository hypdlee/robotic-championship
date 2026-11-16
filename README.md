# IEEE Global Robot Championships Malta 2026: Team Genesis

Team Genesis, University of Malta. This repository holds everything we are building for the championship, held on 28 and 29 November 2026 at Montekristo Estate Arena, Malta (rulebook V0.6, draft). It is split into three parts that match the three things the rulebook asks of a team: an AI Pitch Proposal, the Robot Arm AI Challenge, and the Arena 4 Games.

| Folder | What it is | Rulebook | Run it |
|---|---|---|---|
| [`ai-pitch/`](ai-pitch/) | AI Pitch Proposal demo: a hospital trolley robot with the MII people-aware safety layer, eleven ward deployment cases, the 3-minute pitch video and the 10-slide deck | Section 2 | open `ai-pitch/index.html` |
| [`robo-arms/`](robo-arms/) | Real-time simulator of the Robot Arm AI Challenge: two SO-101-style arms through every round, scored live, with a LeRobot-style observation and policy panel | Section 5.8 | open `robo-arms/index.html` |
| [`robo-games/`](robo-games/) | Real-time simulator of the Arena 4 Games we compete in: Time of Fall, Climate CHAOS, Weight in Line and Hanging Hazard, with a robot design panel, compliance checks and the MII layer as driver assist | Section 4 | open `robo-games/index.html` |

**30-second explainer videos:** [`media/robo-arms_30s.mp4`](media/robo-arms_30s.mp4) and [`media/robo-games_30s.mp4`](media/robo-games_30s.mp4), recorded from the simulators with captions and narration.

**Run online (no download):**

- Robo arms: https://claude.ai/artifact/EbszmDB3qzK2uCmCrVV2Wo
- Robo games: https://claude.ai/artifact/1ausj2KtMiUPyJk2ZwApYc

All three demos are single `index.html` files with no dependencies: download the file and open it in any browser, no server or install needed. GitHub shows HTML files as code, so it cannot run the pages itself; use the download icon on the file view, or enable GitHub Pages (below).

Each folder has its own README with full details. What follows is a summary of all three.

---

## 1. `ai-pitch/`: AI Pitch Proposal demo

**Theme:** Assistive Robotics for Healthcare and Wellbeing (rule 2.1.3 a). The proposal shows how a hospital trolley robot behaves around people using the MII people-aware safety layer together with a battery power supervisor, and why that matters for ward deployment.

**Contents**

- `index.html`: the whole interactive demo in one file (same page as `deployment-cases/deployment-cases.html`).
- `media/Genesis_Ward_Safety_Pitch.mp4`: 3-minute pitch video built from the simulation (synthetic narrator, burnt-in captions).
- `Ward_Deployment_Cases.pptx`: 10-slide animated deck for the 5-minute day-one presentation (rule 2.1.5).
- `deployment-cases/src/`: page template, simulation core (`cases_core.js`), battery core (`core.js`) and the `build.sh` that inlines three.js and the Genesis CAD data.

**What it shows**

Eleven scripted ward situations, each runnable on three versions of the safety layer (v4.0, v5.2, v6.0) and two sensor layouts (the current mast camera, and the proposed front LiDAR at 0.29 m with the camera lowered to 1.0 m): corridor crowds, a visitor crossing, ghost alerts on glass and IV poles, a nurse already present at boot, stale alerts, a night ward, a slow computer at 0.5 fps, a tired battery with a 20 kg trolley, a side approach, and a parked bed forcing a detour. Each case has a 3D ward view (follow, robot camera with detections, overhead), the danger state and alerts, an alert log with false alerts tagged, a scrubbable timeline and a pass or fail acceptance test. Floor zones mark 1.0 m (MOVE AWAY) and 1.5 m (TOO CLOSE).

**Read before relying on it:** these are simulations driven by the layer's rules, not recordings or real detector output. Only the distance and speed rules of the danger ladder are modelled; the detector, intent classifier and facing check are not. The LiDAR layout, map, planner and speed response are proposed extensions; the MII layer itself only warns.

---

## 2. `robo-arms/`: Robot Arm AI Challenge simulator

A browser simulation of the whole Robot Arm AI Challenge (section 5.8), useful for understanding the rounds, the scoring and what a learning-based policy has to cope with. Two SO-101-style arms (5 DOF plus gripper, 500 mm apart, 500 mm reach) play on the 1700 x 1200 mm workspace with the H-rig top camera and the side pole camera.

**Rounds covered, with the rulebook scoring reproduced**

- Game 1, Pick-and-Place: Colour Sort, Shape Sort, Dynamic Sort. 32 printed objects (4 colours x 4 shapes x 2 sizes), points 1/2/3/5 small and 2/3/4/5 large, wrong-bin deductions, +10 completion bonus, 0 to 100 clamp, game score = (R1 + R2 + R3) / 3. In Dynamic Sort a judge moves bins and nudges objects while the arms run.
- Game 2, Two-Arm Coordination: Pouring (grams in bowl vs spilled, time bands) and Folding (two-stage fold, alignment bands, 5 s validation). Both arms must contribute meaningfully in every round. Wrap and Pack is not simulated because the team is not playing it.
- Final, Tower Construction Duel: two teams, blue build-only blocks and red dual-use blocks, 30 s whistle command windows where the operator may only give high-level intent, throws only once the own tower is at least 6 blocks, red block recovery by the opponent, collapses and penalties.

**Panels:** policy cards (ACT, Diffusion Policy, SmolVLA) that change chunk length, inference latency, placement accuracy and whether operator commands in the final are understood; `observation.images.top` and `.side` camera feeds; joint-state bars for both arms; an action-chunk plot; episode and frame counters; an episode log with rule references per event; a per-round insight on why that round matters for policy and data design; an E-STOP (rule 5.7.2).

**Simplifications:** planar inverse kinematics with the gripper kept vertical, no contact physics, a simple collision-avoidance rule between teammate arms, bin locking so two grippers never share a bin. Where the rulebook is inconsistent (four cloths listed in Folding against sub-caps that assume three) the published maximums are enforced.

---

## 3. `robo-games/`: Arena 4 Games simulator

A browser simulation of the category Team Genesis is entered in (section 4), played by a Genesis robot against a scripted opponent on the 6 x 6 m plywood arena with the 4 x 4 m sticker.

**Games covered, with the rulebook limits reproduced**

1. Time of Fall (4.9): chained tug of war over the pit, 60 s, tow point must fit the 11 mm connector or the robot is disqualified for the game, no add-ons. Pull force is min(motor force, mu x m x g), so traction and ballast decide it; the slip indicator shows wasted motor power. Timeouts are settled by distance from the edge.
2. Climate CHAOS (4.10): a randomised city maze (the real map is only revealed on the day), flood cells that cause a washout and end the run, sinkholes, parked vehicles, a forest fire front that escalates after each tower, and towers A/B/C with 5/4/3 columns, 60 cm clearance, +50/+100/+150 points and -25/-50/-100 if collapsed. Cutouts count once the whole object is inside the safe zone. 5 minutes.
3. Weight in Line (4.11): six weights per colour (5 + 10 + 15 + 20 + 20 + 30 = 100), ring placement tiers 100 / 75 / 50 / 0 percent, opponent weights in your corner count negative, pit pushes only after all your weights are scored (own pit 100, opponent pit 50), illegal pushes lose the match, yellow and red cards. 5 minutes.
4. Hanging Hazard (4.12): three rugby balls scored in the opponent's pocket, swinging hanging weights that can flip a robot (a flipped robot loses), locks limited to 10 s with repeated locks counting against you. 8 minutes.

**Panels:** a Genesis robot design panel (mass, tyre grip, drive force, footprint, height, tow point, add-ons) with live compliance checks against rules 4.3, 4.4, 4.9.3 and the 60 cm tower clearance; the MII safety layer shown as the driver assist allowed by rule 4.1.8 (danger tiers, 1.0 m and 1.5 m envelopes, throttle limiting near tower columns and closing pendulums; switch it off to watch collision and flip rates rise); a championship-path card that follows the qualifying and knockout structure of rule 4.8 and fills in Genesis results; a kill switch (rule 4.4.2.2); three opponent skill levels.

**Simplifications:** top-down 2D kinematics with a simple pushing model between robots, maze walls are planner constraints rather than physics, tower contact is probabilistic, and the referee's cards arrive at random intervals.

---

## Rulebook facts the whole team should know

- Event: 28 and 29 November 2026, Montekristo Estate Arena. Opening ceremony Saturday 14:00, awards Sunday 18:00. Robots are registered at the venue the day before (rule 8.2) and handed over before the competition (rule 8.1); late robots do not compete.
- Arena team: up to 3 members plus 1 mentor (rules 1.4.1, 1.4.3). Only one person may be in both the Arena and Robot Arm teams (1.4.6). All members must attend the whole event (1.1.5).
- Robot: Featherweight, at most 20 kg in the heaviest add-on setup, base at most 80 x 80 cm, height at most 50 cm, 24 V DC, sealed batteries, a visible and marked kill switch that fully disconnects drive, add-ons and comms (4.3, 4.4). The robot must retract to its initial configuration after every game or it loses that match (4.3.4).
- Automation: driver-assist tools are allowed only with emergency stops and a manual override (4.1.8); autonomous add-ons need strict safety procedures and should be discussed with the organisers first (4.6.10, 1.3.2). Anything not explicitly permitted is prohibited (1.2.4).
- Deliverables: AI pitch proposal and the making-of video were due 6 November 2026; the making-of video is compulsory and its absence costs qualifying points (section 7). An inventory sheet of panels, add-ons and batteries is due at least one day before the competition; only declared items may be used.

Sources: [rulebook V0.6 (PDF)](https://www.ieeemalta.org/wp-content/uploads/2026/11/Robot-Championship-2026-V0.6-1.pdf), [IEEE Malta championship page](https://www.ieeemalta.org/ieee-robot-championship-2026/).

## Live demos (GitHub Pages)

With Pages enabled (repo Settings, Pages, Deploy from a branch, `main`, folder `/ (root)`), the pages run at:

- https://hypdlee.github.io/robotic-championship/ (landing page)
- https://hypdlee.github.io/robotic-championship/ai-pitch/
- https://hypdlee.github.io/robotic-championship/robo-arms/
- https://hypdlee.github.io/robotic-championship/robo-games/

## Team

Team Genesis: John Paul Bayliss, Miriam Cini, Merve Guneyhan, James Gatt, Ankit Kumar. Supervisors: Dr Conrad Attard (University of Malta) and Prof. Filippo Sanfilippo (University of Agder).
