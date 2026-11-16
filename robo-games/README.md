# Robo games: Arena 4 Games simulator

Real-time, browser-only simulation of the IEEE Global Robot Championships Malta 2026 **Arena 4 Games** category (rulebook V0.6, section 4), played by a Team Genesis robot against a simulated opponent on the 6 x 6 m plywood arena with the 4 x 4 m sticker.

**Run it:** download `index.html` and open it in any browser. No server or install needed.

## Games

1. **Time of Fall** (4.9): chained tug of war over the pit, 60 s, tow point must fit the 11 mm connector, no add-ons. Pull force is min(motor force, mu x m x g), so traction and ballast decide it. Timeout is settled by distance from the edge (4.9.7).
2. **Climate CHAOS** (4.10): a randomised city maze (the real map is revealed on the day) with flood cells (washout ends the run), sinkholes, parked vehicles, an advancing forest fire, and towers A/B/C with 5/4/3 columns, 60 cm clearance, +50/+100/+150 points and -25/-50/-100 if collapsed. Cutouts count once the whole object is inside the safe zone (4.10.6). 5 minutes.
3. **Weight in Line** (4.11): six weights per colour (5 + 10 + 15 + 20 + 20 + 30 = 100), ring placement tiers 100 / 75 / 50 / 0 %, opponent weights in your corner count negative, pit pushes only after all weights are scored (own pit 100, opponent pit 50), illegal pushes lose the match, yellow and red cards. 5 minutes.
4. **Hanging Hazard** (4.12): three rugby balls scored in the opponent's pocket, swinging hanging weights that can flip a robot (a flipped robot loses), locks limited to 10 s, 8 minutes.

## Panels

- **Genesis robot design**: mass, tyre grip, drive force, footprint, height, tow point and add-ons, with live compliance checks against 4.3, 4.4, 4.9.3 and the 60 cm tower clearance. Changing the design changes how the games play out.
- **MII driver assist**: the people-aware safety layer re-used as a driver assist (4.1.8): danger tiers, 1.0 m and 1.5 m envelopes, throttle limiting near tower columns and closing pendulums. Toggle it off to see collision and flip rates rise.
- **Championship path**: qualifying rounds, knockout structure and the retract-to-initial-configuration rule (4.3.4), with Genesis results filled in as games finish.
- **Kill switch** (4.4.2.2) that deactivates drive, add-ons and comms.

## Simplifications

Top-down 2D kinematics with a simple pushing model between robots, no wall physics in the maze (the planner treats walls, flood, sinkholes and vehicles as impassable), probabilistic column contact under towers, and a referee that issues cards at random intervals. Opponent behaviour is scripted at three skill levels.
