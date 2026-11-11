# Ward deployment cases

Single-file HTML demo (open `deployment-cases.html` in a browser, no server needed) showing nine scripted hospital cases for the Genesis robot: the Chapter 5 people-aware safety layer plus the battery power supervisor.

These are simulations driven by the chapter's rules, not recordings. Robot slowing in response to alerts is a proposed extension.

`src/` holds the template and the simulation and battery cores. `build.sh` inlines three.js, the CAD data and both cores into the single HTML (it expects the Genesis CAD JSON and three.js source at the paths set inside it).
