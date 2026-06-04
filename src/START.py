#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
START.py - convenience launcher for the CARLA navigation scripts.

It simply forwards all arguments to the chosen planner script, so you do not
have to type the full path. Both planners share the same command-line flags.

Examples
--------
    # Run the A* planner (default) with defaults:
    python START.py

    # Run the A* planner explicitly, custom start/destination and port:
    python START.py astar --a 0 --b 100 --port 2000

    # Run the Dijkstra planner:
    python START.py dijkstra --a 0 --b 120

Any flag accepted by the target script (--host, --port, --a, --b, --speed,
--vehicle, --no-draw) can be passed after the algorithm name.
"""

import os
import sys
import subprocess

ALGOS = {
    "astar": "carla_astar_navigation.py",
    "dijkstra": "carla_dijkstra_navigation.py",
}


def main():
    args = sys.argv[1:]

    # First positional argument selects the algorithm; default is A*.
    algo = "astar"
    if args and args[0] in ALGOS:
        algo = args[0]
        args = args[1:]
    elif args and args[0] in ("-h", "--help"):
        print(__doc__)
        return

    script = os.path.join(os.path.dirname(os.path.abspath(__file__)), ALGOS[algo])
    print("[START] Launching {} planner -> {}".format(algo, script))
    # Forward the remaining arguments to the target script unchanged.
    sys.exit(subprocess.call([sys.executable, script] + args))


if __name__ == "__main__":
    main()
