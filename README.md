# 3-Phase Oil Separator Simulation

## Description
Python-based simulation of a horizontal 3-phase oil separator, models the water, oil, and gas levels as well as internal pressure. Includes 3 actuator models to control the levels of each substance. 

## Features
- Simulation of liquid levels and gas pressures.
- PID controlled pressure regulating valve actuator.
- 'Bang-Bang' (On/Off) control for fluid valves.
- Live-plotting example with Matplotlib.
- MQTT functionality for publishing simulation data.

## Usage
Run main.py for an automatic run of the simulation with MQTT disabled

or

Run the simulation from the command line:
```bash
py main.py --broker [options]
```