# 3-Phase Oil Separator Simulation

## Description
Python-based simulation of a horizontal 3-phase oil separator, models the water, oil, and gas levels as well as internal pressure. Includes 3 actuator models to control the levels of each substance. 

## Features
- Simulation of liquid levels and gas pressures.
- PID controlled pressure regulating valve actuator.
- 'Bang-Bang' (On/Off) control for fluid valves.
- Live-plotting example with Matplotlib.
- MQTT functionality for publishing simulation data.

## Installation
1. Clone the repository:
   ```bash
   git clone [https://github.com/sonofodin14/Oil_Separator_Demo](https://github.com/sonofodin14/Oil_Separator_Demo)
   cd oil_separator_sim
   ```
2. Create and activate a virtual environment (recommended):
   ```bash
   python -m venv venv
   # On Windows:
   venv\Scripts\activate
   # On macOS/Linux:
   source venv/bin/activate
   ```
3. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Usage
Run main.py for an automatic run of the simulation with MQTT disabled

or

Run the simulation from the command line:
```bash
py oil_separator_sim.py --broker [options]
```