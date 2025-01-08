# N-Body Simulation

A dynamic and visually engaging n-body simulation inspired by **'The Three-Body Problem'**. This application simulates gravitational interactions between stars, planets, and particles, supporting features such as zooming, time control, camera movement, and preloaded celestial systems like the Solar System and Alpha Centauri.

## Features

- **Realistic Gravitational Physics**:  
  - Leapfrog integration and Plummer softening for stability and accuracy.
- **Celestial Bodies**:  
  - **Stars**: Color-coded by mass and size based on astrophysical principles.
  - **Planets**: Radius and color based on type (rocky, gas giant, etc.).
  - **Particles**: Lightweight objects for additional realism in systems.
- **Interactive UI**:  
  - **Zoom & Scale**: Adjust scale from astronomical units to planetary levels.  
  - **Time Control**: Change simulation speed or pause.  
  - **Camera Movement**: Focus on specific objects or freely navigate the simulation.  
  - **Body Addition**: Dynamically add stars and planets with mass and velocity control via mouse interactions.
  - **Labels**: Mass, velocity, and type of focused bodies displayed on-screen.
- **Preset Systems**:  
  - **Solar System**: Includes the Sun, planets, and Pluto.  
  - **Alpha Centauri**: Binary stars with hypothetical planets and Proxima Centauri.  
  - **Particle Cloud**: Dense particle systems for clustering and chaos. (still experimental, large amount of particles slow down the simulation significantly)
- **Velocity Calculations**:  
  - Compute escape velocity and orbital velocity during body addition.
- **Custom Visuals**:  
  - Dynamic rendering of body size and color.  
  - Background-free clean visualization for focus on objects.

## Controls

### General
- `Space` : Pause/Resume simulation.
- `=` : Zoom in.
- `-` : Zoom out.
- `Arrow Up/Down` : Adjust time speed.
- `R` : Reset the simulation.

### Camera
- `W/A/S/D` : Move camera when unfocused.
- `Arrow Left/Right` : Cycle focus between bodies.
- `U` : Unfocus the camera.

### Body Interaction
- **Add Bodies**:  
  - `C` : Switch to "Star" mode.  
  - `P` : Switch to "Planet" mode.  
  - Left-Click + Drag: Set velocity for a new body based on drag direction.  
  - Hold Left-Click: Incrementally increase body mass.
- `X` : Delete the currently focused body.

### Presets
- `1` : Load Solar System.  
- `2` : Load Alpha Centauri system.  
- `3` : Load particle cloud.

## GIF Previews
1. Solar System Simulation
2. Alpha Centauri Interaction
3. Particle Cloud Chaos
4. Adding Bodies Dynamically

![Simulation Preview](https://via.placeholder.com/800x450?text=GIF+Placeholder)

## Requirements

- Python 3.8+
- Libraries: `pygame`, `numpy`

## How to Run

1. Install required libraries:
   ```bash
   pip install pygame numpy
   ```
2. Run 
   ```
    python main.py
   ```
