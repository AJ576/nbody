# N-Body Simulation

A dynamic and visually engaging n-body simulation inspired by **'The Three-Body Problem'**. This application simulates gravitational interactions between stars, planets, and particles, supporting features such as zooming, time control, camera movement, and preloaded systems like the Solar System and Alpha Centauri system.

<img src='https://media1.giphy.com/media/v1.Y2lkPTc5MGI3NjExbzdwZnk2OTNhZXZyZHpkYjJ1ejQ3ZnE0ODlnbHl1ZTl6NzMwc2Q3YSZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/JfXHoVupwiljVT866u/giphy.gif' title='Video Walkthrough' alt='Video Walkthrough' />

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
  - **Alpha Centauri**: Binary stars with hypothetical planets and Proxima Centauri with known planet proxima centauri b.  
  - **Particle Cloud**: Dense particle systems for clustering and visualizing formation of planets naturally. (still experimental, large amount of particles slow down the simulation significantly)
- **Velocity Calculations**:  
  - Computes and displays escape velocity and orbital velocity during body addition.
- **Custom Visuals**:  
  - Dynamic rendering of body size and color.  

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
    - Drag velocity is relative w.r.t the focused body and raw velocity otherwise  
  - Hold Left-Click: Incrementally increase body mass.
- `X` : Delete the currently focused body.

### Presets
- `1` : Load Solar System.  
- `2` : Load Alpha Centauri system.  
- `3` : Load particle cloud.

## GIF Previews
1. Solar System Simulation
<img src='https://media2.giphy.com/media/v1.Y2lkPTc5MGI3NjExOTc0OHp3bDloZ3d6MDQ3ZHRveG5nMWt5Y25ldGs0M241M2ZqaXZjNCZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/KVhz36PLsePtgZHKht/giphy.gif' title='Video Walkthrough' alt='Video Walkthrough' />
2. Alpha Centauri Interaction
<img src='https://media4.giphy.com/media/v1.Y2lkPTc5MGI3NjExMnp4MjltaDhxMTVjdHBuejV0Zm81d3c1M3A2OHBnNjFqMXR4ZGQ2cCZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/VIZVjC6nWPvqFMZbdT/giphy.gif' title='Video Walkthrough' alt='Video Walkthrough' />
3. Adding Bodies Dynamically
<img src='https://media0.giphy.com/media/v1.Y2lkPTc5MGI3NjExbWc2NWc1ZnZlajNhazJxYnFpMWQ5aDZ0dmc2NTZ3ZmFmOTRteHJsMSZlcD12MV9pbnRlcm5hbF9naWZfYnlfaWQmY3Q9Zw/eI1ouNWCFBfqGvtw7j/giphy.gif' title='Video Walkthrough' alt='Video Walkthrough' />


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
