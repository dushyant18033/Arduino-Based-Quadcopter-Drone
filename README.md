# Arduino-Based-Quadcopter-Drone
Arduino based firmware for flight control of a quadcopter drone

This code is still under development.

## Browser flight simulator

The repository includes an educational 3D quadcopter simulator in `simulator/`. It models motor thrust, gravity, drag, ground contact, and angle stabilization; adjust the pilot controls and orbit/zoom the scene to explore their effects. It is self-contained and does **not** compile or execute the Arduino firmware.

Serve the repository root locally (ES modules require HTTP), then open `/simulator/`:

```sh
python3 -m http.server 8000
```

Open `http://localhost:8000/simulator/`. No external libraries or network connection are required. Run the physics checks with `node --test simulator/physics.test.mjs`.
