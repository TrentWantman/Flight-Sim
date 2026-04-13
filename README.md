My goal for this project is to build a complete flight simulation. This includes launch, orbit insertion, rendezvous, docking, and landing. The project will feature autonomous guidance, navigation, and control. Every system is written from scratch in C++. I don't plan on using engines or libraries, just physics and math.

I'm using this project to learn / implement the concepts that aerospace and flight software depends on. Some examples are:

- Linear algebra: vectors, rotation matrices, coordinate frame transforms
- Newtonian physics: F=ma, momentum, variable-mass dynamics
- Numerical integration: Euler method, Runge-Kutta (RK4)
- Control theory: PID controllers, closed-loop feedback
- Sensor fusion: triple-redundant voting, Kalman filtering
- Real-time systems: deterministic control loops, multi-threaded architecture
- Lock-free data structures: double-buffered producer-consumer pipelines
- State machines: mission sequencing, phase transitions
- Atmospheric modeling: barometric pressure, altitude-dependent air density
- Aerodynamics: quadratic drag, drag coefficient, cross-sectional area
- Rocket propulsion: Tsiolkovsky ideal rocket equation, specific impulse, exhaust velocity
- Gravity models: flat gravity, inverse square law, two-body problem
- Orbital mechanics: Kepler's laws, orbital elements, circular orbits
- Trajectory planning: gravity turn, ascent profiles
- Delta-v budgets: mission fuel planning, burn scheduling
- Orbital maneuvers: Hohmann transfers, phase angle calculations
- Rendezvous: Clohessy-Wiltshire equations, relative motion dynamics
- Proximity operations: approach trajectories, docking conditions
- Coordinate frames: Earth-centered inertial, body-fixed, local orbital frames
- Guidance, Navigation, and Control (GNC): autonomous mission execution
- HOOTL architecture: hardware-out-of-the-loop simulation

Huge shoutout to Alfonso Gonzalez - Astrodynamics & SE Podcast on youtube for providing extremely well presented lectures on many of these concepts. As well as Daft Punk for creating the music I have been listening to while working on this.
