# Interactions.jl — A Physics Engine for Julia Games

Julia is a high-performance programming language designed for simulation, visualization, and scientific computing — all essential in game physics.  
**Interactions.jl** is a lightweight 2D/3D physics engine written in Julia, built with game development in mind.

---

## Installation

```julia
julia> ]add Interactions
```

And for the development version

```julia
julia> ]add https://github.com/Gesee-y/Interactions.jl
```

---

## Features

* **Mass-Aggregate Simulation**: Simulate individual or grouped particles.
* **Forces System**: Built-in forces like `SpringForce`, `BuoyancyForce`, and a simple interface to define your own.
* **Dynamic Constraints**: Connect objects using constraints — joints, limits, and more — or define custom ones.
* **Collision Detection**:

  * 2D: Fast grid-based detection using bitboards (O(n) in practice).
  * 3D: Uses spatial partitioning structures.
* **Verlet Integration**: Stable and efficient physics integration method.
* **Rigid and Static Bodies**: Supports both dynamic and immobile objects.

---

## Example

```julia
using Interactions

# Particle2D(inverse_mass, position, velocity, acceleration, force_accumulation, damping)
p = Particle2D(0.005, Vec2f(1.5, 0), Vec2f(-30, 40), Vec2f(20, 0), Vec2f(0, 0), 0.99)

# Update particle position over time
integrate!(p, Δ)
```

---

## License

This package is licensed under the MIT License.

---

## Bug Reports

If you encounter a bug or numerical issue, feel free to open an issue on GitHub — your feedback helps improve the engine.