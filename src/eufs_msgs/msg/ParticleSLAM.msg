# Represents the state of a particle-based SLAM algorithm

# The index position current best guess of the SLAM algorithm in `particles`
uint64 best_particle_idx

# The set of particles of the SLAM algorithm
Particle[] particles