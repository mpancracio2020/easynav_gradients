# Algorithm

## 1. Navigation graph

Each NavCel is a triangle with a cached centroid c_i. Neighbor transitions are taken from NavMap. When robot_width > 0, a transition must share exactly one edge whose length (with a small numerical tolerance) is at least robot_width.

## 2. Potential propagation

Dijkstra starts at the goal cell g with U(g)=0. Other potentials start at infinity. For each traversable transition i to j, the relaxation is:

```text
U(j) = min(U(j), U(i) + ||c_j - c_i||_2)
```

The centroid distance is three-dimensional. Unreachable cells retain infinite potential.

## 3. Gradient estimation

For finite-potential neighboring cells j, define the horizontal displacement d_ij and its length r_ij. Skip near-zero displacements. With N_i the remaining valid neighbors:

```text
G_i = -(1 / |N_i|) sum_j [(U(j)-U(i))/(r_ij+0.001)] * (d_ij/r_ij)
```

The negative sign directs the vector toward decreasing potential. If there are no valid neighbors, the gradient is zero. A subsequent pass blends each gradient with the mean of itself and its finite-potential traversable neighbors using smoothing_factor.

## 4. Discrete descent

For each candidate neighbor with strictly lower potential, normalize the current gradient and horizontal direction to the neighbor, then score:

```text
score(i,j) = dot(normalize(G_i), normalize(d_ij))
             + potential_drop_weight * (U(i)-U(j))
```

Choose the maximum score. If the gradient norm is near zero or no valid transition is selected, choose a traversable neighbor of minimum strictly lower potential. Abort with an empty path if the goal cell is not reached within the iteration limit. Preserve the requested final orientation.

## 5. Elastic Band

The endpoints remain fixed during the smoothing iteration. For intermediate waypoint i, with original position x_i^0:

```text
candidate_xy = x_i.xy + w_data * (x_i^0.xy - x_i.xy)
                      + w_smooth * (x_(i-1).xy + x_(i+1).xy - 2*x_i.xy)
```

Project the candidate onto its associated triangle using closest_point_on_triangle. Iterate with separate current and next buffers until maximum displacement is below the convergence threshold or the iteration budget is exhausted. Recompute intermediate orientations from the path tangent and preserve the goal orientation. The implementation uses single-precision points during smoothing.

## 6. Caching and invalidation

Geometry/topology signatures hash vertex coordinates and triangle indices. A changed signature invalidates cached neighbor topology, centroids, traversable transitions, potentials and gradients. A goal position change invalidates potential and gradient fields; an orientation-only change regenerates the path. When continuous replanning is disabled, an existing path can be reused if map and goal are unchanged.

## Scope and limitations

This is a discrete mesh-based planner, not a continuous Eikonal solver. It does not guarantee globally optimal smoothed paths. Waypoint projection alone does not validate complete connecting segments or robot footprints. Cached path invalidation on cancellation and invalid inputs, fallback localization distance, and geometry validation require additional testing.
