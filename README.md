# Formation Control and APF Obstacle Avoidance

A modular Python framework for **Multi-Agent Formation Control** combined with **Artificial Potential Fields (APF)** for real-time obstacle avoidance and dynamic formation switching in constrained environments.

---

## Key Features

- **Negative Gradient APF Formulation:** Repulsive forces are derived analytically from the negative gradient of safe-distance distance potentials $-\nabla J_{\text{rep}}(\mathbf{q})$ using the chain rule.
- **Distance-Based Rigidity Control:** Directed mutual distance constraints keep rigid 2D geometric patterns (e.g., regular triangle meshes) without global coordinate broadcasting.
- **Dynamic Topology Switching:** Real-time geometric probes anticipate upcoming narrow corridors/doors and trigger decentralized transitions from 2D formations to a 1D single-file column (and vice versa).
- **Transient & Symmetry Analysis:** Systematic benchmark analyzing reflection equivariance, saddle-point bifurcations, and initial condition chirality in distance-only rigidity graphs.

---

## 1. Multi-Agent Obstacle & Inter-Agent Collision Avoidance

Obstacles and neighbouring agents generate repulsive potential fields $J_{\text{rep}}(\mathbf{q})$ when entering an influence region $\delta_0$:

$$J_{\text{rep}}(\mathbf{q}) = \frac{1}{\beta} \left( \frac{1}{d(\mathbf{q}) - \delta} - \frac{1}{\delta_0 - \delta} \right)^\beta$$

$$\mathbf{F}_{\text{rep}}(\mathbf{q}) = -\nabla J_{\text{rep}}(\mathbf{q}) = \frac{1}{(d(\mathbf{q}) - \delta)^2} \left( \frac{1}{d(\mathbf{q}) - \delta} - \frac{1}{\delta_0 - \delta} \right)^{\beta - 1} \hat{\mathbf{n}}$$

where $\hat{\mathbf{n}} = \frac{\mathbf{q} - \mathbf{c}^\star}{\Vert \mathbf{q} - \mathbf{c}^\star \Vert}$ is the unit normal pointing away from the closest Euclidean projection $\mathbf{c}^\star$ on the obstacle surface, and $\delta$ is the minimum safe clearance distance.

![Two APF Agents Avoiding Each Other](assets/two_agents_apf.gif)

---

## 2. Rigid Formation Control

Formation control is achieved through mutual distance constraints over a directed acyclic graph (DAG). For each constraint between parent $i$ and child $j$ with desired distance $d_{ij}$:

$$\mathbf{u}_{j,\text{form}} = -k (\Vert \mathbf{p}_i - \mathbf{p}_j \Vert^2 - d_{ij}^2)(\mathbf{p}_j - \mathbf{p}_i)$$

![Triangle Formation Control](assets/triangle_formation.gif)

---

## 3. Dynamic Formation Switching in Narrow Bottlenecks

When navigating through narrow passages (e.g. gates, doors), a forward virtual probe detects lateral clearance constraints and coordinates a smooth state transition:

$$\text{Triangle Formation (2D)} \underset{\text{Gap Detected}}{\overset{\text{Traveled Distance}}{\rightleftharpoons}} \text{Line Formation (1D)}$$

![Formation Switching](assets/formation_switching.gif)

---

## 4. Initial Condition Chirality & Reflection Symmetry

Distance-based potential energy surfaces possess symmetric, isometric local minima (**flipped embeddings**). The final geometric orientation (upward vs downward tilt) is governed by the **basin of attraction** of the launch trajectory. To systematically investigate this behavior, seven benchmark scenarios are evaluated:

- **Case A ($y_0 = +0.8$):** Original asymmetric launch; converges to an upward-tilted formation.
- **Case B ($y_0 = -0.8$, $y \to -y$):** Exact spatial reflection of Case A; strictly mirrors trajectories and final formation across $y = 0$.
- **Case C ($y_0 = 0.0$):** Symmetric bilateral launch; collapses onto the $y=0$ saddle point inside the narrow gate, where numerical roundoff breaks symmetry.
- **Case D ($y_0 = -1.0$):** Entire fleet shifted 1.0 unit lower than Case C; approaches the gate from below ($\dot{y} > 0$) and settles into a downward-tilted formation.
- **Case E ($y_0 = +1.0$, $y \to -y$ of D):** Exact reflection of Case D; approaches from above ($\dot{y} < 0$) and strictly mirrors Case D across $y = 0$.
- **Case F ($y_0 = +1.0$):** Entire fleet shifted 1.0 unit higher than Case C; settles into an upward-tilted formation.
- **Case G ($y_0 = -1.0$, $y \to -y$ of F):** Exact reflection of Case F; strictly mirrors Case F across $y = 0$.

### Key Observations & Takeaways:
1. **Strict Reflection Invariance:** Inverting initial transverse positions ($y \to -y$) rigorously mirrors the entire state evolution ($y(t) \to -y(t)$) across $y = 0$, confirming zero algorithmic bias.
2. **Transverse Momentum Governs Basin Attraction:** Launching below the gate axis ($y_0 = -1.0$, Cases D & G) induces positive lateral approach velocity and obstacle interaction forces that channel the post-gate re-expansion into the downward-tilted minimum. Conversely, launching above ($y_0 = +1.0$, Cases E & F) directs the fleet into the upward-tilted minimum.
3. **Topological Invariance Under Agent Permutation:** Cases D and G (and likewise E and F) share identical initial coordinate footprints but with transposed agent indices (top vs bottom wings). Both converge to the exact same geometric formation, proving that global basin dynamics dominate over individual agent assignment permutations.
4. **Information Bottleneck:** Bottleneck passage compresses the 2D formation into a 1D line, but transverse momentum and asymmetric boundary repulsions preserve sufficient phase-space memory to guide post-gate expansion.
5. **Mitigation for Real Robotics:** When unique agent roles (e.g. left vs right wings) are mandatory, pure distance-based control:
   $$\Vert \mathbf{p}_i - \mathbf{p}_j \Vert = d_{ij}$$
   must be augmented with signed bearing/angle constraints or displacement consensus $\mathbf{R}(\theta)\mathbf{z}^*$.

![Formation Symmetry Comparison](assets/formation_symmetry_comparison.png)

---

## Project Structure

```
.
├── assets/
│   ├── simple_agent.gif                    # Single-agent target following
│   ├── two_agents_apf.gif                  # Two-agent APF collision avoidance
│   ├── triangle_formation.gif              # Multi-agent triangle navigation
│   ├── formation_switching.gif             # Dynamic corridor passage
│   └── formation_symmetry_comparison.png   # 7x1 comparative benchmark
├── main.ipynb                              # Interactive Jupyter Notebook
├── README.md                               # Project documentation
└── .gitignore                              # Git tracking configuration
```

---

## Getting Started

### Prerequisites
- Python 3.9+
- `numpy`, `matplotlib`, `shapely`, `pillow`

### Installation & Execution
```bash
# Clone the repository
git clone https://github.com/fsciamma/Formation-Control-and-APF-Obstacle-Avoidance.git
cd Formation-Control-and-APF-Obstacle-Avoidance

# Run the Jupyter Notebook
jupyter notebook main.ipynb
```
