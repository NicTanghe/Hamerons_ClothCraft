# Physics-inspired mesh resolution: independent CPU experiment

Branch: `codex/physics-inspired-mesh-resolution`, based on upstream
`7d0f78171757ab0be538c0bb9fa1c66921bb402e`.

This is an original Rust implementation of the **material estimate and static
sizing-map/meshing pipeline** described by Zhang et al., *Physics-inspired
Estimation of Optimal Cloth Mesh Resolution*, SIGGRAPH 2025,
[DOI 10.1145/3721238.3730619](https://doi.org/10.1145/3721238.3730619).
The [full paper](https://wanghmin.github.io/publication/zhang-2025-pie/Zhang-2025-PIE.pdf)
was read, including equations and figures in sections 3--4 and Table 1.
It is a research tool, not an editor backend, automatic physical calibration,
full reproduction of the paper's simulator, or proof of optimal resolution.

## Run

```sh
cargo run --locked --no-default-features -j 2 --example pie_resolution -- --compare
cargo run --locked --no-default-features -j 2 --example pie_resolution -- \
  --input examples/pie_resolution/shirring.json --output output/pie-custom --compare
cargo test --locked --no-default-features -j 2 --test pie_resolution -- --test-threads=2
cargo clippy --locked --no-default-features -j 2 --example pie_resolution --test pie_resolution -- -D warnings
```

All commands are CPU-only. No desktop feature or GPU/window is required.
There are no dependency, solver, production mesher, editable format or default
behavior changes. The new example has private modules reused by its integration
tests. It can be evaluated separately from the coordinated native authoring work.

The default original fixture is a 180 by 240 mm rectangle with an internal
120 mm constraint representing a gather line. Its base fabric has B = 2e-6
N m and E = 110000 Pa. The explicit shirring computation uses sewn/rest length
ratio 0.5, added Young's modulus zero and exponent n = 1, yielding 4.603 mm
source spacing from 9.206 mm base spacing. The transition doubles at 40 mm.
The example does not simulate the gathering forces or produce a draped garment.

The output directory receives:

- `input.json`: resolved research inputs, seed, calibration and budgets.
- `adaptive.json`: material-space vertices, triangles, per-vertex directional
  spacing, named directed constraint chains and measured mesh quality.
- `adaptive.obj`: flat geometry and UVs in metres, with aligned vertex/UV indices.
- `adaptive.svg`: millimetre-scale mesh preview; dark blue denotes finer local
  spacing and orange denotes constraints. Preview colors are scaled per mesh.
- `sizing.csv`: a 101 by 101 bounding-box sampling grid, restricted to material.
- `summary.json`: mesh counts, area, quality and complete CPU generation time.
- With `--compare`, corresponding uniform-base and uniform-fine mesh artifacts.

The JSON is a separate research input/output contract, **not** a new
`GarmentTemplate`, `FittingProject` or persistent measurement representation.
Output indices are rebuildable mesh indices, not editable semantic identities.
The tool does not install a new mesh in a live simulation or transfer state.
Reusing its output in production would require compiling masses, material
elements, hinges, seams and attachments and invalidating dependent state caches.

## Implemented method and units

`examples/pie_resolution/sizing.rs` implements equation 4:

```text
c1 * r^6 + c2 * r^4 = B/E
c1 = 32.08 m^-3
c2 = -1.876e-4 m^-1
B: N m; E: N/m^2 = Pa; r: m
```

Bisection selects the positive monotone branch over the explicit interval
[2.5, 50] mm. Inputs below/above that interval's ratios return the corresponding
endpoint spacing; invalid, zero or overflowing B/E values are rejected. Custom
coefficients must define a positive, monotone branch throughout their interval.
Warp and weft estimates use separate B/E ratios and one shared material-space
axis angle. The calibration is the paper's system-specific empirical choice;
it is not measured or fitted for ClothCraft's constitutive law.

**Do not feed ClothCraft's membrane coefficients directly into E.** They are
in N/m and have a different constitutive interpretation. Converting a sheet
modulus to a volumetric Young's modulus requires an explicit effective thickness
and a constitutive assumption. This tool requires E in Pa rather than guessing
that conversion from garment thickness or inventing calibrated fabric values.

`Fabric::shirring` implements section 4.2.2:
`r_source = rho^n * r(B/(E + E_sh))`. Here `rho` is sewn/rest length, in (0,1],
not `1-rho`. Additional E_sh is in Pa and n is positive. Local sources can
therefore be finer than the material estimator's 2.5 mm floor, as in the paper.
Research field spacings below 1 micrometre are rejected as a resource guard.
The CLI's resolved JSON accepts explicit source spacings; rerunning a JSON file
after changing its fabric does not silently recompute its authored sources.

Section 4.2.1's dimensional wrinkle scaling utilities are also available:

```text
L_tension = prefactor * lambda^2 * sqrt(T/B)
L_compression = prefactor * lambda^(3/2) * (delta/h^2)^(1/4)
```

Lambda, L and h are metres; T is tension in N/m, B is N m and delta is a
dimensionless compressive fraction. The proportionality prefactor is explicit.
Wavelength and mesh spacing are different inputs: this tool does not silently
equate them. `summary.json` includes two labelled numerical scaling examples,
not inferred material calibrations or the default fixture's transition inputs.

## Explicit independent design choices

The paper writes a transition `r + x^m` and derives `m = log(r)/log(L_w)`,
absorbing scale factors into the exponent. Those expressions depend on the
chosen numerical length unit. This implementation instead uses:

```text
h_j(p) = min(base_j, source_ij * (1 + (distance_i(p)/L_i)^exponent_i))
```

This is a deliberate normalized transition, not a literal reproduction of
that exponent fit. It doubles the source spacing exactly at the declared L,
until capped by the base spacing, and changes consistently when every length
unit changes. The default exponent is 1/2 for the tension-like demo; 2/3 is
also tested for the compression-like law. Spacing components combine by
minimum, so sources cannot coarsen the base field. Each source supplies two
directional spacings; all sources share the fabric's axis orientation.

Sources support a material-space polyline, a labelled disk region, or a
forward-facing point with a direction. The latter models the directional
influence convention described for folding points, using a half-plane gate.
That gate can be discontinuous across its plane; no smoothness is claimed there.
Region and polyline transitions are continuous. Source geometry controls the
field only; constraints in `domain.lines` must be declared separately.

The code does not automatically discover folding endpoints/curvature or elbow
compression. Folding source spacings and any parallel fold constraints must
be authored explicitly. Down-filling pressure is not converted into the paper's
empirical exponent: its pressure normalization and scale would require a
separate stated calibration. Actual stitches, fold mechanics, internal pressure,
unexpected collision adaptation and dynamic remeshing are outside this tool.

## Geometry, sampling and checked bounds

`examples/pie_resolution/mesh.rs` accepts a straight sampled CCW outer ring,
clockwise holes and disjoint internal polylines, in material metres. Each ring
segment has a unique semantic name; each internal line has a name. Curves can
be sampled before supplying this input, but the tool does not change or promote
editable curve points. Current ClothCraft domain validation is reused for
holes/internal paths; additional bounded outer-ring validation prevents malformed
indexing and rejects crossings, duplicate names and segments below 1 micrometre.

The sampling process is original code informed by
[Bridson's 2007 sketch](https://www.cs.ubc.ca/~rbridson/docs/bridson-siggraph07-poissondisk.pdf).
It uses deterministic xorshift randomness, an active list and uniform-area
annular candidates in the local directional metric. Endpoint ellipse exclusion
is symmetric. Boundary and internal constraint samples are compulsory and can
be closer together than the Poisson exclusion distance. This variable-spacing
extension does not inherit Bridson's uniform-radius linear-time proof or prove
maximal coverage. A spatial grid accelerates proximity queries.

The existing pinned **Spade 2.15.1** dependency supplies constrained Delaunay
triangulation. Its base metric is fixed by the base directional spacings and
material axis. Local sizing controls sampling/refinement only: the result is
not claimed Delaunay-optimal in a varying Riemannian metric. No fabric energy
or material parameter is passed to Spade itself.

Constraint segments are subdivided until their metric length, evaluated with a
conservative lower spacing bound over the entire segment, is at most one.
Material triangles are refined at their centroids until every face satisfies:

```text
face_material_area / (h0_lower_bound * h1_lower_bound) <= sqrt(3)/4
```

The lower bounds cover a ball enclosing the whole face. Distance to each
source is 1-Lipschitz, so tiny features between ordinary sample locations are
covered. This can overrefine a face, but does not hide a source by checking
only its centroid. For directed sources, a ball intersecting the forward
half-plane conservatively includes their influence.

An exterior frame separates the convex hull from rounded constraint samples,
preventing zero-area hull slivers under rotated metrics. Frame and nonmaterial
faces are excluded. Every returned named path is reconstructed in directed
order, its endpoints are checked, and every chain link must be a real mesh
edge. Whole-domain area is checked to relative tolerance 1e-8. Exhausted point,
depth or refinement-pass budgets return an error, never a partial mesh.

The enforced size condition is **area**, not a maximum edge or minimum angle
guarantee. Actual maximum metric edge and minimum metric angle are reported;
no 25-degree angle bound from the production mesher is claimed. Sliver removal,
variable metric angle optimization and convergence tests on the physical
simulator are future integration gates.

## Evidence and handoff

The 12 focused CPU tests cover the published table, equation residuals,
monotonicity/clamps, invalid units and coefficients, directional estimates,
shirring ratios, wrinkle scaling powers, exact transition doubling, source
ordering, length-unit covariance, conservative region/directional bounds,
repeatable meshes, rotated axes, holes/internal constraints, area and boundary
length conservation, fixed-metric local Delaunay checks, concave material,
small features and explicit budget failure.

Table 1 and the printed equation are not numerically identical for every
rounded entry. In particular, Dark Blue's published B = 1.6e-4, E = 1.3e5 and
the printed coefficients give **18.418278 mm**, whereas the table says 18.5 mm.
The test allows 0.1 mm difference from the rounded table but separately requires
the actual equation's relative residual below 1e-12. We retain this discrepancy
rather than changing the implementation to fit that table entry.

Observed on the original demo with seed 42, CPU development profile:

| Mesh | Vertices | Triangles | Area (m2) | Min metric angle |
|---|---:|---:|---:|---:|
| Uniform base | 1,039 | 1,948 | 0.0432 | 21.37 degrees |
| Adaptive | 1,313 | 2,496 | 0.0432 | 20.72 degrees |
| Uniform fine | 4,062 | 7,866 | 0.0432 | 19.22 degrees |

Adaptive has about 68% fewer vertices than uniform fine on this fixture.
One local run measured approximately 25 ms adaptive, 10 ms uniform base and
39 ms uniform fine for complete CPU mesh generation. These are illustrative
single-run wall times, not a statistical performance result, simulation step
times or rendering timings. SVG output was rendered and visually inspected.
No cloth dynamics, wrinkle quality, GPU work or garment-scale speedup was tested.

CLI acceptance also replayed the saved research input and compared all three
mesh JSON files byte-for-byte after parsing: vertices, faces, boundary chains,
spacings and quality were identical. A 64-vertex budget failed before creating
the requested output directory. Focused strict Clippy and rustfmt checks passed.
The full locked CPU regression command also completed successfully:
`cargo test --locked --no-default-features -j 2 -- --test-threads=2`, including
all 12 new cases and the existing library, integration and documentation tests.

Original implementation and original synthetic fixture; no authors' code,
paper figures, garment assets or EXO source were copied. Existing project
licensing remains unchanged; Spade remains the existing declared dependency.
The next useful experiment is matched rest geometry and physical parameters
on uniform-base, adaptive and uniform-fine meshes, comparing area/mass,
boundary/seam lengths, static deflection, wrinkle measurements and full-step
cost. Do not enable material-driven sizing by default before that evidence.
