# Seam allowance and notches (MARK-01)

This slice adds per-boundary **2D seam allowance** and **notches** as stable draft data in
material coordinates, carried onto the preview and the physical SVG sheet. The cutting contour is
drawn beside the sewing contour; notches stay on the sewing line, as described below. Everything
here is CPU authoring and export; it does not change the solver, the document format version, or
the meaning of the existing physical fields.

The implementation is based on the reviewed contribution in PR #6, rebased onto current main.
It deliberately keeps one serialized notch representation; it does not merge both existing
contributions as interchangeable schemas. See **Reconciliation with PR #1** below.

## What it is, and what it is not

- **It is** two maps on `PolygonPattern`: `seam_allowances` (metres, keyed by the stable
  boundary id) and `notches` (a list per boundary, each with a position, a marker length and a
  shape). A joined **cutting contour** is derived on demand from the sewing contour plus the
  allowances. Both maps are *draft data*: they add no area, mass, hinge or constraint.
- **It is not** a change to the physical model. A marking never rebuilds the mesh, never resets
  the drape, and never enters the mesh fingerprint. `authoring::same_physics` and
  `fitting::mesh_signature` both clear the two maps, exactly as they already clear
  `target_lengths`. Installing a marking-only edit also keeps playback, the pending worker
  generation and the retained drape: `authoring::only_appearance_changed` treats allowance and
  notch maps as nonphysical draft metadata.
- **It is not** `MARK-02`. Notch **correspondence** (a matched pair across a seam and a
  length-difference report) is a separate, unstarted slice. The seam compiler's own `notches`
  field is untouched.
- **It is not** production cutting certification. The offset is a geometric draft, not a
  Minkowski dilation, and the printed sheet is not tiled or nesting-aware.

## The data model

`PolygonPattern` gains two optional, `skip_serializing_if`-empty fields. Historical files load
unchanged, and a fresh pattern serialises neither key.

```json
{
  "seam_allowances": { "hem": 0.01, "side-left": 0.01 },
  "notches": {
    "hem": [{ "at": 0.5, "length_m": 0.005, "kind": "v" }],
    "side-left": [{ "at": 0.3, "length_m": 0.008, "kind": "slit" }]
  }
}
```

- `at` is a fraction of the boundary's **sampled arclength** in the open interval `(0, 1)`.
  Because it is a fraction of the named boundary, it survives resizing, remeshing and mirroring
  (mirroring stores `1 - at`).
- `length_m` is the marker depth from the sewing line, in metres; `kind` is `v` (default) or
  `slit`.
- Allowances are absolute widths in metres, keyed by the same boundary ids the sewing editor
  already uses. The identifier is the contract; no vertex or triangle index is stored.

A marking boundary is a **closed** boundary: an outer edge or a hole rim. An open internal line
has two sides and is refused with a clear message, so an allowance is never silently
one-sided.

### Reconciliation with PR #1

PR #1 proposed a different, incompatible notch payload: `{ "t": ..., "depth_m": ... }` and also
carried material-space text annotations. PR #6 uses `{ "at": ..., "length_m": ..., "kind": ... }`.
Both were strict readers, so accepting both as aliases would create two persistent
representations and ambiguous round trips. This branch keeps the PR #6 schema because it carries
the marker shape and describes the notch as a depth rather than an unrelated annotation offset.
PR #1's annotations remain outside this slice; the `t`/`depth_m` spelling is not accepted by the
current reader. The contributors and reviews are credited in the PR; no source or assets were
copied from PR #1.

## The cutting contour

`PolygonPattern::cutting_contour()` returns `None` when no allowance is set, otherwise a
`CuttingContour { outer, holes }`:

1. Every named boundary is sampled by the existing `edge_points`, and each segment is offset
   outward from the material by its boundary's allowance. Outward is to the right of the
   directed boundary, which points away from the material for both the counter-clockwise outer
   contour and a clockwise hole.
2. Adjacent offset segments are joined at their line intersection. A normal corner receives a
   **miter**. A parallel or numerically missed intersection becomes an explicit **bevel** with two
   points. A miter longer than eight times the local allowance is also replaced by a bevel, so a
   spike cannot silently become the cutting line. A miter at a narrow concavity is never treated
   as an implicit round or a silent extension.
3. The result is validated: the loop must be simple, keep its winding, and not intrude into the
   material. That last check catches a closed gap: a per-segment offset can overshoot a thin
   feature and fold shut while still tracing a simple loop. Intrusion is not decided from a few
   samples: every candidate cut segment is split at each crossing with the original outline and
   hole rims, and every interval between crossings is tested at its midpoint. A narrow excursion
   that enters and leaves the material between two vertices is therefore refused, while a line
   that only touches or runs along the boundary is not.
4. Every hole is retained. A hole with no allowance is copied unchanged into the complete cutting
   contour; it is not dropped merely because the outer edge has an allowance.
5. A hole allowance that closes the hole is refused. The checks cover the threshold from both
   sides: a degenerate inset at half the hole width, and the larger offset that can cross both
   opposite edges, re-form a simple square and evade a winding-only test. Each offset segment is
   compared with the nearest original hole edge, so the first collapse is rejected rather than
   printed as a spurious hole.

A contour that cannot be represented is **refused**, not approximated. The command returns an
error, the transaction is abandoned, and the document is untouched.

## Notches

`PolygonPattern::notch_marker(edge, notch)` returns the marker polyline in material
coordinates. A `v` notch is a small wedge pointing outward: its two bases sit on the sampled
sewing boundary at `length_m / 2` arclength on either side of the center, and its tip is
`length_m` outward. A `slit` is a single outward segment from the center. The bases follow the
sampled boundary, so on a curved edge they stay on the flattened sewing line instead of
floating off at a straight tangent offset, within the same sub-0.2 mm curve tolerance as the
contour. The marker is derived from the named boundary and its sampled arclength, not from a
vertex or triangle index.

A notch must satisfy the published editor rule: it stays **at least 1 mm from either end** of
its boundary, its length is in `(0, 50 mm]`, and its allowance is in `[0, 100 mm]`. For a `v`
notch that clearance applies to the **whole span** (center ± `length_m / 2`), so a V whose base
would reach or cross a corner is refused; a slit keeps the center-only rule. A V that cannot fit
on the boundary at any position is refused rather than clipped. A violation is a transaction
error, never a silent drop. `validate_markings` also rejects a marking that references an
unknown boundary, so a rename, split or mirror that forgets to migrate a marking fails loudly.

### The marker is a sewing-line guide

The stored notch is attached to the **sewing line**, not to the cutting contour. `notch_marker`
starts at the sampled boundary point and points outward along the local boundary normal. For a
600 mm edge with a 10 mm allowance and a 5 mm slit at its midpoint it returns
`(0.300, 0.000) -> (0.300, -0.005)` m, while the cut edge runs at `y = -0.010` m: the marker
stops 5 mm short of the cut edge. Both the preview and the true-scale sheet draw exactly that
sewing-line marker. A cutter must therefore transfer the mark to the cut edge; the cut edge
carries no automatic notch mark.

Duplicating the notch onto the allowance is **not implemented**. It needs a chosen outward
direction at miters and bevels, a rule for allowances smaller or larger than the marker, hole
orientation and curved boundaries, plus matching export and validation. That is deferred
(`MARK-02`) rather than approximated silently.

Splitting a boundary copies the allowance to both halves and keeps each notch at its material
arclength. On a curved boundary the cubic split is exact, but the two halves are flattened
separately; the marker follows the resampled halves to the same sub-0.2 mm curve tolerance as
the contour. A split inside a notch's end clearance is refused before mutation: 1 mm for a
slit, and the V's full span (1 mm plus half its length) for a V, because the split becomes a new
boundary end. A notch exactly at the split cannot be represented in the open interval and is
likewise refused.

## Migration and refusal

| Operation | Markings |
|---|---|
| `Mirror` | reversed contour ⇒ `at` becomes `1 - at`; allowances are absolute, unchanged |
| `SplitBoundary` | the allowance is copied to both halves; each notch keeps its material arclength position; a split inside a notch's end clearance (1 mm, or 1 mm plus half the length for a V) is refused |
| `RemovePoint` | refused when either rejoined boundary carries an allowance or notch (ambiguous) |
| `Boolean` | refused when an operand carries an allowance or notch, like lines and target lengths |
| `Shape`, `Dimensions`, `Anchor`, `MovePoints` | carried through unchanged: fractions and widths are geometry-independent |
| Clipboard | copied with the panel, like every other field |

## Export

- `export::pattern_svg` (the existing unscaled preview) draws the joined cutting contour as a
  dashed orange path and each sewing-line notch as a red marker, with accessible `<title>` text.
- `export::print_pattern_svg` is the true-scale sheet in millimetres: one panel per page,
  1 mm = 1 mm, with the stitch line, any cutting line, all holes, the internal lines, the
  sewing-line notches, the grain line and a **100 mm ruler** that proves the scale. It is written to
  `patterns-print.svg` beside `patterns.svg` by `write_artifacts`.
- Page bounds include the sampled outer contour, every cutting-contour ring and every notch
  marker. A valid long notch therefore expands the sheet instead of being clipped; the marker
  stroke is also kept inside the page.
- It is **not** nesting and **not** tiling: pages are never packed together, and a panel wider
  than the page is not split. Multi-page tiling and deterministic nesting remain `LAYOUT-01`, and
  a PDF remains `PDF-01`.

## Use the tools

In **Edit**, click an outer boundary on the 2D board. The inspector shows the boundary length
and, below the boundary tools, **Seam allowance** and **Notches**:

- Type a width in mm and **Apply allowance** to offset that boundary; **Clear allowance**
  removes it. The cutting line is drawn on the board immediately. A boundary without a stored
  allowance opens the field on the **10 mm** default width, so the draft is one click from being
  applied; the stored value replaces it from then on.
- Type a position (mm from the start), a marker length and a shape, then **Add notch**. Existing
  notches are listed with their position, length and shape and can be removed one at a time.

The board draws the cutting contour and the sewing-line notches in every tool, so the drafter
always sees both lines.

## Limits

- The offset is a per-segment offset with miters and explicit bevel fallback, not a Minkowski
  dilation or a straight skeleton. It is exact for convex and gently concave boundaries and
  refuses the cases it cannot represent; it does not round corners or repair itself.
- The public marking helpers (`cutting_contour`, `named_boundary`, `closed_boundary`,
  `notch_marker`, `validate_markings`) check the outer point/edge structure before indexing it.
  A hand-edited polygon with fewer or more edge names than points, an empty outline or a
  non-finite coordinate is refused instead of panicking; `edge_points` bounds-checks both the
  point and the edge name. `build` repeats the full geometric validation.
- The crossing check keeps a uniform bounding-box grid over the boundary edges, so a dense panel
  does not compare every cut segment with every edge. The remaining quadratic work is the
  pre-existing whole-loop simplicity test. `cutting_contour` is currently recomputed when the
  board or an export asks for it; no derived-contour cache exists. Measured CPU cost with a 5 mm
  allowance on every edge of a regular polygon is about 0.03 ms at 64 vertices, 0.2–0.5 ms at 256,
  2.4–3 ms at 1,024, 10 ms at 2,048 and 35 ms at 4,096, so ordinary draft panels are far below a
  frame budget while a several-thousand-vertex panel is not. A cache or partial update is deferred
  until real garment sizes and frame profiling justify it; no GPU claim is made.
- A dropped notch reproduces the published 1 mm end rule; there is no "snap to a corner
  recommendation".
- Notches are draft markers on the sewing line. A V's whole span must stay inside its boundary,
  and its bases follow the sampled boundary on curves; a V that would cross a corner is refused.
  They are not cut into the mesh, they add no notch energy, the cut edge is drawn without an
  automatic mark, and the seam compiler does not consume them yet (`MARK-02`).
- The true-scale sheet has no nesting, tiling, cut count or PDF output (`LAYOUT-01`, `BOM-01`,
  `PDF-01`). It is not a physical manufacturing guarantee.
- No GPU, native-window or printed-paper calibration is claimed by this slice. The default test
  suite remains CPU-only.

## Sources and behaviour

The workflow wording — per-boundary seam allowance width, notch placement with a 1 mm
clearance from a segment point, a default notch type, inner-shape versus pattern-outline display,
and duplicating a notch onto the seam allowance — is consistent with the public CLO / Marvelous
Designer articles already cited by [essential-authoring-tools.md](essential-authoring-tools.md)
(*Edit Seam Allowance Properties*, *Notch*) and with the option and validation labels shipped
with the application, cross-checked against the installed build for wording. No proprietary
code, resource or garment example was imported; the model, geometry and tests here are
independently written. The contribution history and the independent reviews of PR #1 and PR #6
are recorded on the coordination issue and PR.

## Implementation and validation

| File | Responsibility |
|---|---|
| `src/garment/markings.rs` | `Notch`, `NotchKind`, validation, joining and the cutting contour |
| `src/garment/polygon.rs` | the two optional fields; `build` validates the markings |
| `src/authoring.rs`, `src/authoring/geometry.rs` | `SeamAllowance`, `SetNotch`, `RemoveNotch`; migration and refusal |
| `src/garment/export.rs` | preview markings and the true-scale print SVG |
| `src/atelier/board.rs`, `authoring_ui.rs` | boundary inspector, board overlay and the markings draft fields |
| `src/garment/shapes.rs`, `src/fitting.rs` | mirror migration; cache-signature cleanup |
| `src/pattern_import.rs`, `src/garment/svg_import.rs` | initialise the two empty maps when constructing a polygon |
| `tests/pattern_markings.rs` | CPU acceptance tests |

`tests/pattern_markings.rs` covers: a 10 mm allowance grows the cutting contour while leaving
meshed area and `same_physics` unchanged and not resetting the drape; a one-sided allowance moves
only that edge; a horseshoe whose gap a 3 mm allowance would close is refused transactionally;
an unchanged hole remains in a partial cutting contour; a hole allowance is refused at and after
the collapse threshold; a notch survives a JSON round trip and a dangling reference fails to
compile; the end rule for a slit and the whole-span rule for a V, including both boundary ends, a
short boundary and a refused split inside a V span; append/remove; split migration and
exact/near-split refusal; mirror reflection; refusal by `RemovePoint` and `Boolean`; a curved
boundary marker whose V bases stay on the sampled sewing line, curved split/mirror preservation,
and loading a legacy document without the new keys; malformed holes and malformed outer contours
rejected before any marking helper indexes them; and the cutting line, notch and 100 mm ruler in
both SVGs, including a long notch that expands the physical page. A five-point counterexample
from review whose cut segment dips into the material between its endpoints and midpoint is
refused, while the same unmarked outline still builds. A 600 mm edge with a 10 mm allowance and
a 5 mm slit asserts the marker stays on the sewing line and stops 5 mm short of the
cut edge. A CPU named-widget test stages, cancels, applies and rejects invalid allowance/notch
drafts without opening a window, and a CPU transition test proves that allowance/notch apply and
undo preserve playback, the pending worker generation, the retained drape and the cached mesh
signature.

The library also has unit tests for the offset geometry and marker shape. The default CPU suite
is run with `cargo test --locked --no-default-features -j 2 -- --test-threads=2`, followed by
formatting and CPU Clippy. The desktop UI is compiled separately; this slice does not claim a
native pointer run or physical-printer calibration.
