//! Cubic Bézier subdivision in material metres. Independent of mesh resolution.
use glam::DVec2;

pub const FLATNESS_M: f64 = 0.00025;
pub fn cubic(p: [DVec2; 4], t: f64) -> DVec2 {
    let a = p[0].lerp(p[1], t);
    let b = p[1].lerp(p[2], t);
    let c = p[2].lerp(p[3], t);
    a.lerp(b, t).lerp(b.lerp(c, t), t)
}
fn distance_to_segment(p: DVec2, a: DVec2, b: DVec2) -> f64 {
    let d = b - a;
    p.distance(a + d * ((p - a).dot(d) / d.length_squared().max(1e-30)).clamp(0., 1.))
}
pub fn flatten(p: [DVec2; 4], tolerance: f64) -> Result<Vec<DVec2>, String> {
    if p.iter().any(|v| !v.is_finite()) || !tolerance.is_finite() || tolerance <= 0. {
        return Err("Invalid Bézier controls or tolerance".into());
    }
    fn split(
        p: [DVec2; 4],
        tolerance: f64,
        depth: u32,
        out: &mut Vec<DVec2>,
    ) -> Result<(), String> {
        // Distance to the segment (not the infinite line) also catches collinear
        // overshoot/backtracking. The convex hull bounds curve-to-chord error.
        if distance_to_segment(p[1], p[0], p[3]).max(distance_to_segment(p[2], p[0], p[3]))
            <= tolerance
        {
            out.push(p[3]);
            return Ok(());
        }
        if depth == 16 || out.len() > 4096 {
            return Err("Bézier subdivision budget exceeded".into());
        }
        let a = (p[0] + p[1]) * 0.5;
        let b = (p[1] + p[2]) * 0.5;
        let c = (p[2] + p[3]) * 0.5;
        let d = (a + b) * 0.5;
        let e = (b + c) * 0.5;
        let f = (d + e) * 0.5;
        split([p[0], a, d, f], tolerance, depth + 1, out)?;
        split([f, e, c, p[3]], tolerance, depth + 1, out)
    }
    let mut out = vec![p[0]];
    split(p, tolerance, 0, &mut out)?;
    Ok(out)
}

pub fn arclength_point(points: &[DVec2], fraction: f64) -> DVec2 {
    let length: f64 = points.windows(2).map(|p| p[0].distance(p[1])).sum();
    let mut remaining = length * fraction.clamp(0., 1.);
    for pair in points.windows(2) {
        let d = pair[0].distance(pair[1]);
        if remaining <= d {
            return pair[0].lerp(pair[1], remaining / d.max(1e-30));
        }
        remaining -= d;
    }
    points.last().copied().unwrap_or_default()
}

/// Experimental three-landmark attachment matching, retained as CPU tests.
/// This is the local prototype, not a persisted POM representation or a
/// production edit-recovery guarantee. Candidate identities and landmarks
/// must be supplied by the caller; the weights and thresholds are trials.
#[cfg(test)]
mod three_point_tracking_prototype {
    use super::*;
    // Private, test-only evidence: callers still supply candidate and landmark
    // identities. These values never become persisted document references.
    struct Fingerprint {
        material: DVec2,
        distances: [f64; 3],
        relative: [f64; 3],
        other: DVec2,
        scale: f64,
    }

    #[derive(Debug)]
    struct CandidateScore {
        predicted: DVec2,
        relative_error: f64,
        distance_error: f64,
        measurement_error: f64,
    }
    impl CandidateScore {
        fn total(&self) -> f64 {
            // Diagnostic hypothesis, not calibrated confidence.
            0.5 * self.relative_error + 0.35 * self.distance_error + 0.15 * self.measurement_error
        }
    }

    fn finite_point(p: DVec2) -> Result<DVec2, String> {
        if p.is_finite() {
            Ok(p)
        } else {
            Err("Non-finite attachment geometry".into())
        }
    }
    fn distance(a: DVec2, b: DVec2) -> Result<f64, String> {
        let d = finite_point(finite_point(a)? - finite_point(b)?)?;
        let length = d.x.hypot(d.y);
        if length.is_finite() {
            Ok(length)
        } else {
            Err("Attachment distance overflowed".into())
        }
    }

    struct Frame {
        origin: DVec2,
        u: DVec2,
        v: DVec2,
        scale: f64,
        determinant: f64,
    }
    impl Frame {
        fn new([a, b, c]: [DVec2; 3]) -> Result<Self, String> {
            // Longest side makes normalization independent of landmark ordering.
            let scale = distance(a, b)?.max(distance(a, c)?).max(distance(b, c)?);
            if scale <= 0. {
                return Err("Reference scale collapsed".into());
            }
            let u = (b - a) / scale;
            let v = (c - a) / scale;
            let determinant = u.perp_dot(v);
            // Dimensionless twice-area / longest-side². This is a conditioning
            // guard, not an absolute tolerance measured in square metres.
            if !determinant.is_finite() || determinant.abs() < 1e-8 {
                return Err("Three reference landmarks are collinear or poorly conditioned".into());
            }
            Ok(Self {
                origin: a,
                u,
                v,
                scale,
                determinant,
            })
        }
        fn coordinates(&self, p: DVec2) -> Result<[f64; 3], String> {
            let offset = finite_point((finite_point(p)? - self.origin) / self.scale)?;
            let beta = offset.perp_dot(self.v) / self.determinant;
            let gamma = self.u.perp_dot(offset) / self.determinant;
            let weights = [1. - beta - gamma, beta, gamma];
            if weights.iter().any(|w| !w.is_finite()) {
                return Err("Attachment coordinates overflowed".into());
            }
            Ok(weights)
        }
    }
    fn fingerprint(p: DVec2, landmarks: [DVec2; 3], other: DVec2) -> Result<Fingerprint, String> {
        let frame = Frame::new(landmarks)?;
        let relative = frame.coordinates(p)?;
        distance(p, other)?;
        let [a, b, c] = landmarks;
        Ok(Fingerprint {
            material: p,
            distances: [distance(a, p)?, distance(b, p)?, distance(c, p)?],
            relative,
            other,
            scale: frame.scale,
        })
    }
    fn candidate_score(
        f: &Fingerprint,
        candidate: DVec2,
        landmarks: [DVec2; 3],
        other: DVec2,
    ) -> Result<CandidateScore, String> {
        let frame = Frame::new(landmarks)?;
        frame.coordinates(candidate)?;
        let [a, b, c] = landmarks;
        let predicted = finite_point(a + (b - a) * f.relative[1] + (c - a) * f.relative[2])?;
        let distances = [
            distance(a, candidate)?,
            distance(b, candidate)?,
            distance(c, candidate)?,
        ];
        let errors = std::array::from_fn::<_, 3, _>(|i| {
            (distances[i] / frame.scale - f.distances[i] / f.scale) / 3_f64.sqrt()
        });
        let result = CandidateScore {
            predicted,
            relative_error: distance(candidate, predicted)? / frame.scale,
            distance_error: errors[0].hypot(errors[1]).hypot(errors[2]),
            measurement_error: (distance(candidate, other)? / frame.scale
                - distance(f.material, f.other)? / f.scale)
                .abs(),
        };
        if !result.total().is_finite() {
            return Err("Attachment score overflowed".into());
        }
        Ok(result)
    }
    fn select_candidate(
        f: &Fingerprint,
        points: &[DVec2],
        landmarks: [DVec2; 3],
        other: DVec2,
    ) -> Result<usize, String> {
        let mut ranked = points
            .iter()
            .enumerate()
            .map(|(i, p)| candidate_score(f, *p, landmarks, other).map(|s| (i, s.total())))
            .collect::<Result<Vec<_>, _>>()?;
        ranked.sort_by(|a, b| a.1.total_cmp(&b.1));
        let Some(best) = ranked.first() else {
            return Err("No candidate attachment".into());
        };
        // Trial thresholds only. Keep weak and ambiguous matches explicit.
        if best.1 > 0.05 {
            return Err("No sufficiently matching attachment".into());
        }
        if ranked.get(1).is_some_and(|other| other.1 - best.1 < 0.01) {
            return Err("Ambiguous attachment candidates".into());
        }
        Ok(best.0)
    }

    fn fixture() -> (Fingerprint, [DVec2; 3], DVec2) {
        let landmarks = [
            DVec2::new(0.4, 0.),
            DVec2::new(0.4, 0.3),
            DVec2::new(0., 0.3),
        ];
        let other = DVec2::new(0.3, 0.3);
        (
            fingerprint(DVec2::new(0.3, 0.), landmarks, other).unwrap(),
            landmarks,
            other,
        )
    }

    #[test]
    fn selects_the_intended_material_station_after_a_boundary_split() {
        use crate::{
            authoring::{self, Command},
            garment::{
                meshing::MeshingSettings,
                presets::{self, GarmentKind},
            },
        };
        let original = authoring::apply(
            &presets::basic(GarmentKind::Tshirt, 0),
            Command::Polygon {
                points: vec![
                    DVec2::ZERO,
                    DVec2::new(0.4, 0.),
                    DVec2::new(0.4, 0.3),
                    DVec2::new(0., 0.3),
                ],
                tangents: vec![],
                meshing: MeshingSettings {
                    target_edge_m: 0.05,
                    ..Default::default()
                },
                fabric: "woven-reference".into(),
                replace: true,
            },
        )
        .unwrap()
        .template;
        let (saved, landmarks, other) = fixture();
        let edit = authoring::apply(
            &original,
            Command::SplitBoundary {
                panel: original.panels[0].id.clone(),
                edge: "edge_1".into(),
                at_m: 0.2,
            },
        )
        .unwrap();
        let before = original.panels[0].polygon.as_ref().unwrap();
        let after = edit.template.panels[0].polygon.as_ref().unwrap();
        let first = after.edges.iter().position(|e| e == "edge_1").unwrap();
        let child = after
            .edges
            .iter()
            .position(|e| !before.edges.contains(e))
            .unwrap();
        let wrong = arclength_point(&after.edge_points(first).unwrap(), 0.5);
        let intended = arclength_point(&after.edge_points(child).unwrap(), 0.5);
        assert!(intended.distance(DVec2::new(0.3, 0.)) < 1e-12);
        assert!(wrong.distance(DVec2::new(0.1, 0.)) < 1e-12);
        assert_eq!(
            select_candidate(&saved, &[wrong, intended], landmarks, other).unwrap(),
            1
        );
        let actual = candidate_score(&saved, intended, landmarks, other).unwrap();
        let incorrect = candidate_score(&saved, wrong, landmarks, other).unwrap();
        assert!(actual.total() < 1e-12);
        assert!(incorrect.total() > 0.1);
    }

    #[test]
    fn fingerprint_is_invariant_under_uniform_scale_and_translation() {
        let (saved, landmarks, other) = fixture();
        let shift = DVec2::new(0.05, -0.02);
        let scaled = candidate_score(
            &saved,
            DVec2::new(0.3, 0.) * 1.25 + shift,
            landmarks.map(|p| p * 1.25 + shift),
            other * 1.25 + shift,
        )
        .unwrap();
        assert!(scaled.total() < 1e-12);
    }

    #[test]
    fn rejects_ambiguous_weak_and_degenerate_matches() {
        let (saved, landmarks, other) = fixture();
        let intended = DVec2::new(0.3, 0.);
        assert!(
            select_candidate(&saved, &[intended, intended], landmarks, other)
                .unwrap_err()
                .contains("Ambiguous")
        );
        assert!(select_candidate(&saved, &[DVec2::new(0.1, 0.)], landmarks, other).is_err());
        assert!(select_candidate(&saved, &[], landmarks, other).is_err());
        assert!(fingerprint(intended, [DVec2::ZERO, DVec2::X, DVec2::X * 2.], other).is_err());
    }

    #[test]
    fn third_landmark_distinguishes_two_distance_reflection_ambiguity() {
        let a = DVec2::ZERO;
        let b = DVec2::new(0.4, 0.);
        let point = DVec2::new(0.2, 0.1);
        let reflected = DVec2::new(0.2, -0.1);
        assert!((point.distance(a) - reflected.distance(a)).abs() < 1e-12);
        assert!((point.distance(b) - reflected.distance(b)).abs() < 1e-12);
        let third = DVec2::new(0., 0.3);
        assert!((point.distance(third) - reflected.distance(third)).abs() > 0.1);
    }

    #[test]
    fn rejects_nonfinite_capture_inputs_and_overflowing_geometry() {
        let (_, landmarks, other) = fixture();
        for bad in [
            DVec2::NAN,
            DVec2::new(f64::INFINITY, 0.),
            DVec2::new(0., f64::NEG_INFINITY),
        ] {
            assert!(
                fingerprint(bad, landmarks, other).is_err(),
                "invalid material position"
            );
            assert!(
                fingerprint(DVec2::ZERO, landmarks, bad).is_err(),
                "invalid other endpoint"
            );
            for i in 0..3 {
                let mut invalid = landmarks;
                invalid[i] = bad;
                assert!(
                    fingerprint(DVec2::ZERO, invalid, other).is_err(),
                    "invalid landmark {i}"
                );
            }
        }
        let huge = [
            DVec2::new(f64::MAX, 0.),
            DVec2::new(-f64::MAX, 0.),
            DVec2::new(0., 1.),
        ];
        assert!(fingerprint(DVec2::ZERO, huge, other).is_err());
    }

    #[test]
    fn invalid_scoring_inputs_return_errors_instead_of_panicking() {
        let (saved, landmarks, other) = fixture();
        for bad in [
            DVec2::NAN,
            DVec2::new(f64::INFINITY, 0.),
            DVec2::new(0., f64::NEG_INFINITY),
        ] {
            assert!(candidate_score(&saved, bad, landmarks, other).is_err());
            assert!(candidate_score(&saved, DVec2::ZERO, landmarks, bad).is_err());
            assert!(
                select_candidate(&saved, &[DVec2::new(0.3, 0.), bad], landmarks, other).is_err()
            );
            for i in 0..3 {
                let mut invalid = landmarks;
                invalid[i] = bad;
                assert!(candidate_score(&saved, DVec2::ZERO, invalid, other).is_err());
            }
        }
    }

    #[test]
    fn captures_geometrically_equivalent_small_and_large_patterns() {
        let (_, landmarks, other) = fixture();
        for scale in [1e-12, 1e-6, 1., 1e6, 1e12] {
            let scaled = landmarks.map(|p| p * scale);
            let position = DVec2::new(0.3, 0.) * scale;
            let saved = fingerprint(position, scaled, other * scale).unwrap();
            assert_eq!(
                select_candidate(&saved, &[position], scaled, other * scale).unwrap(),
                0
            );
        }
    }

    #[test]
    fn near_collinear_rejection_is_relative_to_landmark_size() {
        for scale in [1e-6, 1., 1e6] {
            let landmarks = [
                DVec2::ZERO,
                DVec2::new(scale, 0.),
                DVec2::new(0.5 * scale, 1e-12 * scale),
            ];
            assert!(
                fingerprint(DVec2::ZERO, landmarks, DVec2::Y).is_err(),
                "scale {scale}"
            );
        }
    }
    #[test]
    fn score_is_independent_of_consistent_landmark_order() {
        let (_, landmarks, other) = fixture();
        let mut scores = Vec::new();
        for [i, j, k] in [
            [0, 1, 2],
            [0, 2, 1],
            [1, 0, 2],
            [1, 2, 0],
            [2, 0, 1],
            [2, 1, 0],
        ] {
            let ordered = [landmarks[i], landmarks[j], landmarks[k]];
            let saved = fingerprint(DVec2::new(0.3, 0.), ordered, other).unwrap();
            scores.push(
                candidate_score(&saved, DVec2::new(0.31, 0.01), ordered, other)
                    .unwrap()
                    .total(),
            );
        }
        assert!(scores[0] > 0.01);
        assert!(scores.iter().all(|s| (s - scores[0]).abs() < 1e-12));
    }

    #[test]
    fn similarity_transforms_preserve_the_intended_candidate() {
        let (saved, landmarks, other) = fixture();
        for angle in [0_f64, 0.5, -1.2, std::f64::consts::PI] {
            for scale in [0.001, 1., 1000.] {
                let transform = |p: DVec2| {
                    let (sin, cos) = angle.sin_cos();
                    DVec2::new(cos * p.x - sin * p.y, sin * p.x + cos * p.y) * scale
                        + DVec2::new(0.05, -0.02)
                };
                let point = transform(saved.material);
                let moved = landmarks.map(transform);
                let score = candidate_score(&saved, point, moved, transform(other)).unwrap();
                assert!(score.predicted.distance(point) < 1e-10);
                assert!(score.total() < 1e-12);
                assert_eq!(
                    select_candidate(
                        &saved,
                        &[transform(DVec2::ZERO), point],
                        moved,
                        transform(other)
                    )
                    .unwrap(),
                    1
                );
            }
        }
    }

    #[test]
    fn a_real_measurement_change_can_reject_the_correct_identity() {
        let (saved, landmarks, other) = fixture();
        let changed_other = other + DVec2::Y;
        let score = candidate_score(&saved, saved.material, landmarks, changed_other).unwrap();
        assert!(score.relative_error < 1e-12);
        assert!(score.distance_error < 1e-12);
        assert!(score.measurement_error > 1.);
        // A weighted score alone cannot certify identity through arbitrary edits.
        assert!(select_candidate(&saved, &[saved.material], landmarks, changed_other).is_err());
    }

    #[test]
    fn curved_station_survives_repeated_supplied_child_candidates() {
        use crate::{
            authoring::{self, Command},
            garment::{
                meshing::MeshingSettings,
                presets::{self, GarmentKind},
            },
        };
        let mut template = authoring::apply(
            &presets::basic(GarmentKind::Tshirt, 0),
            Command::Polygon {
                points: vec![
                    DVec2::ZERO,
                    DVec2::new(0.4, 0.),
                    DVec2::new(0.4, 0.3),
                    DVec2::new(0., 0.3),
                ],
                tangents: vec![],
                meshing: MeshingSettings {
                    target_edge_m: 0.05,
                    ..Default::default()
                },
                fabric: "woven-reference".into(),
                replace: true,
            },
        )
        .unwrap()
        .template;
        let panel_id = template.panels[0].id.clone();
        let mut polygon = template.panels[0].polygon.clone().unwrap();
        polygon.curves.insert(
            "edge_1".into(),
            [DVec2::new(0.12, -0.04), DVec2::new(0.29, -0.045)],
        );
        template = authoring::apply(
            &template,
            Command::Shape {
                panel: panel_id.clone(),
                polygon,
            },
        )
        .unwrap()
        .template;
        // Coordinates use the board translation to compensate for local contour
        // normalization; this fixture has zero presentation rotation. Child IDs
        // and arclength fractions are supplied explicitly, not recovered POMs.
        let station = |t: &crate::garment::GarmentTemplate, edge: &str, fraction| {
            let panel = &t.panels[0];
            let polygon = panel.polygon.as_ref().unwrap();
            let i = polygon.edges.iter().position(|id| id == edge).unwrap();
            arclength_point(&polygon.edge_points(i).unwrap(), fraction)
                + panel.layout_2d.unwrap_or_default()
        };
        let landmarks = |t: &crate::garment::GarmentTemplate| {
            ["edge_2", "edge_3", "edge_4"].map(|edge| station(t, edge, 0.))
        };
        let other = station(&template, "edge_3", 0.2);
        let original = station(&template, "edge_1", 0.83);
        let saved = fingerprint(original, landmarks(&template), other).unwrap();
        let mut edge = "edge_1".to_owned();
        let mut fraction = 0.83;
        for _ in 0..3 {
            let before = template.panels[0].polygon.as_ref().unwrap();
            let old_ids = before.edges.clone();
            let at_m = 0.35 * authoring::edge_length(before, &edge).unwrap();
            template = authoring::apply(
                &template,
                Command::SplitBoundary {
                    panel: panel_id.clone(),
                    edge: edge.clone(),
                    at_m,
                },
            )
            .unwrap()
            .template;
            let after = template.panels[0].polygon.as_ref().unwrap();
            if fraction >= 0.35 {
                edge = after
                    .edges
                    .iter()
                    .find(|id| !old_ids.contains(id))
                    .unwrap()
                    .clone();
                fraction = (fraction - 0.35) / 0.65;
            } else {
                fraction /= 0.35;
            }
            let intended = station(&template, &edge, fraction);
            assert!(intended.distance(original) < 3. * FLATNESS_M);
            let other = station(&template, "edge_3", 0.2);
            assert_eq!(
                select_candidate(
                    &saved,
                    &[station(&template, "edge_1", 0.1), intended],
                    landmarks(&template),
                    other
                )
                .unwrap(),
                1
            );
            // Verify the supplied geometry survives document serialization too.
            let json = serde_json::to_string(&template).unwrap();
            template = serde_json::from_str(&json).unwrap();
            assert!(station(&template, &edge, fraction).distance(intended) < 1e-12);
        }
    }
}
