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
    use serde_json::{Value, json};

    fn xy(p: DVec2) -> Value {
        json!([p.x * 1000., p.y * 1000.])
    }
    // Local prototype: a multi-reference attachment fingerprint, not a second
    // persisted POM schema. No production attachment is silently retargeted.
    #[derive(Clone)]
    struct Fingerprint {
        material: DVec2,
        landmarks: [DVec2; 3],
        distances: [f64; 3],
        relative: [f64; 3],
        other: DVec2,
    }
    fn relative_coordinates(p: DVec2, landmarks: [DVec2; 3]) -> Result<[f64; 3], String> {
        let [a, b, c] = landmarks;
        let u = b - a;
        let v = c - a;
        let determinant = u.perp_dot(v);
        if determinant.abs() < 1e-10 {
            return Err("Three reference landmarks are collinear or too close".into());
        }
        let beta = (p - a).perp_dot(v) / determinant;
        let gamma = u.perp_dot(p - a) / determinant;
        Ok([1. - beta - gamma, beta, gamma])
    }
    fn fingerprint(p: DVec2, landmarks: [DVec2; 3], other: DVec2) -> Result<Fingerprint, String> {
        Ok(Fingerprint {
            material: p,
            landmarks,
            distances: landmarks.map(|a| a.distance(p)),
            relative: relative_coordinates(p, landmarks)?,
            other,
        })
    }
    fn candidate_score(
        f: &Fingerprint,
        candidate: DVec2,
        landmarks: [DVec2; 3],
        other: DVec2,
    ) -> Result<Value, String> {
        // Three-landmark relative coordinates predict the point through an affine
        // change. Distances are normalized by landmark scale to tolerate scaling.
        let predicted = landmarks
            .into_iter()
            .zip(f.relative)
            .map(|(p, w)| p * w)
            .sum::<DVec2>();
        let old_scale = (f.landmarks[1] - f.landmarks[0]).length();
        let new_scale = (landmarks[1] - landmarks[0]).length();
        if old_scale < 1e-9 || new_scale < 1e-9 {
            return Err("Reference scale collapsed".into());
        }
        relative_coordinates(candidate, landmarks)?;
        let distances = landmarks.map(|p| p.distance(candidate));
        let distance_error = (distances
            .into_iter()
            .zip(f.distances)
            .map(|(a, b)| (a / new_scale - b / old_scale).powi(2))
            .sum::<f64>()
            / 3.)
            .sqrt();
        let relative_error = candidate.distance(predicted) / new_scale;
        let measurement_error = (candidate.distance(other) / new_scale
            - f.material.distance(f.other) / old_scale)
            .abs();
        // These weights are a diagnostic hypothesis, not calibrated confidence.
        let score = 0.5 * relative_error + 0.35 * distance_error + 0.15 * measurement_error;
        Ok(
            json!({"score":score,"relative_error":relative_error,"distance_error":distance_error,
        "measurement_error":measurement_error,"predicted":xy(predicted),"distances_mm":distances.map(|d|d*1000.),
        "weights":{"relative_position":0.5,"three_distances":0.35,"measurement_length":0.15}}),
        )
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
            .map(|(i, p)| {
                candidate_score(f, *p, landmarks, other).map(|s| (i, s["score"].as_f64().unwrap()))
            })
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
        assert!(actual["score"].as_f64().unwrap() < 1e-12);
        assert!(incorrect["score"].as_f64().unwrap() > 0.1);
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
        assert!(scaled["score"].as_f64().unwrap() < 1e-12);
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
}
