//! Opt-in CPU experiment. Run with --no-default-features; no editor changes.
#[path = "pie_resolution/mesh.rs"]
mod mesh;
#[path = "pie_resolution/sizing.rs"]
mod sizing;

use clap::Parser;
use glam::DVec2;
use mesh::{Domain, Options, Output, Ring};
use serde::{Deserialize, Serialize};
use sizing::{Calibration, Fabric, Feature, SizingMap, Source};
use std::{
    collections::BTreeMap,
    fmt::Write as _,
    fs,
    path::{Path, PathBuf},
    time::Instant,
};

#[derive(Parser)]
#[command(about = "Independent physics-inspired rest-space mesh sizing experiment (CPU only)")]
struct Args {
    /// Research JSON input; omitted runs the original shirring fixture.
    #[arg(long)]
    input: Option<PathBuf>,
    #[arg(long, default_value = "output/pie-resolution")]
    output: PathBuf,
    /// Compare against uniform base-minimum and uniform source-minimum spacings.
    #[arg(long)]
    compare: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct Input {
    fabric: Fabric,
    #[serde(default)]
    calibration: Calibration,
    domain: Domain,
    #[serde(default)]
    sources: Vec<Source>,
    #[serde(default)]
    options: Options,
}
fn fixture() -> Result<Input, String> {
    let calibration = Calibration::default();
    let fabric = Fabric {
        young_pa: [1.1e5; 2],
        bending_nm: [2e-6; 2],
        axis_degrees: 0.,
    };
    let gather = vec![DVec2::new(0.03, 0.07), DVec2::new(0.15, 0.07)];
    let spacing_m = fabric.shirring(&calibration, [0.; 2], 0.5, 1.)?;
    Ok(Input {
        fabric,
        calibration,
        domain: Domain {
            outer: Ring {
                points: vec![
                    DVec2::ZERO,
                    DVec2::new(0.18, 0.),
                    DVec2::new(0.18, 0.24),
                    DVec2::new(0., 0.24),
                ],
                edges: ["top", "right", "bottom", "left"]
                    .map(str::to_owned)
                    .to_vec(),
            },
            holes: vec![],
            lines: BTreeMap::from([("gather".into(), gather.clone())]),
        },
        sources: vec![Source {
            feature: Feature::Polyline { points: gather },
            spacing_m,
            doubling_distance_m: 0.04,
            exponent: 0.5,
        }],
        options: Options::default(),
    })
}
fn write_mesh(
    dir: &Path,
    name: &str,
    mesh: &Output,
    domain: &Domain,
) -> Result<(), Box<dyn std::error::Error>> {
    fs::write(
        dir.join(format!("{name}.json")),
        serde_json::to_vec_pretty(mesh)?,
    )?;
    let mut obj = String::from("# Rebuildable rest-space research mesh; metres, no draped state\n");
    for p in &mesh.uv {
        writeln!(obj, "v {:.17} {:.17} 0", p.x, p.y)?;
    }
    for p in &mesh.uv {
        writeln!(obj, "vt {:.17} {:.17}", p.x, p.y)?;
    }
    for f in &mesh.faces {
        writeln!(
            obj,
            "f {0}/{0} {1}/{1} {2}/{2}",
            f[0] + 1,
            f[1] + 1,
            f[2] + 1
        )?;
    }
    fs::write(dir.join(format!("{name}.obj")), obj)?;
    let lo = domain
        .outer
        .points
        .iter()
        .copied()
        .fold(DVec2::splat(f64::INFINITY), DVec2::min)
        * 1000.;
    let hi = domain
        .outer
        .points
        .iter()
        .copied()
        .fold(DVec2::splat(f64::NEG_INFINITY), DVec2::max)
        * 1000.;
    let width = hi.x - lo.x;
    let height = hi.y - lo.y;
    let mut svg = format!(
        "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"{}mm\" height=\"{}mm\" viewBox=\"{} {} {} {}\"><rect x=\"{}\" y=\"{}\" width=\"{}\" height=\"{}\" fill=\"#faf9f5\"/>\n",
        width + 10.,
        height + 10.,
        lo.x - 5.,
        lo.y - 5.,
        width + 10.,
        height + 10.,
        lo.x - 5.,
        lo.y - 5.,
        width + 10.,
        height + 10.
    );
    let hmin = mesh
        .spacing_m
        .iter()
        .flatten()
        .copied()
        .fold(f64::INFINITY, f64::min);
    let hmax = mesh.spacing_m.iter().flatten().copied().fold(0., f64::max);
    for f in &mesh.faces {
        let h = f
            .iter()
            .map(|i| mesh.spacing_m[*i][0].min(mesh.spacing_m[*i][1]))
            .sum::<f64>()
            / 3.;
        let t = ((h - hmin) / (hmax - hmin).max(1e-12)).clamp(0., 1.);
        let color = format!(
            "rgb({},{},{})",
            (40. + 150. * t) as u8,
            (125. + 90. * t) as u8,
            (185. + 45. * t) as u8
        );
        let points = f
            .map(|i| format!("{},{}", mesh.uv[i].x * 1000., mesh.uv[i].y * 1000.))
            .join(" ");
        writeln!(
            svg,
            "<polygon points=\"{points}\" fill=\"{color}\" stroke=\"#223344\" stroke-width=\"0.12\"/>"
        )?;
    }
    svg.push_str("<g fill=\"none\" stroke=\"#e87528\" stroke-width=\"0.6\">\n");
    for chain in mesh.boundaries.values() {
        let points = chain
            .iter()
            .map(|i| format!("{},{}", mesh.uv[*i].x * 1000., mesh.uv[*i].y * 1000.))
            .collect::<Vec<_>>()
            .join(" ");
        writeln!(svg, "<polyline points=\"{points}\"/>")?;
    }
    svg.push_str("</g></svg>\n");
    fs::write(dir.join(format!("{name}.svg")), svg)?;
    Ok(())
}
fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args = Args::parse();
    let input: Input = if let Some(path) = args.input {
        if fs::metadata(&path)?.len() > 1_048_576 {
            return Err("Research input exceeds 1 MiB".into());
        }
        serde_json::from_slice(&fs::read(path)?)?
    } else {
        fixture()?
    };
    let base = input.fabric.resolution(&input.calibration)?;
    let map = SizingMap::new(base, input.fabric.axis_degrees, input.sources.clone())?;
    let mut runs = vec![("adaptive", map.clone())];
    if args.compare {
        runs.push((
            "uniform-base",
            SizingMap::new([base[0].min(base[1]); 2], input.fabric.axis_degrees, vec![])?,
        ));
        runs.push((
            "uniform-fine",
            SizingMap::new([map.finest(); 2], input.fabric.axis_degrees, vec![])?,
        ));
    }
    // Finish every computation before writing a successful run's output files.
    let mut computed = vec![];
    for (name, map) in runs {
        let start = Instant::now();
        let mesh = mesh::generate(&input.domain, &map, &input.options)?;
        computed.push((name, mesh, start.elapsed().as_secs_f64() * 1000.));
    }
    fs::create_dir_all(&args.output)?;
    fs::write(
        args.output.join("input.json"),
        serde_json::to_vec_pretty(&input)?,
    )?;
    let mut summary = BTreeMap::new();
    for (name, mesh, ms) in &computed {
        write_mesh(&args.output, name, mesh, &input.domain)?;
        println!(
            "{name}: {} vertices, {} triangles, {:.3} ms (complete CPU mesh generation)",
            mesh.quality.vertices, mesh.quality.triangles, ms
        );
        summary.insert(
            *name,
            serde_json::json!({"quality":mesh.quality, "mesh_generation_ms":ms}),
        );
    }
    let tension_distance = sizing::tension_doubling_distance(0.01, 1., 2e-6, 1.)?;
    let compression_distance = sizing::compression_doubling_distance(0.01, 0.25, 0.0005, 1.)?;
    fs::write(
        args.output.join("summary.json"),
        serde_json::to_vec_pretty(&serde_json::json!({
            "research_only":true, "base_spacing_m":base, "runs":summary,
            "units":"metres, Pa, N m; mesh_generation_ms is CPU wall time, not simulation or rendering",
            "scaling_examples_not_calibration":{"tension_distance_m":tension_distance,"compression_distance_m":compression_distance},
            "warning":"No wrinkle-accuracy, solver-speed or production-optimality claim. See docs/pie-resolution.md."
        }))?,
    )?;
    let mut csv = String::from("u_m,v_m,axis_0_spacing_m,axis_1_spacing_m\n");
    let lo = input
        .domain
        .outer
        .points
        .iter()
        .copied()
        .fold(DVec2::splat(f64::INFINITY), DVec2::min);
    let hi = input
        .domain
        .outer
        .points
        .iter()
        .copied()
        .fold(DVec2::splat(f64::NEG_INFINITY), DVec2::max);
    for y in 0..=100 {
        for x in 0..=100 {
            let p = lo + (hi - lo) * DVec2::new(x as f64 / 100., y as f64 / 100.);
            if input.domain.contains(p) {
                let h = map.spacing(p);
                writeln!(csv, "{},{},{},{}", p.x, p.y, h[0], h[1])?;
            }
        }
    }
    fs::write(args.output.join("sizing.csv"), csv)?;
    Ok(())
}
