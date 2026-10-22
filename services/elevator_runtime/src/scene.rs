//! Measured scene geometry. Missing depth is unknown space, never free space.
//! Point-cloud work runs outside the action supervisor.
use crate::unix_us;
use anyhow::{Context, Result, ensure};
use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeMap, BTreeSet, VecDeque},
    sync::{Arc, RwLock},
};

pub type V3 = [f64; 3];
pub fn wrap(a: f64) -> f64 {
    a.sin().atan2(a.cos())
}
pub fn rotate(p: [f64; 2], yaw: f64) -> [f64; 2] {
    let (s, c) = yaw.sin_cos();
    [c * p[0] - s * p[1], s * p[0] + c * p[1]]
}
pub fn world(p: [f64; 2], pose: V3) -> [f64; 2] {
    let p = rotate(p, pose[2]);
    [p[0] + pose[0], p[1] + pose[1]]
}
pub fn local(p: [f64; 2], pose: V3) -> [f64; 2] {
    rotate([p[0] - pose[0], p[1] - pose[1]], -pose[2])
}
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct CloudSource {
    pub topic: String,
    pub frame: String,
    pub parent_link: String,
    pub translation: V3,
    pub quaternion_xyzw: [f64; 4],
    pub provenance: String,
    pub heading: bool,
}
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SceneSettings {
    pub sources: Vec<CloudSource>,
    /// Measured complete travel envelope, half length, half width, top height.
    pub footprint: V3,
    pub travel_joints: BTreeMap<String, f64>,
    pub margin_m: f64,
    pub max_age_s: f64,
    pub max_skew_s: f64,
    pub floor_z_m: f64,
    pub floor_tolerance_m: f64,
    pub floor_grid_m: f64,
    pub minimum_door_width_m: f64,
    pub maximum_door_width_m: f64,
    pub max_step_m: f64,
    pub max_turn_rad: f64,
    pub max_drift_m: f64,
    pub linear_speed_mps: f64,
    pub angular_speed_radps: f64,
    pub acceleration_mps2: f64,
    pub angular_acceleration_radps2: f64,
    pub position_tolerance_m: f64,
    pub angle_tolerance_rad: f64,
    pub no_progress_s: f64,
}
impl SceneSettings {
    pub fn validate(&self) -> Result<()> {
        ensure!(
            !self.sources.is_empty() && self.sources.len() <= 2,
            "configure one or two point-cloud sources"
        );
        ensure!(
            self.sources.iter().filter(|s| s.heading).count() == 1,
            "exactly one source must supply lidar heading"
        );
        let mut topics = BTreeSet::new();
        for s in &self.sources {
            ensure!(
                topics.insert(&s.topic)
                    && s.topic.starts_with('/')
                    && !s.frame.is_empty()
                    && !s.parent_link.is_empty()
                    && !s.provenance.trim().is_empty(),
                "invalid cloud identity or calibration provenance"
            );
            ensure!(
                s.translation
                    .iter()
                    .chain(&s.quaternion_xyzw)
                    .all(|x| x.is_finite())
                    && (s.quaternion_xyzw.iter().map(|x| x * x).sum::<f64>() - 1.).abs() < 1e-3,
                "invalid cloud transform"
            );
        }
        for v in self.footprint.into_iter().chain([
            self.margin_m,
            self.max_age_s,
            self.max_skew_s,
            self.floor_tolerance_m,
            self.floor_grid_m,
            self.minimum_door_width_m,
            self.maximum_door_width_m,
            self.max_step_m,
            self.max_turn_rad,
            self.max_drift_m,
            self.linear_speed_mps,
            self.angular_speed_radps,
            self.acceleration_mps2,
            self.angular_acceleration_radps2,
            self.position_tolerance_m,
            self.angle_tolerance_rad,
            self.no_progress_s,
        ]) {
            ensure!(
                v.is_finite() && v > 0.,
                "scene limits must be finite and positive"
            );
        }
        ensure!(
            self.floor_z_m.is_finite()
                && self.max_age_s <= 0.5
                && self.max_skew_s <= 0.05
                && (0.025..=0.1).contains(&self.floor_grid_m)
                && (2. * self.footprint[0] / self.floor_grid_m).ceil()
                    * (2. * self.footprint[1] / self.floor_grid_m).ceil()
                    <= 4096.
                && self.footprint[0] <= 1.
                && self.footprint[1] <= 1.
                && self.footprint[2] <= 3.
                && self.floor_tolerance_m <= 0.03
                && self.position_tolerance_m <= 0.03
                && self.angle_tolerance_rad <= 0.05
                && self.max_step_m <= 2.
                && self.max_turn_rad <= std::f64::consts::PI
                && self.linear_speed_mps <= 0.15
                && self.angular_speed_radps <= 0.15
                && self.minimum_door_width_m < self.maximum_door_width_m,
            "scene limits exceed supported bounds"
        );
        let required: BTreeSet<_> = ["left", "right"]
            .into_iter()
            .flat_map(|s| (1..=7).map(move |i| format!("{s}_joint{i}")))
            .chain(["carriage_joint".to_owned()])
            .collect();
        ensure!(
            required.iter().all(|n| self.travel_joints.contains_key(n))
                && self.travel_joints.values().all(|v| v.is_finite()),
            "travel posture must name both arms and carriage"
        );
        Ok(())
    }
}
#[derive(Clone, Debug, Serialize)]
pub struct Plane {
    pub point: V3,
    pub normal: V3,
    pub uncertainty_m: f64,
}
#[derive(Clone, Debug, Serialize)]
pub struct Doorway {
    pub id: String,
    pub center: V3,
    pub normal: V3,
    pub width_m: f64,
    pub state: String,
    pub stable_samples: usize,
    pub uncertainty_m: f64,
}
#[derive(Clone, Debug, Serialize)]
pub struct Scene {
    pub observation_id: String,
    pub capture_us: u64,
    pub frame: String,
    pub pose: V3,
    pub heading_source: String,
    pub heading_rmse_m: f64,
    pub panel: Option<Plane>,
    pub doorway: Option<Doorway>,
    pub sources: Vec<String>,
    #[serde(skip)]
    pub points: Vec<V3>,
    #[serde(skip)]
    pub clearance: Clearance,
}
#[derive(Clone, Debug, Default)]
pub struct Clearance {
    floor: BTreeMap<(i32, i32), Vec<[f64; 2]>>,
    obstacles: Vec<[f64; 2]>,
}
impl Clearance {
    pub fn from_points(points: &[V3], cfg: &SceneSettings) -> Self {
        let mut result = Self::default();
        let mut obstacles = BTreeSet::new();
        for p in points {
            let cell = (
                (p[0] / cfg.floor_grid_m).floor() as i32,
                (p[1] / cfg.floor_grid_m).floor() as i32,
            );
            if (p[2] - cfg.floor_z_m).abs() <= cfg.floor_tolerance_m {
                result.floor.entry(cell).or_default().push([p[0], p[1]]);
            } else if p[2] > cfg.floor_z_m + cfg.floor_tolerance_m && p[2] < cfg.footprint[2] {
                obstacles.insert(cell);
            }
        }
        result.obstacles = obstacles
            .into_iter()
            .map(|(x, y)| {
                [
                    (x as f64 + 0.5) * cfg.floor_grid_m,
                    (y as f64 + 0.5) * cfg.floor_grid_m,
                ]
            })
            .collect();
        result
    }
}
#[derive(Default)]
pub struct SceneStore {
    value: RwLock<Option<Arc<Scene>>>,
    history: RwLock<VecDeque<Arc<Scene>>>,
}
impl SceneStore {
    pub fn set(&self, scene: Scene) {
        let scene = Arc::new(scene);
        *self.value.write().unwrap() = Some(scene.clone());
        let mut history = self.history.write().unwrap();
        history.push_back(scene);
        while history.len() > 32 {
            history.pop_front();
        }
    }
    pub fn at(&self, stamp: u64, max_skew_s: f64) -> Result<Arc<Scene>> {
        self.history
            .read()
            .unwrap()
            .iter()
            .min_by_key(|s| s.capture_us.abs_diff(stamp))
            .filter(|s| s.capture_us.abs_diff(stamp) <= (max_skew_s * 1e6) as u64)
            .cloned()
            .context("detector capture has no synchronized point cloud")
    }
    pub fn invalidate(&self) {
        *self.value.write().unwrap() = None;
    }
    pub fn read(&self, settings: &SceneSettings) -> Result<Arc<Scene>> {
        let scene = self.value.read().unwrap().clone().context(
            "scene unavailable; calibrated point clouds and lidar tracking are required",
        )?;
        ensure!(
            unix_us()
                .checked_sub(scene.capture_us)
                .is_some_and(|age| age as f64 <= settings.max_age_s * 1e6),
            "scene is stale or has a future timestamp"
        );
        Ok(scene)
    }
}

/// Planar scan-to-fixed-reference alignment, as used for partial turns in Evo #1102.
/// Wheel yaw is never substituted for a rejected lidar fit.
#[derive(Default)]
pub struct Heading {
    reference: Vec<([f64; 2], [f64; 2])>,
    pose: V3,
    last_stamp: u64,
}

/// Fit local wall tangents. Point-coordinate spread alone does not constrain
/// sliding along parallel walls, even if nearest-point ICP reports a small error.
fn wall_samples(scan: &[[f64; 2]]) -> Vec<([f64; 2], [f64; 2])> {
    scan.iter()
        .filter_map(|p| {
            let neighbors: Vec<_> = scan
                .iter()
                .filter(|q| (p[0] - q[0]).hypot(p[1] - q[1]) <= 0.22)
                .collect();
            if neighbors.len() < 4 {
                return None;
            }
            let count = neighbors.len() as f64;
            let mean = [
                neighbors.iter().map(|q| q[0]).sum::<f64>() / count,
                neighbors.iter().map(|q| q[1]).sum::<f64>() / count,
            ];
            let (mut xx, mut yy, mut xy) = (0., 0., 0.);
            for q in neighbors {
                let (x, y) = (q[0] - mean[0], q[1] - mean[1]);
                xx += x * x;
                yy += y * y;
                xy += x * y;
            }
            let spread = (xx - yy).hypot(2. * xy);
            let major = (xx + yy + spread) / 2.;
            let minor = (xx + yy - spread) / 2.;
            if major < 0.001 || minor / major > 0.1 {
                return None;
            }
            let tangent = (2. * xy).atan2(xx - yy) / 2.;
            Some((*p, [-tangent.sin(), tangent.cos()]))
        })
        .collect()
}

fn solve_pose(mut h: [[f64; 3]; 3], mut rhs: V3) -> Result<V3> {
    let scale = h.iter().flatten().map(|v| v.abs()).fold(0., f64::max);
    ensure!(
        scale.is_finite() && scale > 0.,
        "lidar pose is unconstrained"
    );
    for column in 0..3 {
        let pivot = (column..3)
            .max_by(|a, b| h[*a][column].abs().total_cmp(&h[*b][column].abs()))
            .unwrap();
        ensure!(
            h[pivot][column].abs() > scale * 1e-6,
            "lidar pose is unconstrained"
        );
        h.swap(column, pivot);
        rhs.swap(column, pivot);
        let divisor = h[column][column];
        for j in column..3 {
            h[column][j] /= divisor;
        }
        rhs[column] /= divisor;
        for row in 0..3 {
            if row == column {
                continue;
            }
            let factor = h[row][column];
            for j in column..3 {
                h[row][j] -= factor * h[column][j];
            }
            rhs[row] -= factor * rhs[column];
        }
    }
    ensure!(rhs.iter().all(|v| v.is_finite()), "invalid lidar pose fit");
    Ok(rhs)
}

fn fit_walls(pairs: &[([f64; 2], [f64; 2], [f64; 2], f64)]) -> Result<(V3, f64)> {
    let (mut h, mut rhs) = ([[0.; 3]; 3], [0.; 3]);
    let (mut xx, mut yy, mut xy, mut error2) = (0., 0., 0., 0.);
    for (q, r, n, _) in pairs {
        let error = n[0] * (q[0] - r[0]) + n[1] * (q[1] - r[1]);
        let j = [n[0], n[1], -n[0] * q[1] + n[1] * q[0]];
        for row in 0..3 {
            rhs[row] -= j[row] * error;
            for column in 0..3 {
                h[row][column] += j[row] * j[column];
            }
        }
        xx += n[0] * n[0];
        yy += n[1] * n[1];
        xy += n[0] * n[1];
        error2 += error * error;
    }
    let count = pairs.len() as f64;
    ensure!(
        (xx * yy - xy * xy) / (count * count) > 0.02,
        "lidar wall normals cannot constrain both translation axes"
    );
    Ok((solve_pose(h, rhs)?, (error2 / count).sqrt()))
}

impl Heading {
    pub fn update(&mut self, points: &[V3], stamp: u64) -> Result<(V3, f64)> {
        ensure!(
            stamp > self.last_stamp,
            "repeated or reversed lidar timestamp"
        );
        let mut cells = BTreeMap::new();
        for p in points
            .iter()
            .filter(|p| p[2] > 0.15 && p[2] < 1.8 && p[0].hypot(p[1]) < 8.)
        {
            cells
                .entry(((p[0] / 0.06).floor() as i32, (p[1] / 0.06).floor() as i32))
                .or_insert([p[0], p[1]]);
        }
        let all: Vec<_> = cells.into_values().collect();
        let scan: Vec<_> = all
            .iter()
            .step_by(all.len().div_ceil(600).max(1))
            .copied()
            .collect();
        ensure!(scan.len() >= 30, "insufficient lidar geometry");
        if self.reference.is_empty() {
            let reference = wall_samples(&scan);
            ensure!(reference.len() >= 30, "insufficient lidar wall geometry");
            let pairs: Vec<_> = reference.iter().map(|(p, n)| (*p, *p, *n, 0.)).collect();
            fit_walls(&pairs)?;
            self.reference = reference;
            self.last_stamp = stamp;
            return Ok((self.pose, 0.));
        }
        ensure!(
            stamp - self.last_stamp <= 500_000,
            "lidar tracking interrupted; start a new scene reference"
        );
        let mut pose = self.pose;
        let mut rmse = 0.;
        let mut converged = false;
        for _ in 0..25 {
            let mut pairs = Vec::new();
            for p in &scan {
                let q = world(*p, pose);
                let best = self
                    .reference
                    .iter()
                    .map(|(r, n)| (*r, *n, (r[0] - q[0]).powi(2) + (r[1] - q[1]).powi(2)))
                    .min_by(|a, b| a.2.total_cmp(&b.2))
                    .unwrap();
                if best.2 < 0.15_f64.powi(2) {
                    pairs.push((q, best.0, best.1, best.2));
                }
            }
            ensure!(
                pairs.len() >= 30 && pairs.len() as f64 / scan.len() as f64 >= 0.6,
                "lidar overlap is insufficient"
            );
            pairs.sort_by(|a, b| a.3.total_cmp(&b.3));
            pairs.truncate((pairs.len() * 9 / 10).max(3));
            let (delta, residual) = fit_walls(&pairs)?;
            ensure!(
                delta[0].hypot(delta[1]) <= 0.1 && delta[2].abs() <= 0.1,
                "lidar fit exceeded its local correction bound"
            );
            let pr = rotate([pose[0], pose[1]], delta[2]);
            pose = [pr[0] + delta[0], pr[1] + delta[1], wrap(pose[2] + delta[2])];
            rmse = residual;
            if delta[2].abs() < 1e-4 && delta[0].hypot(delta[1]) < 1e-4 {
                converged = true;
                break;
            }
        }
        ensure!(
            converged && rmse < 0.04,
            "lidar fit did not converge within the residual limit"
        );
        let dt = (stamp - self.last_stamp) as f64 / 1e6;
        ensure!(
            (pose[0] - self.pose[0]).hypot(pose[1] - self.pose[1]) <= 0.25 * dt + 0.015
                && wrap(pose[2] - self.pose[2]).abs() <= 0.3 * dt + 0.015,
            "lidar pose jump"
        );
        self.pose = pose;
        self.last_stamp = stamp;
        Ok((pose, rmse))
    }
}

pub fn panel(points: &[V3]) -> Option<Plane> {
    let points: Vec<_> = points
        .iter()
        .filter(|p| p[0] > 0.15 && p[0] < 3. && p[1].abs() < 2. && p[2] > 0.25 && p[2] < 1.8)
        .collect();
    if points.len() < 30 {
        return None;
    }
    let points: Vec<_> = points
        .iter()
        .step_by(points.len().div_ceil(2000))
        .copied()
        .collect();
    let sample: Vec<_> = points
        .iter()
        .step_by((points.len() / 80).max(1))
        .take(80)
        .collect();
    let mut best = (0, [0.; 3], [0.; 3], 0.);
    for (i, a) in sample.iter().enumerate() {
        for b in sample.iter().skip(i + 1).step_by(12) {
            let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
            let length = dx.hypot(dy);
            if length < 0.3 {
                continue;
            }
            let mut n = [-dy / length, dx / length, 0.];
            if n[0] * a[0] + n[1] * a[1] < 0. {
                n[0] = -n[0];
                n[1] = -n[1];
            }
            if n[0] < 0.5 {
                continue;
            }
            let distance = n[0] * a[0] + n[1] * a[1];
            let inliers: Vec<_> = points
                .iter()
                .filter(|p| (p[0] * n[0] + p[1] * n[1] - distance).abs() < 0.025)
                .collect();
            if inliers.len() > best.0 {
                let residual = inliers
                    .iter()
                    .map(|p| (p[0] * n[0] + p[1] * n[1] - distance).abs())
                    .sum::<f64>()
                    / inliers.len() as f64;
                best = (
                    inliers.len(),
                    [distance * n[0], distance * n[1], 0.],
                    n,
                    residual,
                );
            }
        }
    }
    (best.0 >= 30).then(|| Plane {
        point: best.1,
        normal: best.2,
        uncertainty_m: best.3.max(0.015),
    })
}

/// A plane alone is not enough: require local depth returns at the press pixel.
pub fn surface_from_ray(scene: &Scene, origin: V3, direction: V3) -> Result<serde_json::Value> {
    ensure!(
        origin.iter().chain(&direction).all(|v| v.is_finite()),
        "nonfinite camera ray"
    );
    let plane = scene.panel.as_ref().context("panel plane unavailable")?;
    let denominator = direction
        .iter()
        .zip(plane.normal)
        .map(|(d, n)| d * n)
        .sum::<f64>();
    ensure!(denominator >= 0.2, "camera ray is too oblique to the panel");
    let distance = (0..3)
        .map(|i| (plane.point[i] - origin[i]) * plane.normal[i])
        .sum::<f64>()
        / denominator;
    ensure!(distance > 0., "panel is behind the camera");
    let point: V3 = std::array::from_fn(|i| origin[i] + distance * direction[i]);
    let support = scene
        .points
        .iter()
        .filter(|p| {
            let radius = (0..3)
                .map(|i| (p[i] - point[i]).powi(2))
                .sum::<f64>()
                .sqrt();
            let offset = (0..3)
                .map(|i| (p[i] - plane.point[i]) * plane.normal[i])
                .sum::<f64>()
                .abs();
            radius < 0.08 && offset < 0.025
        })
        .count();
    ensure!(
        support >= 6,
        "press point has insufficient local depth support"
    );
    for p in &scene.points {
        let along = (0..3)
            .map(|i| (p[i] - origin[i]) * direction[i])
            .sum::<f64>();
        let perpendicular = (0..3)
            .map(|i| (p[i] - origin[i] - along * direction[i]).powi(2))
            .sum::<f64>()
            .sqrt();
        ensure!(
            !(along > 0.05 && along < distance - 0.05 && perpendicular < 0.025),
            "press point is occluded by a nearer surface"
        );
    }
    Ok(
        serde_json::json!({"position":point,"normal":plane.normal,"uncertainty_m":plane.uncertainty_m,"capture_us":scene.capture_us,"support_points":support}),
    )
}

#[derive(Default)]
pub struct DoorTracker {
    previous: Option<(Doorway, [f64; 2], f64, u64)>,
    sequence: u64,
}
impl DoorTracker {
    pub fn reset_confirmation(&mut self) {
        if let Some((door, _, _, _)) = self.previous.as_mut() {
            door.stable_samples = 0;
            door.state = "unknown".into();
        }
    }
    pub fn observe(
        &mut self,
        points: &[V3],
        plane: Option<&Plane>,
        pose: V3,
        stamp: u64,
        cfg: &SceneSettings,
    ) -> Option<Doorway> {
        let result = self.observe_inner(points, plane, pose, stamp, cfg);
        if result.is_none() {
            self.reset_confirmation();
        }
        result
    }
    fn observe_inner(
        &mut self,
        points: &[V3],
        plane: Option<&Plane>,
        pose: V3,
        stamp: u64,
        cfg: &SceneSettings,
    ) -> Option<Doorway> {
        let tracked = self
            .previous
            .as_ref()
            .filter(|(_, _, _, ts)| stamp > *ts && stamp - *ts < 500_000)
            .map(|(door, center, yaw, _)| {
                let center = local(*center, pose);
                let normal = rotate([yaw.cos(), yaw.sin()], -pose[2]);
                Plane {
                    point: [center[0], center[1], cfg.floor_z_m],
                    normal: [normal[0], normal[1], 0.],
                    uncertainty_m: door.uncertainty_m,
                }
            });
        let plane = tracked.as_ref().or(plane)?;
        let n = plane.normal;
        let t = [-n[1], n[0]];
        let distance = n[0] * plane.point[0] + n[1] * plane.point[1];
        let mut bins = BTreeSet::new();
        for p in points.iter().filter(|p| {
            p[2] > 0.2 && p[2] < 1.7 && (p[0] * n[0] + p[1] * n[1] - distance).abs() < 0.04
        }) {
            bins.insert(((p[0] * t[0] + p[1] * t[1]) / 0.025).round() as i32);
        }
        let bins: Vec<_> = bins.into_iter().collect();
        let gap = bins
            .windows(2)
            .filter_map(|w| {
                let width = (w[1] - w[0] - 1) as f64 * 0.025;
                (width >= cfg.minimum_door_width_m && width <= cfg.maximum_door_width_m)
                    .then_some(((w[0] + w[1]) as f64 * 0.0125, width))
            })
            .min_by(|a, b| a.0.abs().total_cmp(&b.0.abs()));
        let (offset, width) = gap?;
        let center = [
            distance * n[0] + offset * t[0],
            distance * n[1] + offset * t[1],
            cfg.floor_z_m,
        ];
        // Require jamb support at different heights on both sides of the aperture.
        let jambs = [offset - width / 2., offset + width / 2.];
        if !jambs.iter().all(|j| {
            [0.3, 0.9].iter().all(|z| {
                points.iter().any(|p| {
                    (p[0] * n[0] + p[1] * n[1] - distance).abs() < 0.04
                        && (p[0] * t[0] + p[1] * t[1] - j).abs() < 0.08
                        && (p[2] - z).abs() < 0.2
                })
            })
        }) {
            return None;
        }
        // Returns beyond the aperture are positive evidence of an opening.
        let beyond = points
            .iter()
            .filter(|p| {
                let along = (p[0] - center[0]) * n[0] + (p[1] - center[1]) * n[1];
                let across = (p[0] - center[0]) * t[0] + (p[1] - center[1]) * t[1];
                along > 0.2
                    && along < 2.
                    && across.abs() < width / 2. - cfg.margin_m
                    && p[2] > cfg.floor_z_m - 0.03
            })
            .count();
        let center_world = world([center[0], center[1]], pose);
        let yaw = wrap(n[1].atan2(n[0]) + pose[2]);
        let previous = self.previous.as_ref().filter(|(_, c, a, ts)| {
            stamp > *ts
                && stamp - *ts < 500_000
                && (c[0] - center_world[0]).hypot(c[1] - center_world[1]) < 0.12
                && wrap(*a - yaw).abs() < 0.1
        });
        let (id, mut stable, motion) = if let Some((old, _, _, _)) = previous {
            let delta = width - old.width_m;
            (
                old.id.clone(),
                if delta.abs() < 0.015 {
                    old.stable_samples + 1
                } else {
                    0
                },
                if delta < -0.015 {
                    "closing"
                } else if delta > 0.015 {
                    "opening"
                } else {
                    "open"
                },
            )
        } else {
            self.sequence += 1;
            (format!("door-{}", self.sequence), 1, "opening")
        };
        let state = if beyond < 20 {
            "unknown"
        } else if width < 2. * (cfg.footprint[1] + cfg.margin_m + plane.uncertainty_m) {
            "blocked"
        } else if stable < 3 && motion == "open" {
            "opening"
        } else {
            motion
        };
        if state == "unknown" || state == "blocked" {
            stable = 0;
        }
        let door = Doorway {
            id,
            center,
            normal: n,
            width_m: width,
            state: state.into(),
            stable_samples: stable,
            uncertainty_m: plane.uncertainty_m,
        };
        self.previous = Some((door.clone(), center_world, yaw, stamp));
        Some(door)
    }
}

/// Check a sampled swept footprint and measured floor support. Unknown cells refuse.
pub fn clear_path(scene: &Scene, cfg: &SceneSettings, delta: V3) -> Result<()> {
    let floor = &scene.clearance.floor;
    let distance = delta[0].hypot(delta[1]);
    let samples = ((distance / cfg.floor_grid_m)
        .max(delta[2].abs() / 0.04)
        .ceil() as usize)
        .max(1);
    ensure!(samples <= 100, "base path too long");
    for step in 0..=samples {
        let f = step as f64 / samples as f64;
        let center = [f * delta[0], f * delta[1]];
        let yaw = f * delta[2];
        let (sin, cos) = yaw.sin_cos();
        // Include the complete occupied cell, not just its center.
        let cell_margin = cfg.floor_grid_m / std::f64::consts::SQRT_2;
        for p in &scene.clearance.obstacles {
            let (x, y) = (p[0] - center[0], p[1] - center[1]);
            let q = [cos * x + sin * y, -sin * x + cos * y];
            let inside = q[0].abs() < cfg.footprint[0] + cfg.margin_m + cell_margin
                && q[1].abs() < cfg.footprint[1] + cfg.margin_m + cell_margin;
            ensure!(!inside, "obstacle in swept robot footprint");
        }
        // The currently occupied footprint is supported; require evidence for new floor.
        let nx = (2. * cfg.footprint[0] / cfg.floor_grid_m).ceil() as usize;
        let ny = (2. * cfg.footprint[1] / cfg.floor_grid_m).ceil() as usize;
        for ix in 0..=nx {
            for iy in 0..=ny {
                let p = rotate(
                    [
                        -cfg.footprint[0] + ix as f64 * 2. * cfg.footprint[0] / nx as f64,
                        -cfg.footprint[1] + iy as f64 * 2. * cfg.footprint[1] / ny as f64,
                    ],
                    yaw,
                );
                let p = [p[0] + center[0], p[1] + center[1]];
                if p[0].abs() <= cfg.footprint[0] && p[1].abs() <= cfg.footprint[1] {
                    continue;
                }
                let cell = (
                    (p[0] / cfg.floor_grid_m).floor() as i32,
                    (p[1] / cfg.floor_grid_m).floor() as i32,
                );
                let supported = (-1..=1).any(|dx| {
                    (-1..=1).any(|dy| {
                        floor
                            .get(&(cell.0 + dx, cell.1 + dy))
                            .is_some_and(|values| {
                                values.iter().any(|q| {
                                    (q[0] - p[0]).hypot(q[1] - p[1]) <= cfg.floor_grid_m * 0.75
                                })
                            })
                    })
                });
                ensure!(
                    supported,
                    "floor support is unknown or sill exceeds the configured limit"
                );
            }
        }
    }
    Ok(())
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    pub fn config() -> SceneSettings {
        SceneSettings {
            sources: vec![CloudSource {
                topic: "/test/cloud".into(),
                frame: "test_lidar".into(),
                parent_link: "base_link".into(),
                translation: [0.; 3],
                quaternion_xyzw: [0., 0., 0., 1.],
                provenance: "synthetic-test-only".into(),
                heading: true,
            }],
            footprint: [0.25, 0.2, 1.6],
            travel_joints: ["left", "right"]
                .into_iter()
                .flat_map(|s| (1..=7).map(move |i| (format!("{s}_joint{i}"), 0.)))
                .chain([("carriage_joint".into(), 0.)])
                .collect(),
            margin_m: 0.05,
            max_age_s: 0.3,
            max_skew_s: 0.05,
            floor_z_m: 0.,
            floor_tolerance_m: 0.02,
            floor_grid_m: 0.05,
            minimum_door_width_m: 0.7,
            maximum_door_width_m: 1.5,
            max_step_m: 2.,
            max_turn_rad: std::f64::consts::PI,
            max_drift_m: 0.08,
            linear_speed_mps: 0.1,
            angular_speed_radps: 0.1,
            acceleration_mps2: 0.1,
            angular_acceleration_radps2: 0.1,
            position_tolerance_m: 0.015,
            angle_tolerance_rad: 0.02,
            no_progress_s: 3.,
        }
    }
    pub fn scene() -> Scene {
        let points: Vec<V3> = (-100..=100)
            .flat_map(|x| (-80..=80).map(move |y| [x as f64 * 0.025, y as f64 * 0.025, 0.]))
            .collect();
        Scene {
            observation_id: "test".into(),
            capture_us: unix_us(),
            frame: "base_link".into(),
            pose: [0.; 3],
            heading_source: "lidar_fixed_reference".into(),
            heading_rmse_m: 0.,
            panel: None,
            doorway: None,
            sources: vec!["test".into()],
            clearance: Clearance::from_points(&points, &config()),
            points,
        }
    }
    pub fn aperture(width: f64) -> Vec<V3> {
        let mut points = scene().points;
        for y in -80..=80 {
            let y = y as f64 * 0.025;
            if y.abs() >= width / 2. {
                for z in 1..=16 {
                    points.push([1., y, z as f64 * 0.1]);
                }
            }
        }
        for y in -20..=20 {
            for z in 1..=16 {
                points.push([2., y as f64 * 0.025, z as f64 * 0.1]);
            }
        }
        points
    }
    #[test]
    fn settings_require_both_arms_and_calibration() {
        let mut c = config();
        c.validate().unwrap();
        c.travel_joints.remove("right_joint4");
        assert!(c.validate().is_err());
        c = config();
        c.sources[0].provenance.clear();
        assert!(c.validate().is_err());
        c = config();
        c.sources[0].heading = false;
        assert!(c.validate().is_err());
    }
    #[test]
    fn swept_path_rejects_obstacles_unknown_floor_and_sill() {
        let c = config();
        let mut s = scene();
        clear_path(&s, &c, [0.5, 0., 0.]).unwrap();
        clear_path(&s, &c, [0., 0., 1.]).unwrap();
        s.points.push([0.5, 0., 0.5]);
        s.clearance = Clearance::from_points(&s.points, &c);
        assert!(clear_path(&s, &c, [0.5, 0., 0.]).is_err());
        s = scene();
        s.points.retain(|p| p[0] < 0.4 || p[0] > 0.6);
        s.clearance = Clearance::from_points(&s.points, &c);
        assert!(clear_path(&s, &c, [0.5, 0., 0.]).is_err());
        s = scene();
        for p in &mut s.points {
            if p[0] > 0.35 && p[0] < 0.5 {
                p[2] = 0.06;
            }
        }
        s.clearance = Clearance::from_points(&s.points, &c);
        assert!(clear_path(&s, &c, [0.5, 0., 0.]).is_err());
    }
    #[test]
    fn door_requires_stable_width_and_detects_closure() {
        let c = config();
        let mut tracker = DoorTracker::default();
        let p = aperture(1.2);
        let plane = Plane {
            point: [1., 0., 0.],
            normal: [1., 0., 0.],
            uncertainty_m: 0.015,
        };
        for i in 1..=3 {
            let door = tracker
                .observe(&p, Some(&plane), [0.; 3], i * 100_000, &c)
                .unwrap();
            assert_eq!(door.state, if i == 3 { "open" } else { "opening" });
        }
        let door = tracker
            .observe(&aperture(1.1), Some(&plane), [0.; 3], 400_000, &c)
            .unwrap();
        assert_eq!(door.state, "closing");
        assert!(
            tracker
                .observe(&aperture(0.), Some(&plane), [0.; 3], 500_000, &c)
                .is_none()
        );
    }
    #[test]
    fn lost_door_evidence_requires_new_stable_samples() {
        let c = config();
        let mut tracker = DoorTracker::default();
        let p = aperture(1.2);
        let plane = Plane {
            point: [1., 0., 0.],
            normal: [1., 0., 0.],
            uncertainty_m: 0.015,
        };
        for i in 1..=3 {
            tracker
                .observe(&p, Some(&plane), [0.; 3], i * 100_000, &c)
                .unwrap();
        }
        assert!(tracker.observe(&[], None, [0.; 3], 400_000, &c).is_none());
        let d = tracker
            .observe(&p, Some(&plane), [0.; 3], 500_000, &c)
            .unwrap();
        assert_ne!(d.state, "open");
        assert_eq!(d.stable_samples, 1);
    }
    #[test]
    fn press_ray_needs_local_support_and_no_nearer_surface() {
        let mut s = scene();
        s.panel = Some(Plane {
            point: [1., 0., 0.],
            normal: [1., 0., 0.],
            uncertainty_m: 0.015,
        });
        assert!(surface_from_ray(&s, [0., 0., 1.], [1., 0., 0.]).is_err());
        for y in -3..=3 {
            for z in -3..=3 {
                s.points.push([1., y as f64 * 0.01, 1. + z as f64 * 0.01]);
            }
        }
        surface_from_ray(&s, [0., 0., 1.], [1., 0., 0.]).unwrap();
        s.points.push([0.5, 0., 1.]);
        assert!(surface_from_ray(&s, [0., 0., 1.], [1., 0., 0.]).is_err());
    }
    #[test]
    fn doorway_stays_identified_after_crossing() {
        let c = config();
        let mut tracker = DoorTracker::default();
        let p = aperture(1.2);
        let plane = Plane {
            point: [1., 0., 0.],
            normal: [1., 0., 0.],
            uncertainty_m: 0.015,
        };
        for i in 1..=3 {
            tracker
                .observe(&p, Some(&plane), [0.; 3], i * 100_000, &c)
                .unwrap();
        }
        let shifted: Vec<_> = p.iter().map(|p| [p[0] - 1.5, p[1], p[2]]).collect();
        let d = tracker
            .observe(&shifted, None, [1.5, 0., 0.], 400_000, &c)
            .unwrap();
        assert_eq!(d.id, "door-1");
        assert_eq!(d.state, "open");
        assert!((d.center[0] + 0.5).abs() < 0.03);
    }
    #[test]
    fn stale_future_and_missing_scenes_refuse() {
        let c = config();
        let store = SceneStore::default();
        assert!(store.read(&c).is_err());
        let mut s = scene();
        s.capture_us = unix_us() - 1_000_000;
        store.set(s);
        assert!(store.read(&c).is_err());
        let mut s = scene();
        s.capture_us = unix_us() + 1_000_000;
        store.set(s);
        assert!(store.read(&c).is_err());
        store.set(scene());
        assert!(store.read(&c).is_ok());
        store.invalidate();
        assert!(store.read(&c).is_err());
    }
    fn localization_cloud() -> Vec<V3> {
        let mut points = aperture(1.2);
        // Nonparallel surfaces are needed to measure both translation axes.
        for x in -80..=120 {
            points.push([x as f64 * 0.025, 2., 1.]);
            points.push([x as f64 * 0.025, -2., 1.]);
        }
        points
    }
    #[test]
    fn lidar_rejects_parallel_walls() {
        let mut h = Heading::default();
        assert!(h.update(&aperture(1.2), 100_000).is_err());
        h.update(&localization_cloud(), 100_000).unwrap();
        // Losing the constraining walls during motion also refuses the fit.
        assert!(h.update(&aperture(1.2), 200_000).is_err());
    }
    #[test]
    fn lidar_rejects_degenerate_geometry_and_timestamp_reuse() {
        let mut h = Heading::default();
        let line: Vec<_> = (0..100).map(|i| [i as f64 * 0.07, 1., 1.]).collect();
        assert!(h.update(&line, 100_000).is_err());
        let p = localization_cloud();
        h.update(&p, 100_000).unwrap();
        h.update(&p, 200_000).unwrap();
        assert!(h.update(&p, 200_000).is_err());
        assert!(h.update(&p, 900_000).is_err());
    }
    #[test]
    fn lidar_measures_translation_and_signed_yaw() {
        for sign in [-1., 1.] {
            let mut h = Heading::default();
            let p = localization_cloud();
            h.update(&p, 100_000).unwrap();
            let pose = [0.008, -0.004, sign * 0.008];
            let moved: Vec<_> = p
                .iter()
                .map(|p| {
                    let q = local([p[0], p[1]], pose);
                    [q[0], q[1], p[2]]
                })
                .collect();
            let (measured, _) = h.update(&moved, 200_000).unwrap();
            for i in 0..3 {
                assert!(
                    (measured[i] - pose[i]).abs() < 0.005,
                    "{measured:?} != {pose:?}"
                );
            }
        }
    }
}
