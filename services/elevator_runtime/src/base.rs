//! RAX swerve feedback controller. Evo executes deadline-bounded velocity pulses.
use crate::scene::*;
use anyhow::{Result, ensure};
use serde::{Deserialize, Serialize};
use serde_json::{Value, json};

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BaseGoal {
    pub delta: V3,
    pub doorway_id: Option<String>,
}
impl BaseGoal {
    pub fn validate(&self, cfg: &SceneSettings) -> Result<()> {
        ensure!(
            self.delta.iter().all(|x| x.is_finite()),
            "base delta must be finite"
        );
        let length = self.delta[0].hypot(self.delta[1]);
        ensure!(
            length <= cfg.max_step_m && self.delta[2].abs() <= cfg.max_turn_rad,
            "base move exceeds configured step bound"
        );
        ensure!(
            2.5 * (length / cfg.linear_speed_mps + self.delta[2].abs() / cfg.angular_speed_radps)
                + 5.
                <= 90.,
            "base goal cannot finish within the action timeout at configured speed"
        );
        ensure!(
            length == 0. || self.delta[2] == 0.,
            "separate base rotation from translation"
        );
        ensure!(
            length > 0. || self.delta[2] != 0.,
            "base move has zero length"
        );
        Ok(())
    }
}
pub fn posture(status: &Value, cfg: &SceneSettings, limits: &Value) -> Result<()> {
    ensure!(
        status["feedback_fresh"] == true && status["motion_blocked"] == false,
        "fresh measured robot state is required"
    );
    ensure!(
        cfg.travel_joints.iter().all(|(name, target)| {
            status["state"]["positions"][name]
                .as_f64()
                .zip(limits[name]["tolerance"].as_f64())
                .is_some_and(|(actual, tolerance)| (actual - target).abs() <= tolerance)
        }),
        "base motion requires the configured complete travel posture"
    );
    Ok(())
}
#[derive(Debug)]
pub struct Controller {
    pub goal: BaseGoal,
    origin: V3,
    target: V3,
    last_pose: V3,
    last_stamp: u64,
    previous_velocity: V3,
    measured_velocity: V3,
    travel: f64,
    rotation: f64,
    best_error: f64,
    last_progress_s: f64,
    checked_stamp: u64,
    door_plane: Option<([f64; 2], [f64; 2])>,
}
impl Controller {
    pub fn new(goal: BaseGoal, scene: &Scene, cfg: &SceneSettings) -> Result<Self> {
        goal.validate(cfg)?;
        clear_path(scene, cfg, goal.delta)?;
        let target = world([goal.delta[0], goal.delta[1]], scene.pose);
        let door_plane = if let Some(id) = &goal.doorway_id {
            let d = scene
                .doorway
                .as_ref()
                .ok_or_else(|| anyhow::anyhow!("doorway not observed"))?;
            ensure!(
                &d.id == id && d.state == "open" && d.stable_samples >= 3,
                "doorway must be stably open before base motion"
            );
            // Orient the crossing plane from this starting side toward the aperture.
            let sign = if d.center[0] * d.normal[0] + d.center[1] * d.normal[1] < 0. {
                -1.
            } else {
                1.
            };
            Some((
                world([d.center[0], d.center[1]], scene.pose),
                rotate([sign * d.normal[0], sign * d.normal[1]], scene.pose[2]),
            ))
        } else {
            None
        };
        Ok(Self {
            goal,
            origin: scene.pose,
            target: [target[0], target[1], scene.pose[2]],
            last_pose: scene.pose,
            last_stamp: scene.capture_us,
            previous_velocity: [0.; 3],
            measured_velocity: [0.; 3],
            travel: 0.,
            rotation: 0.,
            best_error: f64::INFINITY,
            last_progress_s: 0.,
            checked_stamp: 0,
            door_plane,
        }
        .with_yaw())
    }
    fn with_yaw(mut self) -> Self {
        self.target[2] = wrap(self.origin[2] + self.goal.delta[2]);
        self
    }
    pub fn residual(&self, scene: &Scene) -> V3 {
        let p = local([self.target[0], self.target[1]], scene.pose);
        [p[0], p[1], wrap(self.target[2] - scene.pose[2])]
    }
    pub fn timeout_s(&self, _cfg: &SceneSettings) -> f64 {
        90.
    }
    pub fn step(&mut self, scene: &Scene, cfg: &SceneSettings, elapsed: f64) -> Result<Option<V3>> {
        ensure!(elapsed < self.timeout_s(cfg), "base target timeout");
        ensure!(
            scene.capture_us >= self.last_stamp,
            "scene timestamp reversed"
        );
        if scene.capture_us > self.last_stamp {
            let dt = (scene.capture_us - self.last_stamp) as f64 / 1e6;
            let dx = scene.pose[0] - self.last_pose[0];
            let dy = scene.pose[1] - self.last_pose[1];
            let dyaw = wrap(scene.pose[2] - self.last_pose[2]);
            ensure!(
                dx.hypot(dy) <= 0.25 * dt + 0.015 && dyaw.abs() <= 0.3 * dt + 0.015,
                "base localization jumped"
            );
            self.measured_velocity = [dx / dt, dy / dt, dyaw / dt];
            self.travel += dx.hypot(dy);
            self.rotation += dyaw.abs();
            self.last_pose = scene.pose;
            self.last_stamp = scene.capture_us;
        }
        let error = self.residual(scene);
        let length = error[0].hypot(error[1]);
        let translated = self.goal.delta[0].hypot(self.goal.delta[1]);
        if translated == 0. {
            ensure!(
                (scene.pose[0] - self.origin[0]).hypot(scene.pose[1] - self.origin[1])
                    <= cfg.max_drift_m,
                "rotation translation drift exceeded"
            );
        } else {
            let offset = local([scene.pose[0], scene.pose[1]], self.origin);
            let cross =
                (offset[0] * self.goal.delta[1] - offset[1] * self.goal.delta[0]) / translated;
            ensure!(
                cross.abs() <= cfg.max_drift_m && error[2].abs() < 0.15,
                "translation left the allowed heading or lateral band"
            );
        }
        ensure!(
            self.travel <= 1.5 * translated + cfg.max_drift_m
                && self.rotation <= 1.5 * self.goal.delta[2].abs() + 0.15,
            "cumulative base travel exceeded"
        );
        if let Some(id) = &self.goal.doorway_id {
            let d = scene
                .doorway
                .as_ref()
                .ok_or_else(|| anyhow::anyhow!("doorway visibility lost"))?;
            ensure!(
                &d.id == id && d.state == "open" && d.stable_samples >= 3,
                "doorway closed, changed, or became uncertain"
            );
            let across = (d.center[0] * (-d.normal[1]) + d.center[1] * d.normal[0]).abs();
            let lateral_extent =
                cfg.footprint[0] * d.normal[1].abs() + cfg.footprint[1] * d.normal[0].abs();
            // While centering, allow an offset only if the base has not reached the sill.
            let along = d.center[0] * d.normal[0] + d.center[1] * d.normal[1];
            if along.abs() <= cfg.footprint[0] + cfg.margin_m {
                ensure!(
                    across + lateral_extent + cfg.margin_m + d.uncertainty_m < d.width_m / 2.,
                    "doorway side clearance lost"
                );
            }
        }
        // Check stopping travel in the measured direction, including overshoot.
        let speed = self.measured_velocity[0].hypot(self.measured_velocity[1]);
        let reaction = cfg.max_age_s + crate::TICK_S;
        let brake = speed * reaction + speed * speed / (2. * cfg.acceleration_mps2);
        let velocity = rotate(
            [self.measured_velocity[0], self.measured_velocity[1]],
            -scene.pose[2],
        );
        let angular = self.measured_velocity[2];
        let stopping = [
            if speed > 1e-8 {
                velocity[0] / speed * brake
            } else {
                0.
            },
            if speed > 1e-8 {
                velocity[1] / speed * brake
            } else {
                0.
            },
            angular.signum()
                * (angular.abs() * reaction
                    + angular * angular / (2. * cfg.angular_acceleration_radps2)),
        ];
        let mut remaining = error;
        if translated == 0. {
            remaining[0] = 0.;
            remaining[1] = 0.;
        }
        if scene.capture_us != self.checked_stamp {
            clear_path(scene, cfg, remaining)?;
            clear_path(scene, cfg, stopping)?;
            self.checked_stamp = scene.capture_us;
        }
        let metric = if translated > 0. {
            length
        } else {
            error[2].abs()
        };
        if metric < self.best_error - 0.001 {
            self.best_error = metric;
            self.last_progress_s = elapsed;
        }
        ensure!(
            elapsed - self.last_progress_s < cfg.no_progress_s,
            "base made no measured progress"
        );
        let reached =
            length <= cfg.position_tolerance_m && error[2].abs() <= cfg.angle_tolerance_rad;
        if reached && speed < 0.005 && self.measured_velocity[2].abs() < 0.01 {
            return Ok(None);
        }
        let mut v = [0.; 3];
        if !reached {
            if translated > 0. && length > cfg.position_tolerance_m {
                let rate = (length * 0.8)
                    .min(cfg.linear_speed_mps)
                    .min((2. * cfg.acceleration_mps2 * length).sqrt());
                v[0] = rate * error[0] / length;
                v[1] = rate * error[1] / length;
            }
            v[2] = (error[2] * 1.5).clamp(-cfg.angular_speed_radps, cfg.angular_speed_radps);
        }
        for i in 0..3 {
            let acceleration = if i == 2 {
                cfg.angular_acceleration_radps2
            } else {
                cfg.acceleration_mps2
            };
            v[i] = v[i].clamp(
                self.previous_velocity[i] - acceleration * crate::TICK_S,
                self.previous_velocity[i] + acceleration * crate::TICK_S,
            );
        }
        self.previous_velocity = v;
        Ok(Some(v))
    }
    pub fn final_check(&self, scene: &Scene, cfg: &SceneSettings) -> Result<()> {
        let e = self.residual(scene);
        ensure!(
            e[0].hypot(e[1]) <= cfg.position_tolerance_m && e[2].abs() <= cfg.angle_tolerance_rad,
            "base moved outside target tolerance during stopping"
        );
        Ok(())
    }
    pub fn report(&self, scene: &Scene, cfg: &SceneSettings) -> Value {
        let crossing = self.door_plane.map(|(c, n)| {
            let distance = (scene.pose[0] - c[0]) * n[0] + (scene.pose[1] - c[1]) * n[1];
            let yaw = scene.pose[2] - n[1].atan2(n[0]);
            let extent = cfg.footprint[0] * yaw.cos().abs()
                + cfg.footprint[1] * yaw.sin().abs()
                + cfg.margin_m;
            if distance > extent {
                "after"
            } else if distance < -extent {
                "before"
            } else {
                "straddling"
            }
        });
        json!({"residual":self.residual(scene),"initial_pose":self.origin,"final_pose":scene.pose,"distance_travelled_m":self.travel,"angle_travelled_rad":self.rotation,"crossing_state":crossing,"scene_id":scene.observation_id})
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scene::tests::{config, scene};
    fn goal(delta: V3) -> BaseGoal {
        BaseGoal {
            delta,
            doorway_id: None,
        }
    }
    #[test]
    fn commanded_motion_does_not_imply_arrival_and_stalls_fail() {
        let c = config();
        let s = scene();
        let mut ctl = Controller::new(goal([0.3, 0., 0.]), &s, &c).unwrap();
        assert!(ctl.step(&s, &c, 0.).unwrap().unwrap()[0] > 0.);
        assert!(ctl.step(&s, &c, 3.1).is_err());
    }
    #[test]
    fn measured_target_requires_zero_rate_and_final_residual() {
        let c = config();
        let mut s = scene();
        let mut ctl = Controller::new(goal([0.03, 0., 0.]), &s, &c).unwrap();
        ctl.step(&s, &c, 0.).unwrap();
        s.capture_us += 200_000;
        s.pose[0] = 0.03;
        assert!(ctl.step(&s, &c, 0.2).unwrap().is_some());
        s.capture_us += 100_000;
        assert!(ctl.step(&s, &c, 0.3).unwrap().is_none());
        ctl.final_check(&s, &c).unwrap();
        s.pose[0] += 0.04;
        assert!(ctl.final_check(&s, &c).is_err());
    }
    #[test]
    fn braking_checks_measured_motion_away_from_target() {
        let c = config();
        let mut s = scene();
        let mut ctl = Controller::new(goal([0.3, 0., 0.]), &s, &c).unwrap();
        ctl.step(&s, &c, 0.).unwrap();
        s.capture_us += 200_000;
        s.pose[0] = -0.02;
        s.points.push([-0.38, 0., 0.5]);
        s.clearance = Clearance::from_points(&s.points, &c);
        // The target is clear, but measured rearward travel cannot stop in time.
        clear_path(&s, &c, ctl.residual(&s)).unwrap();
        let error = ctl.step(&s, &c, 0.2).unwrap_err();
        assert!(error.to_string().contains("obstacle"), "{error}");
    }
    #[test]
    fn rotation_wraps_and_translation_drift_refuses() {
        let c = config();
        let mut s = scene();
        s.pose[2] = std::f64::consts::PI - 0.01;
        let mut ctl = Controller::new(goal([0., 0., 0.03]), &s, &c).unwrap();
        assert!(ctl.residual(&s)[2] > 0.);
        s.pose[2] = -std::f64::consts::PI + 0.02;
        assert!(ctl.residual(&s)[2].abs() < 1e-8);
        s.pose[0] = 0.1;
        s.capture_us += 400_000;
        assert!(ctl.step(&s, &c, 0.4).is_err());
    }
    #[test]
    fn door_gate_is_live_and_crossing_uses_full_envelope_both_directions() {
        let c = config();
        for sign in [-1., 1.] {
            let mut s = scene();
            s.doorway = Some(Doorway {
                id: "door".into(),
                center: [sign * 0.5, 0., 0.],
                normal: [1., 0., 0.],
                width_m: 1.2,
                state: "closed".into(),
                stable_samples: 4,
                uncertainty_m: 0.015,
            });
            let g = BaseGoal {
                delta: [sign * 1., 0., 0.],
                doorway_id: Some("door".into()),
            };
            assert!(Controller::new(g.clone(), &s, &c).is_err());
            s.doorway.as_mut().unwrap().state = "open".into();
            let mut ctl = Controller::new(g, &s, &c).unwrap();
            s.pose[0] = sign * 0.6;
            assert_eq!(ctl.report(&s, &c)["crossing_state"], "straddling");
            s.pose[0] = sign * 0.9;
            assert_eq!(ctl.report(&s, &c)["crossing_state"], "after");
            s.pose[0] = 0.;
            s.doorway.as_mut().unwrap().state = "closing".into();
            assert!(ctl.step(&s, &c, 0.1).is_err());
        }
    }
}
