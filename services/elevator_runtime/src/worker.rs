//! RAX task state. Evo remains the only actuator owner.
use crate::{
    base::{self, BaseGoal, Controller},
    buttons, camera,
    rpc::Evo,
    scene::*,
    unix_us,
};
use anyhow::{Context, Result, bail, ensure};
use serde::Deserialize;
use serde_json::{Value, json};
use std::{
    collections::VecDeque,
    sync::{Arc, Mutex},
    time::Instant,
};

#[derive(Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Config {
    pub scene: SceneSettings,
    pub button_service: String,
    pub ros_domain_id: u32,
}
struct Running {
    id: String,
    control: Controller,
    started: Instant,
}
struct Capture {
    id: String,
    camera: String,
    frame: Value,
    scene: Arc<Scene>,
}
pub struct Worker {
    pub config: Config,
    pub evo: Arc<Evo>,
    pub scenes: Arc<SceneStore>,
    limits: Value,
    running: Mutex<Option<Running>>,
    captures: Mutex<VecDeque<Capture>>,
    sequence: std::sync::atomic::AtomicU64,
}
impl Worker {
    pub fn new(config: Config, evo: Arc<Evo>) -> Result<Self> {
        config.scene.validate()?;
        ensure!(
            config.button_service.starts_with('/'),
            "detector service must be explicit"
        );
        let caps = evo.call("capabilities", json!({}))?;
        ensure!(
            caps["drive_validity_max_us"] == 200_000,
            "Evo does not enforce command validity"
        );
        for name in config.scene.travel_joints.keys() {
            ensure!(
                caps["joint_limits"].get(name).is_some(),
                "unknown travel joint: {name}"
            );
        }
        Ok(Self {
            config,
            evo,
            scenes: Arc::default(),
            limits: caps["joint_limits"].clone(),
            running: Mutex::new(None),
            captures: Mutex::new(VecDeque::new()),
            sequence: std::sync::atomic::AtomicU64::new(0),
        })
    }
    fn id(&self, prefix: &str) -> String {
        format!(
            "rax-{prefix}-{}",
            self.sequence
                .fetch_add(1, std::sync::atomic::Ordering::Relaxed)
        )
    }
    fn stationary(&self) -> Result<Value> {
        let state = self.evo.call("status", json!({}))?;
        ensure!(
            state["feedback_fresh"] == true
                && state["motion_blocked"] == false
                && state["active_action"].is_null(),
            "robot must be idle with fresh feedback"
        );
        let velocities = state["state"]["velocities"]
            .as_object()
            .context("missing joint rates")?;
        let base = state["state"]["odometry"]["velocity"]
            .as_array()
            .context("missing base rates")?;
        ensure!(
            !velocities.is_empty()
                && base.len() == 3
                && velocities
                    .values()
                    .chain(base.iter())
                    .all(|v| v.as_f64().is_some_and(|v| v.is_finite() && v.abs() < 0.005)),
            "robot is moving"
        );
        Ok(state)
    }
    pub fn capabilities(&self) -> Value {
        json!({"configured":true,"button_detection":true,"settings":self.config.scene,
            "heading_source":"lidar_fixed_reference","doorway_control":true,"physical_qualification":"required"})
    }
    pub fn dispatch(&self, method: &str, params: Value) -> Result<Value> {
        match method {
            "capabilities" => Ok(self.capabilities()),
            "scene" => Ok(serde_json::to_value(
                &*self.scenes.read(&self.config.scene)?,
            )?),
            "preview" | "begin" => {
                let goal: BaseGoal = serde_json::from_value(params)?;
                let snapshot = self.stationary()?;
                base::posture(&snapshot, &self.config.scene, &self.limits)?;
                let scene = self.scenes.read(&self.config.scene)?;
                let control = Controller::new(goal, &scene, &self.config.scene)?;
                if method == "preview" {
                    return Ok(json!({"valid":true,"duration_s":90}));
                }
                let mut active = self.running.lock().unwrap();
                ensure!(active.is_none(), "a RAX base goal is already active");
                let id = self.id("goal");
                *active = Some(Running {
                    id: id.clone(),
                    control,
                    started: Instant::now(),
                });
                Ok(json!({"goal_id":id}))
            }
            "step" | "abort" => {
                #[derive(Deserialize)]
                #[serde(deny_unknown_fields)]
                struct Step {
                    goal_id: String,
                    #[serde(default)]
                    after_us: u64,
                }
                let p: Step = serde_json::from_value(params)?;
                let mut active = self.running.lock().unwrap();
                let run = active.as_mut().context("no active base goal")?;
                ensure!(p.goal_id == run.id, "base goal changed");
                if method == "abort" {
                    *active = None;
                    return Ok(json!({"aborted":true}));
                }
                ensure!(
                    run.started.elapsed().as_secs_f64() < 90.,
                    "base goal timeout"
                );
                let snapshot = self.stationary()?;
                base::posture(&snapshot, &self.config.scene, &self.limits)?;
                let scene = self.scenes.read(&self.config.scene)?;
                // Each pulse ends at a measured stop. Wait for subsequent lidar evidence.
                if scene.capture_us <= p.after_us {
                    return Ok(json!({"waiting":true}));
                }
                match run.control.step(
                    &scene,
                    &self.config.scene,
                    run.started.elapsed().as_secs_f64(),
                )? {
                    None => {
                        run.control.final_check(&scene, &self.config.scene)?;
                        let result = run.control.report(&scene, &self.config.scene);
                        *active = None;
                        Ok(json!({"done":true,"base_result":result}))
                    }
                    Some(velocity) => {
                        let cfg = &self.config.scene;
                        // Include the entire command validity and braking at the proposed rate.
                        let speed = velocity[0].hypot(velocity[1]);
                        let travel_s = 0.2 + crate::TICK_S + speed / (2. * cfg.acceleration_mps2);
                        clear_path(
                            &scene,
                            cfg,
                            [
                                velocity[0] * travel_s,
                                velocity[1] * travel_s,
                                velocity[2]
                                    * (0.2
                                        + crate::TICK_S
                                        + velocity[2].abs()
                                            / (2. * cfg.angular_acceleration_radps2)),
                            ],
                        )?;
                        let now = unix_us();
                        let until =
                            (now + 150_000).min(scene.capture_us + (cfg.max_age_s * 1e6) as u64);
                        ensure!(
                            until > now + 80_000,
                            "scene too old for another drive pulse"
                        );
                        Ok(json!({"velocity":velocity,"duration_s":0.06,"valid_until_us":until}))
                    }
                }
            }
            "detect" => {
                #[derive(Deserialize)]
                #[serde(deny_unknown_fields)]
                struct Detect {
                    camera: String,
                    score_threshold: f32,
                }
                let p: Detect = serde_json::from_value(params)?;
                ensure!(
                    self.running.lock().unwrap().is_none(),
                    "base goal is active"
                );
                let before = self.stationary()?;
                let mut frame = tokio::runtime::Handle::current().block_on(buttons::detect(
                    &self.config.button_service,
                    &p.camera,
                    p.score_threshold,
                    self.config.ros_domain_id,
                ))?;
                let after = self.stationary()?;
                ensure!(
                    before["last_action"] == after["last_action"],
                    "robot acted during detection"
                );
                let geometry = self.evo.call("camera_geometry", json!({"camera":p.camera,
                    "capture_us":frame["capture_us"],"width":frame["width"],"height":frame["height"]}))?;
                frame
                    .as_object_mut()
                    .unwrap()
                    .extend(geometry.as_object().context("invalid geometry")?.clone());
                let stamp = frame["capture_us"]
                    .as_u64()
                    .context("missing capture timestamp")?;
                let scene = self.scenes.at(stamp, self.config.scene.max_skew_s)?;
                let id = self.id("capture");
                let result = json!({"observation_id":id,"cameras":{&p.camera:&frame},
                    "scene":&*scene,"detections":frame["detections"],"activation":"unknown","errors":{}});
                frame.as_object_mut().unwrap().remove("jpeg");
                let mut captures = self.captures.lock().unwrap();
                captures.push_back(Capture {
                    id,
                    camera: p.camera,
                    frame,
                    scene,
                });
                while captures.len() > 32 {
                    captures.pop_front();
                }
                Ok(result)
            }
            "ray" => {
                #[derive(Deserialize)]
                #[serde(deny_unknown_fields)]
                struct Ray {
                    observation_id: String,
                    camera: String,
                    pixel: [f64; 2],
                }
                let p: Ray = serde_json::from_value(params)?;
                let captures = self.captures.lock().unwrap();
                let c = captures
                    .iter()
                    .find(|c| c.id == p.observation_id && c.camera == p.camera)
                    .context("unknown or expired RAX capture")?;
                let mut ray = camera::ray(&c.frame, p.pixel)?;
                ray["surface"] = surface_from_ray(
                    &c.scene,
                    serde_json::from_value(ray["origin"].clone())?,
                    serde_json::from_value(ray["direction"].clone())?,
                )?;
                ray["depth_available"] = json!(true);
                Ok(ray)
            }
            _ => bail!("unknown RAX runtime operation"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{
        io::{BufRead, BufReader, Write},
        os::unix::net::UnixListener,
    };

    #[test]
    fn pulse_uses_fresh_scene_and_waits_for_measured_stop_evidence() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("evo.sock");
        let listener = UnixListener::bind(&path).unwrap();
        let cfg = crate::scene::tests::config();
        let joints = cfg.travel_joints.clone();
        let server = std::thread::spawn(move || {
            for stream in listener.incoming() {
                let mut stream = stream.unwrap();
                let mut line = String::new();
                BufReader::new(&stream).read_line(&mut line).unwrap();
                if line.is_empty() {
                    break;
                }
                let request: Value = serde_json::from_str(&line).unwrap();
                let result = match request["method"].as_str().unwrap() {
                    "capabilities" => {
                        let limits: serde_json::Map<String, Value> = joints
                            .keys()
                            .map(|k| (k.clone(), json!({"tolerance":0.01})))
                            .collect();
                        json!({"drive_validity_max_us":200_000,"joint_limits":limits})
                    }
                    "status" => {
                        json!({"feedback_fresh":true,"motion_blocked":false,"active_action":null,
                        "state":{"positions":joints,"velocities":joints,"odometry":{"velocity":[0,0,0]}}})
                    }
                    method => panic!("unexpected operation: {method}"),
                };
                writeln!(
                    stream,
                    "{}",
                    json!({"version":1,"id":"rax-perception","ok":true,"result":result})
                )
                .unwrap();
            }
        });
        let evo = Arc::new(Evo {
            socket: path.clone(),
        });
        assert!(
            evo.call("drive", json!({}))
                .unwrap_err()
                .to_string()
                .contains("read-only")
        );
        let worker = Worker::new(
            Config {
                scene: cfg,
                button_service: "/button_recognition/detect".into(),
                ros_domain_id: 0,
            },
            evo,
        )
        .unwrap();
        let scene = crate::scene::tests::scene();
        let stamp = scene.capture_us;
        worker.scenes.set(scene);
        let begin = worker
            .dispatch("begin", json!({"delta":[0.1,0.,0.]}))
            .unwrap();
        let id = begin["goal_id"].clone();
        assert!(
            worker
                .dispatch("abort", json!({"goal_id":"other"}))
                .is_err()
        );
        assert_eq!(
            worker
                .dispatch("step", json!({"goal_id":id,"after_us":stamp}))
                .unwrap()["waiting"],
            true
        );
        let pulse = worker.dispatch("step", json!({"goal_id":id})).unwrap();
        assert_eq!(pulse["duration_s"], 0.06);
        let deadline = pulse["valid_until_us"].as_u64().unwrap();
        assert!(deadline <= stamp + 300_000 && deadline <= unix_us() + 150_000);
        assert!(pulse["velocity"][0].as_f64().unwrap() > 0.);
        worker.scenes.invalidate();
        assert!(worker.dispatch("step", json!({"goal_id":id})).is_err());
        assert_eq!(
            worker.dispatch("abort", json!({"goal_id":id})).unwrap()["aborted"],
            true
        );
        drop(std::os::unix::net::UnixStream::connect(path).unwrap());
        server.join().unwrap();
    }
}
