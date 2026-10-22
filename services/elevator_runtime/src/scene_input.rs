//! Existing PointCloud2 transports feed the typed scene store; no motion publisher.
use crate::{rpc::Evo, scene::*, unix_us};
use anyhow::{Context, Result, ensure};
use hiroz::{
    Builder,
    qos::{QosProfile, QosReliability},
};
use hiroz_msgs::sensor_msgs::PointCloud2;
use serde_json::json;
use std::{collections::BTreeMap, sync::Arc};

pub fn decode(cloud: &PointCloud2) -> Result<Vec<V3>> {
    let count = cloud.width as usize * cloud.height as usize;
    let data_len: usize = cloud.data.zslices().map(|s| s.as_slice().len()).sum();
    ensure!(
        count > 0 && count <= 4_000_000 && data_len <= 64 * 1024 * 1024 && cloud.point_step >= 12,
        "invalid point-cloud dimensions"
    );
    ensure!(
        cloud.row_step as usize >= cloud.width as usize * cloud.point_step as usize
            && data_len >= cloud.row_step as usize * cloud.height as usize,
        "truncated point cloud"
    );
    let fields: Vec<_> = ["x", "y", "z"]
        .iter()
        .map(|name| {
            let f = cloud
                .fields
                .iter()
                .find(|f| f.name == *name)
                .context("cloud lacks XYZ field")?;
            ensure!(
                f.count == 1
                    && (f.datatype == 7 || f.datatype == 8)
                    && f.offset
                        .checked_add(if f.datatype == 7 { 4 } else { 8 })
                        .is_some_and(|end| end <= cloud.point_step),
                "invalid XYZ field layout"
            );
            Ok((f.offset as usize, f.datatype))
        })
        .collect::<Result<_>>()?;
    let data = cloud.data.to_zslice();
    let data = data.as_slice();
    let mut points = Vec::new();
    for i in (0..count).step_by(count.div_ceil(30000)) {
        let offset = (i / cloud.width as usize) * cloud.row_step as usize
            + (i % cloud.width as usize) * cloud.point_step as usize;
        let mut p = [0.; 3];
        for (axis, (field, kind)) in fields.iter().enumerate() {
            let bytes = &data[offset + field..];
            p[axis] = if *kind == 7 {
                let b = bytes[..4].try_into().unwrap();
                (if cloud.is_bigendian {
                    f32::from_be_bytes(b)
                } else {
                    f32::from_le_bytes(b)
                }) as f64
            } else {
                let b = bytes[..8].try_into().unwrap();
                if cloud.is_bigendian {
                    f64::from_be_bytes(b)
                } else {
                    f64::from_le_bytes(b)
                }
            };
        }
        if p.iter().all(|v| v.is_finite()) && p.iter().map(|v| v * v).sum::<f64>() > 0.0001 {
            points.push(p);
        }
    }
    ensure!(points.len() >= 30, "insufficient valid depth returns");
    Ok(points)
}
pub(crate) fn transform(p: V3, q: [f64; 4], t: V3) -> V3 {
    let v = [
        2. * (q[1] * p[2] - q[2] * p[1]),
        2. * (q[2] * p[0] - q[0] * p[2]),
        2. * (q[0] * p[1] - q[1] * p[0]),
    ];
    [
        p[0] + q[3] * v[0] + q[1] * v[2] - q[2] * v[1] + t[0],
        p[1] + q[3] * v[1] + q[2] * v[0] - q[0] * v[2] + t[1],
        p[2] + q[3] * v[2] + q[0] * v[1] - q[1] * v[0] + t[2],
    ]
}
#[derive(Default)]
struct Worker {
    latest: BTreeMap<usize, (u64, Vec<V3>)>,
    heading: Heading,
    pose: Option<(u64, V3, f64)>,
    doors: DoorTracker,
    last_scene: u64,
}
impl Worker {
    fn accept(
        &mut self,
        index: usize,
        cloud: PointCloud2,
        cfg: &SceneSettings,
        evo: &Evo,
    ) -> Result<Option<Scene>> {
        let source = &cfg.sources[index];
        ensure!(
            cloud.header.frame_id == source.frame,
            "cloud frame differs from calibrated source"
        );
        ensure!(
            cloud.header.stamp.sec >= 0 && cloud.header.stamp.nanosec < 1_000_000_000,
            "invalid cloud timestamp"
        );
        let stamp =
            cloud.header.stamp.sec as u64 * 1_000_000 + cloud.header.stamp.nanosec as u64 / 1000;
        ensure!(
            unix_us()
                .checked_sub(stamp)
                .is_some_and(|age| age as f64 <= cfg.max_age_s * 1e6),
            "cloud source timestamp is stale or in the future"
        );
        ensure!(
            self.latest
                .get(&index)
                .is_none_or(|(previous, _)| stamp > *previous),
            "cloud timestamp did not advance"
        );
        let history = evo.call("history", json!({}))?;
        let measured = history["states"]
            .as_array()
            .context("cloud has no state history")?
            .iter()
            .min_by_key(|s| s["source_us"].as_u64().unwrap_or(0).abs_diff(stamp))
            .filter(|s| s["source_us"].as_u64().unwrap_or(0).abs_diff(stamp) <= 20_000)
            .context("cloud and joint state are not synchronized")?;
        let tf = evo.transform(
            &source.parent_link,
            measured["positions"].clone(),
            source.translation,
            source.quaternion_xyzw,
        )?;
        let t: V3 = serde_json::from_value(tf["position"].clone())?;
        let q: [f64; 4] = serde_json::from_value(tf["quaternion_xyzw"].clone())?;
        ensure!(
            t.iter().chain(&q).all(|v| v.is_finite())
                && (q.iter().map(|v| v * v).sum::<f64>() - 1.).abs() < 1e-3,
            "invalid cloud transform result"
        );
        let points: Vec<_> = decode(&cloud)?
            .into_iter()
            .map(|p| transform(p, q, t))
            .filter(|p| {
                p[0].hypot(p[1]) < 8.
                    && !(p[0].abs() < cfg.footprint[0]
                        && p[1].abs() < cfg.footprint[1]
                        && p[2] >= cfg.floor_z_m
                        && p[2] < cfg.footprint[2])
            })
            .collect();
        if source.heading {
            let (pose, rmse) = self.heading.update(&points, stamp)?;
            self.pose = Some((stamp, pose, rmse));
        }
        self.latest.insert(index, (stamp, points));
        Ok(self.assemble(cfg))
    }

    fn assemble(&mut self, cfg: &SceneSettings) -> Option<Scene> {
        if self.latest.len() != cfg.sources.len() {
            return None;
        }
        let (pose_stamp, pose, rmse) = self.pose?;
        let min = self.latest.values().map(|(s, _)| *s).min().unwrap();
        let max = self.latest.values().map(|(s, _)| *s).max().unwrap();
        // Sensors can publish at different rates. Wait for a matching pair;
        // do not treat a valid unpaired frame as sensor loss. SceneStore still
        // expires the last complete scene at the configured age limit.
        if max - min > (cfg.max_skew_s * 1e6) as u64
            || max.abs_diff(pose_stamp) > (cfg.max_skew_s * 1e6) as u64
            || min <= self.last_scene
        {
            return None;
        }
        let points: Vec<_> = self
            .latest
            .values()
            .flat_map(|(_, p)| p.iter().copied())
            .collect();
        let panel = panel(&points);
        let doorway = self.doors.observe(&points, panel.as_ref(), pose, min, cfg);
        self.last_scene = min;
        Some(Scene {
            observation_id: format!("cloud-{min}"),
            capture_us: min,
            frame: "base_link".into(),
            pose,
            heading_source: "lidar_fixed_reference".into(),
            heading_rmse_m: rmse,
            panel,
            doorway,
            sources: cfg.sources.iter().map(|s| s.topic.clone()).collect(),
            clearance: Clearance::from_points(&points, cfg),
            points,
        })
    }
}
pub async fn run(
    cfg: SceneSettings,
    evo: Arc<Evo>,
    domain: u32,
    store: Arc<SceneStore>,
    mut stop: tokio::sync::watch::Receiver<bool>,
) -> Result<()> {
    cfg.validate()?;
    let context = crate::transport::local(domain)?
        .build()
        .map_err(crate::box_err)?;
    let node = context
        .create_node("agent_scene_geometry")
        .build()
        .map_err(crate::box_err)?;
    let (tx, mut rx) = tokio::sync::mpsc::channel(2);
    let mut producers = tokio::task::JoinSet::new();
    for (index, source) in cfg.sources.iter().enumerate() {
        let sub = node
            .create_sub::<PointCloud2>(&source.topic)
            .with_qos(QosProfile {
                reliability: QosReliability::BestEffort,
                ..Default::default()
            })
            .build()
            .map_err(crate::box_err)?;
        let tx = tx.clone();
        let store = store.clone();
        producers.spawn(async move {
            loop {
                let cloud = sub.async_recv().await.map_err(crate::box_err)?;
                if tx.try_send((index, cloud)).is_err() {
                    store.invalidate();
                    anyhow::bail!("geometry worker overflow");
                }
            }
            #[allow(unreachable_code)]
            Ok::<(), anyhow::Error>(())
        });
    }
    drop(tx);
    let mut worker = Worker::default();
    loop {
        tokio::select! {biased;
            _=stop.changed()=>return Ok(()),
            _=producers.join_next()=>{store.invalidate();anyhow::bail!("geometry input worker stopped");},
            packet=rx.recv()=>{
                let (index,cloud)=packet.context("cloud inputs closed")?;
                let (cfg,evo)=(cfg.clone(),evo.clone());
                let (next,result)=tokio::task::spawn_blocking(move||{let result=worker.accept(index,cloud,&cfg,&evo);(worker,result)}).await?;
                worker=next;
                match result {Ok(Some(scene))=>store.set(scene),Ok(None)=>{},Err(error)=>{worker.doors.reset_confirmation();store.invalidate();eprintln!("scene unavailable: {error}");}}
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use hiroz_msgs::sensor_msgs::PointField;
    fn cloud(big: bool) -> PointCloud2 {
        let mut c = PointCloud2::default();
        c.width = 32;
        c.height = 2;
        c.point_step = 16;
        c.row_step = 32 * 16 + 8;
        c.is_bigendian = big;
        c.fields = ["x", "y", "z"]
            .into_iter()
            .enumerate()
            .map(|(i, n)| PointField {
                name: n.into(),
                offset: (i * 4) as u32,
                datatype: 7,
                count: 1,
            })
            .collect();
        let mut data = vec![0; c.row_step as usize * c.height as usize];
        for i in 0..64 {
            let start = i / 32 * c.row_step as usize + i % 32 * 16;
            for (axis, value) in [1.0_f32, 0.2, 0.3].into_iter().enumerate() {
                let bytes = if big {
                    value.to_be_bytes()
                } else {
                    value.to_le_bytes()
                };
                data[start + axis * 4..start + axis * 4 + 4].copy_from_slice(&bytes);
            }
        }
        c.data = data.into();
        c
    }
    #[test]
    fn reads_padded_rows_and_both_byte_orders() {
        for big in [false, true] {
            let p = decode(&cloud(big)).unwrap();
            assert_eq!(p.len(), 64);
            assert!((p[63][2] - 0.3).abs() < 1e-6);
        }
    }
    #[test]
    fn bad_field_layout_and_truncated_payload_refuse() {
        let mut c = cloud(false);
        let mut data = c.data.to_zslice().as_slice().to_vec();
        data.pop();
        c.data = data.into();
        assert!(decode(&c).is_err());
        let mut c = cloud(false);
        c.fields[0].offset = 15;
        assert!(decode(&c).is_err());
        c.fields[0].offset = u32::MAX;
        assert!(decode(&c).is_err());
        let mut c = cloud(false);
        c.fields[1].datatype = 4;
        assert!(decode(&c).is_err());
    }
    #[test]
    fn different_sensor_rates_wait_for_pairs_without_losing_open_confirmation() {
        let mut cfg = crate::scene::tests::config();
        let mut depth = cfg.sources[0].clone();
        depth.heading = false;
        depth.topic = "/test/depth".into();
        cfg.sources.push(depth);
        let points = crate::scene::tests::aperture(1.2);
        let stamp = unix_us() - 150_000;
        let mut worker = Worker::default();
        worker.latest.insert(0, (stamp, points.clone()));
        worker.latest.insert(1, (stamp, points.clone()));
        worker.pose = Some((stamp, [0.; 3], 0.));
        let first = worker.assemble(&cfg).unwrap();
        assert_eq!(first.doorway.as_ref().unwrap().stable_samples, 1);
        let store = SceneStore::default();
        store.set(first);
        worker.latest.insert(1, (stamp + 66_000, points.clone()));
        assert!(worker.assemble(&cfg).is_none());
        assert_eq!(store.read(&cfg).unwrap().capture_us, stamp);
        worker.latest.insert(0, (stamp + 100_000, points.clone()));
        worker.pose = Some((stamp + 100_000, [0.; 3], 0.));
        let second = worker.assemble(&cfg).unwrap();
        assert_eq!(second.doorway.as_ref().unwrap().stable_samples, 2);
        worker.latest.insert(1, (stamp + 100_000, points));
        let third = worker.assemble(&cfg).unwrap();
        assert_eq!(third.capture_us, stamp + 100_000);
        assert_eq!(third.doorway.as_ref().unwrap().state, "open");
        // Reusing the same pair cannot count as another stable capture.
        assert!(worker.assemble(&cfg).is_none());
    }
    #[test]
    fn optical_transform_uses_rotation_and_translation() {
        let q = [
            0.,
            0.,
            (std::f64::consts::FRAC_PI_4).sin(),
            (std::f64::consts::FRAC_PI_4).cos(),
        ];
        let p = transform([1., 0., 0.], q, [0.1, 0.2, 0.3]);
        assert!(
            (p[0] - 0.1).abs() < 1e-9 && (p[1] - 1.2).abs() < 1e-9 && (p[2] - 0.3).abs() < 1e-9
        );
    }
}
