//! Broker the existing detector and preserve its exact camera capture.
use crate::ros::evo_interfaces::{DetectButtonsRequest, srv::DetectButtons};
use anyhow::{Result, ensure};
use base64::{Engine as _, engine::general_purpose::STANDARD};
use hiroz::Builder;
use serde_json::{Value, json};

pub async fn detect(service: &str, camera: &str, score: f32, domain: u32) -> Result<Value> {
    ensure!(
        ["base", "left_arm", "right_arm"].contains(&camera)
            && score.is_finite()
            && (0.0..=1.0).contains(&score),
        "invalid button detection request"
    );
    let started = crate::unix_us();
    let ctx = crate::transport::local(domain)?
        .build()
        .map_err(crate::box_err)?;
    let node = ctx
        .create_node("agent_button_query")
        .build()
        .map_err(crate::box_err)?;
    let client = node
        .create_client::<DetectButtons>(service)
        .build()
        .map_err(crate::box_err)?;
    let request = DetectButtonsRequest {
        source: camera.into(),
        score_threshold: score,
        timeout_s: 3.,
    };
    let reply = client
        .call_with_timeout(&request, std::time::Duration::from_secs(4))
        .await
        .map_err(crate::box_err)?;
    ensure!(reply.success, "button detector refused: {}", reply.message);
    let d = reply.detections;
    ensure!(
        d.header.stamp.sec >= 0 && d.header.stamp.nanosec < 1_000_000_000,
        "invalid detector capture timestamp"
    );
    let stamp = d.header.stamp.sec as u64 * 1_000_000 + d.header.stamp.nanosec as u64 / 1000;
    ensure!(
        d.source == camera
            && stamp > started
            && crate::unix_us()
                .checked_sub(stamp)
                .is_some_and(|age| age <= 1_000_000)
            && d.source_generation > 0
            && d.capture_monotonic_ns > 0,
        "detector did not return a fresh identified capture"
    );
    let jpeg_len: usize = d.source_jpeg.zslices().map(|s| s.as_slice().len()).sum();
    ensure!(
        jpeg_len > 0
            && jpeg_len <= 8 * 1024 * 1024
            && d.image_width > 0
            && d.image_height > 0
            && d.detections.len() <= 256,
        "invalid detector image or result size"
    );
    ensure!(
        d.detections.iter().all(|b| b.score.is_finite()
            && (0.0..=1.0).contains(&b.score)
            && b.x_min >= 0
            && b.y_min >= 0
            && b.x_max > b.x_min
            && b.y_max > b.y_min
            && b.x_max as u32 <= d.image_width
            && b.y_max as u32 <= d.image_height),
        "invalid detector box or score"
    );
    let detections:Vec<_>=d.detections.iter().map(|b|json!({"text":b.text,"text_readable":b.text_readable,"score":b.score,"box":[b.x_min,b.y_min,b.x_max,b.y_max]})).collect();
    Ok(
        json!({"camera":camera,"capture_us":stamp,"width":d.image_width,"height":d.image_height,"source_generation":d.source_generation,"capture_monotonic_ns":d.capture_monotonic_ns,"jpeg":STANDARD.encode(d.source_jpeg.to_zslice().as_slice()),"detections":detections}),
    )
}
