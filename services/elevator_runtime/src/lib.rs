//! Trusted RAX elevator perception and feedback. This process never commands actuators.
pub use hiroz_msgs::*;
include!(concat!(env!("OUT_DIR"), "/generated.rs"));
pub mod base;
pub mod buttons;
pub mod camera;
pub mod rpc;
pub mod scene;
pub mod scene_input;
pub mod worker;
pub const TICK_S: f64 = 0.02;
pub fn unix_us() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_micros() as u64
}
pub fn box_err(e: impl std::fmt::Display) -> anyhow::Error {
    anyhow::anyhow!("{e}")
}

pub mod transport;
