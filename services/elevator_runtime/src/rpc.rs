//! Read-only access to Evo state and capture geometry. No actuator methods here.
use anyhow::{Context, Result, ensure};
use serde_json::{Value, json};
use std::{
    io::{BufRead, BufReader, Write},
    os::unix::net::UnixStream,
    path::PathBuf,
    time::Duration,
};
pub struct Evo {
    pub socket: PathBuf,
}
impl Evo {
    pub fn call(&self, method: &str, params: Value) -> Result<Value> {
        ensure!(
            ["status", "history", "fk", "capabilities", "camera_geometry"].contains(&method),
            "read-only Evo interface"
        );
        let mut socket = UnixStream::connect(&self.socket)?;
        socket.set_read_timeout(Some(Duration::from_millis(200)))?;
        socket.set_write_timeout(Some(Duration::from_millis(200)))?;
        let request = json!({"version":1,"id":"rax-perception","method":method,"params":params});
        let mut bytes = serde_json::to_vec(&request)?;
        ensure!(bytes.len() < 64 * 1024, "Evo request too large");
        bytes.push(b'\n');
        socket.write_all(&bytes)?;
        let mut response = Vec::new();
        use std::io::Read;
        BufReader::new(socket)
            .take(40 * 1024 * 1024 + 1)
            .read_until(b'\n', &mut response)?;
        ensure!(
            response.len() <= 40 * 1024 * 1024 && response.last() == Some(&b'\n'),
            "invalid Evo response size"
        );
        let reply: Value = serde_json::from_slice(&response)?;
        ensure!(
            reply["version"] == 1 && reply["id"] == "rax-perception" && reply["ok"] == true,
            "Evo read refused: {}",
            reply["error"]
        );
        Ok(reply["result"].clone())
    }
    pub fn transform(
        &self,
        parent: &str,
        joints: Value,
        t: [f64; 3],
        q: [f64; 4],
    ) -> Result<Value> {
        let fk = self.call("fk", json!({"joints":joints,"links":[parent]}))?;
        let pose = fk["poses"]
            .get(parent)
            .context("sensor parent transform unavailable")?;
        let origin: [f64; 3] = serde_json::from_value(pose["position"].clone())?;
        let r: [f64; 4] = serde_json::from_value(pose["quaternion_xyzw"].clone())?;
        let position = crate::scene_input::transform(t, r, origin);
        let orientation = [
            r[3] * q[0] + r[0] * q[3] + r[1] * q[2] - r[2] * q[1],
            r[3] * q[1] - r[0] * q[2] + r[1] * q[3] + r[2] * q[0],
            r[3] * q[2] + r[0] * q[1] - r[1] * q[0] + r[2] * q[3],
            r[3] * q[3] - r[0] * q[0] - r[1] * q[1] - r[2] * q[2],
        ];
        Ok(json!({"position":position,"quaternion_xyzw":orientation}))
    }
}
