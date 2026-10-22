use anyhow::{Context, Result, ensure};
use rax_elevator_runtime::{
    rpc::Evo,
    scene_input,
    worker::{Config, Worker},
};
use serde::Deserialize;
use serde_json::{Value, json};
use std::{
    os::unix::fs::{MetadataExt, PermissionsExt},
    path::PathBuf,
    sync::Arc,
};
use tokio::{
    io::{AsyncBufReadExt, AsyncReadExt, AsyncWriteExt, BufReader},
    net::UnixListener,
};
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Request {
    method: String,
    #[serde(default)]
    params: Value,
}
#[tokio::main]
async fn main() -> Result<()> {
    rax_elevator_runtime::transport::validate_environment(
        std::env::var_os("ZENOH_SESSION_CONFIG_URI"),
        std::env::var_os("ZENOH_CONFIG_OVERRIDE"),
    )?;
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    ensure!(
        args.len() == 3,
        "usage: rax-elevator-runtime CONFIG EVO_SOCKET RAX_SOCKET"
    );
    let config = PathBuf::from(&args[0]);
    ensure!(
        std::fs::metadata(&config)?.len() <= 64 * 1024,
        "config too large"
    );
    let config: Config = serde_json::from_slice(&std::fs::read(config)?)?;
    let evo = Arc::new(Evo {
        socket: PathBuf::from(&args[1]),
    });
    let worker = Arc::new(Worker::new(config, evo.clone())?);
    let path = PathBuf::from(&args[2]);
    let metadata = std::fs::symlink_metadata(path.parent().context("socket needs a directory")?)?;
    ensure!(
        path.is_absolute()
            && metadata.is_dir()
            && metadata.uid() == unsafe { libc::geteuid() }
            && metadata.mode() & 0o077 == 0,
        "runtime socket requires a private owned directory"
    );
    let listener = UnixListener::bind(&path)?;
    std::fs::set_permissions(&path, std::fs::Permissions::from_mode(0o600))?;
    let (_stop, rx) = tokio::sync::watch::channel(false);
    let mut sensors = tokio::spawn(scene_input::run(
        worker.config.scene.clone(),
        evo,
        worker.config.ros_domain_id,
        worker.scenes.clone(),
        rx,
    ));
    let slots = Arc::new(tokio::sync::Semaphore::new(4));
    loop {
        tokio::select! {
            result = &mut sensors => { worker.scenes.invalidate(); result??; anyhow::bail!("sensor worker stopped"); },
            connection = listener.accept() => {
                let (mut socket, _) = connection?;
                ensure!(socket.peer_cred()?.uid() == unsafe {libc::geteuid()}, "socket peer UID differs");
                let slot = slots.clone().acquire_owned().await?;
                let worker = worker.clone();
                tokio::spawn(async move {
                    let _slot = slot;
                    let result = async {
                        let mut bytes = Vec::new();
                        tokio::time::timeout(std::time::Duration::from_secs(1),
                            BufReader::new(&mut socket).take(65537).read_until(b'\n', &mut bytes)).await??;
                        ensure!(bytes.len() <= 65536 && bytes.last() == Some(&b'\n'), "invalid request size");
                        let request: Request = serde_json::from_slice(&bytes)?;
                        tokio::task::spawn_blocking(move || worker.dispatch(&request.method, request.params)).await?
                    }.await;
                    let response = match result {
                        Ok(value) => json!({"ok":true,"result":value}),
                        Err(error) => json!({"ok":false,"error":format!("{error:#}")}),
                    };
                    if let Ok(mut bytes) = serde_json::to_vec(&response) {
                        bytes.push(b'\n');
                        let _ = tokio::time::timeout(std::time::Duration::from_secs(1), socket.write_all(&bytes)).await;
                    }
                });
            }
        }
    }
}
