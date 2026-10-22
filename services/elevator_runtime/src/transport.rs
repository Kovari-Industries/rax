//! Local robot transport. No discovery, remote endpoint, or environment override.
use anyhow::{Result, ensure};
use hiroz::context::ZContextBuilder;

pub fn validate_environment(
    config: Option<std::ffi::OsString>,
    overrides: Option<std::ffi::OsString>,
) -> Result<()> {
    ensure!(
        config.is_none() && overrides.is_none(),
        "elevator transport does not accept ZENOH_SESSION_CONFIG_URI or ZENOH_CONFIG_OVERRIDE"
    );
    Ok(())
}

pub fn local(domain: u32) -> Result<ZContextBuilder> {
    validate_environment(
        std::env::var_os("ZENOH_SESSION_CONFIG_URI"),
        std::env::var_os("ZENOH_CONFIG_OVERRIDE"),
    )?;
    Ok(ZContextBuilder::default()
        .with_domain_id(domain as usize)
        .with_mode("client")
        .with_connect_endpoints(["tcp/127.0.0.1:7447"])
        .with_listen_endpoints([] as [&str; 0])
        .disable_multicast_scouting()
        .with_json("scouting/gossip/enabled", false))
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn inherited_transport_settings_cannot_redirect_local_clients() {
        assert!(validate_environment(None, None).is_ok());
        assert!(validate_environment(Some("remote.json5".into()), None).is_err());
        assert!(validate_environment(None, Some("connect/endpoints=[]".into())).is_err());
        assert!(validate_environment(None, Some("".into())).is_err());
    }
}
