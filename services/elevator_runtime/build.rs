use std::{collections::HashSet, path::PathBuf};
fn main() -> anyhow::Result<()> {
    let source =
        PathBuf::from(std::env::var("CARGO_MANIFEST_DIR")?).join("interfaces/evo_interfaces");
    println!("cargo:rerun-if-changed={}", source.display());
    let config = hiroz_codegen::GeneratorConfig {
        generate_cdr: true,
        generate_protobuf: false,
        generate_type_info: true,
        is_humble: false,
        output_dir: std::env::var("OUT_DIR")?.into(),
        external_crate: Some("hiroz_msgs".into()),
        local_packages: HashSet::from(["evo_interfaces".into()]),
        json_out: None,
    };
    let dependencies = hiroz_codegen::discover_bundled_packages(false)?;
    hiroz_codegen::MessageGenerator::new(config).generate_from_msg_files_with_deps(
        &[source.as_path()],
        &dependencies.iter().map(|p| p.as_path()).collect::<Vec<_>>(),
    )?;
    Ok(())
}
