use std::{env, fs::File, io::Write, path::PathBuf};

fn main() {
    // put in .cargo/config.toml
    //println!("cargo:rustc-link-arg-bins=--nmagic");
    //println!("cargo:rustc-link-arg-bins=-Tlink.x");
    //println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    // Put `memory.x` in our output directory and ensure it's
    // on the linker search path.
    let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());
    File::create(out.join("memory.x"))
        .unwrap()
        .write_all(include_bytes!("memory_stm32f469ni.x")) // TODO: Change this based on features or similar
        .unwrap();
    println!("cargo:rustc-link-search={}", out.display());

    // By default, Cargo will re-run a build script whenever
    // any file in the project changes. By specifying `memory.x`
    // here, we ensure the build script is only re-run when
    // `memory.x` is changed.
    println!("cargo:rerun-if-changed=memory_stm32f469ni.x");

    let config = slint_build::CompilerConfiguration::new()
        .embed_resources(slint_build::EmbedResourcesKind::EmbedForSoftwareRenderer)
        .with_style("native".into());
    //slint_build::compile_with_config("ui/display_mock.slint", config).unwrap();
    slint_build::compile_with_config("window.slint", config).unwrap();
    slint_build::print_rustc_flags().unwrap();

    println!("cargo:EMBED_TEXTURES=1");
}
