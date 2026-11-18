//! `cargo xtask dist`: the release `stela` with its matching `stela.efi`
//! embedded. The boot image builds first, from the pinned sources `gpu
//! fetch` checks out; `stela-cli`'s build script then embeds the file
//! `STELA_EFI` names. The embedded image carries no debug sections: setup
//! writes it beside the old one on the USB's 200 MiB EFI partition. With
//! `--radv` the image also carries amdgpu and RADV, and `stela` the AMD
//! port's firmware, for a PC with a supported Radeon GPU.
use crate::tools::{cargo, exec, root};
use anyhow::Result;

pub fn run(radv: bool) -> Result<()> {
    crate::boot::vram_cap(
        std::env::var_os(crate::boot::VRAM_CAP_VARIABLE).is_some(),
        true,
    )?;
    crate::preflight::run()?;
    crate::gpu::fetch::fetch(false)?;
    let built = if radv {
        crate::boot::build_radv(true)?
    } else {
        crate::boot::build(true)?
    };
    let efi = built.with_file_name("stela-dist.efi");
    crate::boot::deploy_image(&built, &efi)?;
    let mut command = cargo();
    command
        .args(["build", "--release", "--locked", "-p", "stela-cli"])
        .env("STELA_EFI", &efi);
    if radv {
        // The sticks it makes carry the AMD port's firmware too.
        command.args(["--features", "amdgpu-port"]);
    }
    crate::clang::toolset()?.apply(&mut command);
    exec(&mut command)?;
    let release = root().join("target/release");
    println!("{}", release.join("stela").display());
    println!("{}", efi.display());
    Ok(())
}
