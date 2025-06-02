use cmake::Config;
use std::{env, path::PathBuf};

fn main() {
    println!("cargo:rerun-if-changed=wrapper.h");
    println!("cargo:rerun-if-changed=build.rs");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());

    build_from_source(&out_path);

    // Link the libraries
    println!("cargo:rustc-link-lib=small_gicp_c");
    println!("cargo:rustc-link-lib=small_gicp");
    println!("cargo:rustc-link-lib=stdc++");

    // Generate bindings
    let include_dir = env::var("SMALL_GICP_INCLUDE_DIR").unwrap_or_else(|_| {
        // Default to local install if available
        let local = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap())
            .parent()
            .unwrap()
            .join("small_gicp_c")
            .join("install")
            .join("include");

        if local.exists() {
            local.to_string_lossy().into_owned()
        } else {
            "/usr/local/include".to_string()
        }
    });

    let bindings = bindgen::Builder::default()
        .header("wrapper.h")
        .clang_arg(format!("-I{}", include_dir))
        .parse_callbacks(Box::new(bindgen::CargoCallbacks::new()))
        // Allowlist the functions and types we want to expose
        .allowlist_function("small_gicp_.*")
        .allowlist_type("small_gicp_.*")
        .allowlist_var("SMALL_GICP_.*")
        // Generate rust enums for C enums
        .rustified_enum("small_gicp_error_t")
        .rustified_enum("small_gicp_registration_type_t")
        .generate()
        .expect("Unable to generate bindings");

    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}

fn build_from_source(out_path: &PathBuf) {
    let small_gicp_c_dir = PathBuf::from(env::var("CARGO_MANIFEST_DIR").unwrap())
        .parent()
        .unwrap()
        .join("small_gicp_c");

    // Build small_gicp_c using cmake
    let dst = Config::new(&small_gicp_c_dir)
        .define("BUILD_SHARED_LIBS", "ON")
        .define("BUILD_EXAMPLES", "OFF")
        .define("CMAKE_INSTALL_PREFIX", out_path.to_str().unwrap())
        .build();

    println!("cargo:rustc-link-search=native={}/lib", dst.display());
    println!("cargo:include={}/include", dst.display());

    // Set environment variable for bindgen
    env::set_var("SMALL_GICP_INCLUDE_DIR", dst.join("include"));
}
