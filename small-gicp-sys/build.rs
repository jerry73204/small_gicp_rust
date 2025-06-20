fn main() {
    let mut build = cxx_build::bridge("src/ffi.rs");

    // Add include directories
    build
        .include("include")
        .include("../small_gicp/include") // small_gicp headers
        .file("include/wrapper.cpp") // Add the implementation file
        .file("../small_gicp/src/small_gicp/registration/registration_helper.cpp") // Add registration helper
        .flag_if_supported("-std=c++17")
        .flag_if_supported("-fopenmp")
        .flag_if_supported("-Wno-unused-parameter")
        .flag_if_supported("-Wno-sign-compare");

    // Add Eigen include path
    if let Ok(output) = std::process::Command::new("pkg-config")
        .args(["--cflags", "eigen3"])
        .output()
    {
        if output.status.success() {
            let flags = String::from_utf8_lossy(&output.stdout);
            for flag in flags.split_whitespace() {
                if let Some(include_path) = flag.strip_prefix("-I") {
                    build.include(include_path);
                }
            }
        }
    } else {
        // Fallback to common Eigen locations
        build.include("/usr/include/eigen3");
    }

    // Platform-specific flags
    if cfg!(target_os = "macos") {
        build.flag("-Xpreprocessor");
    }

    // Compile the bridge
    build.compile("small_gicp_sys");

    // Link OpenMP
    if cfg!(target_os = "linux") {
        println!("cargo:rustc-link-lib=gomp");
    } else if cfg!(target_os = "macos") {
        println!("cargo:rustc-link-lib=omp");
    }

    // Rebuild if headers change
    println!("cargo:rerun-if-changed=src/ffi.rs");
    println!("cargo:rerun-if-changed=include/wrapper.h");
    println!("cargo:rerun-if-changed=include/wrapper.cpp");
}
