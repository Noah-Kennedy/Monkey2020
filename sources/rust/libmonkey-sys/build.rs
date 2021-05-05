fn main() {
    invoke_cmake();
    link_static_libs();
    link_dynamic_libs()
}

fn invoke_cmake() {
    let mut cfg = cmake::Config::new("../../cpp");

    let dst = if cfg!(target_os = "windows") {
        &mut cfg
    } else {
        cfg.generator("Ninja")
    }.build().join("lib");

    // only see this if run with -vv
    println!("Path is {}\n", dst.to_str().unwrap());

    // add directory where cmake dumps our libraries to the linker search path
    println!("cargo:rustc-link-search=native={}", dst.display());
}

//*********************************************************************************************
// # Linking
//
// Note that we are putting things in in the REVERSE ORDER of our directed graph.
// For example, cameralot-capture comes before its OpenCV dependencies, which in turn come
// before their dependency on stdc++ (the C++ standard library).
//
// That last lib is crucial: a program like g++ implicitly assumes that it needs to link in
// the C++ standard library. rustc might implicitly link in glibc, but it does not know that we
// have a C++ dependency.
//*********************************************************************************************

fn link_static_libs() {
    if cfg!(feature = "monkey-vision") {
        println!("cargo:rustc-link-lib=static=monkey-vision");
    }

    if cfg!(feature = "cameralot") {
        println!("cargo:rustc-link-lib=static=cameralot");
    }
}

fn link_dynamic_libs() {
    if cfg!(feature = "link_zed") {
        println!("cargo:rustc-link-lib=dylib=zed");
    }

    if cfg!(feature = "link_opencv_aruco") {
        println!("cargo:rustc-link-lib=dylib=opencv_aruco");
    }

    if cfg!(feature = "link_opencv_core") {
        println!("cargo:rustc-link-lib=dylib=opencv_core");
    }

    if cfg!(feature = "link_opencv_imgproc") {
        println!("cargo:rustc-link-lib=dylib=opencv_imgproc");
    }

    if cfg!(feature = "link_opencv_imgcodecs") {
        println!("cargo:rustc-link-lib=dylib=opencv_imgcodecs");
    }

    if cfg!(feature = "link_opencv_videoio") {
        println!("cargo:rustc-link-lib=dylib=opencv_videoio");
    }

    // make sure this is last!
    if cfg!(feature = "link_stdcpp") {
        println!("cargo:rustc-link-lib=dylib=stdc++");
    }
}