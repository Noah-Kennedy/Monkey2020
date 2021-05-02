fn main() {
    // invoke cmake and ninja
    let mut cfg = cmake::Config::new("../cpp");

    let dst = if cfg!(target_os = "windows") {
        &mut cfg
    } else {
        cfg.generator("Ninja")
    }.build().join("lib");

    // only see this if run with -vv
    println!("Path is {}\n", dst.to_str().unwrap());

    // add directory where cmake dumps our libraries to the linker search path
    println!("cargo:rustc-link-search=native={}", dst.display());

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

    // statically link library built with cmake
    println!("cargo:rustc-link-lib=static=monkey-vision");

    // Dynamically link needed OpenCV libraries
    // OpenCV is modular; we do not need to link in everything, doing so may nuke our
    // compilation times
    println!("cargo:rustc-link-lib=dylib=opencv_core");
    println!("cargo:rustc-link-lib=dylib=opencv_imgproc");
    println!("cargo:rustc-link-lib=dylib=opencv_imgcodecs");
    println!("cargo:rustc-link-lib=dylib=opencv_videoio");

    // Dynamically link C++ standard library
    println!("cargo:rustc-link-lib=dylib=stdc++");
}