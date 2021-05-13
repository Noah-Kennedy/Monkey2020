use actix_web::{App, HttpServer, web};
use actix_web::middleware::Logger;
use actix_web::web::Bytes;
use env_logger::WriteStyle;
use log::LevelFilter;

use cameralot::prelude::*;
use monkey_api::MotorSpeeds;
use monkey_api::requests::AutonomousParams;
use monkey_unified_command::{camera, nav};
use monkey_unified_command::camera::CameraManager;
use monkey_unified_command::nav::NavManager;
use monkey_vision::prelude::*;
use space_monkeys::ZhuLi;
use std::time::Instant;

const FILE_FORMAT: &str = ".jpg";
const WIDTH: u32 = 1280;
const HEIGHT: u32 = 720;

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    let mesh_file = "mesh.ply";

    env_logger::builder()
        .filter_level(LevelFilter::Info)
        .default_format()
        .write_style(WriteStyle::Always)
        .init();

    log::info!("Initializing camera");

    let mut thot = OpenCVCameraFeed::new();
    assert!(thot.open(0), "Failed to open camera, exiting!");

    log::info!("Initialized the streams");

    let (cam_tx, cam_rx) = tokio::sync::watch::channel(
        actix_web::web::Bytes::from(vec![0])
    );

    let camera_man = CameraManager {
        feeds: vec![cam_rx]
    };

    let server = HttpServer::new(move || {
        App::new()
            .wrap(Logger::default())
            .service(web::scope("cameras")
                .data(camera_man.clone())
                .service(camera::static_image)
                .service(camera::ws_camera)
            )
    })
        .bind(("0.0.0.0", 8080))?
        .run();

    let mut td = TimerData {
        grab_millis: 0,
        retrieve_millis: 0,
        resize_millis: 0,
        encode_millis: 0,
    };

    let mut i = 0;

    loop {
        let timer = std::time::Instant::now();

        let buf = unsafe {
            thot.read(WIDTH, HEIGHT, FILE_FORMAT, &mut td).unwrap()
        };

        let time = timer.elapsed().as_millis();

        log::trace!("{}:\t{} millis", i, time);
        log::trace!("\tGrab: {} millis", td.grab_millis);
        log::trace!("\tRetrieve: {} millis", td.retrieve_millis);
        log::trace!("\tResize: {} millis", td.resize_millis);
        log::trace!("\tEncode: {} millis", td.encode_millis);

        cam_tx.send(Bytes::from(buf.to_vec())).unwrap();

        i += 1;
    }

    Ok(())
}