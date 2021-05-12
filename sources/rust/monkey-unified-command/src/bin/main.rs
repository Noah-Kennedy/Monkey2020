use actix_web::{App, HttpServer, web};
use actix_web::middleware::Logger;
use actix_web::web::Bytes;
use env_logger::WriteStyle;
use log::LevelFilter;

use cameralot::prelude::*;
use monkey_unified_command::{camera, command};
use monkey_unified_command::camera::CameraManager;

const FILE_FORMAT: &str = ".jpg";
const WIDTH: u32 = 1280;
const HEIGHT: u32 = 720;

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    env_logger::builder()
        .filter_level(LevelFilter::Info)
        .default_format()
        .write_style(WriteStyle::Always)
        .init();

    let mut instagram_thot = OpenCVCameraFeed::new();
    instagram_thot.open(0);

    let (tx, rx) = tokio::sync::watch::channel(
        actix_web::web::Bytes::from(vec![0])
    );

    let camera_man = CameraManager {
        feeds: vec![rx]
    };

    let server = HttpServer::new(move || {
        App::new()
            .wrap(Logger::default())
            .service(web::scope("cameras")
                .data(camera_man.clone())
                .service(camera::static_image)
                .service(camera::ws_camera)
            )
            .service(command::get_speed)
            .service(command::set_speed)
            .service(command::set_target)
            .service(command::end_autonomous)
    })
        .bind(("0.0.0.0", 8080))?
        .run();

    let mut td = TimerData {
        grab_millis: 0,
        retrieve_millis: 0,
        resize_millis: 0,
        encode_millis: 0,
    };

    for i in 0..1_000_000 {
        let timer = std::time::Instant::now();

        let buf = unsafe {
            instagram_thot.read(WIDTH, HEIGHT, FILE_FORMAT, &mut td).unwrap()
        };

        let time = timer.elapsed().as_millis();

        log::trace!("{}:\t{} millis", i, time);
        log::trace!("\tGrab: {} millis", td.grab_millis);
        log::trace!("\tRetrieve: {} millis", td.retrieve_millis);
        log::trace!("\tResize: {} millis", td.resize_millis);
        log::trace!("\tEncode: {} millis", td.encode_millis);

        tx.send(Bytes::from(buf.to_vec())).unwrap();
    }

    server.stop(false).await;

    Ok(())
}