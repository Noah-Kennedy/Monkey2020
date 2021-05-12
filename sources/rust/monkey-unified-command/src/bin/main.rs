use actix_web::{App, HttpServer, web};
use actix_web::middleware::Logger;
use actix_web::web::Bytes;
use env_logger::WriteStyle;
use log::LevelFilter;
use std::thread;

use cameralot::prelude::*;
use monkey_unified_command::{camera, nav};
use monkey_unified_command::camera::CameraManager;
use monkey_vision::prelude::*;
use monkey_unified_command::nav::NavManager;
use monkey_api::MotorSpeeds;
use space_monkeys::{ZhuLi, Command};
use monkey_api::requests::AutonomousParams;

const FILE_FORMAT: &str = ".jpg";
const WIDTH: u32 = 1280;
const HEIGHT: u32 = 720;

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    let mesh_file = "mesh.ply";

    let (mut vision, mut thot) = monkey_vision::core::create(
        mesh_file,
        ZedCameraResolution::Res720HD60,
        ZedDepthQuality::DepthPerformance,
        ZedMappingResolution::MapMediumRes,
        ZedMappingRange::MapMedium,
        ZedMeshFilter::FilterMedium,
    ).unwrap();

    env_logger::builder()
        .filter_level(LevelFilter::Info)
        .default_format()
        .write_style(WriteStyle::Always)
        .init();

    let mut instagram_thot = OpenCVCameraFeed::new();
    instagram_thot.open(0);

    let (cam_tx, cam_rx) = tokio::sync::watch::channel(
        actix_web::web::Bytes::from(vec![0])
    );

    let (speed_send, speed_rec) = tokio::sync::watch::channel(MotorSpeeds::default());
    let (command_send, command_rec) = crossbeam::channel::unbounded();

    let camera_man = CameraManager {
        feeds: vec![cam_rx]
    };

    let nav_manager = NavManager { command_send, speed_rec };

    let server = HttpServer::new(move || {
        App::new()
            .wrap(Logger::default())
            .service(web::scope("cameras")
                .data(camera_man.clone())
                .service(camera::static_image)
                .service(camera::ws_camera)
            )
            .service(web::scope("nav")
                .data(nav_manager.clone())
                .service(nav::start)
                .service(nav::get_speed)
                .service(nav::set_speed)
                .service(nav::set_target)
                .service(nav::end)
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

    let mut zhu_li = ZhuLi {
        command_rec,
        speed_send,
        params: AutonomousParams::default(),
        state: None
    };

    for i in 0..1_000_000 {
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

        zhu_li.poll_commands();
        zhu_li.do_the_thing(&mut vision, mesh_file);
    }

    nav_manager.command_send.send(Command::EndNav);

    server.stop(false).await;

    Ok(())
}