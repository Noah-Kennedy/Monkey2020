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

    log::warn!("Hold my beer, this initialization subroutine may crash!");

    let (mut vision, mut thot) = monkey_vision::core::create(
        mesh_file,
        ZedCameraResolution::Res720HD60,
        ZedDepthQuality::DepthPerformance,
        ZedMappingResolution::MapMediumRes,
        ZedMappingRange::MapMedium,
        ZedMeshFilter::FilterMedium,
    ).unwrap();

    log::info!("Initialized the vision");

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
        state: None,
        last_time: Instant::now()
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

        zhu_li.do_the_thing(&mut vision, mesh_file);

        i += 1;
    }

    Ok(())
}