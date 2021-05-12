use actix_web::{App, HttpServer};
use cameralot::prelude::*;
use std::fs;

mod util;
mod camera;
pub mod command;

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    let mut instagram_hoe = OpenCVCameraFeed::new();
    instagram_hoe.open(0);

    let server = HttpServer::new(|| {
        App::new()
            .service(camera::static_image)
            .service(camera::ws_camera)
            .service(command::get_speed)
            .service(command::set_speed)
            .service(command::set_target)
            .service(command::end_autonomous)
    })
        .bind(("127.0.0.1", 8080))?
        .run();

    let mut td = TimerData {
        grab_millis: 0,
        retrieve_millis: 0,
        resize_millis: 0,
        encode_millis: 0
    };

    for i in 0..1000 {
        let path = format!("images/{}{}", i, FILE_FORMAT);

        let timer = std::time::Instant::now();

        let buf = unsafe {
            instagram_hoe.read(WIDTH, HEIGHT, FILE_FORMAT, &mut td).unwrap()
        };

        let time = timer.elapsed().as_millis();

        println!("{}:\t{} millis", i, time);
        println!("\tGrab: {} millis", td.grab_millis);
        println!("\tRetrieve: {} millis", td.retrieve_millis);
        println!("\tResize: {} millis", td.resize_millis);
        println!("\tEncode: {} millis", td.encode_millis);

        fs::write(&path, buf).unwrap();
    }

    server.await
}