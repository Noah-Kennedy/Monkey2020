use actix_web::{App, HttpServer};

mod camera;
mod command;

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    HttpServer::new(|| {
        App::new()
            .service(camera::static_image)
            .service(camera::ws_camera)
            .service(command::get_speed)
            .service(command::set_speed)
            .service(command::set_target)
            .service(command::end_autonomous)
    })
        .bind(("127.0.0.1", 8080))?
        .run()
        .await
}