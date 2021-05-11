use actix_web::{App, HttpServer};

mod camera;

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    HttpServer::new(|| {
        App::new()
            .service(camera::static_image)
    })
        .bind(("127.0.0.1", 8080))?
        .run()
        .await
}