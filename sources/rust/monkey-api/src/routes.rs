use actix_web::web::{Json, Path};
use crate::objects::Location;
use crate::objects::responses::flight_plan::FlightPlan;
use actix_web::{HttpResponse, Responder};
use actix_files::NamedFile;
use image::imageops::FilterType;

const PATH: &str = "/home/noah/CLionProjects/monkey2020/data/network-test-images";

#[get("/nav/flight-plan/json")]
pub async fn get_flight_plan_json(_location: Json<Location>) -> Json<FlightPlan> {
    // TODO
    Json(FlightPlan::default())
}

#[get("/")]
pub async fn index() -> actix_web::Result<NamedFile> {
    Ok(NamedFile::open("/home/noah/CLionProjects/monkey2020/static/index.html")?)
}

// #[get("/sensors/camera/image/{id}/")]
// pub async fn get_image(camera: Path<u64>) -> actix_web::Result<NamedFile> {
//     let image_path = format!("{}/image_{}.jpg", PATH, camera.0);
//     Ok(NamedFile::open(image_path)?)
// }

#[get("/sensors/camera/image/{id}/")]
pub async fn get_image(camera: Path<u64>) -> impl Responder {
    let image_path = format!("{}/image_{}.jpg", PATH, camera.0);

    let image = image::io::Reader::open(image_path)
        .unwrap()
        .decode()
        .unwrap();

    let image = image.resize(480, 360, FilterType::Nearest);

    let mut bytes: Vec<u8> = Vec::new();
    image.write_to(&mut bytes, image::ImageOutputFormat::Jpeg(10)).unwrap();

    HttpResponse::Ok()
        .content_type("image/jpg")
        .body(bytes)
}