use actix_web::{HttpResponse, web, ResponseError};
use tokio::sync::watch;
use std::error::Error;

#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum CameraFeedError {
    CameraDoesNotExist,
}

impl Error for CameraFeedError {

}


impl ResponseError for CameraFeedError {

}

pub struct CameraManager {
    feeds: Vec<watch::Receiver<Vec<u8>>>,
}

#[actix_web::get("/cameras/{id}/static")]
pub async fn static_image(web::Path((id, )): web::Path<(usize, )>, manager: web::Data<CameraManager>) -> HttpResponse {
    if id < feeds.len
}