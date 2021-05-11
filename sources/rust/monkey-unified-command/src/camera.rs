use std::error::Error;
use std::fmt::{Display, Formatter};
use std::fmt;

use actix_web::{HttpRequest, HttpResponse, ResponseError, web};
use actix_web::http::StatusCode;
use actix_web::web::Bytes;
use actix_web_actors::ws;
use tokio::sync::{mpsc, watch};
use tokio_stream::wrappers::ReceiverStream;

#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum CameraFeedError {
    CameraDoesNotExist(usize),
}

impl Display for CameraFeedError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            CameraFeedError::CameraDoesNotExist(id) => writeln!(f, "Camera {} does not exist", id)?,
        }

        Ok(())
    }
}

impl Error for CameraFeedError {}

impl ResponseError for CameraFeedError {
    fn status_code(&self) -> StatusCode {
        match self {
            CameraFeedError::CameraDoesNotExist(_) => StatusCode::BAD_REQUEST,
        }
    }
}

pub struct CameraManager {
    feeds: Vec<watch::Receiver<Bytes>>,
}

#[actix_web::get("/cameras/{id}/static")]
pub async fn static_image(
    web::Path((id, )): web::Path<(usize, )>,
    manager: web::Data<CameraManager>,
) -> Result<HttpResponse, CameraFeedError>
{
    let feed = manager.feeds.get(id)
        .ok_or(CameraFeedError::CameraDoesNotExist(id))?;

    // borrow holds read lock
    let payload = {
        feed.borrow().clone()
    };

    Ok(HttpResponse::Ok().body(payload))
}

#[actix_web::get("/cameras/{id}/ws-feed")]
pub async fn ws_camera(
    req: HttpRequest,
    web::Path((id, )): web::Path<(usize, )>,
    manager: web::Data<CameraManager>,
) -> actix_web::Result<HttpResponse> {
    let mut feed = manager.feeds.get(id)
        .ok_or(CameraFeedError::CameraDoesNotExist(id))?.clone();

    let mut res = ws::handshake(&req)?;
    let (tx, rx) =
        mpsc::channel::<Result<Bytes, actix_web::Error>>(8);

    tokio::task::spawn_local(async move {
        while feed.changed().await.is_ok() {
            let frame = {
                feed.borrow().clone()
            };

            tx.send(Ok(frame)).await.unwrap();
        }
    });

    Ok(res.streaming(ReceiverStream::new(rx)))
}