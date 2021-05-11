use std::error::Error;
use std::fmt::{Display, Formatter};
use std::fmt;

use actix_web::{HttpRequest, HttpResponse, ResponseError, web};
use actix_web::http::StatusCode;
use actix_web::web::{Bytes, BytesMut};
use actix_web_actors::ws;
use tokio::sync::{mpsc, watch};

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
    mut stream: web::Payload,
) -> actix_web::Result<HttpResponse> {
    let mut res = ws::handshake(&req)?;
    // tokio::task::spawn_local(async move {
    //     while let Some(chunk) = stream.next().await {
    //         let chunk = chunk.unwrap();
    //         let mut codec = Codec::new();
    //         let mut buf = BytesMut::new();
    //         buf.extend_from_slice(&chunk[..]);
    //
    //         let frame = codec.decode(&mut buf).unwrap();
    //         let frame_str = format!("frame: {:?}", frame);
    //
    //         let message = ws::Message::Text(frame_str);
    //         let mut output_buffer = BytesMut::new();
    //         codec.encode(message, &mut output_buffer).unwrap();
    //         let b = output_buffer.split().freeze();
    //         tx.send(Ok(b)).unwrap();
    //     }
    // });

    Ok(res.streaming(manager.feeds[i]))
}