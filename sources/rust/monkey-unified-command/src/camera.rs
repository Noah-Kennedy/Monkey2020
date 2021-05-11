use std::error::Error;
use std::fmt::{Display, Formatter};
use std::fmt;

use actix_web::{HttpRequest, HttpResponse, ResponseError, web};
use actix_web::http::StatusCode;
use actix_web::web::{Bytes, BytesMut, Buf};
use actix_web_actors::ws;
use tokio::sync::{mpsc, watch};
use tokio_stream::wrappers::ReceiverStream;
use tokio_stream::StreamExt;
use tokio_util::codec::Encoder;

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

#[derive(Debug, Clone)]
pub struct CameraManager {
    pub feeds: Vec<watch::Receiver<Bytes>>,
}

#[actix_web::get("/cameras/{id}/static")]
pub async fn static_image(
    path: web::Path<(usize, )>,
    manager: web::Data<CameraManager>,
) -> Result<HttpResponse, CameraFeedError>
{
    let id = path.into_inner().0;
    let feed = manager.feeds.get(id)
        .ok_or(CameraFeedError::CameraDoesNotExist(id))?;

    // borrow holds read lock
    let payload = {
        feed.borrow().clone()
    };

    Ok(HttpResponse::Ok().body(payload))
}

#[actix_web::get("/{id}/ws-feed")]
pub async fn ws_camera(
    req: HttpRequest,
    path: web::Path<(usize, )>,
    manager: web::Data<CameraManager>,
) -> actix_web::Result<HttpResponse> {
    let id = path.into_inner().0;

    let mut feed = manager.feeds.get(id)
        .ok_or(CameraFeedError::CameraDoesNotExist(id))?.clone();

    let mut res = ws::handshake(&req)?;
    let (tx, rx) =
        mpsc::channel::<Result<Bytes, actix_web::Error>>(8);

    let mut codec = actix_http::ws::Codec::new();

    actix_web::rt::spawn(async move {
        while feed.changed().await.is_ok() {
            let frame = {
                feed.borrow().clone()
            };

            let msg = ws::Message::Binary(frame);
            let mut output = BytesMut::new();
            codec.encode(msg, &mut output);

            tx.send(Ok(output.to_bytes())).await.unwrap();
        }
    });

    Ok(res.streaming(ReceiverStream::new(rx)))
}

#[cfg(test)]
mod tests {
    use actix_web::{App, HttpServer, web};
    use actix_web::web::Bytes;
    use futures_util::stream::StreamExt;
    use tokio::sync::watch;

    use crate::camera;
    use crate::camera::CameraManager;
    use futures_util::SinkExt;

    #[actix_rt::test]
    async fn test_ws() {
        let (tx, rx) = watch::channel(Bytes::from("invisible"));
        let manager = CameraManager {
            feeds: vec![rx]
        };

        let server = HttpServer::new(move || {
            App::new()
                .service(web::scope("/cameras")
                    .service(camera::ws_camera)
                    .data(manager.clone()))
        })
            .bind("127.0.0.1:8080").unwrap();


        let server = server.run();

        let (stream, _) = tokio_tungstenite::connect_async(
            "ws://127.0.0.1:8080/cameras/0/ws-feed"
        )
            .await
            .unwrap();

        let (mut write, mut read) = stream.split();

        write.send(tokio_tungstenite::tungstenite::Message::Binary(b"^^^^".to_vec())).await.unwrap();

        tx.send(Bytes::from("hello")).unwrap();

        assert_eq!(
            tokio_tungstenite::tungstenite::Message::Binary("hello".as_bytes().to_owned()),
            read.next().await.unwrap().unwrap()
        );

        tx.send(Bytes::from("world")).unwrap();

        assert_eq!(
            tokio_tungstenite::tungstenite::Message::Binary("world".as_bytes().to_owned()),
            read.next().await.unwrap().unwrap()
        );

        server.stop(false).await;
    }
}