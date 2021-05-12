use actix_web::{HttpRequest, HttpResponse};
use actix_web::web::{Buf, Bytes, BytesMut};
use serde::Serialize;
use tokio::sync::{mpsc, watch};
use tokio_stream::wrappers::ReceiverStream;
use tokio_util::codec::Encoder;

pub fn throw_bytes_at_wall(
    req: HttpRequest,
    mut input: watch::Receiver<Bytes>,
) -> actix_web::Result<HttpResponse> {
    let mut res = actix_web_actors::ws::handshake(&req)?;
    let (tx, rx) =
        mpsc::channel::<Result<Bytes, actix_web::Error>>(8);

    let mut codec = actix_http::ws::Codec::new();

    actix_web::rt::spawn(async move {
        while input.changed().await.is_ok() {
            let frame = {
                input.borrow().clone()
            };

            let msg = actix_web_actors::ws::Message::Binary(frame);
            let mut output = BytesMut::new();
            codec.encode(msg, &mut output).unwrap();

            if tx.send(Ok(output.to_bytes())).await.is_err() {
                log::info!("Exiting wall");
                break;
            }
        }
    });

    Ok(res.streaming(ReceiverStream::new(rx)))
}

pub fn throw_text_at_wall<T: 'static>(
    req: HttpRequest,
    mut input: watch::Receiver<T>,
) -> actix_web::Result<HttpResponse>
    where T: Serialize + Clone
{
    let mut res = actix_web_actors::ws::handshake(&req)?;
    let (tx, rx) =
        mpsc::channel::<Result<Bytes, actix_web::Error>>(8);

    let mut codec = actix_http::ws::Codec::new();

    actix_web::rt::spawn(async move {
        while input.changed().await.is_ok() {
            let value = {
                input.borrow().clone()
            };

            let msg = actix_web_actors::ws::Message::Text(serde_json::to_string(&value).unwrap());
            let mut output = BytesMut::new();
            codec.encode(msg, &mut output).unwrap();

            if tx.send(Ok(output.to_bytes())).await.is_err() {
                break;
            }
        }
    });

    Ok(res.streaming(ReceiverStream::new(rx)))
}