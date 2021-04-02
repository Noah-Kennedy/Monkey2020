#[cfg_attr(feature = "actix-web", macro_use)]
#[cfg(feature = "actix-web")]
extern crate actix_web;
#[cfg_attr(feature = "serde", macro_use)]
#[cfg(feature = "serde")]
extern crate serde;

use actix_web::{App, HttpServer, middleware};

use crate::routes::{get_image, index};
use env_logger::WriteStyle;
use log::LevelFilter;
use openssl::ssl::{SslAcceptor, SslMethod, SslFiletype};
use actix_web::middleware::Logger;

#[cfg(feature = "objects")]
pub mod objects;

#[cfg(feature = "server")]
pub mod error;

#[cfg(feature = "server")]
pub mod routes;

#[actix_web::main]
async fn main() -> std::io::Result<()> {
    env_logger::builder()
        .filter_level(LevelFilter::Info)
        .default_format()
        .write_style(WriteStyle::Always)
        .init();

    let mut builder = SslAcceptor::mozilla_intermediate(SslMethod::tls())
        .unwrap();

    builder
        .set_private_key_file("/home/noah/CLionProjects/monkey2020/certs/key.pem", SslFiletype::PEM)
        .expect("Error, no certificate key found");

    builder.set_certificate_chain_file("/home/noah/CLionProjects/monkey2020/certs/cert.pem")
        .expect("Error, no certificate file");

    HttpServer::new(|| {
        App::new()
            .wrap(Logger::default())
            .wrap(middleware::Compress::default())
            .wrap(middleware::normalize::NormalizePath::default())
            .service(get_image)
            .service(index)
    })
        // .bind("127.0.0.1:8080")?
        .bind_openssl("127.0.0.1:8080", builder)?
        .run()
        .await
}