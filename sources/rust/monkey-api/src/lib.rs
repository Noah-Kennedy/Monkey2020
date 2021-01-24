#[cfg_attr(feature = "serde", macro_use)]
#[cfg(feature = "serde")]
extern crate serde;

#[cfg_attr(feature = "actix-web", macro_use)]
#[cfg(feature = "actix-web")]
extern crate actix_web;

#[cfg(feature = "objects")]
pub mod objects;

#[cfg(feature = "server")]
pub mod error;

#[cfg(feature = "server")]
pub mod routes;