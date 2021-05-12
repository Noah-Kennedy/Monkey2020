use std::error::Error;
use std::fmt::{Display, Formatter};
use std::{fmt, thread};

use actix_web::{HttpResponse, ResponseError, Result, HttpRequest};
use actix_web::http::StatusCode;
use actix_web::web::{Data, Json, Path};

use monkey_api::{Location, MotorSpeeds};
use space_monkeys::{Command, NavState, ZhuLi};
use monkey_api::requests::AutonomousParams;
use crate::util::throw_text_at_wall;
use space_monkeys::mesh_to_grid::Grid;
use std::time::{Duration, Instant};
use tokio::sync::watch;
use crossbeam::channel;

#[derive(Debug)]
pub enum NavError {
    Send(channel::SendError<Command>)
}

impl Display for NavError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            NavError::Send(err) => writeln!(f, "{:?}", err)?
        }
        Ok(())
    }
}

impl Error for NavError {}

impl ResponseError for NavError {
    fn status_code(&self) -> StatusCode {
        match self {
            NavError::Send(_) => StatusCode::INTERNAL_SERVER_ERROR
        }
    }
}

#[derive(Debug, Clone)]
pub struct NavManager {
    pub command_send: channel::Sender<Command>,
    pub speed_rec: watch::Receiver<MotorSpeeds>
}

#[actix_web::get("/nav/start")]
pub async fn start(params: Json<AutonomousParams>, manager: Data<NavManager>) -> actix_web::Result<HttpResponse, NavError> {
    match manager.command_send.send(Command::StartNav(params.0)) {
        Ok(_) => Ok(HttpResponse::Ok().finish()),
        Err(err) => Err(NavError::Send(err))
    }
}

#[actix_web::get("/nav/get_speed")]
pub async fn get_speed(manager: Data<NavManager>) -> Result<Json<MotorSpeeds>, NavError> {
    Ok(Json(*manager.speed_rec.borrow()))
}

#[actix_web::post("/nav/set_speed")]
pub async fn set_speed(speed: Json<MotorSpeeds>, manager: Data<NavManager>) -> Result<HttpResponse, NavError> {
    match manager.command_send.send(Command::SetSpeed(speed.0)) {
        Ok(_) => Ok(HttpResponse::Ok().finish()),
        Err(err) => Err(NavError::Send(err))
    }
}

#[actix_web::post("/nav/set_target")]
pub async fn set_target(target: Json<Option<Location>>, manager: Data<NavManager>) -> Result<HttpResponse, NavError> {
    match manager.command_send.send(Command::SetTarget(target.0)) {
        Ok(_) => Ok(HttpResponse::Ok().finish()),
        Err(err) => Err(NavError::Send(err))
    }
}

#[actix_web::post("/nav/end")]
pub async fn end(manager: Data<NavManager>) -> Result<HttpResponse, NavError> {
    match manager.command_send.send(Command::EndNav) {
        Ok(_) => Ok(HttpResponse::Ok().finish()),
        Err(err) => Err(NavError::Send(err))
    }
}
