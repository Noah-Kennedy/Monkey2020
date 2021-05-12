use std::error::Error;
use std::fmt::{Display, Formatter};
use std::fmt;

use actix_web::{HttpResponse, ResponseError, HttpRequest, Result};
use actix_web::web::{Data, Json};
use tokio::sync::watch::{Receiver, Sender, error::SendError};
use crossbeam::channel::TryRecvError;

use monkey_api::{Location, MotorSpeeds};
use space_monkeys::Command;
use actix_web::http::StatusCode;

#[derive(Debug, Clone)]
pub enum CommandError {
    Send(SendError<Command>),
    TryRecv(TryRecvError)
}

impl Display for CommandError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            CommandError::Send(err) => writeln!(f, "{:?}", err)?,
            CommandError::TryRecv(err) => writeln!(f, "{:?}", err)?
        }
        Ok(())
    }
}

impl Error for CommandError {}

impl ResponseError for CommandError {
    fn status_code(&self) -> StatusCode {
        match self {
            CommandError::Send(_) => StatusCode::INTERNAL_SERVER_ERROR,
            CommandError::TryRecv(_) => StatusCode::NOT_FOUND
        }
    }
}

pub struct CommandManager {
    pub command_send: Sender<Command>,
    pub speed_rec: Receiver<MotorSpeeds>,
}

#[actix_web::get("/get_speed")]
pub async fn get_speed(manager: Data<CommandManager>) -> Result<Json<MotorSpeeds>, CommandError> {
    match manager.speed_rec.try_recv() {
        Ok(speed) => Ok(Json(speed)),
        Err(err) => Err(CommandError::TryRecv(err))
    }
}

#[actix_web::post("/command/set_speed")]
pub async fn set_speed(speed: Json<MotorSpeeds>, manager: Data<CommandManager>) -> Result<HttpResponse, CommandError> {
    match manager.command_send.send(Command::SetSpeed(speed.0)) {
        Ok(_) => Ok(HttpResponse::Ok().finish()),
        Err(err) => Err(CommandError::Send(err))
    }
}

#[actix_web::post("/command/set_target")]
pub async fn set_target(target: Json<Option<Location>>, manager: Data<CommandManager>) -> Result<HttpResponse, CommandError> {
    match manager.command_send.send(Command::SetTarget(target.0)) {
        Ok(_) => Ok(HttpResponse::Ok().finish()),
        Err(err) => Err(CommandError::Send(err))
    }
}

#[actix_web::post("/command/end_autonomous")]
pub async fn end_autonomous(manager: Data<CommandManager>) -> Result<HttpResponse, CommandError> {
    match manager.command_send.send(Command::EndAutonomous) {
        Ok(_) => Ok(HttpResponse::Ok().finish()),
        Err(err) => Err(CommandError::Send(err))
    }
}
