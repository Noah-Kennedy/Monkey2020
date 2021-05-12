use std::error::Error;
use std::fmt::{Display, Formatter};
use std::fmt;

use actix_web::{HttpResponse, ResponseError, HttpRequest, Result};
use actix_web::web::{Data, Json};
use tokio::sync::watch::{Receiver, Sender};

use monkey_api::{Location, MotorSpeeds};
use space_monkeys::Command;

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
    match manager.command_send.try_send(Command::SetSpeed(speed.0)) {
        Ok(_) => Ok(HttpResponse::Ok().finish()),
        Err(err) => Err(CommandError::TrySend(err))
    }
}

#[actix_web::post("/command/set_target")]
pub async fn set_target(target: Json<Option<Location>>, manager: Data<CommandManager>) -> Result<HttpResponse, CommandError> {
    match manager.command_send.try_send(Command::SetTarget(target.0)) {
        Ok(_) => Ok(HttpResponse::Ok().finish()),
        Err(err) => Err(CommandError::TrySend(err))
    }
}

#[actix_web::post("/command/end_autonomous")]
pub async fn end_autonomous(manager: Data<CommandManager>) -> Result<HttpResponse, CommandError> {
    match manager.command_send.try_send(Command::EndAutonomous) {
        Ok(_) => Ok(HttpResponse::Ok().finish()),
        Err(err) => Err(CommandError::TrySend(err))
    }
}
