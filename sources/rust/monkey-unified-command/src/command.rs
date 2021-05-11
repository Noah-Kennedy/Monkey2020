use crossbeam::channel::{Receiver, Sender, TrySendError, TryRecvError};
use actix_web::web::{Data, Path, Payload, Json};
use actix_web::{HttpResponse, ResponseError};
use space_monkeys::Command;
use monkey_api::{MotorSpeeds, Location};
use std::fmt::{Display, Formatter};
use std::fmt;
use actix_web::http::StatusCode;
use std::error::Error;

#[derive(Debug, Copy, Clone, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum CommandError {
    TryRecv(TryRecvError),
    TrySend(TrySendError<Command>)
}

impl Display for CommandError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            CommandError::CameraDoesNotExist(id) => writeln!(f, "Camera {} does not exist", id)?,
        }

        Ok(())
    }
}

impl Error for CommandError {}

impl ResponseError for CommandError {
    fn status_code(&self) -> StatusCode {
        match self {
            CommandError::CameraDoesNotExist(_) => StatusCode::BAD_REQUEST,
        }
    }
}

pub struct CommandManager {
    pub command_send: Sender<Command>,
    pub speed_rec: Receiver<MotorSpeeds>
}

#[actix_web::get("/get_speed")]
pub async fn get_speed(Path(()): Path<()>, manager: Data<CommandManager>) -> Result<HttpResponse, CommandError> {
    match manager.speed_rec.try_recv() {
        Ok(speed) => Ok(HttpResponse::Ok().body(speed)),
        Err(err) => CommandError::TryRecv(err)
    }
}

#[actix_web::post("/command/set_speed")]
pub async fn set_speed(speed: Json<MotorSpeeds>, manager: Data<CommandManager>) -> Result<HttpResponse, CommandError> {
    match manager.command_send.try_send(Command::SetSpeed(speed.0)) {
        Ok(_) => Ok(HttpResponse::Ok().body({})),
        Err(err) => CommandError::TrySend(err)
    }
}

#[actix_web::post("/command/set_target")]
pub async fn set_target(target: Json<Option<Location>>, manager: Data<CommandManager>) -> Result<HttpResponse, CommandError> {
    match manager.command_send.try_send(Command::SetTarget(target.0)) {
        Ok(_) => Ok(HttpResponse::Ok().body({})),
        Err(err) => CommandError::TrySend(err)
    }
}

#[actix_web::post("/command/end_autonomous")]
pub async fn end_autonomous(manager: Data<CommandManager>) -> Result<HttpResponse, CommandError> {
    match manager.command_send.try_send(Command::EndAutonomous) {
        Ok(_) => Ok(HttpResponse::Ok().body({})),
        Err(err) => CommandError::TrySend(err)
    }
}
