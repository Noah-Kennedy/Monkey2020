use crossbeam::channel::{Receiver, Sender, TrySendError, TryRecvError};
use actix_web::web::{Data, Path, Payload, Json};
use actix_web::HttpResponse;
use space_monkeys::Command;
use monkey_api::{MotorSpeeds, Location};

pub struct CommandManager {
    pub command_send: Sender<Command>,
    pub speed_rec: Receiver<MotorSpeeds>
}

#[actix_web::get("/get_speed")]
pub async fn get_speed(Path(()): Path<()>, manager: Data<CommandManager>) -> Result<HttpResponse, TryRecvError> {
    manager.speed_rec.try_recv().map(|speed| HttpResponse::Ok().body(speed))
}

#[actix_web::post("/command/set_speed")]
pub async fn set_speed(speed: Json<MotorSpeeds>, manager: Data<CommandManager>) -> Result<HttpResponse, TrySendError<Command>> {
    manager.command_send.try_send(Command::SetSpeed(speed.0))?;
    Ok(HttpResponse::Ok().body({}))
}

#[actix_web::post("/command/set_target")]
pub async fn set_target(target: Json<Option<Location>>, manager: Data<CommandManager>) -> Result<HttpResponse, TrySendError<Command>> {
    manager.command_send.try_send(Command::SetTarget(target.0))?;
    Ok(HttpResponse::Ok().body({}))
}

#[actix_web::post("/command/end_autonomous")]
pub async fn end_autonomous(manager: Data<CommandManager>) -> Result<HttpResponse, TrySendError<Command>> {
    manager.command_send.try_send(Command::EndAutonomous)?;
    Ok(HttpResponse::Ok().body({}))
}
