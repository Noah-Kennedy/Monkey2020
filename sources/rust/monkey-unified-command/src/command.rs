use std::error::Error;
use std::fmt::{Display, Formatter};
use std::fmt;

use actix_web::{HttpResponse, ResponseError, Result, HttpRequest};
use actix_web::http::StatusCode;
use actix_web::web::{Data, Json, Path};

use monkey_api::{Location, MotorSpeeds};
use space_monkeys::{Command, AutonomousState};
use monkey_api::requests::AutonomousParams;
use crate::util::throw_text_at_wall;
use space_monkeys::mesh_to_grid::Grid;
use std::time::{Duration, Instant};
use tokio::sync::watch;
use crossbeam::channel;

#[derive(Debug)]
pub enum CommandError {
    Send(channel::SendError<Command>)
}

impl Display for CommandError {
    fn fmt(&self, f: &mut Formatter<'_>) -> fmt::Result {
        match self {
            CommandError::Send(err) => writeln!(f, "{:?}", err)?
        }
        Ok(())
    }
}

impl Error for CommandError {}

impl ResponseError for CommandError {
    fn status_code(&self) -> StatusCode {
        match self {
            CommandError::Send(_) => StatusCode::INTERNAL_SERVER_ERROR
        }
    }
}

pub struct NavManager {
    pub command_send: channel::Sender<Command>,
    pub speed_rec: watch::Receiver<MotorSpeeds>
}

#[actix_web::get("/nav/start")]
pub async fn start(params: Json<AutonomousParams>) -> actix_web::Result<HttpResponse> {
    let state = AutonomousState {
        speed: Default::default(),
        target: None,
        keep_going: true,
        imu_data: Default::default(),
        path: None,
        grid: Grid::new(params.min_x, params.max_x, params.min_z, params.max_z, params.res_x, params.res_z),
        time_since_last_spatial_map_update: Duration::from_secs(0),
        last_time: Instant::now(),
    };
    Ok(HttpResponse::Ok().finish())
}

#[actix_web::get("/nav/get_speed")]
pub async fn get_speed(manager: Data<NavManager>) -> Result<Json<MotorSpeeds>, CommandError> {
    Ok(Json(*manager.speed_rec.borrow()))
}

#[actix_web::post("/nav/set_speed")]
pub async fn set_speed(speed: Json<MotorSpeeds>, manager: Data<NavManager>) -> Result<HttpResponse, CommandError> {
    match manager.command_send.send(Command::SetSpeed(speed.0)) {
        Ok(_) => Ok(HttpResponse::Ok().finish()),
        Err(err) => Err(CommandError::Send(err))
    }
}

#[actix_web::post("/nav/set_target")]
pub async fn set_target(target: Json<Option<Location>>, manager: Data<NavManager>) -> Result<HttpResponse, CommandError> {
    match manager.command_send.send(Command::SetTarget(target.0)) {
        Ok(_) => Ok(HttpResponse::Ok().finish()),
        Err(err) => Err(CommandError::Send(err))
    }
}

#[actix_web::post("/nav/end_autonomous")]
pub async fn end_autonomous(manager: Data<NavManager>) -> Result<HttpResponse, CommandError> {
    match manager.command_send.send(Command::EndAutonomous) {
        Ok(_) => Ok(HttpResponse::Ok().finish()),
        Err(err) => Err(CommandError::Send(err))
    }
}
