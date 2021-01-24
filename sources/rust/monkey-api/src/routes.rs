use actix_web::web::{Json};
use crate::objects::responses::flight_plan::FlightPlan;
use crate::objects::Location;

#[get("/nav/flight-plan/json")]
pub async fn get_flight_plan_json(_location: Json<Location>) -> Json<FlightPlan> {
    // TODO
    Json(FlightPlan::default())
}

