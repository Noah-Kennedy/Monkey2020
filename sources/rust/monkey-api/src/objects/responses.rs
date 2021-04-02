pub mod flight_plan {
    use crate::objects::responses::TurningArc;

    #[derive(Default, Debug, PartialOrd, PartialEq, Clone)]
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    pub struct FlightPlan {
        pub turning_arcs: Vec<PlannedTurningArc>
    }

    #[derive(Default, Debug, PartialOrd, PartialEq, Clone)]
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    pub struct PlannedTurningArc {
        /// Arc to turn along
        #[cfg_attr(feature = "serde", serde(flatten))]
        pub arc: TurningArc,
        /// Length of time to turn along art
        pub time: f32,
    }
}

pub mod flight_director {
    use crate::objects::responses::TurningArc;

    #[derive(Debug, PartialOrd, PartialEq, Clone, Copy)]
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    pub enum FlightDirectorInstruction {
        HoldCalculatingRoute,
        HoldAbortRecalculatingRoute,
        HoldAbortDriverControl,
        Turn(TurningArc),
        Complete,
    }
}

#[derive(Default, Debug, PartialOrd, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TurningArc {
    /// The speed for the right motor, in radians per second
    pub right_speed: f32,
    /// The speed for the left motor, in radians per second
    pub left_speed: f32,
}
