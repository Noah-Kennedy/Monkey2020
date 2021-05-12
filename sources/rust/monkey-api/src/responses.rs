pub mod flight_plan {
    use crate::MotorSpeeds;

    #[derive(Default, Debug, PartialOrd, PartialEq, Clone)]
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    pub struct FlightPlan {
        pub turning_arcs: Vec<PlannedTurningArc>,
    }

    #[derive(Debug, PartialOrd, PartialEq, Clone)]
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    pub struct PlannedTurningArc {
        /// Arc to turn along, defined by motor speeds
        #[cfg_attr(feature = "serde", serde(flatten))]
        pub arc: MotorSpeeds,
        /// Length of time to turn along art
        pub time: f32,
    }
}

pub mod flight_director {
    use crate::MotorSpeeds;

    #[derive(Debug, PartialOrd, PartialEq, Clone, Copy)]
    #[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
    pub enum FlightDirectorInstruction {
        HoldCalculatingRoute,
        HoldAbortRecalculatingRoute,
        HoldAbortDriverControl,
        Turn(MotorSpeeds),
        Complete,
    }
}
