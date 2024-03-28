mod cheats;

use std::default;
use std::time::Duration;

use hs_hackathon::prelude::tracing_subscriber::filter::targets;
use hs_hackathon::prelude::*;

use cheats::angles::Vector;
use cheats::approaching::Hint;
use cheats::positioning::Position;
use cheats::TeamColors;

const CAR: Color = Color::Red;
const TARGET: Color = Color::Green;

#[allow(unused)]
struct MapState {
    car: Position,
    target: Position,
}

#[allow(unused)]
impl MapState {
    pub async fn infer(drone: &mut Camera) -> eyre::Result<Self> {
        let snapshot = drone.snapshot().await?;
        let leds = hs_hackathon::vision::detect(&snapshot.0, &Default::default())?;
        let car = leds.iter().find(|led| led.color == CAR);
        let target = leds.iter().find(|led| led.color == TARGET);

        tracing::info!("pos car: {car:?}, pos target {target:?}");

        if let (Some(car), Some(target)) = (car, target) {
            Ok(Self {
                car: Position::from(car.bbox),
                target: Position::from(target.bbox),
            })
        } else {
            Err(eyre::eyre!("Car or target not found"))
        }
    }

    async fn car_orientation(
        current: Position,
        drone: &mut Camera,
        motor: &mut MotorSocket,
        wheels: &mut WheelOrientation,
    ) -> eyre::Result<Vector> {
        unimplemented!()
    }
}

#[derive(Debug)]
#[allow(unused)]
enum State {
    /// Turn the cars direction by doing consecutive front and back movements
    /// until the angle between the cars orientation and the target converges to be under
    /// a specified threshold
    Turning,
    /// Approach the car by doing incremental actions of approaching and measuring interleaved.
    /// So we approach the target a bit, measure if we decreased the distance, if yes repeat, if no
    /// then calibrate. We do this until we hit the target.
    Approaching,
    /// Simply idling on the target and identifying when the target moves away from our current
    /// position.
    Idle,
}

impl State {
    async fn execute(
        &mut self,
        drone: &mut Camera,
        motor: &mut MotorSocket,
        wheels: &mut WheelOrientation,
    ) -> eyre::Result<()> {
        tracing::info!("state: {self:?}");
        let mut last_pos = MapState::infer(drone).await?;

        match self {
            State::Turning => loop {
                motor
                    .move_for(Velocity::forward(), Duration::from_millis(500))
                    .await?;

                let cur_pos = MapState::infer(drone).await?;

                let car_vec = Vector::from((last_pos.car, cur_pos.car));

                let target_vec = Vector::from((cur_pos.car, cur_pos.target));

                let angle = car_vec.angle(target_vec);
                let on_target = angle.abs() < 10.0;

                last_pos = cur_pos;

                *self = if on_target {
                    wheels.set(Angle::straight()).await?;
                    State::Approaching
                } else {
                    let wheel_angle = if angle.is_sign_positive() {
                        Angle::right()
                    } else {
                        Angle::left()
                    };
                    wheels.set(wheel_angle).await?;
                    motor
                        .move_for(Velocity::backward(), Duration::from_millis(500))
                        .await?;

                    let wheel_angle = if angle.is_sign_positive() {
                        Angle::left()
                    } else {
                        Angle::right()
                    };
                    wheels.set(wheel_angle).await?;
                    State::Turning
                }
            },
            State::Approaching => {
                let hint = cheats::approaching::auto(
                    &TeamColors {
                        car: CAR,
                        target: TARGET,
                    },
                    drone,
                    motor,
                    wheels,
                )
                .await?;

                *self = match hint {
                    Hint::TargetWasHit => Self::Idle,
                    Hint::OrientationIsOff => Self::Turning,
                };
            }
            State::Idle => {
                cheats::idling::auto(
                    &TeamColors {
                        car: CAR,
                        target: TARGET,
                    },
                    drone,
                    motor,
                    wheels,
                )
                .await?;

                *self = Self::Turning;
            }
        }

        Ok(())
    }
}

#[hs_hackathon::main]
async fn main() -> eyre::Result<()> {
    let mut wheels = WheelOrientation::new().await?;
    let mut motor = MotorSocket::open().await?;
    let mut drone = Camera::connect().await?;

    let mut machine = State::Turning;

    loop {
        machine.execute(&mut drone, &mut motor, &mut wheels).await?;
        tracing::debug!("{:?}", machine);
    }
}
