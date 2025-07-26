use std::f64::consts::PI;

use nalgebra::{Matrix1, Matrix2, RowVector2, Vector2};

#[derive(Clone)]
pub struct KalmanFilter {
    // [phase offset, phase offset / second]
    pub state: Vector2<f64>,
    pub uncertainty: Matrix2<f64>,
    time_wander: f64,
}

impl KalmanFilter {
    pub fn new() -> Self {
        Self {
            state: Vector2::new(0.0, 0.0),
            uncertainty: Matrix2::from_diagonal(&Vector2::new(PI * PI, PI * PI)),
            time_wander: 1e-8,
        }
    }

    pub fn advance_time(&mut self, duration: f64) {
        let update = Matrix2::new(1.0, duration, 0.0, 1.0);
        let noise = Matrix2::new(
            self.time_wander * duration * duration * duration / 3.0,
            self.time_wander * duration * duration / 2.0,
            self.time_wander * duration * duration / 2.0,
            self.time_wander * duration,
        );

        self.state = update * self.state;
        self.uncertainty = update * self.uncertainty * update.transpose() + noise;
    }

    pub fn measurement(&mut self, measurement: f64, noise: f64) {
        let to_measurement = RowVector2::<f64>::new(1.0, 0.0);

        let measurement = Matrix1::new(measurement);
        let noise = Matrix1::new(noise);

        let prediction = to_measurement * self.state;
        let mut difference = measurement - prediction;
        while difference.x < -PI {
            difference.x += 2.0 * PI;
        }
        while difference.x > PI {
            difference.x -= 2.0 * PI;
        }
        let covariance = to_measurement * self.uncertainty * to_measurement.transpose() + noise;

        let measurement_strength =
            self.uncertainty * to_measurement.transpose() * covariance.try_inverse().unwrap();

        self.state += measurement_strength * difference;
        self.uncertainty =
            (Matrix2::identity() - measurement_strength * to_measurement) * self.uncertainty;
    }
}
