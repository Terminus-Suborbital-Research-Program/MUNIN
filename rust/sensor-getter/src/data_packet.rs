use iceoryx2::prelude::*;

pub type MagData = bmm350::Sensor3DData;
pub type GyroData = bmi323::Sensor3DData;
pub type AccelData = bmi323::Sensor3DData;

#[derive(Debug, ZeroCopySend)]
#[repr(C)]
pub struct Position {
    pub lat: f64,
    pub lon: f64,
    pub alt: f64,
}

#[derive(Debug, ZeroCopySend)]
#[repr(C)]
pub struct PositionECEF {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Debug, ZeroCopySend)]
#[repr(C)]
pub struct Velocity {
    speed: f64,
    heading: f64,
}

#[derive(Debug, ZeroCopySend)]
#[repr(C)]
pub struct GPSData {
    gps_pos: Position,
    gps_pos_ecef: PositionECEF,
    gps_vel: Velocity,
}

#[derive(Debug, ZeroCopySend)]
#[repr(C)]
pub struct IMUData {
    gyro_x: i16,
    gyro_y: i16,
    gyro_z: i16,
    accel_x: i16,
    accel_y: i16,
    accel_z: i16,
    mag_x: i32,
    mag_y: i32,
    mag_z: i32,
}

#[derive(Debug, ZeroCopySend)]
#[repr(C)]
pub struct DataPack {
    imu0_data: IMUData,
        imu1_data: IMUData,
    gps_data: GPSData,
}
