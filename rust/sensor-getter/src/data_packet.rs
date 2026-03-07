pub type MagData = bmm350::Sensor3DData;
pub type GyroData = bmi323::Sensor3DData;
pub type AccelData = bmi323::Sensor3DData;


pub struct DataPack {
    tg1: GyroData,
    ta1: AccelData, 
    tm1: MagData,
    tg2: GyroData,
    ta2: AccelData, 
    tm2: MagData,
    pos_gps: ublox::Position,
    pos_gps_ecef: ublox::PositionECEF,
    vel_gps: ublox::Velocity,
}
