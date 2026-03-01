// src/cpp/interface/imu_sensor.hpp
#pragma once
#include "sensor_input.hpp"
#include "../lib/shm_data_format.hpp"

class ImuSensor : public SensorInput {
public:
    virtual ~ImuSensor() = default;

    ImuSensor(const ImuSensor&) = delete;
    ImuSensor& operator=(const ImuSensor&) = delete;

    // -- IMU 固有メソッド --
    // 最新の IMU データを取得（スレッドセーフ）
    virtual ImuData GetLatestData() = 0;

    // SensorInput の Read() は継承するが、IMU では GetLatestData() を使用

protected:
    ImuSensor() = default;
};
