# cs_embedded_software — Project file structure

Top-level layout for the embedded software course project.

```
Services
├── Calibration
│   ├── imu_calibration.c
│   ├── imu_calibration.h
│   ├── barometer_calibration.c
│   └── barometer_calibration.h
├── Control
│   ├── pid.c
│   ├── pid.h
│   ├── sigma_controller.c
│   └── sigma_controller.h
├── Filters
│   ├── biquad_filter.c
│   ├── biquad_filter.h
│   ├── fir_filter.c
│   ├── fir_filter.h
│   ├── moving_average.c
│   └── moving_average.h
├── Health
│   ├── error_manager.c
│   ├── error_manager.h
│   ├── health_monitor.c
│   ├── health_monitor.h
│   ├── sensor_validator.c
│   └── sensor_validator.h
├── Navigation
│   ├── altitude_estimator.c
│   ├── altitude_estimator.h
│   ├── vertical_speed.c
│   └── vertical_speed.h
├── Recovery
│   ├── apam.c
│   ├── apam.h
│   ├── landing_detector.c
│   └── landing_detector.h
├── SensorFusion
│   ├── madgwick.c
│   ├── madgwick.h
│   ├── orientation.c
│   └── orientation.h
├── Storage
│   ├── data_logger.c
│   ├── data_logger.h
│   ├── log_buffer.c
│   └── log_buffer.h
├── Telemetry
│   ├── command_packet.c
│   ├── command_packet.h
│   ├── crc.c
│   ├── crc.h
│   ├── telemetry_packet.c
│   └── telemetry_packet.h
└── Time
    ├── time_service.c
    └── time_service.h
```

Use this as a starting point; adapt directories to your board, toolchain, and workflow.
```
