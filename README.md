# ESP32-C3 Real-Time Physical UV Sensor Monitoring System

High-precision, real-time IoT UV monitoring ecosystem featuring an ESP32-C3 microcontroller, GUVA-S12SD UV sensor, 3-tier digital signal processing (DSP), InfluxDB Cloud, Grafana dashboards, and a cross-platform Flutter mobile application.

## System Architecture
* **Edge Sensor Node:** ESP32-C3 reading analog voltage from GUVA-S12SD via `GPIO2` (ADC1_CH2).
* **Signal Processing (DSP):** 
  * 9-sample sliding window **Median Filter** (outlier/spike rejection).
  * **1D Kalman Filter** (Gaussian noise reduction).
  * **Hybrid Filter** (Median + Kalman cascaded for zero lag during rapid light shifts).
* **Data Pipelines:**
  * **MQTT Protocol:** Real-time JSON publishing via HiveMQ Cloud (`venkat_iot/sensors/uv/node_01`).
  * **Time-Series Database:** InfluxDB Cloud (`uv_sensor_data` bucket, `uv_measurement` measurement).
  * **Visualization:** Grafana Cloud (Stat gauges, 24h peaks, time-series voltage/UVI graphs, 7-day peak bar charts).
  * **Mobile Control:** Flutter app supporting multi-period history (Date, Week, Month, Year), 24-hour descending UV ranking, and CSV/PNG data exports.
