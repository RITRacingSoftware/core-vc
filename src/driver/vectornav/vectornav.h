#include <stdint.h>
#include <stdbool.h>

#ifndef VECTORNAV_H
#define VECTORNAV_H

#define MEMBER_OFFSET(member) ((uint32_t)(&(((vectornav_result_t*)0)->member)))

typedef struct vectornav_time_utc_s {
    int8_t TimeUtcYear;
    uint8_t TimeUtcMonth, TimeUtcDay,
            TimeUtcHour, TimeUtcMinute, TimeUtcSecond;
    uint16_t TimeUtcFracSec;
} vectornav_time_utc_t;

typedef struct vectornav_result_s {
    // IMU data
    float UncompMagX, UncompMagY, UncompMagZ,
          UncompAccelX, UncompAccelY, UncompAccelZ,
          UncompGyroX, UncompGyroY, UncompGyroZ,
          Temperature,
          Pressure,
          DeltaThetaT, DeltaThetaX, DeltaThetaY, DeltaThetaZ,
          DeltaVelX, DeltaVelY, DeltaVelZ,
          MagX, MagY, MagZ,
          AccelX, AccelY, AccelZ,
          AngularRateX, AngularRateY, AngularRateZ;
    uint16_t SensSat;
    // INS data
    uint16_t InsStatus;
    double PosLlaL, PosLlaO, PosLlaA,
           PosEcefX, PosEcefY, PosEcefZ;
    float VelBodyX, VelBodyY, VelBodyZ,
          VelNedN, VelNedE, VelNedD,
          VelEcefX, VelEcefY, VelEcefZ,
          MagEcefX, MagEcefY, MagEcefZ,
          AccelEcefX, AccelEcefY, AccelEcefZ,
          LinAccelEcefX, LinAccelEcefY, LinAccelEcefZ,
          PosU,
          VelU;
    // time data
    uint64_t TimeStartup, TimeGps, GpsTow;
    vectornav_time_utc_t TimeUtc;
    uint16_t GpsWeek;
    uint64_t TimeSyncIn, TimeGpsPps;
    uint32_t SyncInCnt, SyncOutCnt;
    uint8_t TimeStatus;
    // GNSS1 data
    uint8_t NumSats1, Gnss1Fix;
    /*double Gnss1PosLlaL, Gnss1PosLlaO, Gnss1PosLlaA,
           Gnss1PosEcefX, Gnss1PosEcefY, Gnss1PosEcefZ;
    float Gnss1VelNedN, Gnss1VelNedE, Gnss1VelNedD,
          Gnss1VelEcefX, Gnss1VelEcefY, Gnss1VelEcefZ,
          Gnss1PosUncertaintyN, Gnss1PosUncertaintyE, Gnss1PosUncertaintyD,
          Gnss1VelUncertainty,
          Gnss1TimeUncertainty;*/

    // Attitude data
    float YprY, YprP, YprR;

    // GNSS2 data
    uint8_t NumSats2, Gnss2Fix;
    /*double Gnss1PosLlaL, Gnss1PosLlaO, Gnss1PosLlaA,
           Gnss1PosEcefX, Gnss1PosEcefY, Gnss1PosEcefZ;
    float Gnss1VelNedN, Gnss1VelNedE, Gnss1VelNedD,
          Gnss1VelEcefX, Gnss1VelEcefY, Gnss1VelEcefZ,
          Gnss1PosUncertaintyN, Gnss1PosUncertaintyE, Gnss1PosUncertaintyD,
          Gnss1VelUncertainty,
          Gnss1TimeUncertainty;*/
    // attitude data
} vectornav_result_t;

extern vectornav_result_t vn_data_raw;

bool vectornav_parse(const uint8_t *buf, uint16_t buflen, vectornav_result_t *data);
void vectornav_send_errors();
void vectornav_init();

#endif
