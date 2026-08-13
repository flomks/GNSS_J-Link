#include "nmea.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static const char *sentence(const char *payload)
{
    static char buffer[256];
    unsigned checksum = 0u;

    for (const char *p = payload; *p != '\0'; p++)
    {
        checksum ^= (unsigned char)*p;
    }

    (void)snprintf(buffer, sizeof(buffer), "$%s*%02X", payload, checksum);
    return buffer;
}

static const NMEA_Satellite *find_satellite(
    const NMEA_Data *data, NMEA_Constellation constellation, uint16_t svid)
{
    for (uint8_t i = 0u; i < data->satellite_count; i++)
    {
        if ((data->satellites[i].constellation == constellation) &&
            (data->satellites[i].svid == svid))
        {
            return &data->satellites[i];
        }
    }

    return NULL;
}

int main(void)
{
    NMEA_Data data;

    NMEA_Reset(&data);
    assert(data.position_valid == 0u);

    assert(NMEA_Parse(
        &data,
        "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A") == 1);
    assert(data.data_valid == 1u);
    assert(data.position_valid == 1u);
    assert(data.latitude_udeg == 48117300);
    assert(data.longitude_udeg == 11516666);
    assert(data.time_valid == 1u);
    assert(data.utc_hour == 12u && data.utc_minute == 35u && data.utc_second == 19u);
    assert(data.date_valid == 1u);
    assert(data.utc_day == 23u && data.utc_month == 3u && data.utc_year == 1994u);
    assert(data.speed_valid == 1u && data.speed_kmh_milli == 41484u);
    assert(data.course_valid == 1u && data.course_mdeg == 84400u);

    NMEA_Reset(&data);
    assert(NMEA_Parse(
        &data,
        "$GPGGA,123520,3450.0000,S,05822.0000,W,1,08,0.9,-0.5,M,46.9,M,,*67") == 1);
    assert(data.position_valid == 1u); /* GGA is enough; GSA is optional. */
    assert(data.latitude_udeg == -34833333);
    assert(data.longitude_udeg == -58366666);
    assert(data.altitude_valid == 1u && data.altitude_dm == -5);

    assert(NMEA_Parse(
        &data,
        "$GPGGA,123521,4807.038,N,,E,1,08,0.9,10.0,M,46.9,M,,*51") == 1);
    assert(data.position_valid == 0u); /* A half position must not be accepted. */

    assert(NMEA_Parse(
        &data,
        "$GPGGA,123522,4807.038,N,01131.000,E,0,00,9.9,10.0,M,46.9,M,,*7E") == 1);
    assert(data.position_valid == 0u); /* Coordinates with GGA quality 0 are invalid. */

    assert(NMEA_Parse(
        &data,
        "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*00") == 0);
    assert(data.checksum_errors == 1u);

    NMEA_Reset(&data);
    assert(NMEA_Parse(&data, sentence(
        "GPGSV,2,1,05,01,40,083,41,02,17,273,38,03,25,123,35,08,62,201,44")) == 1);
    assert(NMEA_Parse(&data, sentence(
        "GPGSV,2,2,05,14,09,310,")) == 1);
    assert(data.satellites_visible_valid == 1u);
    assert(data.satellites_visible == 5u);

    const NMEA_Satellite *gps03 = find_satellite(
        &data, NMEA_CONSTELLATION_GPS, 3u);
    const NMEA_Satellite *gps14 = find_satellite(
        &data, NMEA_CONSTELLATION_GPS, 14u);
    assert(gps03 != NULL);
    assert(gps03->visible == 1u);
    assert(gps03->elevation_valid == 1u && gps03->elevation_deg == 25u);
    assert(gps03->azimuth_valid == 1u && gps03->azimuth_deg == 123u);
    assert(gps03->snr_valid == 1u && gps03->snr_dbhz == 35u);
    assert(gps14 != NULL && gps14->visible == 1u && gps14->snr_valid == 0u);

    assert(NMEA_Parse(&data, sentence(
        "GPGSA,A,3,03,08,,,,,,,,,,,1.5,0.9,1.2,1")) == 1);
    gps03 = find_satellite(&data, NMEA_CONSTELLATION_GPS, 3u);
    assert(gps03 != NULL && gps03->used == 1u);
    assert(find_satellite(&data, NMEA_CONSTELLATION_GPS, 8u)->used == 1u);
    assert(find_satellite(&data, NMEA_CONSTELLATION_GPS, 1u)->used == 0u);

    assert(NMEA_Parse(&data, sentence(
        "GAGSV,1,1,02,11,30,100,37,21,45,200,42")) == 1);
    assert(data.satellites_visible == 7u);
    const NMEA_Satellite *gal21 = find_satellite(
        &data, NMEA_CONSTELLATION_GALILEO, 21u);
    assert(gal21 != NULL && gal21->snr_dbhz == 42u);

    /* A new GPS GSV cycle retires old GPS visibility, but not Galileo. */
    assert(NMEA_Parse(&data, sentence(
        "GPGSV,1,1,01,14,10,311,20")) == 1);
    assert(data.satellites_visible == 3u);
    assert(gps03->visible == 0u);
    assert(gal21->visible == 1u);

    puts("nmea_test: all assertions passed");
    return 0;
}
