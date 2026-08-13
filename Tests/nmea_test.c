#include "nmea.h"

#include <assert.h>
#include <stdio.h>

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

    puts("nmea_test: all assertions passed");
    return 0;
}
