/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// main.cpp is a dummy application to ensure AP_NavEKF2 can be compiled and linked correctly.

#include <AP_NavEKF2/AP_NavEKF2.h>
#include <iostream>

int main (int argc, char *argv[])
{

    //TODO: need some simulation data
    NavEKF2 navEKF2;
    IMUData imuData[] = {IMUData(),IMUData(),IMUData(),IMUData(),};
    GpsData gpsData;

    MagnetoData magnetoData;//magnetometer data
    AirSpdData airSpdData;//air speed data, here air speed is true air speed
    airSpdData.airSpd_use = false;
    RngBcnData rngBcnData;//range beacon data
    BaroData baroData;//barometer data

    //Extended Kalman Filter initialise phase
    navEKF2.InitialiseFilter(20, 10, 4, imuData, gpsData, magnetoData, baroData);

    //Extended Kalman Filter update phase
    navEKF2.UpdateFilter(20, 0, imuData,gpsData, magnetoData, airSpdData, rngBcnData, baroData);

    Vector3f gyro_1;
    //Extended Kalman Filter predict phase
    navEKF2.getGyroBias(0, gyro_1);

    std::cout<<"gyro_1="<<gyro_1.x<<","<<gyro_1.y<<","<<gyro_1.z<<""<<std::endl;

    //Extended Kalman Filter update phase
    navEKF2.UpdateFilter(20, 0, imuData, gpsData, magnetoData, airSpdData, rngBcnData, baroData);

    Vector3f gyro_2;
    //Extended Kalman Filter predict phase
    navEKF2.getGyroBias(0, gyro_1);
    std::cout<<"gyro_2="<<gyro_2.x<<","<<gyro_2.y<<","<<gyro_2.z<<""<<std::endl;

    return 0;
}