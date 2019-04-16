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
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace std;

enum data_type {
    data_type_accel = 1,
    data_type_gyro,
    data_type_magneto,
    data_type_gps
};

struct accel_data {
    data_type type;
    uint64_t timestamp;
    double x;
    double y;
    double z;
};

typedef accel_data gyro_data;
typedef accel_data magnet_data;

struct gps_data {
    data_type type;
    double latitude;
    double longitude;
    double horizontal_accuracy;
    double vertical_accuracy;
    double altitude;
    double course;
    double true_heading;
    double magnetic_heading;
    double speed;
    uint64_t timestamp;

};

struct data_container {
    union{
        accel_data data1;
        gps_data data2;
    };
};

void read_csv_3_data(ifstream & in, vector<accel_data> & data_vector, data_type type){
    string line;
    string field;
    vector<string> fields;

    bool first_line = true;
    while (getline(in, line))
    {
        istringstream sin(line);
        vector<string> fields;
        string field;
        while (getline(sin, field, ','))
        {
            fields.push_back(field);
        }

        if(!first_line && fields.size() == 4){
            gyro_data data;
            stringstream ss1(fields[0]);
            ss1>>data.x;
            stringstream ss2(fields[1]);
            ss2>>data.y;
            stringstream ss3(fields[2]);
            ss3>>data.z;
            stringstream ss4(fields[3]);
            double ts;
            ss4>>ts;
            data.timestamp = ts*1e6;

            data.type = type;

            data_vector.insert(data_vector.end(), data);
        }

        first_line = false;
    }
}

void read_csv_10_data(ifstream & in, vector<gps_data> & data_vector, data_type type){
    string line;
    string field;
    vector<string> fields;

    bool first_line = true;
    while (getline(in, line))
    {
        istringstream sin(line);
        vector<string> fields;
        string field;
        while (getline(sin, field, ','))
        {
            fields.push_back(field);
        }

        if(!first_line && fields.size() == 10){
            gps_data data;
            stringstream ss1(fields[0]);
            ss1>>data.latitude;
            stringstream ss2(fields[1]);
            ss2>>data.longitude;
            stringstream ss3(fields[2]);
            ss3>>data.horizontal_accuracy;
            stringstream ss4(fields[3]);
            ss4>>data.vertical_accuracy;
            stringstream ss5(fields[4]);
            ss5>>data.altitude;
            stringstream ss6(fields[5]);
            ss6>>data.course;
            stringstream ss7(fields[6]);
            ss7>>data.true_heading;
            stringstream ss8(fields[7]);
            ss8>>data.magnetic_heading;
            stringstream ss9(fields[8]);
            ss9>>data.speed;
            stringstream ss10(fields[9]);

            double ts;
            ss10>>ts;

            data.timestamp = ts*1e6;

            data.type = type;

            if(data.horizontal_accuracy < 64)
                data_vector.insert(data_vector.end(), data);
        }

        first_line = false;
    }
}

int main (int argc, char *argv[])
{

    NavEKF2 navEKF2;
    IMUData imuData[] = {IMUData()};

    char gps_file[] = "./gps.log";//gps.log为手机生成的GPS历史信息，包含了时间戳，经纬度，速度，高度等信息
    char gyro_file[] = "./gps.gyro.log";//手机的gyro数据，包含了时间戳和x,y,z数据
    char accel_file[] = "./gps.accel.log";//手机记录下来的加速度数据，包含了时间戳和对应的x,y,z数据，已经去除了重力加速度的影响
    char magneto_file[] = "./gps.magneto.log";//手机的magnetometer数据，包含了时间戳和x,y,z数据

    ifstream gps_in(gps_file);
    ifstream gyro_in(gyro_file);
    ifstream accel_in(accel_file);
    ifstream magneto_in(magneto_file);

    vector<gps_data> gps_data_m;
    vector<gyro_data> gyro_data_m;
    vector<accel_data>  accel_data_m;
    vector<magnet_data> magnet_data_m;

    read_csv_10_data(gps_in, gps_data_m, data_type_gps);
    read_csv_3_data(gyro_in, gyro_data_m, data_type_gyro);
    read_csv_3_data(accel_in, accel_data_m, data_type_accel);
    read_csv_3_data(magneto_in, magnet_data_m, data_type_magneto);

    cout<<gps_data_m[0].timestamp<<endl;
    cout<<gyro_data_m[0].timestamp<<endl;
    cout<<accel_data_m[0].timestamp<<endl;
    cout<<magnet_data_m[0].timestamp<<endl;

    GpsData gpsData;
    gpsData.gps_have_vertical_velocity = false;
    gpsData.gps_vertical_accuracy_ok = false;
    gpsData.gps_horizontal_accuracy_ok = false;
    gpsData.gps_horizontal_accuracy = 5000.0;
    gpsData.gps_vertical_accuracy = 5000.0;

    MagnetoData magnetoData;//magnetometer data
    AirSpdData airSpdData;//air speed data, here air speed is true air speed
    airSpdData.airSpd_use = false;
    RngBcnData rngBcnData;//range beacon data
    rngBcnData.has_rngBcn = false;

    BaroData baroData;//barometer data
    baroData.baro_last_update = 0;
    baroData.baro_altitude = 0;

    uint32_t index_gps = 0, index_gyro = 0, index_accel = 0, index_magnet = 0;

    bool first_run = true;

    gps_data max_gps;
    max_gps.timestamp = -1;
    gyro_data max_gyro;
    max_gyro.timestamp = -1;
    accel_data max_accel;
    max_accel.timestamp = -1;
    magnet_data max_magnet;
    max_magnet.timestamp = -1;

    bool hal_ms_init = false;
    uint64_t hal_ms = 0;

    while (index_gps < gps_data_m.size()
           || index_gyro < gyro_data_m.size()
           || index_accel < accel_data_m.size()
           || index_magnet < magnet_data_m.size()) {

        gps_data gps = index_gps<gps_data_m.size()?gps_data_m[index_gps]:max_gps;
        gyro_data gyro = index_gyro<gyro_data_m.size()?gyro_data_m[index_gyro]:max_gyro;
        accel_data accel = index_accel<accel_data_m.size()?accel_data_m[index_accel]:max_accel;
        magnet_data magnet = index_magnet<magnet_data_m.size()?magnet_data_m[index_magnet]:max_magnet;

        uint64_t last_time_usec = 0;

        bool choose_gps = false;
        bool choose_gyro = false;
        bool choose_accel = false;
        bool choose_magnet = false;

        if(gps.timestamp < gyro.timestamp){
            //no gyro
            if(gps.timestamp < accel.timestamp){
                //no accel
                if(gps.timestamp < magnet.timestamp){
                    last_time_usec = gps.timestamp;
                    choose_gps = true;
                    index_gps = MIN(gps_data_m.size(), index_gps+1);
                }
                else{
                    last_time_usec = magnet.timestamp;
                    choose_magnet = true;
                    index_magnet =  MIN(magnet_data_m.size(), index_magnet+1);
                }
            }
            else{
                //no gps and gyro
                if(accel.timestamp < magnet.timestamp){
                    last_time_usec = accel.timestamp;
                    choose_accel = true;
                    index_accel = MIN(accel_data_m.size(), index_accel+1);
                }
                else{
                    last_time_usec = magnet.timestamp;
                    choose_magnet = true;
                    index_magnet =  MIN(magnet_data_m.size(), index_magnet+1);
                }
            }
        }
        else{
            //no gps
            if (gyro.timestamp < accel.timestamp){
                //no accel and gps
                if(gyro.timestamp < magnet.timestamp){
                    last_time_usec = gyro.timestamp;
                    choose_gyro = true;
                    index_gyro =  MIN(gyro_data_m.size(), index_gyro+1);
                }
                else{
                    last_time_usec = magnet.timestamp;
                    choose_magnet = true;
                    index_magnet =  MIN(magnet_data_m.size(), index_magnet+1);
                }
            }
            else{
                //no gyro and gps
                if(accel.timestamp < magnet.timestamp){
                    last_time_usec = accel.timestamp;
                    choose_accel = true;
                    index_accel = MIN(accel_data_m.size(), index_accel+1);
                }
                else{
                    last_time_usec = magnet.timestamp;
                    choose_magnet = true;
                    index_magnet =  MIN(magnet_data_m.size(), index_magnet+1);
                }
            }
        }

        memset(&imuData, 0, sizeof(imuData));
        memset(&magnetoData, 0, sizeof(magnetoData));
        memset(&gpsData, 0, sizeof(gpsData));

        if(!hal_ms_init){
            hal_ms_init = true;
            hal_ms = last_time_usec;
        }
        imuData[0].imuDataNew_delAngDT = 1.0;
        imuData[0].hal_millis = last_time_usec/1000 - hal_ms/1000;
        if(choose_accel){
            imuData[0].hal_millis = last_time_usec/1000 - hal_ms/1000;
        }

        imuData[0].ins_loop_delta_t = 0.01;
        imuData[0].ins_accelPosOffset.x = accel.x;
        imuData[0].ins_accelPosOffset.y = accel.y;
        imuData[0].ins_accelPosOffset.z = accel.z;

        if(choose_gyro){
            imuData[0].hal_millis = last_time_usec/1000 - hal_ms/1000;
        }
        imuData[0].ins_loop_delta_t = 0.01;
        imuData[0].imuDataNew_delAng.x = gyro.x;
        imuData[0].imuDataNew_delAng.y = gyro.y;
        imuData[0].imuDataNew_delAng.z = gyro.z;

        magnetoData.compass_count = 1;
        magnetoData.magneto_use = false;
        magnetoData.compass_last_update_usec = last_time_usec - hal_ms;
        magnetoData.learn_offsets_enabled = false;
        magnetoData.compass_primary_index = 0;
        magnetoData.compassData = new MagnetoCompassData();

        if(choose_magnet){
            magnetoData.magneto_use = true;
            magnetoData.compassData[0].compass_offset = Vector3f{0,0,0};
            magnetoData.compassData[0].last_update_usec = last_time_usec - hal_ms;
            magnetoData.compassData[0].use_for_yaw = true;
            magnetoData.compassData[0].compass_field = Vector3f{static_cast<float>(magnet.x),static_cast<float>(magnet.y),static_cast<float>(magnet.z)};
        }

#define FACTOR 1e7
        if(choose_gps){
            gpsData.gps_delay = 0;
            gpsData.gps_horizontal_accuracy = gps.horizontal_accuracy;
            gpsData.gps_last_message_time_ms = last_time_usec/1000 - hal_ms/1000;
            gpsData.gps_location.lng = gps.longitude*1e8;
            gpsData.gps_location.lat = gps.latitude*1e8;
            gpsData.gps_location.alt = gps.altitude*1e8;
            gpsData.gps_horizontal_accuracy = gps.horizontal_accuracy;
            gpsData.gps_vertical_accuracy = gps.vertical_accuracy;
            gpsData.gps_have_vertical_velocity = false;
            gpsData.gps_horizontal_accuracy_ok = gps.horizontal_accuracy < 65;
            gpsData.gps_status = GPS_OK_FIX_3D;
            gpsData.gps_num_sats = 5;

        }


        if(first_run){
            //Extended Kalman Filter initialise phase
            navEKF2.InitialiseFilter(last_time_usec, 10, 1, imuData, gpsData, magnetoData, baroData);
            first_run = false;
        }
        else{
            //Extended Kalman Filter update phase
            navEKF2.UpdateFilter(last_time_usec, 0, imuData, gpsData, magnetoData, airSpdData, rngBcnData, baroData);

            Vector3f velNed;
            navEKF2.getVelNED(0, velNed);

            Vector3f magNed;
            navEKF2.getMagNED(0, magNed);
            Vector3f magXYZ;
            navEKF2.getMagXYZ(0, magXYZ);

            cout<<"magnet="<<magnet.x<<","<<magnet.y<<","<<magnet.z<<""<<endl;
            cout<<"mag NED="<<magNed.x<<","<<magNed.y<<","<<magNed.z<<""<<endl;
            cout<<"mag XYZ="<<magXYZ.x<<","<<magXYZ.y<<","<<magXYZ.z<<""<<endl;
            cout<<"accel from csv="<<imuData[0].ins_accelPosOffset.x<<","<<imuData[0].ins_accelPosOffset.y<<","<<imuData[0].ins_accelPosOffset.z<<""<<endl;
            cout<<"velocity NED="<<velNed.x<<","<<velNed.y<<","<<velNed.z<<""<<endl<<endl;
        }
        delete magnetoData.compassData;

    }
    return 0;
}