#ifndef GEODETIC_CONVERT_H
#define GEODETIC_CONVERT_H

#include <Arduino.h>

class GeodeticConvert {
public:
    GeodeticConvert();

    // Инициализация
    bool isInitialised();
    void initialiseReference(double latitude, double longitude, double altitude);
    void getReference(double &latitude, double &longitude, double &altitude);

    // Преобразования
    void ecef2Ned(double x, double y, double z, double &north, double &east, double &down);
    void geodetic2Ecef(double latitude, double longitude, double altitude, double &x, double &y, double &z);
    void ecef2Geodetic(double x, double y, double z, double &latitude, double &longitude, double &altitude);
    void geodetic2Ned(double latitude, double longitude, double altitude, double &north, double &east, double &down);
    void ned2Geodetic(double north, double east, double down, double &latitude, double &longitude, double &altitude);
    
private:
    bool have_reference_;
    double initial_latitude_, initial_longitude_, initial_altitude_;
    double initial_ecef_x_, initial_ecef_y_, initial_ecef_z_;
    double ecef_to_ned_matrix_[3][3];
    double ned_to_ecef_matrix_[3][3];

    // Вспомогательные функции
    double deg2Rad(double degrees);
    double rad2Deg(double radians);
    void nRe(double lat_radians, double lon_radians, double matrix[3][3]);
};

#endif
