#include "GeodeticConvert.h"
#include <math.h>

// Константы для расчётов
const double kSemimajorAxis = 6378137.0;
const double kSemiminorAxis = 6356752.3142;
const double kFirstEccentricitySquared = 6.69437999014e-3;
const double kSecondEccentricitySquared = 6.73949674228e-3;
const double kFlattening = 1 / 298.257223563;

GeodeticConvert::GeodeticConvert() {
    have_reference_ = false;
}

bool GeodeticConvert::isInitialised() {
    return have_reference_;
}

void GeodeticConvert::initialiseReference(double latitude, double longitude, double altitude) {
    initial_latitude_ = deg2Rad(latitude);
    initial_longitude_ = deg2Rad(longitude);
    initial_altitude_ = altitude;

    geodetic2Ecef(latitude, longitude, altitude, initial_ecef_x_, initial_ecef_y_, initial_ecef_z_);

    double phiP = atan2(initial_ecef_z_, sqrt(initial_ecef_x_ * initial_ecef_x_ + initial_ecef_y_ * initial_ecef_y_));
    
    nRe(phiP, initial_longitude_, ecef_to_ned_matrix_);
    nRe(initial_latitude_, initial_longitude_, ned_to_ecef_matrix_);

    // Транспонируем матрицу NED -> ECEF
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            ned_to_ecef_matrix_[i][j] = ecef_to_ned_matrix_[j][i];
        }
    }

    have_reference_ = true;
}

void GeodeticConvert::geodetic2Ecef(double latitude, double longitude, double altitude, double &x, double &y, double &z) {
    double lat_rad = deg2Rad(latitude);
    double lon_rad = deg2Rad(longitude);
    double xi = sqrt(1 - kFirstEccentricitySquared * sin(lat_rad) * sin(lat_rad));
    x = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * cos(lon_rad);
    y = (kSemimajorAxis / xi + altitude) * cos(lat_rad) * sin(lon_rad);
    z = (kSemimajorAxis / xi * (1 - kFirstEccentricitySquared) + altitude) * sin(lat_rad);
}

void GeodeticConvert::ecef2Geodetic(double x, double y, double z, double &latitude, double &longitude, double &altitude) {
    double r = sqrt(x * x + y * y);
    double Esq = kSemimajorAxis * kSemimajorAxis - kSemiminorAxis * kSemiminorAxis;
    double F = 54 * kSemiminorAxis * kSemiminorAxis * z * z;
    double G = r * r + (1 - kFirstEccentricitySquared) * z * z - kFirstEccentricitySquared * Esq;
    double C = (kFirstEccentricitySquared * kFirstEccentricitySquared * F * r * r) / (G * G * G);
    double S = pow(1 + C + sqrt(C * C + 2 * C), 1.0 / 3.0);
    double P = F / (3 * pow(S + 1 / S + 1, 2) * G * G);
    double Q = sqrt(1 + 2 * kFirstEccentricitySquared * kFirstEccentricitySquared * P);
    double r_0 = -(P * kFirstEccentricitySquared * r) / (1 + Q) + sqrt(0.5 * kSemimajorAxis * kSemimajorAxis * (1 + 1.0 / Q) - P * (1 - kFirstEccentricitySquared) * z * z / (Q * (1 + Q)) - 0.5 * P * r * r);
    double U = sqrt(pow(r - kFirstEccentricitySquared * r_0, 2) + z * z);
    double V = sqrt(pow(r - kFirstEccentricitySquared * r_0, 2) + (1 - kFirstEccentricitySquared) * z * z);
    double Z_0 = kSemiminorAxis * kSemiminorAxis * z / (kSemimajorAxis * V);
    altitude = U * (1 - kSemiminorAxis * kSemiminorAxis / (kSemimajorAxis * V));
    latitude = rad2Deg(atan((z + kSecondEccentricitySquared * Z_0) / r));
    longitude = rad2Deg(atan2(y, x));
}

void GeodeticConvert::ecef2Ned(double x, double y, double z, double &north, double &east, double &down) {
    double vect[3] = {x - initial_ecef_x_, y - initial_ecef_y_, z - initial_ecef_z_};
    north = ecef_to_ned_matrix_[0][0] * vect[0] + ecef_to_ned_matrix_[0][1] * vect[1] + ecef_to_ned_matrix_[0][2] * vect[2];
    east = ecef_to_ned_matrix_[1][0] * vect[0] + ecef_to_ned_matrix_[1][1] * vect[1] + ecef_to_ned_matrix_[1][2] * vect[2];
    down = -(ecef_to_ned_matrix_[2][0] * vect[0] + ecef_to_ned_matrix_[2][1] * vect[1] + ecef_to_ned_matrix_[2][2] * vect[2]);
}

void GeodeticConvert::geodetic2Ned(double latitude, double longitude, double altitude, double &north, double &east, double &down) {
    double x, y, z;
    geodetic2Ecef(latitude, longitude, altitude, x, y, z);
    ecef2Ned(x, y, z, north, east, down);
}

double GeodeticConvert::deg2Rad(double degrees) {
    return degrees * M_PI / 180.0;
}

double GeodeticConvert::rad2Deg(double radians) {
    return radians * 180.0 / M_PI;
}

void GeodeticConvert::nRe(double lat_radians, double lon_radians, double matrix[3][3]) {
    double sLat = sin(lat_radians);
    double cLat = cos(lat_radians);
    double sLon = sin(lon_radians);
    double cLon = cos(lon_radians);

    matrix[0][0] = -sLat * cLon;
    matrix[0][1] = -sLat * sLon;
    matrix[0][2] = cLat;
    matrix[1][0] = -sLon;
    matrix[1][1] = cLon;
    matrix[1][2] = 0.0;
    matrix[2][0] = cLat * cLon;
    matrix[2][1] = cLat * sLon;
    matrix[2][2] = sLat;
}
