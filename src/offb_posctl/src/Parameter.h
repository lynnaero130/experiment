//
// Created by zm on 18-12-1.
//

#ifndef OFFB_POSCTL_PARAMETER_H
#define OFFB_POSCTL_PARAMETER_H


class Parameter {

public:
    float pos_x;
    float pos_y;
    float pos_z;

    float x_p;
    float y_p;
    float z_p;

    float vx_p;
    float vy_p;
    float vz_p;

    float vx_i;
    float vy_i;
    float vz_i;

    float vx_d;
    float vy_d;
    float vz_d;

    float txp_p;
    float txp_i;
    float txp_d;
    float txv_p;
    float txv_i;
    float txv_d;

    float typ_p;
    float typ_i;
    float typ_d;
    float tyv_p;
    float tyv_i;
    float tyv_d;

    float tzp_p;
    float tzp_i;
    float tzp_d;
    float tzv_p;
    float tzv_i;
    float tzv_d;

    bool readParam(const char* addr);

};


#endif //OFFB_POSCTL_PARAMATER_H
