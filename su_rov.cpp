#include "su_rov.h"

#include <math.h>

SU_ROV::SU_ROV(QObject *parent) : QObject(parent)
{
    K_Protocol = new Qkx_coeffs("protocols.conf", "ki");
    X_Protocol = new x_protocol("protocols.conf", "xi", X);

    T = 0.01;
    time.start(T*1000);
    connect(&time, SIGNAL(timeout()), SLOT(tick()));
}

void SU_ROV::tick()
{
    X[1][0]=K[1];

    getDataFromModel();

    yawControlChannel();

    BFS_DRK(X[49][0],0,0,0);

    model->runge(X[27][0],X[28][0],X[29][0],X[30][0],0.01);
}

void SU_ROV::getDataFromModel()
{
    X[32][0] = model->Fx;
    X[33][0] = model->vx_global;
    X[34][0] = model->vx_local;
    X[35][0] = model->x_global;
    X[36][0] = model->y_global;
    X[37][0] = model->z_global;
    X[39][0] = model->Tetta_g;
    X[40][0] = model->Gamma_g;
    X[42][0] = model->Psi_g;
    X[50][0] = model->W_Psi_g;
}

void SU_ROV::BFS_DRK(double Upsi, double Uteta, double Ugamma, double Ux)
{
    X[11][0] = saturation(Ux,       K[11]);
    X[12][0] = saturation(Upsi,     K[12]);
    X[13][0] = saturation(Uteta,    K[13]);
    X[14][0] = saturation(Ugamma,   K[14]);

    X[15][0] = X[11][0] + X[12][0];
    X[16][0] = X[11][0] + X[12][0];
    X[17][0] = X[11][0] - X[12][0];
    X[18][0] = X[11][0] - X[12][0];

    X[19][0] = X[15][0] - X[13][0];
    X[20][0] = X[16][0] + X[13][0];
    X[21][0] = X[17][0] - X[13][0];
    X[22][0] = X[18][0] + X[13][0];

    X[23][0] = X[19][0] - X[14][0];
    X[24][0] = X[20][0] + X[14][0];
    X[25][0] = X[21][0] + X[14][0];
    X[26][0] = X[22][0] - X[14][0];

    X[27][0] = saturation(X[23][0],K[23])*K[27];
    X[28][0] = saturation(X[24][0],K[24])*K[28];
    X[29][0] = saturation(X[25][0],K[25])*K[29];
    X[30][0] = saturation(X[26][0],K[26])*K[30];
}

void SU_ROV::yawControlChannel()
{
    X[41][0] = X[1][0];
    X[43][0] = X[41][0] - X[42][0];
    X[44][0] = X[43][0]*K[44];

    X[45][0] = X[50][0]*K[45];
    X[46][0] = X[44][0] - X[45][0] + X[41][0]*K[43];
    X[48][0] = X[46][0];
    X[49][0] = saturation(X[48][0],K[49]);
}

float SU_ROV::saturation(float input, float max)
{
    if (fabs(input)>= max)
        return (sign(input)*max);
    else return input;
}
