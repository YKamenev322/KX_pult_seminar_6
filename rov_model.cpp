#include "rov_model.h"

#include <math.h>


ROV_Model::ROV_Model(QObject *parent) : QObject(parent) {
    resetModel();
    k_gamma = 0.3;
    m = 100;
    delta_m = 2;
    cv1[0] = 109; cv1[1] = 950; cv1[2] = 633;
    cv2[0] = 10.9; cv2[1] = 114; cv2[2] = 76;
    cw1[0] = 228.6; cw1[1] = 366; cw1[2] = 366; // kak v rabote Egorova
    cw2[0] = 2.29; cw2[1] = 36.6; cw2[2] = 36.6;
    lambda[1][1] = 50; lambda[2][2] = 101; lambda[3][3] = 101;
    lambda[4][4] = 50; lambda[5][5] = 50; lambda[6][6] = 50;
    J[0] = 4; J[1] = 19.8; J[2] = 19.8; //moment inercii apparata vdol sootvetstvuushih osei
    kd = 3; //koefficient usilenija dvizhitelei
    h = 0.018; //metacentricheskaya vysota
    Td = 0.15; //postojannaya vremeni dvizhitelei
    //koordinaty uporov dvizhitelei otnositelno centra mass apparata
    l1=0.14;
    l2=0.14;
    depth_limit=100;
    max_depth=100;
}

void ROV_Model::model(const float Umvl,const float Umnl,const float Umvp,const float Umnp) {
    int limit1, limit2;
    double G,delta_f;

    //модули упоров движителей
    Pmvp = a[7];  // маршевый верхний правый
    Pmvl = a[8];  // маршевый верхний левый
    Pmnp = a[9];  // маршевый нижний правый
    Pmnl = a[10];  //маршевый нижний левый

    //проекции упоров движителей на продольную ось апарата
    Pmvp_x = Pmvp;
    Pmnl_x = Pmnl;
    Pmvl_x = Pmvl;
    Pmnp_x = Pmnp;

    double g = 9.81;
    G = m*g; //вес аппарата
    delta_f = delta_m * g; //плавучесть (H)

    //obnulenie verticalnoi polozhitelnoi skorosti apparata pri dostizhenii poverhnosti
    limit1 = limit2 = 0;
    if (a[15] >= max_depth) {
      a[15] = max_depth;
      if (a[2] >= 0) {
          a[2] = 0;
          limit1 = 1;
      }
    };

    //obnulenie verticalnoi polozhitelnoi skorosti apparata pri dostizhenii dna
    if (a[15] <= 0)
    {
      a[15] = 0;
      if (a[2] <= 0)
      {
          a[2] = 0;
          limit2 = 1;
      }
    };

    X[103][0]=Fdx = Pmvp_x + Pmvl_x + Pmnp_x + Pmnl_x;
    Fgx = -cv1[0] * a[1] * fabs(a[1]) - cv2[0] * a[1];
    FloatageX = sin(a[6]) * delta_f;
    //FloatageX = 0; //обнуление остаточной плавучести
    da[1] = (1/(m + lambda[1][1])) * (Fdx + Fgx + FloatageX); //vx'

    Fdy = 0;
    Fgy = -cv1[1] * a[2] * fabs(a[2]) - cv2[1] * a[2];
    FloatageY = cos(a[6]) * cos(a[5]) * delta_f;
    //FloatageY = 0; //обнуление остаточной плавучести
    da[2] = (1/(m + lambda[2][2])) * (Fgy + Fdy + FloatageY); //vy'

    Fdz = 0;
    Fgz = -cv1[2] * a[3] * fabs(a[3]) - cv2[2] * a[3];
    FloatageZ = -cos(a[6]) * sin(a[5]) * delta_f;
    //FloatageZ = 0; //обнуление остаточной плавучести
    da[3] = (1/(m + lambda[3][3])) * (Fdz + Fgz + FloatageZ); //vz'

    da[4] = -(1/cos(a[6]) * ((-a[18]) * cos(a[5]) - sin(a[5]) * a[19]));  //proizvodnaya kursa

    da[5] = a[17] - tan(a[6]) * ((-a[18]) * cos(a[5]) - sin(a[5]) * a[19]);  //proizvodnaya krena

    da[6] = a[19] * cos(a[5]) + sin(a[5]) * (-a[18]); //proizvodnaya differenta

    X[17][0]=da[7] = (1/Td) * (kd * (double)Umvp - Pmvp);  // маршевый верхний правый

    da[8] = (1/Td) * (kd * (double)Umvl - Pmvl); //маршевый верхний левый

    da[9] = (1/Td) * (kd * (double)Umnp - Pmnp);  // маршевый нижний правый

    da[10] = (1/Td) * (kd * (double)Umnl - Pmnl);  //маршевый нижний левый

    da[11] = 0;

    da[12] = 0;

    da[13] = 0;

    double alfa[4][4]; //матрица перевода из связанной СК в глобальную СК
    a[4] = -a[4];
    alfa[1][1] = cos(a[4])*cos(a[6]);
    alfa[2][1] = sin(a[6]);
    alfa[3][1] = -sin(a[4])*cos(a[6]);
    alfa[1][2] = sin(a[5])*sin(a[4])-cos(a[5])*cos(a[4])*sin(a[6]);
    alfa[2][2] = cos(a[5])*cos(a[6]);
    alfa[3][2] = sin(a[5])*cos(a[4])+cos(a[5])*sin(a[4])*sin(a[6]);
    alfa[1][3] = cos(a[5])*sin(a[4])+sin(a[5])*cos(a[4])*sin(a[6]);
    alfa[2][3] = -sin(a[5])*cos(a[6]);
    alfa[3][3] = cos(a[5])*cos(a[4])-sin(a[4])*sin(a[5])*sin(a[6]);
    a[4] = -a[4];

    da[14] = alfa[1][1] * a[1] + alfa[1][2] * a[2] + alfa[1][3] * a[3];
    //dx_global

    da[15] = alfa[2][1] * a[1] + alfa[2][2] * a[2] + alfa[2][3] * a[3];
    //dy_global

    da[16] = alfa[3][1] * a[1] + alfa[3][2] * a[2] + alfa[3][3] * a[3];
    //dz_global

    double Fa = G + delta_f;
    double Fax = sin(a[6])*Fa;
    //float Fay = cos(a[5])*cos(a[6])*Fa;
    double Faz = -sin(a[5])*cos(a[6])*Fa;

    X[101][0]=Mdx = k_gamma*( Pmvp_x + Pmnl_x - Pmnp_x - Pmvl_x );
    Mgx = -cw1[0] * a[17] * fabs(a[17]) - cw2[0] * a[17];
    Max = Faz*h;
    //Max = 0; //obnulenie momenta ot sily Arhimeda
    da[17] = (1/(J[0] + lambda[4][4])) * (Mdx + Mgx + Max);

    X[105][0]=Pmvp_x;
    X[106][0] =Pmvl_x;
    X[107][0] = Pmnl_x;
    X[108][0] = Pmnp_x;

    X[100][0]=Mdy = l2*(-Pmvp_x + Pmvl_x + Pmnl_x - Pmnp_x);
    Mgy = -cw1[1] * a[18] * fabs(a[18]) - cw2[1] * a[18];
    da[18] = (1/(J[1] + lambda[5][5])) * (Mdy + Mgy);

    X[102][0]=Mdz = l1*(-Pmvp_x - Pmvl_x + Pmnl_x + Pmnp_x);
    Mgz = -cw1[2] * a[19] * fabs(a[19]) - cw2[2] * a[19];
    Maz = -h*Fax;
    //Maz = 0; //obnulenie momenta ot sily Arhimeda
    da[19] = (1/(J[2] + lambda[6][6])) * (Mdz + Mgz +Maz);

    da[20] = a[1];
    da[21] = a[2];
    da[22] = a[3];

  

}


void ROV_Model::resetModel(){
    for (int i=0;i<ANPA_MOD_CNT;i++) {a[i] = 0.0f; da[i]=0.0f;}
}


ROV_Model::~ROV_Model(){

}

void ROV_Model::runge(const float Umvl, const float Umnl, const float Umvp, const float Umnp, const float dt) {
    const double Kc = 180/M_PI;
    double a1[23], y[23];
    int i;
    const double H1 = dt;
    const int n = ANPA_MOD_CNT;
    model(Umvl,Umnl,Umvp,Umnp);
    for (i = 1; i < n; i++) {
      a1[i] = a[i];
      y[i] = da[i];
      a[i] = a1[i] + 0.5 * H1 * da[i];
    }

    model(Umvl,Umnl,Umvp,Umnp);
    for (i = 1; i < n; i++)
    {
      y[i] = y[i]+ 2 * da[i];
      a[i] = a1[i] + 0.5 * H1 * da[i];
    }
    model(Umvl,Umnl,Umvp,Umnp);
    for (i = 1; i < n; i++) {
      y[i] = y[i] + 2 * da[i];
      a[i] = a1[i] + H1 * da[i];
    }
    model(Umvl,Umnl,Umvp,Umnp);
    for (i = 1; i < n; i++) {
      a[i] = a1[i] + (H1 / 6) * (y[i] + da[i]);
    }


    //данные в СУ ( с преобразованием координат)

    x_global = a[14]; //koordinata apparata v globalnoi SK
    y_global = a[15];  //otstojanie ot dna otnositelno repernoi tochki, kotoraja na dne
    cur_depth = max_depth - y_global;  //tekush"aya glubina SPA
    z_global = a[16]; //koordinaty apparata v globalnoi SK (преобразование координат)
    Wx = a[17] * Kc; //uglovye skorosti SPA v svyazannyh osyah v gradus/sekunda
    Wy = a[18] * Kc;
    Wz = a[19] * Kc;

    vx_local = a[1]; vy_local = a[2]; vz_local = a[3];  //lineinye skorosti SPA v svyazannyh osyah
    vx_global = da[14]; vy_global = da[15]; vz_global = da[16];  // lineinye skorosti SPA v globalnyh osyah

    Psi_g = a[4] * Kc; // ugol kursa (преобразование координат)
    Gamma_g = a[5] * Kc; // ugol krena
    Tetta_g = a[6] * Kc; // ugol differenta
    W_Psi_g = da[4] * Kc; // proizvodnaya ugla kursa
    W_Gamma_g = da[5] * Kc; // proizvodnaya ugla krena
    W_Tetta_g = da[6] * Kc; // proizvodnaya ugla differenta




}



