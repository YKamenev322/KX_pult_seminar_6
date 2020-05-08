#ifndef SU_ROV_H
#define SU_ROV_H

#include <QObject>
#include <QTimer>

#include "KX_Pult/kx_protocol.h"
#include "KX_Pult/qkx_coeffs.h"
#include "rov_model.h"

#define MODEL_N 3

extern double X[2000][2];
extern QVector<double> K;

class SU_ROV : public QObject
{
    Q_OBJECT
public:
    explicit SU_ROV(QObject *parent = nullptr);

    float saturation(float input, float max);
    int sign(float input){
        return (input>=0) ? 1 : -1;
    }

public slots:
    void tick();

signals:

private:
    Qkx_coeffs *K_Protocol;
    x_protocol *X_Protocol;

    ROV_Model *model;

    QTimer time;
    float T;

    void getDataFromModel();
    void BFS_DRK(double Upsi, double Uteta, double Ugamma, double Ux);
    void yawControlChannel();
};

#endif // SU_ROV_H
