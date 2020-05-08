#include "widget.h"
#include "su_rov.h"

#include <QApplication>

double X[2000][2];

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Widget w;
    w.show();

    SU_ROV SU;

    return a.exec();
}
