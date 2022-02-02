#ifndef WIDGET_H
#define WIDGET_H

#include   <QChartView>
#include "qcustomplot.h"
#include   <QSplineSeries>
using namespace QtCharts;
#include <QWidget>
#include "profile.h"
#include "mcalc.h"
#include <QLabel>
#include <QTimer>

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    void CalMain();
    void get_begin_time();
    void get_bind_para();
    //void getCurveData();
    void mstrcat(char * dst, const char * src);
    //int GetPrivateProfileString(QString strSectionName, QString strKeyName, QString strDefault, char * pReturnedValue, int size, QString strFileName);
    int StrCopy(QString source, char *dest, int size);
    int get_pulse_msg2(char *v_chBuf);
    void soutputProc(void);
    void get_cur_time();
    void getConfigData();
    void setDafultCurve();
    void initDraw();
    void initGlobal();
    void closeEvent(QCloseEvent *e);
    //void drawCurve();
    ~Widget();

private slots:
    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_pushButton_3_clicked();

    void on_pushButton_6_clicked();

    void on_pushButton_7_clicked();

    void on_pushButton_8_clicked();

    void on_pushButton_4_clicked();

    void on_comboBox_currentIndexChanged(int index);

    void slotPointHoverd(const QPointF &point, bool state);

    void on_pushButton_5_clicked();

private slots:
    void drawCurve();
    void on_pushButton_9_clicked();

private:
    Ui::Widget *ui;
    time_t tBeginTime, tCurTime;
    QLabel m_valueLabel;
    QVector<double> VphiE;
    double MinphiE,MaxphiE;
    QVector<double> VphiN;
    double MinphiN,MaxphiN;
    QVector<double> VphiU;
    double MinphiU,MaxphiU;
    QVector<double> VdvE;
    double MindvE,MaxdvE;
    QVector<double> VdvN;
    double MindvN,MaxdvN;
    QVector<double> VdvU;
    double MindvU,MaxdvU;
    QVector<double> VBGx;
    double MinBGx,MaxBGx;
    QVector<double> VBGy;
    double MinBGy,MaxBGy;
    QVector<double> VBGz;
    double MinBGz,MaxBGz;
    QVector<double> VBAx;
    double MinBAx,MaxBAx;
    QVector<double> VBAy;
    double MinBAy,MaxBAy;
    QVector<double> VBAz;
    double MinBAz,MaxBAz;
    QVector<double> VdKGx;
    double MindKGx,MaxdKGx;
    QVector<double> VdKGy;
    double MindKGy,MaxdKGy;
    QVector<double> VdKGz;
    double MindKGz,MaxdKGz;
    QVector<double> VdUGxy;
    double MindUGxy,MaxdUGxy;
    QVector<double> VdUGxz;
    double MindUGxz,MaxdUGxz;
    QVector<double> VdUGyz;
    double MindUGyz,MaxdUGyz;
    QVector<double> VdKAx;
    double MindKAx,MaxdKAx;
    QVector<double> VdKAy;
    double MindKAy,MaxdKAy;
    QVector<double> VdKAz;
    double MindKAz,MaxdKAz;
    QVector<double> VdUAxy;
    double MindUAxy,MaxdUAxy;
    QVector<double> VdUAxz;
    double MindUAxz,MaxdUAxz;
    QVector<double> VdUAyx;
    double MindUAyx,MaxdUAyx;
    QVector<double> VdUAyz;
    double MindUAyz,MaxdUAyz;
    QVector<double> VdUAzx;
    double MindUAzx,MaxdUAzx;
    QVector<double> VdUAzy;
    double MindUAzy,MaxdUAzy;
    QVector<double> VpphiE;
    double MinpphiE,MaxpphiE;
    QVector<double> VpphiN;
    double MinpphiN,MaxpphiN;
    QVector<double> VpphiU;
    double MinpphiU,MaxpphiU;
    QVector<double> VpdvE;
    double MinpdvE,MaxpdvE;
    QVector<double> VpdvN;
    double MinpdvN,MaxpdvN;
    QVector<double> VpdvU;
    double MinpdvU,MaxpdvU;
    QVector<double> VpBGx;
    double MinpBGx,MaxpBGx;
    QVector<double> VpBGy;
    double MinpBGy,MaxpBGy;
    QVector<double> VpBGz;
    double MinpBGz,MaxpBGz;
    QVector<double> VpBAx;
    double MinpBAx,MaxpBAx;
    QVector<double> VpBAy;
    double MinpBAy,MaxpBAy;
    QVector<double> VpBAz;
    double MinpBAz,MaxpBAz;
    QVector<double> VpdKGx;
    double MinpdKGx,MaxpdKGx;
    QVector<double> VpdKGy;
    double MinpdKGy,MaxpdKGy;
    QVector<double> VpdKGz;
    double MinpdKGz,MaxpdKGz;
    QVector<double> VpdUGxy;
    double MinpdUGxy,MaxpdUGxy;
    QVector<double> VpdUGxz;
    double MinpdUGxz,MaxpdUGxz;
    QVector<double> VpdUGyz;
    double MinpdUGyz,MaxpdUGyz;
    QVector<double> VpdKAx;
    double MinpdKAx,MaxpdKAx;
    QVector<double> VpdKAy;
    double MinpdKAy,MaxpdKAy;
    QVector<double> VpdKAz;
    double MinpdKAz,MaxpdKAz;
    QVector<double> VpdUAxy;
    double MinpdUAxy,MaxpdUAxy;
    QVector<double> VpdUAxz;
    double MinpdUAxz,MaxpdUAxz;
    QVector<double> VpdUAyx;
    double MinpdUAyx,MaxpdUAyx;
    QVector<double> VpdUAyz;
    double MinpdUAyz,MaxpdUAyz;
    QVector<double> VpdUAzx;
    double MinpdUAzx,MaxpdUAzx;
    QVector<double> VpdUAzy;
    double MinpdUAzy,MaxpdUAzy;
    QVector<double> VpointNumber;
    int     isRun;
    long pointNumber;
    int inde1IsNew;
    int inde2IsNew;
    int inde3IsNew;
    int inde4IsNew;
    int inde5IsNew;
    int inde6IsNew;
    int inde7IsNew;
    int inde8IsNew;
    int inde9IsNew;
    QTimer *qtime;
    int stopRun;
};
#endif // WIDGET_H
