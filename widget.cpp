/************************************************************************\
功能: 标定参数
版本: V1.0
日期: 2022.01.26
作者: Sky
\************************************************************************/
 #include   <QChartView>
 #include   <QSplineSeries>
using namespace QtCharts;
#include "widget.h"

#include "ui_widget.h"
#include "common.h"
#include "matoper.h"
#include <QFile>
#include <QTextStream>
#include <iostream>
#include <fstream>
#include<set>
#include<sstream>
#include <QFileDialog>
#include <QCoreApplication>
#include <QDebug>
#include "mcalc.h"
#include <QMessageBox>
#include <QValueAxis>
#include "chartview.h"
#include <memory.h>
#include <QPen>
using namespace std;

ifstream inFileName;
ifstream inPulseMsg;
ifstream inImuInfo;
ifstream inRefMsg;
ifstream inGpsMsg;

FILE *fwNavRet;
FILE *fwNavInfo;
FILE *fwFilterInfo;
FILE *fwCheckInfo;
FILE *fwFileNameOut;

char dataPath[100];
#define LEN_PULSE_MSG	32u
#define LEN_SIM_TIME	(12*3600+0*1200)*UPDATE_FREQ
unsigned char ucFlagSimEnd = 0;
char fnNavRet[100];
char fnNavInfo[100];
char fnFilterInfo[100];
char fnCheckInfo[100];
char fnFileNameOut[100] = "filenameout.txt";
char fnFileName[100] = "filename.txt";
char fnImuInfo[100];
char fnRefMsg[100];
//#define PATHNAME "E:\\QTproject\\CalibrationPara\\"
char PATHNAME[100];
char strFileName[1000];
char strImuInfo[1000];
char strRefMsg[1000];
char strPulseMsg[LEN_PULSE_MSG];
typedef struct tag_CHECK_INFO
{
    int iReadCnt;
    int iImuCntCur;
    int iImuCntPre;
    int iCheckCnt;
    int iLostCnt;
    int bLost;
    int bCheck;
}CHECK_INFO_T;
typedef struct tag_NAV_ERR
{
    double dPosnOutErr[3], dVelOutErr[2];
    double dPosnCh1Err[3], dVelCh1Err[2];
    double dPosnRuleErr[3], dVelRuleErr[2];
    double dPosnCh2Err[3], dVelCh2Err[2];
}NAV_ERR_T;

CHECK_INFO_T			g_stCheckInfo;
NAV_ERR_T				g_stNavErr;


Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget),MinphiE(0),MaxphiE(0),MinphiN(0),MaxphiN(0),MinphiU(0),MaxphiU(0),MindvE(0),MaxdvE(0),MindvN(0),MaxdvN(0),MindvU(0),MaxdvU(0),MinBGx(0),MaxBGx(0),MinBGy(0),MaxBGy(0),
    MinBGz(0),MaxBGz(0),MinBAx(0),MaxBAx(0),MinBAy(0),MaxBAy(0),MinBAz(0),MaxBAz(0),MindKGx(0),MaxdKGx(0),MindKGy(0),MaxdKGy(0),MindKGz(0),MaxdKGz(0),MindUGxy(0),MaxdUGxy(0),MindUGxz(0),MaxdUGxz(0),
    MindUGyz(0),MaxdUGyz(0),MindKAx(0),MaxdKAx(0),MindKAy(0),MaxdKAy(0),MindKAz(0),MaxdKAz(0),MindUAxy(0),MaxdUAxy(0),MindUAxz(0),MaxdUAxz(0),MindUAyx(0),MaxdUAyx(0),MindUAyz(0),MaxdUAyz(0),
    MindUAzx(0),MaxdUAzx(0),MindUAzy(0),MaxdUAzy(0),MinpphiE(0),MaxpphiE(0),MinpphiN(0),MaxpphiN(0),MinpphiU(0),MaxpphiU(0),MinpdvE(0),MaxpdvE(0),MinpdvN(0),MaxpdvN(0),MinpdvU(0),MaxpdvU(0),
    MinpBGx(0),MaxpBGx(0),MinpBGy(0),MaxpBGy(0),MinpBGz(0),MaxpBGz(0),MinpBAx(0),MaxpBAx(0),MinpBAy(0),MaxpBAy(0),MinpBAz(0),MaxpBAz(0),MinpdKGx(0),MaxpdKGx(0),MinpdKGy(0),MaxpdKGy(0),
    MinpdKGz(0),MaxpdKGz(0),MinpdUGxy(0),MaxpdUGxy(0),MinpdUGxz(0),MaxpdUGxz(0),MinpdUGyz(0),MaxpdUGyz(0),MinpdKAx(0),MaxpdKAx(0),MinpdKAy(0),MaxpdKAy(0),MinpdKAz(0),MaxpdKAz(0),
    MinpdUAxy(0),MaxpdUAxy(0),MinpdUAxz(0),MaxpdUAxz(0),MinpdUAyx(0),MaxpdUAyx(0),MinpdUAyz(0),MaxpdUAyz(0),MinpdUAzx(0),MaxpdUAzx(0),MinpdUAzy(0),MaxpdUAzy(0),pointNumber(0),isRun(0),inde1IsNew(1),
    inde2IsNew(1),inde3IsNew(1),inde4IsNew(1),inde5IsNew(1),inde6IsNew(1),inde7IsNew(1),inde8IsNew(1),inde9IsNew(1)
{
    ui->setupUi(this);
    qtime = new QTimer(this);
    qtime->setInterval(1000);
    //ui->comboBox->setEnabled(0);



    QString buff = QCoreApplication::applicationDirPath()+ "/";

    //strcpy(PATHNAME,buff.toUtf8());
    strcpy(PATHNAME,buff.toLocal8Bit().data());
    ui->lineEdit->setText(buff.toUtf8());
    getConfigData();
    on_pushButton_6_clicked();
    ui->label_33->setText("等待计算");
    initDraw();
    qtime->start();
    connect(qtime,SIGNAL(timeout()),this,SLOT(drawCurve()));
   //QSplineSeries* line = new QSplineSeries();
    on_pushButton_7_clicked();
    on_pushButton_8_clicked();

}

Widget::~Widget()
{
    qtime->stop();
    delete qtime;
    delete ui;
}


void Widget::get_begin_time()
{
    struct tm *pBeginTime;
    time(&tBeginTime);
    pBeginTime = localtime(&tBeginTime);
    printf ("begin time:%02d-%02d-%02d %02d:%02d:%02d\r\n",pBeginTime->tm_year+1900,pBeginTime->tm_mon,pBeginTime->tm_mday,pBeginTime->tm_hour,pBeginTime->tm_min,pBeginTime->tm_sec);

    char buffer[100];
    sprintf(buffer,"begin time:%02d-%02d-%02d %02d:%02d:%02d\r\n",pBeginTime->tm_year+1900,pBeginTime->tm_mon+1,pBeginTime->tm_mday,pBeginTime->tm_hour,pBeginTime->tm_min,pBeginTime->tm_sec);
//    QString oldString = ui->textEdit->toPlainText();
//    oldString = oldString + QString(buffer);
//    ui->textEdit->clear();
    ui->textEdit->append(buffer);

    sprintf(fnNavRet, "navret1s%02d%02d%02d_%02d.%02d.%02d.txt", (pBeginTime->tm_year+1900)%100,pBeginTime->tm_mon,pBeginTime->tm_mday,pBeginTime->tm_hour,pBeginTime->tm_min,pBeginTime->tm_sec);
    sprintf(fnNavInfo, "navinfo1s%02d%02d%02d_%02d.%02d.%02d.txt", (pBeginTime->tm_year+1900)%100,pBeginTime->tm_mon,pBeginTime->tm_mday,pBeginTime->tm_hour,pBeginTime->tm_min,pBeginTime->tm_sec);
    sprintf(fnFilterInfo, "flrinfo1s%02d%02d%02d_%02d.%02d.%02d.txt", (pBeginTime->tm_year+1900)%100,pBeginTime->tm_mon,pBeginTime->tm_mday,pBeginTime->tm_hour,pBeginTime->tm_min,pBeginTime->tm_sec);
    sprintf(fnCheckInfo, "check%02d%02d%02d_%02d.%02d.%02d.txt", (pBeginTime->tm_year+1900)%100,pBeginTime->tm_mon,pBeginTime->tm_mday,pBeginTime->tm_hour,pBeginTime->tm_min,pBeginTime->tm_sec);
}
void Widget::mstrcat(char * dst, const char * src)
{
    char stmp[100];
    strcpy(stmp, src);
    strcat(stmp, dst);
    strcpy(dst, stmp);
}

void Widget::getConfigData()
{

    char fnBindFile[100] = "config.ini";
    char strAppName[100];
    char strValue[100];

    mstrcat(fnBindFile, PATHNAME);
    //mstrcat(fnBindFile, PATHNAME);

    strcpy(strAppName, "AttiZero");
    GetPrivateProfileString(strAppName, "SysR0", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dSysAttiZero[0] = atof(strValue)*DEG2RAD;
    ui->lineEdit_SysR0->setText(strValue);

    GetPrivateProfileString(strAppName, "SysP0", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dSysAttiZero[1] = atof(strValue)*DEG2RAD;
    ui->lineEdit_SysP0->setText(strValue);

    GetPrivateProfileString(strAppName, "SysH0", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dSysAttiZero[2] = atof(strValue)*DEG2RAD;
    ui->lineEdit_SysH0->setText(strValue);

    atti2dmat(g_stNavData.dCb1b, g_stBindPara.dSysAttiZero);


    /***************************************************************/

    GetPrivateProfileString(strAppName, "ImuR0", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_ImuR0->setText(strValue);

    GetPrivateProfileString(strAppName, "ImuP0", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_ImuP0->setText(strValue);

    GetPrivateProfileString(strAppName, "ImuH0", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_ImuH0->setText(strValue);

    GetPrivateProfileString(strAppName, "OutR0", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_OutR0->setText(strValue);

    GetPrivateProfileString(strAppName, "OutP0", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_OutP0->setText(strValue);

    GetPrivateProfileString(strAppName, "OutH0", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_OutH0->setText(strValue);
    /***************************************************************/

    strcpy(strAppName, "ParaInit");
    GetPrivateProfileString(strAppName, "LatInit", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_LatInit->setText(strValue);

    GetPrivateProfileString(strAppName, "LonInit", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_LonInit->setText(strValue);

    GetPrivateProfileString(strAppName, "VelInit", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_VelInit->setText(strValue);

    GetPrivateProfileString(strAppName, "HeadingInit", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_HeadingInit->setText(strValue);
    /***************************************************************/

    strcpy(strAppName, "LeverArm");
    GetPrivateProfileString(strAppName, "X1", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_X1->setText(strValue);

    GetPrivateProfileString(strAppName, "Y1", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_Y1->setText(strValue);

    GetPrivateProfileString(strAppName, "Z1", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_Z1->setText(strValue);

    GetPrivateProfileString(strAppName, "X2", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_X2->setText(strValue);

    GetPrivateProfileString(strAppName, "Y2", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_Y2->setText(strValue);

    GetPrivateProfileString(strAppName, "Z2", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_Z2->setText(strValue);

    GetPrivateProfileString(strAppName, "X3", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_X3->setText(strValue);

    GetPrivateProfileString(strAppName, "Y3", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_Y3->setText(strValue);

    GetPrivateProfileString(strAppName, "Z3", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_Z3->setText(strValue);

    GetPrivateProfileString(strAppName, "X4", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_X4->setText(strValue);

    GetPrivateProfileString(strAppName, "Y4", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_Y4->setText(strValue);

    GetPrivateProfileString(strAppName, "Z4", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_Z4->setText(strValue);

    GetPrivateProfileString(strAppName, "X5", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_X5->setText(strValue);

    GetPrivateProfileString(strAppName, "Y5", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_Y5->setText(strValue);

    GetPrivateProfileString(strAppName, "Z5", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_Z5->setText(strValue);

    GetPrivateProfileString(strAppName, "X6", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_X6->setText(strValue);

    GetPrivateProfileString(strAppName, "Y6", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_Y6->setText(strValue);

    GetPrivateProfileString(strAppName, "Z6", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_Z6->setText(strValue);
    /***************************************************************/

    strcpy(strAppName, "CaliPara");
    GetPrivateProfileString(strAppName, "KGX", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dKgBind[0] = atof(strValue)*DPH2RPS;
    ui->lineEdit_KGX->setText(strValue);

    GetPrivateProfileString(strAppName, "KGY", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dKgBind[1] = atof(strValue)*DPH2RPS;
    ui->lineEdit_KGY->setText(strValue);

    GetPrivateProfileString(strAppName, "KGZ", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dKgBind[2] = atof(strValue)*DPH2RPS;
    ui->lineEdit_KGZ->setText(strValue);

    GetPrivateProfileString(strAppName, "BGX", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dBgBind[0] = atof(strValue)*DPH2RPS;
    ui->lineEdit_BGX->setText(strValue);

    GetPrivateProfileString(strAppName, "BGY", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dBgBind[1] = atof(strValue)*DPH2RPS;
    ui->lineEdit_BGY->setText(strValue);

    GetPrivateProfileString(strAppName, "BGZ", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dBgBind[2] = atof(strValue)*DPH2RPS;
    ui->lineEdit_BGZ->setText(strValue);

    g_stBindPara.dCgpBind[0] = 1.0;
    GetPrivateProfileString(strAppName, "UGXY", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCgpBind[1] = atof(strValue)*MIN2RAD;
    ui->lineEdit_UGXY->setText(strValue);

    GetPrivateProfileString(strAppName, "UGXZ", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCgpBind[2] = atof(strValue)*MIN2RAD;
    ui->lineEdit_UGXZ->setText(strValue);

    GetPrivateProfileString(strAppName, "UGYX", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCgpBind[3] = atof(strValue)*MIN2RAD;
    ui->lineEdit_UGYX->setText(strValue);

    g_stBindPara.dCgpBind[4] = 1.0;
    GetPrivateProfileString(strAppName, "UGYZ", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCgpBind[5] = atof(strValue)*MIN2RAD;
    ui->lineEdit_UGYZ->setText(strValue);

    GetPrivateProfileString(strAppName, "UGZX", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCgpBind[6] = atof(strValue)*MIN2RAD;
    ui->lineEdit_UGZX->setText(strValue);

    GetPrivateProfileString(strAppName, "UGZY", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCgpBind[7] = atof(strValue)*MIN2RAD;
    ui->lineEdit_UGZY->setText(strValue);

    g_stBindPara.dCgpBind[8] = 1.0;

    GetPrivateProfileString(strAppName, "KAX", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dKaBind[0] = atof(strValue)*1.0e-6;
    ui->lineEdit_KAX->setText(strValue);

    GetPrivateProfileString(strAppName, "KAY", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dKaBind[1] = atof(strValue)*1.0e-6;
    ui->lineEdit_KAY->setText(strValue);

    GetPrivateProfileString(strAppName, "KAZ", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dKaBind[2] = atof(strValue)*1.0e-6;
    ui->lineEdit_KAZ->setText(strValue);

    GetPrivateProfileString(strAppName, "BAX", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dBaBind[0] = atof(strValue)*1; //1.0e-3*GLV_G;
    ui->lineEdit_BAX->setText(strValue);

    GetPrivateProfileString(strAppName, "BAY", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dBaBind[1] = atof(strValue)*1; //1.0e-3*GLV_G;
    ui->lineEdit_BAY->setText(strValue);

    GetPrivateProfileString(strAppName, "BAZ", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dBaBind[2] = atof(strValue)*1; //1.0e-3*GLV_G;
    ui->lineEdit_BAZ->setText(strValue);

    g_stBindPara.dCgpBind[0] = 1.0;

    GetPrivateProfileString(strAppName, "UAXY", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCapBind[1] = atof(strValue)*MIN2RAD;
    ui->lineEdit_UAXY->setText(strValue);

    GetPrivateProfileString(strAppName, "UAXZ", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCapBind[2] = atof(strValue)*MIN2RAD;
    ui->lineEdit_UAXZ->setText(strValue);

    GetPrivateProfileString(strAppName, "UAYX", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCapBind[3] = atof(strValue)*MIN2RAD;
    ui->lineEdit_UAYX->setText(strValue);

    g_stBindPara.dCapBind[4] = 1.0;
    GetPrivateProfileString(strAppName, "UAYZ", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCapBind[5] = atof(strValue)*MIN2RAD;
    ui->lineEdit_UAYZ->setText(strValue);

    GetPrivateProfileString(strAppName, "UAZX", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCapBind[6] = atof(strValue)*MIN2RAD;
    ui->lineEdit_UAZX->setText(strValue);

    GetPrivateProfileString(strAppName, "UAZY", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCapBind[7] = atof(strValue)*MIN2RAD;
    ui->lineEdit_UAZY->setText(strValue);

    g_stBindPara.dCapBind[8] = 1.0;

    GetPrivateProfileString(strAppName, "BGE", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dBgEqu[0] = atof(strValue)*DPH2RPS;
    ui->lineEdit_BGE->setText(strValue);


    GetPrivateProfileString(strAppName, "BGN", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dBgEqu[1] = atof(strValue)*DPH2RPS;
    ui->lineEdit_BGN->setText(strValue);

    GetPrivateProfileString(strAppName, "BGU", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dBgEqu[2] = atof(strValue)*DPH2RPS;
    ui->lineEdit_BGU->setText(strValue);


    strcpy(strAppName, "TempCoef");
    GetPrivateProfileString(strAppName, "BGXB0", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[0][0] = (float)atof(strValue);
    ui->lineEdit_BGXB0->setText(strValue);

    GetPrivateProfileString(strAppName, "BGYB0", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[0][1] = (float)atof(strValue);
    ui->lineEdit_BGYB0->setText(strValue);

    GetPrivateProfileString(strAppName, "BGZB0", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[0][2] = (float)atof(strValue);
    ui->lineEdit_BGZB0->setText(strValue);

    GetPrivateProfileString(strAppName, "BGXB1", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[1][0] = (float)atof(strValue);
    ui->lineEdit_BGXB1->setText(strValue);

    GetPrivateProfileString(strAppName, "BGYB1", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[1][1] = (float)atof(strValue);
    ui->lineEdit_BGYB1->setText(strValue);

    GetPrivateProfileString(strAppName, "BGZB1", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[1][2] = (float)atof(strValue);
    ui->lineEdit_BGZB1->setText(strValue);



    GetPrivateProfileString(strAppName, "BGXB2", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[2][0] = (float)atof(strValue);
    ui->lineEdit_BGXB2->setText(strValue);

    GetPrivateProfileString(strAppName, "BGYB2", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[2][1] = (float)atof(strValue);
    ui->lineEdit_BGYB2->setText(strValue);

    GetPrivateProfileString(strAppName, "BGZB2", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[2][2] = (float)atof(strValue);
    ui->lineEdit_BGZB2->setText(strValue);

    GetPrivateProfileString(strAppName, "BGXB3", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[3][0] = (float)atof(strValue);
    ui->lineEdit_BGXB3->setText(strValue);

    GetPrivateProfileString(strAppName, "BGYB3", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[3][1] = (float)atof(strValue);
    ui->lineEdit_BGYB3->setText(strValue);

    GetPrivateProfileString(strAppName, "BGZB3", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[3][2] = (float)atof(strValue);
    ui->lineEdit_BGZB3->setText(strValue);

    GetPrivateProfileString(strAppName, "BAXB0", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[0][0] = (float)atof(strValue);
    ui->lineEdit_BAXB0->setText(strValue);

    GetPrivateProfileString(strAppName, "BAYB0", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[0][1] = (float)atof(strValue);
    ui->lineEdit_BAYB0->setText(strValue);

    GetPrivateProfileString(strAppName, "BAZB0", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[0][2] = (float)atof(strValue);
    ui->lineEdit_BAZB0->setText(strValue);

    GetPrivateProfileString(strAppName, "BAXB1", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[1][0] = (float)atof(strValue);
    ui->lineEdit_BAXB1->setText(strValue);

    GetPrivateProfileString(strAppName, "BAYB1", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[1][1] = (float)atof(strValue);
    ui->lineEdit_BAYB1->setText(strValue);

    GetPrivateProfileString(strAppName, "BAZB1", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[1][2] = (float)atof(strValue);
    ui->lineEdit_BAZB1->setText(strValue);

    GetPrivateProfileString(strAppName, "BAXB2", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[2][0] = (float)atof(strValue);
    ui->lineEdit_BAXB2->setText(strValue);

    GetPrivateProfileString(strAppName, "BAYB2", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[2][1] = (float)atof(strValue);
    ui->lineEdit_BAYB2->setText(strValue);

    GetPrivateProfileString(strAppName, "BAZB2", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[2][2] = (float)atof(strValue);
    ui->lineEdit_BAZB2->setText(strValue);

    GetPrivateProfileString(strAppName, "BAXB3", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[3][0] = (float)atof(strValue);
    ui->lineEdit_BAXB3->setText(strValue);

    GetPrivateProfileString(strAppName, "BAYB3", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[3][1] = (float)atof(strValue);
    ui->lineEdit_BAYB3->setText(strValue);

    GetPrivateProfileString(strAppName, "BAZB3", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[3][2] = (float)atof(strValue);
    ui->lineEdit_BAZB3->setText(strValue);
}


void Widget::get_bind_para()
{
    char fnBindFile[100] = "config.ini";
    char strAppName[100];
    char strValue[100];
    mstrcat(fnBindFile, PATHNAME);

    strcpy(strAppName, "AttiZero");
    GetPrivateProfileString(strAppName, "SysR0", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dSysAttiZero[0] = atof(strValue)*DEG2RAD;
    GetPrivateProfileString(strAppName, "SysP0", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dSysAttiZero[1] = atof(strValue)*DEG2RAD;
    GetPrivateProfileString(strAppName, "SysH0", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dSysAttiZero[2] = atof(strValue)*DEG2RAD;
    atti2dmat(g_stNavData.dCb1b, g_stBindPara.dSysAttiZero);

    strcpy(strAppName, "CaliPara");
    GetPrivateProfileString(strAppName, "KGX", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dKgBind[0] = atof(strValue)*DPH2RPS;
    GetPrivateProfileString(strAppName, "KGY", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dKgBind[1] = atof(strValue)*DPH2RPS;

    GetPrivateProfileString(strAppName, "KGZ", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dKgBind[2] = atof(strValue)*DPH2RPS;
    GetPrivateProfileString(strAppName, "BGX", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dBgBind[0] = atof(strValue)*DPH2RPS;
    GetPrivateProfileString(strAppName, "BGY", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dBgBind[1] = atof(strValue)*DPH2RPS;
    GetPrivateProfileString(strAppName, "BGZ", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dBgBind[2] = atof(strValue)*DPH2RPS;
    g_stBindPara.dCgpBind[0] = 1.0;
    GetPrivateProfileString(strAppName, "UGXY", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCgpBind[1] = atof(strValue)*MIN2RAD;
    GetPrivateProfileString(strAppName, "UGXZ", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCgpBind[2] = atof(strValue)*MIN2RAD;
    GetPrivateProfileString(strAppName, "UGYX", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCgpBind[3] = atof(strValue)*MIN2RAD;
    g_stBindPara.dCgpBind[4] = 1.0;
    GetPrivateProfileString(strAppName, "UGYZ", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCgpBind[5] = atof(strValue)*MIN2RAD;
    GetPrivateProfileString(strAppName, "UGZX", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCgpBind[6] = atof(strValue)*MIN2RAD;
    GetPrivateProfileString(strAppName, "UGZY", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCgpBind[7] = atof(strValue)*MIN2RAD;
    g_stBindPara.dCgpBind[8] = 1.0;

    GetPrivateProfileString(strAppName, "KAX", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dKaBind[0] = atof(strValue)*1.0e-6;
    GetPrivateProfileString(strAppName, "KAY", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dKaBind[1] = atof(strValue)*1.0e-6;
    GetPrivateProfileString(strAppName, "KAZ", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dKaBind[2] = atof(strValue)*1.0e-6;
    GetPrivateProfileString(strAppName, "BAX", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dBaBind[0] = atof(strValue)*1; //1.0e-3*GLV_G;
    GetPrivateProfileString(strAppName, "BAY", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dBaBind[1] = atof(strValue)*1; //1.0e-3*GLV_G;
    GetPrivateProfileString(strAppName, "BAZ", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dBaBind[2] = atof(strValue)*1; //1.0e-3*GLV_G;
    g_stBindPara.dCgpBind[0] = 1.0;
    GetPrivateProfileString(strAppName, "UAXY", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCapBind[1] = atof(strValue)*MIN2RAD;
    GetPrivateProfileString(strAppName, "UAXZ", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCapBind[2] = atof(strValue)*MIN2RAD;
    GetPrivateProfileString(strAppName, "UAYX", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCapBind[3] = atof(strValue)*MIN2RAD;
    g_stBindPara.dCapBind[4] = 1.0;
    GetPrivateProfileString(strAppName, "UAYZ", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCapBind[5] = atof(strValue)*MIN2RAD;
    GetPrivateProfileString(strAppName, "UAZX", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCapBind[6] = atof(strValue)*MIN2RAD;
    GetPrivateProfileString(strAppName, "UAZY", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dCapBind[7] = atof(strValue)*MIN2RAD;
    g_stBindPara.dCapBind[8] = 1.0;

    GetPrivateProfileString(strAppName, "BGE", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dBgEqu[0] = atof(strValue)*DPH2RPS;
    GetPrivateProfileString(strAppName, "BGN", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dBgEqu[1] = atof(strValue)*DPH2RPS;
    GetPrivateProfileString(strAppName, "BGU", NULL, strValue, 100, fnBindFile);
    g_stBindPara.dBgEqu[2] = atof(strValue)*DPH2RPS;

    strcpy(strAppName, "TempCoef");
    GetPrivateProfileString(strAppName, "BGXB0", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[0][0] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BGYB0", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[0][1] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BGZB0", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[0][2] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BGXB1", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[1][0] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BGYB1", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[1][1] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BGZB1", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[1][2] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BGXB2", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[2][0] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BGYB2", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[2][1] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BGZB2", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[2][2] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BGXB3", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[3][0] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BGYB3", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[3][1] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BGZB3", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaGyroBias[3][2] = (float)atof(strValue);

    GetPrivateProfileString(strAppName, "BAXB0", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[0][0] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BAYB0", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[0][1] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BAZB0", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[0][2] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BAXB1", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[1][0] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BAYB1", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[1][1] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BAZB1", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[1][2] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BAXB2", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[2][0] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BAYB2", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[2][1] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BAZB2", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[2][2] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BAXB3", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[3][0] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BAYB3", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[3][1] = (float)atof(strValue);
    GetPrivateProfileString(strAppName, "BAZB3", NULL, strValue, 100, fnBindFile);
    g_stBindPara.fBetaAccZero[3][2] = (float)atof(strValue);

}
int Widget::get_pulse_msg2(char *v_chBuf)
{
    int i;
    unsigned char ucCheckSum = 0;
    float fPulse1[6];

    memcpy(&fPulse1[0], v_chBuf+6, 24);
    memcpy(&g_stPulseMsg.nSysTime, v_chBuf+2, 4);
    g_stPulseMsg.ucImuCode[g_stPulseMsg.nSysTime%400] = v_chBuf[30];

    g_stPulseMsg.dPulseGyro[0] = (double)(fPulse1[0]);
    g_stPulseMsg.dPulseGyro[1] = (double)(fPulse1[1]);
    g_stPulseMsg.dPulseGyro[2] = (double)(fPulse1[2]);
    g_stPulseMsg.dPulseAcc[0] = (double)(fPulse1[3]);
    g_stPulseMsg.dPulseAcc[1] = (double)(fPulse1[4]);
    g_stPulseMsg.dPulseAcc[2] = (double)(fPulse1[5]);
    //g_stPulseMsg.dPulseAcc[0] = (double)(fPulse1[5]);
    //g_stPulseMsg.dPulseAcc[1] = (double)(fPulse1[4]);
    //g_stPulseMsg.dPulseAcc[2] = (double)(fPulse1[3]);
    if (g_stPulseMsg.nSysTime%400 == 108)
    {
        memcpy(&g_stPulseMsg.n_gyro_temp[0], g_stPulseMsg.ucImuCode+84, 24);
        for (i=0; i<6; i++)
            g_stInertData.fTempGyro[i] = (float)g_stPulseMsg.n_gyro_temp[i]*0.0001f;
    }
    if (g_stPulseMsg.nSysTime%400 == 120)
    {
        memcpy(&g_stPulseMsg.n_accl_temp[0], g_stPulseMsg.ucImuCode+108, 12);
        for (i=0; i<3; i++)
            g_stInertData.fTempAcc[i] = (float)g_stPulseMsg.n_accl_temp[i]*0.0001f;
    }

    for (i=2; i<LEN_PULSE_MSG-1; i++)
        ucCheckSum += *(v_chBuf+i);

    if (ucCheckSum == *(v_chBuf+i))
        g_stCheckInfo.bCheck = 0;
    else
        g_stCheckInfo.bCheck = 1;
    g_stCheckInfo.iReadCnt++;
    g_stCheckInfo.iImuCntCur = g_stPulseMsg.nSysTime;
    if (g_stCheckInfo.iReadCnt <= 1)
        g_stCheckInfo.iImuCntPre = g_stCheckInfo.iImuCntCur-1;
    g_stCheckInfo.bLost = g_stCheckInfo.iImuCntCur-g_stCheckInfo.iImuCntPre-1;
    g_stCheckInfo.iLostCnt += g_stCheckInfo.bLost;
    g_stCheckInfo.iImuCntPre = g_stCheckInfo.iImuCntCur;
    g_stCheckInfo.iCheckCnt += g_stCheckInfo.bCheck;
    return(g_stCheckInfo.bCheck);
}

void Widget::soutputProc(void)
{
    double cosL = cos(g_stRefMsg.fGpsPosn[0]);

    if (g_uiCntClock%(UPDATE_FREQ) == 0)
    {
        g_stNavErr.dPosnOutErr[0] = (g_stNavData.dPosnOut[0]-g_stRefMsg.dPosnRef[0])*RAD2MIN;
        g_stNavErr.dPosnOutErr[1] = (g_stNavData.dPosnOut[1]-g_stRefMsg.dPosnRef[1])*RAD2MIN;
        g_stNavErr.dPosnOutErr[2] = sqrt(g_stNavErr.dPosnOutErr[1]*g_stNavErr.dPosnOutErr[1]*cosL*cosL+g_stNavErr.dPosnOutErr[0]*g_stNavErr.dPosnOutErr[0]);
        g_stNavErr.dVelOutErr[0] = g_stNavData.dVelOut[0]-g_stRefMsg.fGpsVel[0];
        g_stNavErr.dVelOutErr[1] = g_stNavData.dVelOut[1]-g_stRefMsg.fGpsVel[1];

        g_stNavErr.dPosnCh1Err[0] = (g_stNavData.dPosnCh1[0]-g_stRefMsg.dPosnRef[0])*RAD2MIN;
        g_stNavErr.dPosnCh1Err[1] = (g_stNavData.dPosnCh1[1]-g_stRefMsg.dPosnRef[1])*RAD2MIN;
        g_stNavErr.dPosnCh1Err[2] = sqrt(g_stNavErr.dPosnCh1Err[1]*g_stNavErr.dPosnCh1Err[1]*cosL*cosL+g_stNavErr.dPosnCh1Err[0]*g_stNavErr.dPosnCh1Err[0]);
        g_stNavErr.dVelCh1Err[0] = g_stNavData.dVelCh1[0]-g_stRefMsg.fGpsVel[0];
        g_stNavErr.dVelCh1Err[1] = g_stNavData.dVelCh1[1]-g_stRefMsg.fGpsVel[1];

        //////////////////////////////////////////////////////////////////////////
        fprintf (fwNavRet, "%d\t%02x\t", g_stPulseMsg.nSysTime, g_stNavData.ucWorkMode); // 1 2
        fprintf (fwNavRet, "%.6lf\t%.6lf\t%.6lf\t", g_stNavData.dAttiOut[0]*RAD2DEG, g_stNavData.dAttiOut[1]*RAD2DEG, g_stNavData.dAttiOut[2]*RAD2DEG); // 3 4 5
        fprintf (fwNavRet, "%.4lf\t%.4lf\t%.4lf\t", g_stNavData.dVelOut[0], g_stNavData.dVelOut[1], g_stNavData.dVelOut[2]); // 6 7 8
        fprintf (fwNavRet, "%.6lf\t%.6lf\t%.6lf\t", g_stNavData.dPosnOut[0]*RAD2DEG, g_stNavData.dPosnOut[1]*RAD2DEG, g_stNavData.dPosnOut[2]); // 9 10 11
        fprintf (fwNavRet, "%.6lf\t%.6lf\t%.6lf\t", g_stNavData.dAttiCh1[0]*RAD2DEG, g_stNavData.dAttiCh1[1]*RAD2DEG, g_stNavData.dAttiCh1[2]*RAD2DEG); // 12 13 14
        fprintf (fwNavRet, "%.4lf\t%.4lf\t", g_stNavData.dVelCh1[0], g_stNavData.dVelCh1[1]); // 15 16
        fprintf (fwNavRet, "%.6lf\t%.6lf\t", g_stNavData.dPosnCh1[0]*RAD2DEG, g_stNavData.dPosnCh1[1]*RAD2DEG); // 17 18
        fprintf (fwNavRet, "%.6lf\t%.6lf\t%.6lf\t", g_stNavData.dAttiOut[0]*RAD2DEG, g_stNavData.dAttiOut[1]*RAD2DEG, g_stNavData.dAttiOut[2]*RAD2DEG); // 19 20 21
        fprintf (fwNavRet, "%.4lf\t%.4lf\t", g_stNavData.dVelOut[0], g_stNavData.dVelOut[1]); // 22 23
        fprintf (fwNavRet, "%.6lf\t%.6lf\t", g_stNavData.dPosnOut[0]*RAD2DEG, g_stNavData.dPosnOut[1]*RAD2DEG); // 24 25
        fprintf (fwNavRet, "%.6lf\t%.6lf\t%.6lf\t", g_stNavData.dAttiCh1[0]*RAD2DEG, g_stNavData.dAttiCh1[1]*RAD2DEG, g_stNavData.dAttiCh1[2]*RAD2DEG); // 26 27 28
        fprintf (fwNavRet, "%.4lf\t%.4lf\t", g_stNavData.dVelCh1[0], g_stNavData.dVelCh1[1]); // 29 30
        fprintf (fwNavRet, "%.6lf\t%.6lf\t", g_stNavData.dPosnCh1[0]*RAD2DEG, g_stNavData.dPosnCh1[1]*RAD2DEG); // 31 32
        fprintf (fwNavRet, "%.4lf\t%.4lf\t%.4lf\n", g_stNavData.dOmegaAtti[0]*RAD2DEG, g_stNavData.dOmegaAtti[1]*RAD2DEG, g_stNavData.dOmegaAtti[2]*RAD2DEG); // 33 34 35
        //////////////////////////////////////////////////////////////////////////
        fprintf (fwNavInfo, "%d\t%02x\t", g_stPulseMsg.nSysTime, g_stNavData.ucWorkMode); // 1 2
        fprintf (fwNavInfo, "%.6lf\t%.6lf\t%.6lf\t", g_stNavData.dAttiImu1[0]*RAD2DEG, g_stNavData.dAttiImu1[1]*RAD2DEG, g_stNavData.dAttiImu1[2]*RAD2DEG); // 3 4 5
        fprintf (fwNavInfo, "%.6lf\t%.6lf\t%.6lf\t", g_stNavData.dAttiImu1[0]*RAD2DEG, g_stNavData.dAttiImu1[1]*RAD2DEG, g_stNavData.dAttiImu1[2]*RAD2DEG); // 6 7 8
        fprintf (fwNavInfo, "%.4lf\t%.4lf\t", g_stNavErr.dVelOutErr[0], g_stNavErr.dVelOutErr[1]); // 9 10
        fprintf (fwNavInfo, "%.4lf\t%.4lf\t%.4lf\t", g_stNavErr.dPosnOutErr[0], g_stNavErr.dPosnOutErr[1], g_stNavErr.dPosnOutErr[2]); // 11 12 13
        fprintf (fwNavInfo, "%.4lf\t%.4lf\t", g_stNavErr.dVelCh1Err[0], g_stNavErr.dVelCh1Err[1]); // 14 15
        fprintf (fwNavInfo, "%.4lf\t%.4lf\t%.4lf\t", g_stNavErr.dPosnCh1Err[0], g_stNavErr.dPosnCh1Err[1], g_stNavErr.dPosnCh1Err[2]); // 16 17 18
        fprintf (fwNavInfo, "%.4lf\t%.4lf\t", g_stNavErr.dVelRuleErr[0], g_stNavErr.dVelRuleErr[1]); // 19 20
        fprintf (fwNavInfo, "%.4lf\t%.4lf\t%.4lf\t", g_stNavErr.dPosnRuleErr[0], g_stNavErr.dPosnRuleErr[1], g_stNavErr.dPosnRuleErr[2]); // 21 22 23
        fprintf (fwNavInfo, "%.4lf\t%.4lf\t", g_stNavErr.dVelCh2Err[0], g_stNavErr.dVelCh2Err[1]); // 24 25
        fprintf (fwNavInfo, "%.4lf\t%.4lf\t%.4lf\t", g_stNavErr.dPosnCh2Err[0], g_stNavErr.dPosnCh2Err[1], g_stNavErr.dPosnCh2Err[2]); // 26 27 28
        fprintf (fwNavInfo, "%.4f\t%.4f\t%.4f\t", g_stInertData.fTempGyro[0], g_stInertData.fTempGyro[1], g_stInertData.fTempGyro[2]); // 29 30 31
        fprintf (fwNavInfo, "%.4f\t%.4f\t%.4f\n", g_stInertData.fTempAcc[0], g_stInertData.fTempAcc[1], g_stInertData.fTempAcc[2]); // 32 33 34
        //////////////////////////////////////////////////////////////////////////
/*
        g_dLocalG = local_g(g_stNavData.dPosnOut[0]);
        fprintf (fwCheckInfo, "%.6lf\t%.6lf\t%.6lf\t", g_dAttiSeries1s[0][LEN_CHECK-1]*RAD2DEG, g_dAttiSeries1s[1][LEN_CHECK-1]*RAD2DEG, g_dAttiSeries1s[2][LEN_CHECK-1]*RAD2DEG);
        fprintf (fwCheckInfo, "%.6lf\t%.6lf\t", g_dForceTotal1s[LEN_CHECK-1], g_localg);
        fprintf (fwCheckInfo, "%.4lf\t%.4lf\t", g_dVelSeries1s[0][LEN_CHECK-1], g_dVelSeries1s[1][LEN_CHECK-1]);
        fprintf (fwCheckInfo, "%d\t%d\t", g_dwKStatic, g_dwKMoor);
        fprintf (fwCheckInfo, "%d\t%d\t%d\t%d\t%d\t%d\t%d\n", g_byMotionSta, g_byForceChange, g_byVelChange[0], g_byVelChange[1], g_byAttiChange[0], g_byAttiChange[1], g_byAttiChange[2]);
*/

        //////////////////////////////////////////////////////////////////////////
        if ((g_stNavData.ucWorkMode == U8_WORK_SELFALIGN_FINE) || (g_stNavData.ucWorkMode == U8_WORK_INTENAV_GPS))
        {
            fprintf (fwFilterInfo, "%d\t%02x\t", g_stPulseMsg.nSysTime, g_stNavData.ucWorkMode); // 1  2
            fprintf (fwFilterInfo, "%.6lf\t%.6lf\t%.6lf\t", g_stKfPara.dGyroBiasEst[0]*RPS2DPH, g_stKfPara.dGyroBiasEst[1]*RPS2DPH, g_stKfPara.dGyroBiasEst[2]*RPS2DPH); // 3 4 5
            fprintf (fwFilterInfo, "%.6lf\t%.6lf\t%.6lf\t", g_stKfPara.dAccBiasEst[0]*1000/GLV_G, g_stKfPara.dAccBiasEst[1]*1000/GLV_G, g_stKfPara.dAccBiasEst[2]*1000/GLV_G); // 6 7 8
            /*
            fprintf (fwFilterInfo, "%.6lf\t%.6lf\t%.6lf\t", sqrt(g_stKfPara.dPCalc[0*15+0])*RAD2MIN, sqrt(g_stKfPara.dPCalc[1*15+1])*RAD2MIN, sqrt(g_stKfPara.dPCalc[2*15+2])*RAD2MIN);// 9 10 11
            fprintf (fwFilterInfo, "%.6lf\t%.6lf\t%.6lf\t", sqrt(g_stKfPara.dPCalc[3*15+3]), sqrt(g_stKfPara.dPCalc[4*15+4]), sqrt(g_stKfPara.dPCalc[5*15+5])); // 12 13 14
            fprintf (fwFilterInfo, "%.6lf\t%.6lf\t%.6lf\t", sqrt(g_stKfPara.dPCalc[6*15+6])*RAD2MIN, sqrt(g_stKfPara.dPCalc[7*15+7])*RAD2MIN, sqrt(g_stKfPara.dPCalc[8*15+8])); // 15 16 17
            fprintf (fwFilterInfo, "%.6lf\t%.6lf\t%.6lf\t", sqrt(g_stKfPara.dPCalc[9*15+9])*RPS2DPH, sqrt(g_stKfPara.dPCalc[10*15+10])*RPS2DPH, sqrt(g_stKfPara.dPCalc[11*15+11])*RPS2DPH); // 18 19 20
            fprintf (fwFilterInfo, "%.6lf\t%.6lf\t%.6lf\n", sqrt(g_stKfPara.dPCalc[12*15+12])*1.0e3/GLV_G, sqrt(g_stKfPara.dPCalc[13*15+13])*1.0e3/GLV_G, sqrt(g_stKfPara.dPCalc[14*15+14])*1.0e3/GLV_G); // 21 22 23
            */
            fprintf (fwFilterInfo, "%.6lf\t%.6lf\t%.6lf\t", sqrt(g_stKfPara.dPCalc[0*12+0])*RAD2MIN, sqrt(g_stKfPara.dPCalc[1*12+1])*RAD2MIN, sqrt(g_stKfPara.dPCalc[2*12+2])*RAD2MIN);// 9 10 11
            fprintf (fwFilterInfo, "%.6lf\t%.6lf\t%.6lf\t", sqrt(g_stKfPara.dPCalc[3*12+3]), sqrt(g_stKfPara.dPCalc[4*12+4]), 0); // 12 13 14
            fprintf (fwFilterInfo, "%.6lf\t%.6lf\t%.6lf\t", sqrt(g_stKfPara.dPCalc[5*12+5])*RAD2MIN, sqrt(g_stKfPara.dPCalc[6*12+6])*RAD2MIN, 0); // 15 16 17
            fprintf (fwFilterInfo, "%.6lf\t%.6lf\t%.6lf\t", sqrt(g_stKfPara.dPCalc[7*12+7])*RPS2DPH, sqrt(g_stKfPara.dPCalc[8*12+8])*RPS2DPH, sqrt(g_stKfPara.dPCalc[9*12+9])*RPS2DPH); // 18 19 20
            fprintf (fwFilterInfo, "%.6lf\t%.6lf\t%.6lf\n", sqrt(g_stKfPara.dPCalc[10*12+10])*1.0e3/GLV_G, sqrt(g_stKfPara.dPCalc[11*12+11])*1.0e3/GLV_G, 0); // 21 22 23
        }
        if (g_stNavData.ucWorkMode == U8_WORK_CALIBRATE_SYS)
        {
            fprintf (fwCheckInfo, "%.6lf\t%.6lf\t%.6lf\t", g_stBDPara.dXCalc[0]*RAD2MIN, g_stBDPara.dXCalc[1]*RAD2MIN, g_stBDPara.dXCalc[2]*RAD2MIN);
            VphiE.append(g_stBDPara.dXCalc[0]*RAD2MIN); VphiN.append(g_stBDPara.dXCalc[1]*RAD2MIN); VphiU.append(g_stBDPara.dXCalc[2]*RAD2MIN);

            fprintf (fwCheckInfo, "%.4lf\t%.4lf\t%.4lf\t", g_stBDPara.dXCalc[3], g_stBDPara.dXCalc[4], g_stBDPara.dXCalc[5]);
            VdvE.append(g_stBDPara.dXCalc[3]);      VdvN.append(g_stBDPara.dXCalc[4]);       VdvU.append(g_stBDPara.dXCalc[5]);

            fprintf (fwCheckInfo, "%.6lf\t%.6lf\t%.6lf\t", g_stBDPara.dXCalc[6]*RPS2DPH, g_stBDPara.dXCalc[7]*RPS2DPH, g_stBDPara.dXCalc[8]*RPS2DPH);
            VBGx.append(g_stBDPara.dXCalc[6]*RPS2DPH);   VBGy.append(g_stBDPara.dXCalc[7]*RPS2DPH);   VBGz.append(g_stBDPara.dXCalc[8]*RPS2DPH);

            fprintf (fwCheckInfo, "%.6lf\t%.6lf\t%.6lf\t", g_stBDPara.dXCalc[9]/GLV_G*1000.0, g_stBDPara.dXCalc[10]/GLV_G*1000.0, g_stBDPara.dXCalc[11]/GLV_G*1000.0);
            VBAx.append(g_stBDPara.dXCalc[9]/GLV_G*1000.0);   VBAy.append(g_stBDPara.dXCalc[10]/GLV_G*1000.0);   VBAz.append(g_stBDPara.dXCalc[11]/GLV_G*1000.0);

            fprintf (fwCheckInfo, "%.6lf\t%.6lf\t%.6lf\t", g_stBDPara.dXCalc[12]*1000000.0, g_stBDPara.dXCalc[13]*1000000.0, g_stBDPara.dXCalc[14]*1000000.0);
            VdKGx.append(g_stBDPara.dXCalc[12]*1000000.0);   VdKGy.append(g_stBDPara.dXCalc[13]*1000000.0);   VdKGz.append(g_stBDPara.dXCalc[14]*1000000.0);


            fprintf (fwCheckInfo, "%.6lf\t%.6lf\t%.6lf\t", g_stBDPara.dXCalc[15]*RAD2MIN, g_stBDPara.dXCalc[16]*RAD2MIN, g_stBDPara.dXCalc[17]*RAD2MIN);
            VdUGxy.append(g_stBDPara.dXCalc[15]*RAD2MIN);   VdUGxz.append(g_stBDPara.dXCalc[16]*RAD2MIN);   VdUGyz.append(g_stBDPara.dXCalc[17]*RAD2MIN);

            fprintf (fwCheckInfo, "%.6lf\t%.6lf\t%.6lf\t", g_stBDPara.dXCalc[18]*1000000.0, g_stBDPara.dXCalc[19]*1000000.0, g_stBDPara.dXCalc[20]*1000000.0);
            VdKAx.append(g_stBDPara.dXCalc[18]*1000000.0);   VdKAy.append(g_stBDPara.dXCalc[19]*1000000.0);   VdKAz.append(g_stBDPara.dXCalc[20]*1000000.0);

            fprintf (fwCheckInfo, "%.6lf\t%.6lf\t%.6lf\t", g_stBDPara.dXCalc[21]*RAD2MIN, g_stBDPara.dXCalc[22]*RAD2MIN, g_stBDPara.dXCalc[23]*RAD2MIN);
            VdUAxy.append(g_stBDPara.dXCalc[21]*RAD2MIN);   VdUAxz.append(g_stBDPara.dXCalc[22]*RAD2MIN);   VdUAyx.append(g_stBDPara.dXCalc[23]*RAD2MIN);

            fprintf (fwCheckInfo, "%.6lf\t%.6lf\t%.6lf\t", g_stBDPara.dXCalc[24]*RAD2MIN, g_stBDPara.dXCalc[25]*RAD2MIN, g_stBDPara.dXCalc[26]*RAD2MIN);
            VdUAyz.append(g_stBDPara.dXCalc[24]*RAD2MIN);   VdUAzx.append(g_stBDPara.dXCalc[25]*RAD2MIN);   VdUAzy.append(g_stBDPara.dXCalc[26]*RAD2MIN);


            fprintf (fwCheckInfo, "%.6lf\t%.6lf\t%.6lf\t", sqrt(g_stBDPara.dPCalc[0+27*0])*RAD2MIN, sqrt(g_stBDPara.dPCalc[1+27*1])*RAD2MIN, sqrt(g_stBDPara.dPCalc[2+27*2])*RAD2MIN);
            VpphiE.append(sqrt(g_stBDPara.dPCalc[0+27*0])*RAD2MIN);   VpphiN.append(sqrt(g_stBDPara.dPCalc[1+27*1])*RAD2MIN);   VpphiU.append(sqrt(g_stBDPara.dPCalc[2+27*2])*RAD2MIN);

            fprintf (fwCheckInfo, "%.4lf\t%.4lf\t%.4lf\t", sqrt(g_stBDPara.dPCalc[3+27*3]), sqrt(g_stBDPara.dPCalc[4+27*4]), sqrt(g_stBDPara.dPCalc[5+27*5]));
            VpdvE.append(sqrt(g_stBDPara.dPCalc[3+27*3]));   VpdvN.append(sqrt(g_stBDPara.dPCalc[4+27*4]));   VpdvU.append(sqrt(g_stBDPara.dPCalc[5+27*5]));

            fprintf (fwCheckInfo, "%.6lf\t%.6lf\t%.6lf\t", sqrt(g_stBDPara.dPCalc[6+27*6])*RPS2DPH, sqrt(g_stBDPara.dPCalc[7+27*7])*RPS2DPH, sqrt(g_stBDPara.dPCalc[8+27*8])*RPS2DPH);
            VpBGx.append(sqrt(g_stBDPara.dPCalc[6+27*6])*RPS2DPH);   VpBGy.append(sqrt(g_stBDPara.dPCalc[7+27*7])*RPS2DPH);   VpBGz.append(sqrt(g_stBDPara.dPCalc[8+27*8])*RPS2DPH);

            fprintf (fwCheckInfo, "%.6lf\t%.6lf\t%.6lf\t", sqrt(g_stBDPara.dPCalc[9+27*9])/GLV_G*1000.0, sqrt(g_stBDPara.dPCalc[10+27*10])/GLV_G*1000.0, sqrt(g_stBDPara.dPCalc[11+27*11])/GLV_G*1000.0);
            VpBAx.append(sqrt(g_stBDPara.dPCalc[9+27*9])/GLV_G*1000.0);   VpBAy.append(sqrt(g_stBDPara.dPCalc[10+27*10])/GLV_G*1000.0);   VpBAz.append(sqrt(g_stBDPara.dPCalc[11+27*11])/GLV_G*1000.0);

            fprintf (fwCheckInfo, "%.6lf\t%.6lf\t%.6lf\t", sqrt(g_stBDPara.dPCalc[12+27*12])*1000000.0, sqrt(g_stBDPara.dPCalc[13+27*13])*1000000.0, sqrt(g_stBDPara.dPCalc[14+27*14])*1000000.0);
            VpdKGx.append(sqrt(g_stBDPara.dPCalc[12+27*12])*1000000.0);   VpdKGy.append(sqrt(g_stBDPara.dPCalc[13+27*13])*1000000.0);   VpdKGz.append(sqrt(g_stBDPara.dPCalc[14+27*14])*1000000.0);

            fprintf (fwCheckInfo, "%.6lf\t%.6lf\t%.6lf\t", sqrt(g_stBDPara.dPCalc[15+27*15])*RAD2MIN, sqrt(g_stBDPara.dPCalc[16+27*16])*RAD2MIN, sqrt(g_stBDPara.dPCalc[17+27*17])*RAD2MIN);
            VpdUGxy.append(sqrt(g_stBDPara.dPCalc[15+27*15])*RAD2MIN);   VpdUGxz.append(sqrt(g_stBDPara.dPCalc[16+27*16])*RAD2MIN);   VpdUGyz.append(sqrt(g_stBDPara.dPCalc[17+27*17])*RAD2MIN);

            fprintf (fwCheckInfo, "%.6lf\t%.6lf\t%.6lf\t", sqrt(g_stBDPara.dPCalc[18+27*18])*1000000.0, sqrt(g_stBDPara.dPCalc[19+27*19])*1000000.0, sqrt(g_stBDPara.dPCalc[20+27*20])*1000000.0);
            VpdKAx.append(sqrt(g_stBDPara.dPCalc[18+27*18])*1000000.0);   VpdKAy.append(sqrt(g_stBDPara.dPCalc[19+27*19])*1000000.0);   VpdKAz.append(sqrt(g_stBDPara.dPCalc[20+27*20])*1000000.0);

            fprintf (fwCheckInfo, "%.6lf\t%.6lf\t%.6lf\t", sqrt(g_stBDPara.dPCalc[21+27*21])*RAD2MIN, sqrt(g_stBDPara.dPCalc[22+27*22])*RAD2MIN, sqrt(g_stBDPara.dPCalc[23+27*23])*RAD2MIN);
            VpdUAxy.append(sqrt(g_stBDPara.dPCalc[21+27*21])*RAD2MIN);   VpdUAxz.append(sqrt(g_stBDPara.dPCalc[22+27*22])*RAD2MIN);   VpdUAyx.append(sqrt(g_stBDPara.dPCalc[23+27*23])*RAD2MIN);

            fprintf (fwCheckInfo, "%.6lf\t%.6lf\t%.6lf\n", sqrt(g_stBDPara.dPCalc[24+27*24])*RAD2MIN, sqrt(g_stBDPara.dPCalc[25+27*25])*RAD2MIN, sqrt(g_stBDPara.dPCalc[26+27*26])*RAD2MIN);
            VpdUAyz.append(sqrt(g_stBDPara.dPCalc[24+27*24])*RAD2MIN);   VpdUAzx.append(sqrt(g_stBDPara.dPCalc[25+27*25])*RAD2MIN);   VpdUAzy.append(sqrt(g_stBDPara.dPCalc[26+27*26])*RAD2MIN);
            VpointNumber.append(pointNumber);
            pointNumber++;

        }

    //	ClearState();

    }

/*
    if (g_uiCntClock % (1*60*UPDATE_FREQ) == 0)
    {
        printf ("wx=%.6f wy=%.6f wz=%.6f\n",g_stImuInfo.fAngIncP[0]*RPS2DPH,g_stImuInfo.fAngIncP[1]*RPS2DPH,g_stImuInfo.fAngIncP[2]*RPS2DPH);
        printf ("fx=%.6f fy=%.6f fz=%.6f\n",g_stImuInfo.fVelIncP[0]/60.0,g_stImuInfo.fVelIncP[1]/60.0,g_stImuInfo.fVelIncP[2]/60.0);

        for (int i=0; i<3; i++)
        {
            g_stImuInfo.fAngIncR[i] = 0;
            g_stImuInfo.fVelIncR[i] = 0;
            g_stImuInfo.fAngIncP[i] = 0;
            g_stImuInfo.fVelIncP[i] = 0;
        }

    }
*/
    if (g_uiCntClock%(60*5*UPDATE_FREQ) == 0)
    {
        get_cur_time();
        char buffer[100];
        QString oldString;
        printf("%dmin, T=%.2f℃, work=0x%02X, meas=0x%02X, k_part=%dmin, k_sta=%dmin\n", g_uiCntClock/60/UPDATE_FREQ, g_stInertData.fTempGyro[2], g_stNavData.ucWorkMode, g_stNavData.ucMeasMode, g_stNavData.uiKPart/60/UPDATE_FREQ, g_stNavData.uiKSta/60/UPDATE_FREQ);
        sprintf(buffer,"%dmin, T=%.2f℃, work=0x%02X, meas=0x%02X, k_part=%dmin, k_sta=%dmin\n", g_uiCntClock/60/UPDATE_FREQ, g_stInertData.fTempGyro[2], g_stNavData.ucWorkMode, g_stNavData.ucMeasMode, g_stNavData.uiKPart/60/UPDATE_FREQ, g_stNavData.uiKSta/60/UPDATE_FREQ);
        oldString = ui->textEdit->toPlainText();
        oldString = oldString + QString(buffer);
        ui->textEdit->clear();
        ui->textEdit->append(oldString);

        printf("k_damp=%dmin, k_nodamp=%dmin \n",g_stNavData.uiKDamp/60/UPDATE_FREQ, g_stNavData.uiKNoDamp/60/UPDATE_FREQ);
        sprintf(buffer,"k_damp=%dmin, k_nodamp=%dmin \n",g_stNavData.uiKDamp/60/UPDATE_FREQ, g_stNavData.uiKNoDamp/60/UPDATE_FREQ);
        oldString = ui->textEdit->toPlainText();
        oldString = oldString + QString(buffer);
        ui->textEdit->clear();
        ui->textEdit->append(oldString);

        printf("k_damp=%dmin, k_nodamp=%dmin \n",g_stNavData.uiKDamp/60/UPDATE_FREQ, g_stNavData.uiKNoDamp/60/UPDATE_FREQ);
        sprintf(buffer,"%dmin, T=%.2f℃, work=0x%02X, meas=0x%02X, k_part=%dmin, k_sta=%dmin\n", g_uiCntClock/60/UPDATE_FREQ, g_stInertData.fTempGyro[2], g_stNavData.ucWorkMode, g_stNavData.ucMeasMode, g_stNavData.uiKPart/60/UPDATE_FREQ, g_stNavData.uiKSta/60/UPDATE_FREQ);
        oldString = ui->textEdit->toPlainText();
        oldString = oldString + QString(buffer);
        ui->textEdit->clear();
        ui->textEdit->append(oldString);

        printf("R=%.6lf P=%.6lf H=%.6lf\n", g_stNavData.dAttiOut[0]*RAD2DEG, g_stNavData.dAttiOut[1]*RAD2DEG, g_stNavData.dAttiOut[2]*RAD2DEG);
        sprintf(buffer,"R=%.6lf P=%.6lf H=%.6lf\n", g_stNavData.dAttiOut[0]*RAD2DEG, g_stNavData.dAttiOut[1]*RAD2DEG, g_stNavData.dAttiOut[2]*RAD2DEG);
        oldString = ui->textEdit->toPlainText();
        oldString = oldString + QString(buffer);
        ui->textEdit->clear();
        ui->textEdit->append(oldString);

        printf("R=%.6lf P=%.6lf H=%.6lf\n", g_stNavData.dAttiCh1[0]*RAD2DEG, g_stNavData.dAttiCh1[1]*RAD2DEG, g_stNavData.dAttiCh1[2]*RAD2DEG);
        sprintf(buffer,"R=%.6lf P=%.6lf H=%.6lf\n", g_stNavData.dAttiCh1[0]*RAD2DEG, g_stNavData.dAttiCh1[1]*RAD2DEG, g_stNavData.dAttiCh1[2]*RAD2DEG);
        oldString = ui->textEdit->toPlainText();
        oldString = oldString + QString(buffer);
        ui->textEdit->clear();
        ui->textEdit->append(oldString);


        printf("vE1=%.4lf vN1=%.4lf Lat1=%.6lf Lon1=%.6lf\n", g_stNavData.dVelOut[0]*MPS2KN, g_stNavData.dVelOut[1]*MPS2KN, g_stNavData.dPosnOut[0]*RAD2DEG, g_stNavData.dPosnOut[1]*RAD2DEG);
        sprintf(buffer,"vE1=%.4lf vN1=%.4lf Lat1=%.6lf Lon1=%.6lf\n", g_stNavData.dVelOut[0]*MPS2KN, g_stNavData.dVelOut[1]*MPS2KN, g_stNavData.dPosnOut[0]*RAD2DEG, g_stNavData.dPosnOut[1]*RAD2DEG);
        oldString = ui->textEdit->toPlainText();
        oldString = oldString + QString(buffer);
        ui->textEdit->clear();
        ui->textEdit->append(oldString);

        printf("vE2=%.4lf vN2=%.4lf Lat2=%.6lf Lon2=%.6lf\n", g_stNavData.dVelCh1[0]*MPS2KN, g_stNavData.dVelCh1[1]*MPS2KN, g_stNavData.dPosnCh1[0]*RAD2DEG, g_stNavData.dPosnCh1[1]*RAD2DEG);
        sprintf(buffer,"vE2=%.4lf vN2=%.4lf Lat2=%.6lf Lon2=%.6lf\n", g_stNavData.dVelCh1[0]*MPS2KN, g_stNavData.dVelCh1[1]*MPS2KN, g_stNavData.dPosnCh1[0]*RAD2DEG, g_stNavData.dPosnCh1[1]*RAD2DEG);
        oldString = ui->textEdit->toPlainText();
        oldString = oldString + QString(buffer);
        ui->textEdit->clear();
        ui->textEdit->append(oldString);

        printf("vEr=%.4lf vNr=%.4lf Latr=%.6lf Lonr=%.6lf\n", g_stRefMsg.dVelRef[0]*MPS2KN, g_stRefMsg.dVelRef[1]*MPS2KN, g_stRefMsg.dPosnRef[0]*RAD2DEG, g_stRefMsg.dPosnRef[1]*RAD2DEG);
        sprintf(buffer,"vEr=%.4lf vNr=%.4lf Latr=%.6lf Lonr=%.6lf\n", g_stRefMsg.dVelRef[0]*MPS2KN, g_stRefMsg.dVelRef[1]*MPS2KN, g_stRefMsg.dPosnRef[0]*RAD2DEG, g_stRefMsg.dPosnRef[1]*RAD2DEG);
        oldString = ui->textEdit->toPlainText();
        oldString = oldString + QString(buffer);
        ui->textEdit->clear();
        ui->textEdit->append(oldString);

        printf("vEx=%.4lf vNx=%.4lf vE0=%.4lf vN0=%.4lf\n", g_stRefMsg.dVelDampRef[0]*MPS2KN, g_stRefMsg.dVelDampRef[1]*MPS2KN, g_stNavData.dVelDampZero[0]*MPS2KN, g_stNavData.dVelDampZero[1]*MPS2KN);
        sprintf(buffer,"vEx=%.4lf vNx=%.4lf vE0=%.4lf vN0=%.4lf\n", g_stRefMsg.dVelDampRef[0]*MPS2KN, g_stRefMsg.dVelDampRef[1]*MPS2KN, g_stNavData.dVelDampZero[0]*MPS2KN, g_stNavData.dVelDampZero[1]*MPS2KN);
        oldString = ui->textEdit->toPlainText();
        oldString = oldString + QString(buffer);
        ui->textEdit->clear();
        ui->textEdit->append(oldString);

        printf("dLat1=%.4lf dLon1=%.4lf dPosn1=%.4lf\n", g_stNavErr.dPosnOutErr[0], g_stNavErr.dPosnOutErr[1], g_stNavErr.dPosnOutErr[2]);
        sprintf(buffer,"dLat1=%.4lf dLon1=%.4lf dPosn1=%.4lf\n", g_stNavErr.dPosnOutErr[0], g_stNavErr.dPosnOutErr[1], g_stNavErr.dPosnOutErr[2]);
        oldString = ui->textEdit->toPlainText();
        oldString = oldString + QString(buffer);
        ui->textEdit->clear();
        ui->textEdit->append(oldString);

        printf("dLat2=%.4lf dLon2=%.4lf dPosn2=%.4lf\n", g_stNavErr.dPosnRuleErr[0], g_stNavErr.dPosnRuleErr[1], g_stNavErr.dPosnRuleErr[2]);
        sprintf(buffer,"dLat2=%.4lf dLon2=%.4lf dPosn2=%.4lf\n", g_stNavErr.dPosnRuleErr[0], g_stNavErr.dPosnRuleErr[1], g_stNavErr.dPosnRuleErr[2]);
        oldString = ui->textEdit->toPlainText();
        oldString = oldString + QString(buffer);
        ui->textEdit->clear();
        ui->textEdit->append(oldString);

        printf("vU =%.4lf Hgh=%.4lf\n", g_stNavData.dVelOut[2], g_stNavData.dPosnOut[2]);
        sprintf(buffer,"vU =%.4lf Hgh=%.4lf\n", g_stNavData.dVelOut[2], g_stNavData.dPosnOut[2]);
        oldString = ui->textEdit->toPlainText();
        oldString = oldString + QString(buffer);
        ui->textEdit->clear();
        ui->textEdit->append(oldString);

        printf("\n");
        oldString = ui->textEdit->toPlainText();
        oldString = oldString + QString(buffer);
        ui->textEdit->clear();
        ui->textEdit->append(oldString);
        ui->textEdit->append("\n");
    }
}

void Widget::get_cur_time()
{
    struct tm *pCurTime;
    time(&tCurTime);
    pCurTime = localtime(&tCurTime);
    printf ("cur time:%02d-%02d-%02d %02d:%02d:%02d\t last time:%d seconds\r\n",pCurTime->tm_year+1900,pCurTime->tm_mon,pCurTime->tm_mday,pCurTime->tm_hour,pCurTime->tm_min,pCurTime->tm_sec,tCurTime-tBeginTime);
    char buffer[100];
    QString oldString;
    sprintf(buffer,"cur time:%02d-%02d-%02d %02d:%02d:%02d\t last time:%d seconds\r\n",pCurTime->tm_year+1900,pCurTime->tm_mon+1,pCurTime->tm_mday,pCurTime->tm_hour,pCurTime->tm_min,pCurTime->tm_sec,tCurTime-tBeginTime);
    oldString = ui->textEdit->toPlainText();
    oldString = oldString + QString(buffer);
    ui->textEdit->clear();
    ui->textEdit->append(oldString);

}

void Widget::initGlobal()
{

    //ifstream inFileName;
    //ifstream inPulseMsg;
    //ifstream inImuInfo;
    //ifstream inRefMsg;
    //ifstream inGpsMsg;

    //FILE *fwNavRet;
    //FILE *fwNavInfo;
    //FILE *fwFilterInfo;
    //FILE *fwCheckInfo;
    //FILE *fwFileNameOut;

    //char dataPath[100];

    //((void *)g_stBindPara,0,sizeof(g_stBindPara));
    stopRun = 0;
    inde1IsNew=1;
    inde2IsNew=1;
    inde3IsNew=1;
    inde4IsNew=1;
    inde5IsNew=1;
    inde6IsNew=1;
    inde7IsNew=1;
    inde8IsNew=1;
    inde9IsNew=1;
    isRun = 1;
    pointNumber = 0;
    g_uiCntClock = 0;
    g_ucFlagClock ='0';
    g_stBindPara ={0};
    g_stInertData={0};
    g_stKfPara={0};
    g_stBDPara={0};
    g_stImuInfo={0};
    g_stRefMsg={0};
    g_stHostMsg={0};
    g_stPulseMsg={0};
    g_stCheckInfo={0};
    g_stNavErr={0};
    ucFlagSimEnd = 0;
    pointNumber = 0;
    //char fnNavRet[100];
    mstrcat(fnNavRet, PATHNAME);
   // char fnNavInfo[100];
    mstrcat(fnNavInfo, PATHNAME);
    //char fnFilterInfo[100];
    mstrcat(fnFilterInfo, PATHNAME);
    //char fnCheckInfo[100];
    mstrcat(fnCheckInfo, PATHNAME);
    //char fnFileNameOut[100] = "filenameout.txt";
    strcpy(fnFileNameOut,"filenameout.txt");
    mstrcat(fnFileNameOut, PATHNAME);
    //char fnFileName[100] = "filename.txt";
    strcpy(fnFileName,"filename.txt");
    mstrcat(fnFileName, PATHNAME);
    //char fnImuInfo[100];
    //char fnRefMsg[100];
    //#define PATHNAME "E:\\QTproject\\CalibrationPara\\"
    //char PATHNAME[100];
    //char strFileName[1000];
    mstrcat(strFileName, PATHNAME);
    //char strImuInfo[1000];
    //char strRefMsg[1000];
    //strPulseMsg[LEN_PULSE_MSG];
    memset(strPulseMsg,'\0',sizeof(strPulseMsg));
     VphiE.clear();
     VphiN.clear();
     VphiU.clear();
     VdvE.clear();
     VdvN.clear();
     VdvU.clear();
     VBGx.clear();
     VBGy.clear();
     VBGz.clear();
     VBAx.clear();
     VBAy.clear();
     VBAz.clear();
     VdKGx.clear();
     VdKGy.clear();
     VdKGz.clear();
     VdUGxy.clear();
     VdUGxz.clear();
     VdUGyz.clear();
     VdKAx.clear();
     VdKAy.clear();
     VdKAz.clear();
     VdUAxy.clear();
     VdUAxz.clear();
     VdUAyx.clear();
     VdUAyz.clear();
     VdUAzx.clear();
     VdUAzy.clear();
     VpphiE.clear();
     VpphiN.clear();
     VpphiU.clear();
     VpdvE.clear();
     VpdvN.clear();
     VpdvU.clear();
     VpBGx.clear();
     VpBGy.clear();
     VpBGz.clear();
     VpBAx.clear();
     VpBAy.clear();
     VpBAz.clear();
     VpdKGx.clear();
     VpdKGy.clear();
     VpdKGz.clear();
     VpdUGxy.clear();
     VpdUGxz.clear();
     VpdUGyz.clear();
     VpdKAx.clear();
     VpdKAy.clear();
     VpdKAz.clear();
     VpdUAxy.clear();
     VpdUAxz.clear();
     VpdUAyx.clear();
     VpdUAyz.clear();
     VpdUAzx.clear();
     VpdUAzy.clear();
     VpointNumber.clear();
     ui->widget->clearGraphs();
     ui->widget_2->clearGraphs();
     ui->widget_3->clearGraphs();
     ui->widget_4->clearGraphs();
     ui->widget_5->clearGraphs();
     ui->widget_6->clearGraphs();
     ui->pushButton->setEnabled(0);
     ui->pushButton_2->setEnabled(0);
     ui->pushButton_4->setEnabled(0);
     ui->pushButton_5->setEnabled(0);
     ui->pushButton_6->setEnabled(0);
     ui->pushButton_7->setEnabled(0);
     ui->pushButton_8->setEnabled(0);
     ui->pushButton_3->setEnabled(0);
     ui->widget->xAxis->setRange(0, 1, Qt::AlignLeft);
     ui->widget->yAxis->setRange(0, 0.00001, Qt::AlignTop);

     ui->widget_2->xAxis->setRange(0, 1, Qt::AlignLeft);
     ui->widget_2->yAxis->setRange(0, 0.00001, Qt::AlignTop);

     ui->widget_3->xAxis->setRange(0, 1, Qt::AlignLeft);
     ui->widget_3->yAxis->setRange(0, 0.00001, Qt::AlignTop);

     ui->widget_4->xAxis->setRange(0, 1, Qt::AlignLeft);
     ui->widget_4->yAxis->setRange(0, 0.00001, Qt::AlignTop);

     ui->widget_5->xAxis->setRange(0, 1, Qt::AlignLeft);
     ui->widget_5->yAxis->setRange(0, 0.00001, Qt::AlignTop);

     ui->widget_6->xAxis->setRange(0, 1, Qt::AlignLeft);
     ui->widget_6->yAxis->setRange(0, 0.00001, Qt::AlignTop);

     }

void Widget::CalMain()
{
    if(isRun == 1)
        return;
    qtime->start();
    ui->textEdit->clear();
    initGlobal();
    g_stHostMsg.iPosnInit[0] = (int)(29.6848*TRANS_LAT);
    g_stHostMsg.iPosnInit[1] = (int)(115.9875*TRANS_LON);
    InitNavData();
    InitKFData();
    InitBDData();

    get_bind_para();
    // read filename

    inFileName.open(fnFileName, ios::in);
    //	inFileName.getline(fnImuInfo, sizeof(fnImuInfo));
    //	mstrcat(fnImuInfo, PATHNAME);
    //	inImuInfo.open(fnImuInfo, ios::in);
    //	inFileName.getline(fnRefMsg, sizeof(fnRefMsg));
    //	mstrcat(fnRefMsg, PATHNAME);
    //	inRefMsg.open(fnRefMsg, ios::in);
    // write

    fwNavRet = fopen(fnNavRet, "w");


    fwNavInfo = fopen(fnNavInfo, "w");


    fwFilterInfo = fopen(fnFilterInfo, "w");


    fwCheckInfo = fopen(fnCheckInfo, "w");
    //	fprintf (fwCheckInfo, "phiE\tphiN\tphiU\tdvE\tdvN\tdvU\tBGx\tBGy\tBGz\tBAx\tBAy\tBAz\tSGx\tSGy\tSGz\tMGyz\tMGzy\tMGzx\tSAx\tSAy\tSAz\tMAxz\tMAxy\tMAyx\tMAyz\tMAzy\tMAzx\n");


    fwFileNameOut = fopen(fnFileNameOut, "w");
    fprintf(fwFileNameOut, "%s\n", fnNavRet);
    fprintf(fwFileNameOut, "%s\n", fnNavInfo);
    fprintf(fwFileNameOut, "%s\n", fnFilterInfo);
    fprintf(fwFileNameOut, "%s\n", fnCheckInfo);
    fclose(fwFileNameOut);
    // check info init
    g_stCheckInfo.iCheckCnt = 0;
    g_stCheckInfo.iLostCnt = 0;
    g_stCheckInfo.iReadCnt = 0;
    g_stCheckInfo.iImuCntCur = 0;
    g_stCheckInfo.iImuCntPre = 0;
    ui->label_33->setText("正在计算");
    while (inFileName.getline(strFileName, sizeof(strFileName)))
    {

        QCoreApplication::processEvents(QEventLoop::AllEvents, 100);

        inPulseMsg.open(strFileName, ios::in|ios::binary);
        //		inPulseMsg.read(strPulseMsg, 41);
        while (inPulseMsg.read(strPulseMsg, LEN_PULSE_MSG))
        {
            if(stopRun == 1)
            {
                ui->label_33->setText("停止计算");
                isRun = 0;
                ui->pushButton->setEnabled(1);
                ui->pushButton_2->setEnabled(1);
                ui->pushButton_4->setEnabled(1);
                ui->pushButton_5->setEnabled(1);
                ui->pushButton_6->setEnabled(1);
                ui->pushButton_7->setEnabled(1);
                ui->pushButton_8->setEnabled(1);
                ui->pushButton_3->setEnabled(1);
                qtime->stop();
                inPulseMsg.close();
                inFileName.close();

                fclose(fwNavInfo);
                fclose(fwNavRet);
                fclose(fwFilterInfo);
                fclose(fwCheckInfo);
                return;
            }
            QCoreApplication::processEvents(QEventLoop::AllEvents, 100);
            g_uiCntClock++;
            get_pulse_msg2(strPulseMsg);
            //fprintfPLS();
            g_stRefMsg.dVelRef[0] = 0.0;
            g_stRefMsg.dVelRef[1] = 0.0;
            g_stRefMsg.dPosnRef[0] = (29.6848*DEG2RAD);
            g_stRefMsg.dPosnRef[1] = (115.9875*DEG2RAD);
            g_stRefMsg.fGpsPosn[0] = (float)(29.6848*DEG2RAD);
            g_stRefMsg.fGpsPosn[1] = (float)(115.9875*DEG2RAD);

            if (g_uiCntClock == 1*UPDATE_FREQ)
                TurnCoarseAlign();
            if (1 == g_uiCntClock%UPDATE_FREQ)
            {/*
                    inImuInfo.getline(strImuInfo, sizeof(strImuInfo));
                    sscanf(strImuInfo, "%*d %*f %*f %*f %*d %*d %*d %*f %*f %*f %*f %*f %*f %f %f %f %f %f %f",
                        &g_stInertData.fTempGyro[0], &g_stInertData.fTempGyro[1], &g_stInertData.fTempGyro[2],
                        &g_stInertData.fTempAcc[0], &g_stInertData.fTempAcc[1], &g_stInertData.fTempAcc[2]);
                    inRefMsg.getline(strRefMsg, sizeof(strRefMsg));
                    sscanf(strRefMsg, "%*d %*d %d %f %f %f %f %f %f",
                        &g_stRefMsg.uiTimeMain, &g_stRefMsg.fGpsPosn[0], &g_stRefMsg.fGpsPosn[1], &g_stRefMsg.fGpsVel[2],
                        &g_stRefMsg.fTraAng, &g_stRefMsg.fGpsVel[0], &g_stRefMsg.fGpsVel[1]);
                    g_stNavData.ucRefValid = (g_stRefMsg.uiTimeMain&0xFF);
                    g_stRefMsg.fGpsVel[0] *= KN2MPS;
                    g_stRefMsg.fGpsVel[1] *= KN2MPS;
                    g_stRefMsg.fGpsPosn[0] *= DEG2RAD;
                    g_stRefMsg.fGpsPosn[1] *= DEG2RAD;*/
            }

            InertTrans2();
            InputProc();
            CalcProc();
            FilterProc();
            CalcAttiRate();
            soutputProc();
            if (g_uiCntClock > LEN_SIM_TIME)
            {
                ucFlagSimEnd = 1;
            }
            if (ucFlagSimEnd)
                break;
        }
        inPulseMsg.close();
        if (ucFlagSimEnd)
            break;

    }
    inFileName.close();

    fclose(fwNavInfo);
    fclose(fwNavRet);
    fclose(fwFilterInfo);
    fclose(fwCheckInfo);
    printf("\nSuccess");
    QString oldString;
    oldString = ui->textEdit->toPlainText();
    ui->textEdit->clear();
    ui->textEdit->append(oldString);
    ui->textEdit->append("Success！");
    ui->label_33->setText("计算完成");
    //setDafultCurve();
    isRun = 0;
    ui->pushButton->setEnabled(1);
    ui->pushButton_2->setEnabled(1);
    ui->pushButton_4->setEnabled(1);
    ui->pushButton_5->setEnabled(1);
    ui->pushButton_6->setEnabled(1);
    ui->pushButton_7->setEnabled(1);
    ui->pushButton_8->setEnabled(1);
    ui->pushButton_3->setEnabled(1);
    qtime->stop();
}

//void Widget::getCurveData()
//{
//    QFile file;
//    file.setFileName(fnCheckInfo);
//    ui->textBrowser->append(fnCheckInfo);
//    qDebug()<<fnCheckInfo;
//    //file.setFileName("check220025_15.27.41.txt");
//    if(!file.open(QIODevice::ReadOnly))
//    {
//        QMessageBox::warning(this, tr("打开曲线文件"),
//                             tr("打开曲线文件失败，请检查文件名和是否具有读取权限！"));
//        return;
//    }
//    qDebug()<<fnCheckInfo;
//    QTextStream stream(&file);
//    stream.setCodec("utf-8");  // 要读的文件是utf-8编码
//    stream.setGenerateByteOrderMark(true);  // 带bom的utf8
//    while(!stream.atEnd())
//    {

//        QString line = stream.readLine();
//        QStringList lineList = line.split("\t");
//        phiE.append(pointNumber,lineList[0].toDouble());
//        if(pointNumber/100 == 0)
//            qDebug()<<pointNumber;
//        if(lineList[0].toDouble()<MinphiE)
//        {
//            MinphiE = lineList[0].toDouble();
//        }
//        if(lineList[0].toDouble()>MaxphiE)
//        {
//            MaxphiE = lineList[0].toDouble();
//        }
//        phiN.append(pointNumber,lineList[1].toDouble());
//        if(lineList[1].toDouble()<MinphiN)
//        {
//            MinphiN = lineList[1].toDouble();
//        }
//        if(lineList[1].toDouble()>MaxphiN)
//        {
//            MaxphiN = lineList[1].toDouble();
//        }
//        phiU.append(pointNumber,lineList[2].toDouble());
//        if(lineList[2].toDouble()<MinphiU)
//        {
//            MinphiU = lineList[2].toDouble();
//        }
//        if(lineList[2].toDouble()>MaxphiU)
//        {
//            MaxphiU = lineList[2].toDouble();
//        }
//        dvE.append(pointNumber,lineList[3].toDouble());
//        if(lineList[3].toDouble()<MindvE)
//        {
//            MindvE = lineList[3].toDouble();
//        }
//        if(lineList[3].toDouble()>MaxdvE)
//        {
//            MaxdvE = lineList[3].toDouble();
//        }
//        dvN.append(pointNumber,lineList[4].toDouble());
//        if(lineList[4].toDouble()<MindvN)
//        {
//            MindvN = lineList[4].toDouble();
//        }
//        if(lineList[4].toDouble()>MaxdvN)
//        {
//            MaxdvN = lineList[4].toDouble();
//        }
//        dvU.append(pointNumber,lineList[5].toDouble());
//        if(lineList[5].toDouble()<MindvU)
//        {
//            MindvU = lineList[5].toDouble();
//        }
//        if(lineList[5].toDouble()>MaxdvU)
//        {
//            MaxdvU = lineList[5].toDouble();
//        }
//        BGx.append(pointNumber,lineList[6].toDouble());
//        if(lineList[6].toDouble()<MinBGx)
//        {
//            MinBGx = lineList[6].toDouble();
//        }
//        if(lineList[6].toDouble()>MaxBGx)
//        {
//            MaxBGx = lineList[6].toDouble();
//        }
//        BGy.append(pointNumber,lineList[7].toDouble());
//        if(lineList[7].toDouble()<MinBGy)
//        {
//            MinBGy = lineList[7].toDouble();
//        }
//        if(lineList[7].toDouble()>MaxBGy)
//        {
//            MaxBGy = lineList[7].toDouble();
//        }
//        BGz.append(pointNumber,lineList[8].toDouble());
//        if(lineList[8].toDouble()<MinBGz)
//        {
//            MinBGz = lineList[8].toDouble();
//        }
//        if(lineList[8].toDouble()>MaxBGz)
//        {
//            MaxBGz = lineList[8].toDouble();
//        }
//        BAx.append(pointNumber,lineList[9].toDouble());
//        if(lineList[9].toDouble()<MinBAx)
//        {
//            MinBAx = lineList[9].toDouble();
//        }
//        if(lineList[9].toDouble()>MaxBAx)
//        {
//            MaxBAx = lineList[9].toDouble();
//        }
//        BAy.append(pointNumber,lineList[10].toDouble());
//        if(lineList[10].toDouble()<MinBAy)
//        {
//            MinBAy = lineList[10].toDouble();
//        }
//        if(lineList[10].toDouble()>MaxBAy)
//        {
//            MaxBAy = lineList[10].toDouble();
//        }
//        BAz.append(pointNumber,lineList[11].toDouble());
//        if(lineList[11].toDouble()<MinBAz)
//        {
//            MinBAz = lineList[11].toDouble();
//        }
//        if(lineList[11].toDouble()>MaxBAz)
//        {
//            MaxBAz = lineList[11].toDouble();
//        }
//        dKGx.append(pointNumber,lineList[12].toDouble());
//        if(lineList[12].toDouble()<MindKGx)
//        {
//            MindKGx = lineList[12].toDouble();
//        }
//        if(lineList[12].toDouble()>MaxdKGx)
//        {
//            MaxdKGx = lineList[12].toDouble();
//        }
//        dKGy.append(pointNumber,lineList[13].toDouble());
//        if(lineList[13].toDouble()<MindKGy)
//        {
//            MindKGy = lineList[13].toDouble();
//        }
//        if(lineList[13].toDouble()>MaxdKGy)
//        {
//            MaxdKGy = lineList[13].toDouble();
//        }
//        dKGz.append(pointNumber,lineList[14].toDouble());
//        if(lineList[14].toDouble()<MindKGz)
//        {
//            MindKGz = lineList[14].toDouble();
//        }
//        if(lineList[14].toDouble()>MaxdKGz)
//        {
//            MaxdKGz = lineList[14].toDouble();
//        }
//        dUGxy.append(pointNumber,lineList[15].toDouble());
//        if(lineList[15].toDouble()<MindUGxy)
//        {
//            MindUGxy = lineList[15].toDouble();
//        }
//        if(lineList[15].toDouble()>MaxdUGxy)
//        {
//            MaxdUGxy = lineList[15].toDouble();
//        }
//        dUGxz.append(pointNumber,lineList[16].toDouble());
//        if(lineList[16].toDouble()<MindUGxz)
//        {
//            MindUGxz = lineList[16].toDouble();
//        }
//        if(lineList[16].toDouble()>MaxdUGxz)
//        {
//            MaxdUGxz = lineList[16].toDouble();
//        }
//        dUGyz.append(pointNumber,lineList[17].toDouble());
//        if(lineList[17].toDouble()<MindUGyz)
//        {
//            MindUGyz = lineList[17].toDouble();
//        }
//        if(lineList[17].toDouble()>MaxdUGyz)
//        {
//            MaxdUGyz = lineList[17].toDouble();
//        }
//        dKAx.append(pointNumber,lineList[18].toDouble());
//        if(lineList[18].toDouble()<MindKAx)
//        {
//            MindKAx = lineList[18].toDouble();
//        }
//        if(lineList[18].toDouble()>MaxdKAx)
//        {
//            MaxdKAx = lineList[18].toDouble();
//        }
//        dKAy.append(pointNumber,lineList[19].toDouble());
//        if(lineList[19].toDouble()<MindKAy)
//        {
//            MindKAy = lineList[19].toDouble();
//        }
//        if(lineList[19].toDouble()>MaxdKAy)
//        {
//            MaxdKAy = lineList[19].toDouble();
//        }
//        dKAz.append(pointNumber,lineList[20].toDouble());
//        if(lineList[20].toDouble()<MindKAz)
//        {
//            MindKAz = lineList[20].toDouble();
//        }
//        if(lineList[20].toDouble()>MaxdKAz)
//        {
//            MaxdKAz = lineList[20].toDouble();
//        }
//        dUAxy.append(pointNumber,lineList[21].toDouble());
//        if(lineList[21].toDouble()<MindUAxy)
//        {
//            MindUAxy = lineList[21].toDouble();
//        }
//        if(lineList[21].toDouble()>MaxdUAxy)
//        {
//            MaxdUAxy = lineList[21].toDouble();
//        }
//        dUAxz.append(pointNumber,lineList[22].toDouble());
//        if(lineList[22].toDouble()<MindUAxz)
//        {
//            MindUAxz = lineList[22].toDouble();
//        }
//        if(lineList[22].toDouble()>MaxdUAxz)
//        {
//            MaxdUAxz = lineList[22].toDouble();
//        }
//        dUAyx.append(pointNumber,lineList[23].toDouble());
//        if(lineList[23].toDouble()<MindUAyx)
//        {
//            MindUAyx = lineList[23].toDouble();
//        }
//        if(lineList[23].toDouble()>MaxdUAyx)
//        {
//            MaxdUAyx = lineList[23].toDouble();
//        }
//        dUAyz.append(pointNumber,lineList[24].toDouble());
//        if(lineList[24].toDouble()<MindUAyz)
//        {
//            MindUAyz = lineList[24].toDouble();
//        }
//        if(lineList[24].toDouble()>MaxdUAyz)
//        {
//            MaxdUAyz = lineList[24].toDouble();
//        }
//        dUAzx.append(pointNumber,lineList[25].toDouble());
//        if(lineList[25].toDouble()<MindUAzx)
//        {
//            MindUAzx = lineList[25].toDouble();
//        }
//        if(lineList[25].toDouble()>MaxdUAzx)
//        {
//            MaxdUAzx = lineList[25].toDouble();
//        }
//        dUAzy.append(pointNumber,lineList[26].toDouble());
//        if(lineList[26].toDouble()<MindUAzy)
//        {
//            MindUAzy = lineList[26].toDouble();
//        }
//        if(lineList[26].toDouble()>MaxdUAzy)
//        {
//            MaxdUAzy = lineList[26].toDouble();
//        }
//        pphiE.append(pointNumber,lineList[27].toDouble());
//        if(lineList[27].toDouble()<MinpphiE)
//        {
//            MinpphiE = lineList[27].toDouble();
//        }
//        if(lineList[27].toDouble()>MaxpphiE)
//        {
//            MaxpphiE = lineList[27].toDouble();
//        }
//        pphiN.append(pointNumber,lineList[28].toDouble());
//        if(lineList[28].toDouble()<MinpphiN)
//        {
//            MinpphiN = lineList[28].toDouble();
//        }
//        if(lineList[28].toDouble()>MaxpphiN)
//        {
//            MaxpphiN = lineList[28].toDouble();
//        }
//        pphiU.append(pointNumber,lineList[29].toDouble());
//        if(lineList[29].toDouble()<MinpphiU)
//        {
//            MinpphiU = lineList[29].toDouble();
//        }
//        if(lineList[29].toDouble()>MaxpphiU)
//        {
//            MaxpphiU = lineList[29].toDouble();
//        }
//        pdvE.append(pointNumber,lineList[30].toDouble());
//        if(lineList[30].toDouble()<MinpdvE)
//        {
//            MinpdvE = lineList[30].toDouble();
//        }
//        if(lineList[30].toDouble()>MaxpdvE)
//        {
//            MaxpdvE = lineList[30].toDouble();
//        }
//        pdvN.append(pointNumber,lineList[31].toDouble());
//        if(lineList[31].toDouble()<MinpdvN)
//        {
//            MinpdvN = lineList[31].toDouble();
//        }
//        if(lineList[31].toDouble()>MaxpdvN)
//        {
//            MaxpdvN = lineList[31].toDouble();
//        }
//        pdvU.append(pointNumber,lineList[32].toDouble());
//        if(lineList[32].toDouble()<MinpdvU)
//        {
//            MinpdvU = lineList[32].toDouble();
//        }
//        if(lineList[32].toDouble()>MaxpdvU)
//        {
//            MaxpdvU = lineList[32].toDouble();
//        }
//        pBGx.append(pointNumber,lineList[33].toDouble());
//        if(lineList[33].toDouble()<MinpBGx)
//        {
//            MinpBGx = lineList[33].toDouble();
//        }
//        if(lineList[33].toDouble()>MaxpBGx)
//        {
//            MaxpBGx = lineList[0].toDouble();
//        }
//        pBGy.append(pointNumber,lineList[34].toDouble());
//        if(lineList[34].toDouble()<MinpBGy)
//        {
//            MinpBGy = lineList[34].toDouble();
//        }
//        if(lineList[34].toDouble()>MaxpBGy)
//        {
//            MaxpBGy = lineList[34].toDouble();
//        }
//        pBGz.append(pointNumber,lineList[35].toDouble());
//        if(lineList[35].toDouble()<MinpBGz)
//        {
//            MinpBGz = lineList[35].toDouble();
//        }
//        if(lineList[35].toDouble()>MaxpBGz)
//        {
//            MaxpBGz = lineList[35].toDouble();
//        }
//        pBAx.append(pointNumber,lineList[36].toDouble());
//        if(lineList[36].toDouble()<MinpBAx)
//        {
//            MinpBAx = lineList[36].toDouble();
//        }
//        if(lineList[36].toDouble()>MaxpBAx)
//        {
//            MaxpBAx = lineList[36].toDouble();
//        }
//        pBAy.append(pointNumber,lineList[37].toDouble());
//        if(lineList[37].toDouble()<MinpBAy)
//        {
//            MinpBAy = lineList[37].toDouble();
//        }
//        if(lineList[37].toDouble()>MaxpBAy)
//        {
//            MaxpBAy = lineList[37].toDouble();
//        }
//        pBAz.append(pointNumber,lineList[38].toDouble());
//        if(lineList[38].toDouble()<MinpBAz)
//        {
//            MinpBAz = lineList[38].toDouble();
//        }
//        if(lineList[38].toDouble()>MaxpBAz)
//        {
//            MaxpBAz = lineList[38].toDouble();
//        }
//        pdKGx.append(pointNumber,lineList[39].toDouble());
//        if(lineList[39].toDouble()<MinpdKGx)
//        {
//            MinpdKGx = lineList[39].toDouble();
//        }
//        if(lineList[39].toDouble()>MaxpdKGx)
//        {
//            MaxpdKGx = lineList[39].toDouble();
//        }
//        pdKGy.append(pointNumber,lineList[40].toDouble());
//        if(lineList[40].toDouble()<MinpdKGy)
//        {
//            MinpdKGy = lineList[40].toDouble();
//        }
//        if(lineList[40].toDouble()>MaxpdKGy)
//        {
//            MaxpdKGy = lineList[40].toDouble();
//        }
//        pdKGz.append(pointNumber,lineList[41].toDouble());
//        if(lineList[41].toDouble()<MinpdKGz)
//        {
//            MinpdKGz = lineList[41].toDouble();
//        }
//        if(lineList[41].toDouble()>MaxpdKGz)
//        {
//            MaxpdKGz = lineList[41].toDouble();
//        }
//        pdUGxy.append(pointNumber,lineList[42].toDouble());
//        if(lineList[42].toDouble()<MinpdUGxy)
//        {
//            MinpdUGxy = lineList[42].toDouble();
//        }
//        if(lineList[42].toDouble()>MaxpdUGxy)
//        {
//            MaxpdUGxy = lineList[42].toDouble();
//        }
//        pdUGxz.append(pointNumber,lineList[43].toDouble());
//        if(lineList[43].toDouble()<MinpdUGxz)
//        {
//            MinpdUGxz = lineList[43].toDouble();
//        }
//        if(lineList[43].toDouble()>MaxpdUGxz)
//        {
//            MaxpdUGxz = lineList[43].toDouble();
//        }
//        pdUGyz.append(pointNumber,lineList[44].toDouble());
//        if(lineList[44].toDouble()<MinpdUGyz)
//        {
//            MinpdUGyz = lineList[44].toDouble();
//        }
//        if(lineList[44].toDouble()>MaxpdUGyz)
//        {
//            MaxpdUGyz = lineList[44].toDouble();
//        }
//        pdKAx.append(pointNumber,lineList[45].toDouble());
//        if(lineList[45].toDouble()<MinpdKAx)
//        {
//            MinpdKAx = lineList[45].toDouble();
//        }
//        if(lineList[45].toDouble()>MaxpdKAx)
//        {
//            MaxpdKAx = lineList[45].toDouble();
//        }
//        pdKAy.append(pointNumber,lineList[46].toDouble());
//        if(lineList[46].toDouble()<MinpdKAy)
//        {
//            MinpdKAy= lineList[46].toDouble();
//        }
//        if(lineList[46].toDouble()>MaxpdKAy)
//        {
//            MaxpdKAy= lineList[46].toDouble();
//        }
//        pdKAz.append(pointNumber,lineList[47].toDouble());
//        if(lineList[47].toDouble()<MinpdKAz)
//        {
//            MinpdKAz = lineList[47].toDouble();
//        }
//        if(lineList[47].toDouble()>MaxpdKAz)
//        {
//            MaxpdKAz = lineList[47].toDouble();
//        }
//        pdUAxy.append(pointNumber,lineList[48].toDouble());
//        if(lineList[48].toDouble()<MinpdUAxy)
//        {
//            MinpdUAxy = lineList[48].toDouble();
//        }
//        if(lineList[48].toDouble()>MaxpdUAxy)
//        {
//            MaxpdUAxy = lineList[48].toDouble();
//        }
//        pdUAxz.append(pointNumber,lineList[49].toDouble());
//        if(lineList[49].toDouble()<MinpdUAxz)
//        {
//            MinpdUAxz = lineList[49].toDouble();
//        }
//        if(lineList[49].toDouble()>MaxpdUAxz)
//        {
//            MaxpdUAxz = lineList[49].toDouble();
//        }

//        pdUAyx.append(pointNumber,lineList[50].toDouble());
//        if(lineList[50].toDouble()<MinpdUAyx)
//        {
//            MinpdUAyx = lineList[50].toDouble();
//        }
//        if(lineList[50].toDouble()>MaxpdUAyx)
//        {
//            MaxpdUAyx = lineList[50].toDouble();
//        }

//        pdUAyz.append(pointNumber,lineList[51].toDouble());
//        if(lineList[51].toDouble()<MinpdUAyz)
//        {
//            MinpdUAyz = lineList[51].toDouble();
//        }
//        if(lineList[51].toDouble()>MaxpdUAyz)
//        {
//            MaxpdUAyz = lineList[51].toDouble();
//        }

//        pdUAzx.append(pointNumber,lineList[52].toDouble());
//        if(lineList[52].toDouble()<MinpdUAzx)
//        {
//            MinpdUAzx = lineList[52].toDouble();
//        }
//        if(lineList[52].toDouble()>MaxpdUAzx)
//        {
//            MaxpdUAzx = lineList[52].toDouble();
//        }

//        pdUAzy.append(pointNumber,lineList[53].toDouble());
//        if(lineList[53].toDouble()<MinpdUAzy)
//        {
//            MinpdUAzy = lineList[53].toDouble();
//        }
//        if(lineList[53].toDouble()>MaxpdUAzy)
//        {
//            MaxpdUAzy = lineList[53].toDouble();
//        }
//        pointNumber++;
//    }
//    qDebug()<<"read data down";
//    file.close();
//}

void Widget::on_pushButton_clicked()
{
    getConfigData();
}

void Widget::on_pushButton_2_clicked()
{
    char fnBindFile[100] = "config.ini";
    char strAppName[100];
    mstrcat(fnBindFile, PATHNAME);

    strcpy(strAppName, "ParaInit");
    WritePrivateProfileString(strAppName,"LatInit",ui->lineEdit_LatInit->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"LonInit",ui->lineEdit_LonInit->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"VelInit",ui->lineEdit_VelInit->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"HeadingInit",ui->lineEdit_HeadingInit->text().toLatin1(),fnBindFile);

    strcpy(strAppName, "CaliPara");
    WritePrivateProfileString(strAppName,"BGX",ui->lineEdit_BGX->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BGY",ui->lineEdit_BGY->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BGZ",ui->lineEdit_BGZ->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"KGX",ui->lineEdit_KGX->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"KGY",ui->lineEdit_KGY->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"KGZ",ui->lineEdit_KGZ->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UGXY",ui->lineEdit_UGXY->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UGXZ",ui->lineEdit_UGXZ->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UGYX",ui->lineEdit_UGYX->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UGYZ",ui->lineEdit_UGYZ->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UGZX",ui->lineEdit_UGZX->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UGZY",ui->lineEdit_UGZY->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BAX",ui->lineEdit_BAX->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BAY",ui->lineEdit_BAY->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BAZ",ui->lineEdit_BAZ->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"KAX",ui->lineEdit_KAX->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"KAY",ui->lineEdit_KAY->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"KAZ",ui->lineEdit_KAZ->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UAXY",ui->lineEdit_UAXY->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UAXZ",ui->lineEdit_UAXZ->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UAYX",ui->lineEdit_UAYX->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UAYZ",ui->lineEdit_UAYZ->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UAZX",ui->lineEdit_UAZX->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UAZY",ui->lineEdit_UAZY->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BGE",ui->lineEdit_BGE->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BGN",ui->lineEdit_BGN->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BGU",ui->lineEdit_BGU->text().toLatin1(),fnBindFile);
    strcpy(strAppName, "AttiZero");
    WritePrivateProfileString(strAppName,"SysR0",ui->lineEdit_SysR0->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"SysP0",ui->lineEdit_SysP0->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"SysH0",ui->lineEdit_SysH0->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"ImuR0",ui->lineEdit_ImuR0->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"ImuP0",ui->lineEdit_ImuP0->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"ImuH0",ui->lineEdit_ImuH0->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"OutR0",ui->lineEdit_OutR0->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"OutP0",ui->lineEdit_OutP0->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"OutH0",ui->lineEdit_OutH0->text().toLatin1(),fnBindFile);

    strcpy(strAppName, "LeverArm");
    WritePrivateProfileString(strAppName,"X1",ui->lineEdit_X1->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"Y1",ui->lineEdit_Y1->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"Z1",ui->lineEdit_Z1->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"X2",ui->lineEdit_X2->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"Y2",ui->lineEdit_Y2->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"Z2",ui->lineEdit_Z2->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"X3",ui->lineEdit_X3->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"Y3",ui->lineEdit_Y3->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"Z3",ui->lineEdit_Z3->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"X4",ui->lineEdit_X4->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"Y4",ui->lineEdit_Y4->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"Z4",ui->lineEdit_Z4->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"X5",ui->lineEdit_X5->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"Y5",ui->lineEdit_Y5->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"Z5",ui->lineEdit_Z5->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"X6",ui->lineEdit_X6->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"Y6",ui->lineEdit_Y6->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"Z6",ui->lineEdit_Z6->text().toLatin1(),fnBindFile);

    strcpy(strAppName, "TempCoef");
    WritePrivateProfileString(strAppName,"BGXB0",ui->lineEdit_BGXB0->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BGYB0",ui->lineEdit_BGYB0->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BGZB0",ui->lineEdit_BGZB0->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BGXB1",ui->lineEdit_BGXB1->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BGYB1",ui->lineEdit_BGYB1->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BGZB1",ui->lineEdit_BGZB1->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BGXB2",ui->lineEdit_BGXB2->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BGYB2",ui->lineEdit_BGYB2->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BGZB2",ui->lineEdit_BGZB2->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BGXB3",ui->lineEdit_BGXB3->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BGYB3",ui->lineEdit_BGYB3->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BGZB3",ui->lineEdit_BGZB3->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BAXB0",ui->lineEdit_BAXB0->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BAYB0",ui->lineEdit_BAYB0->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BAZB0",ui->lineEdit_BAZB0->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BAXB1",ui->lineEdit_BAXB1->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BAYB1",ui->lineEdit_BAYB1->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BAZB1",ui->lineEdit_BAZB1->text().toLatin1(),fnBindFile);

    WritePrivateProfileString(strAppName,"BAXB2",ui->lineEdit_BAXB2->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BAYB2",ui->lineEdit_BAYB2->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BAZB2",ui->lineEdit_BAZB2->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BAXB3",ui->lineEdit_BAXB3->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BAYB3",ui->lineEdit_BAYB3->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BAZB3",ui->lineEdit_BAZB3->text().toLatin1(),fnBindFile);

}

void Widget::on_pushButton_3_clicked()
{
    get_begin_time();
    CalMain();
}

void Widget::on_pushButton_6_clicked()
{
    char fnBindFile[100] = "config.ini";
    char strAppName[100];
    char strValue[100];
    mstrcat(fnBindFile, PATHNAME);
    strcpy(strAppName, "CaliPara");
    GetPrivateProfileString(strAppName, "KGX", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_KGX_10->setText(strValue);

    GetPrivateProfileString(strAppName, "KGY", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_KGY_10->setText(strValue);

    GetPrivateProfileString(strAppName, "KGZ", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_KGZ_10->setText(strValue);

    GetPrivateProfileString(strAppName, "BGX", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_BGX_10->setText(strValue);

    GetPrivateProfileString(strAppName, "BGY", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_BGY_10->setText(strValue);

    GetPrivateProfileString(strAppName, "BGZ", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_BGZ_10->setText(strValue);

    GetPrivateProfileString(strAppName, "UGXY", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_UGXY_10->setText(strValue);

    GetPrivateProfileString(strAppName, "UGXZ", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_UGXZ_10->setText(strValue);

    GetPrivateProfileString(strAppName, "UGYZ", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_UGYZ_10->setText(strValue);


    GetPrivateProfileString(strAppName, "KAX", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_KAX_10->setText(strValue);

    GetPrivateProfileString(strAppName, "KAY", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_KAY_10->setText(strValue);

    GetPrivateProfileString(strAppName, "KAZ", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_KAZ_10->setText(strValue);

    GetPrivateProfileString(strAppName, "BAX", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_BAX_10->setText(strValue);

    GetPrivateProfileString(strAppName, "BAY", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_BAY_10->setText(strValue);

    GetPrivateProfileString(strAppName, "BAZ", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_BAZ_10->setText(strValue);

    GetPrivateProfileString(strAppName, "UAXY", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_UAXY_10->setText(strValue);

    GetPrivateProfileString(strAppName, "UAXZ", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_UAXZ_10->setText(strValue);

    GetPrivateProfileString(strAppName, "UAYX", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_UAYX_10->setText(strValue);

    GetPrivateProfileString(strAppName, "UAYZ", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_UAYZ_10->setText(strValue);

    GetPrivateProfileString(strAppName, "UAZX", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_UAZX_10->setText(strValue);

    GetPrivateProfileString(strAppName, "UAZY", NULL, strValue, 100, fnBindFile);
    ui->lineEdit_UAZY_10->setText(strValue);

}

void Widget::on_pushButton_7_clicked()
{
    ui->textBrowser->clear();
    //QString file_path = ui->lineEdit->text().toLatin1();
    QString file_path = QCoreApplication::applicationDirPath()+ "/";
//    ui->lineEdit->setText(file_path);
//    strcpy(dataPath,file_path.toLatin1());

    QDir dir(file_path);
    QStringList nameFilters;
    nameFilters << "*.dat";
    QStringList files = dir.entryList(nameFilters, QDir::Files|QDir::Readable, QDir::Name);
    if(files.size() == 0)
    {
        ui->textEdit->setText("错误：无校验数据文件！");
        return;
    }
    for(int i = 0; i< files.size();++i)
    {
        QString tmp = files.at(i);
        ui->textBrowser->append(tmp);
    }

}

void Widget::on_pushButton_8_clicked()
{
    char fnFileName2[100] = "filename.txt";
    mstrcat(fnFileName2, PATHNAME);
    FILE * fp = fopen(fnFileName2,"w");

    QString buff = ui->textBrowser->toPlainText().toUtf8();
     QStringList Str_List = buff.split( "\n" ) ;
     for(int i = 0;i<Str_List.size();i++)
     {

         QString  str = Str_List[i];
         QByteArray ba = str.toLatin1();
        fprintf(fp, "%s\n", ba.data());
     }
     fclose(fp);

}

void Widget::on_pushButton_4_clicked()
{
    ui->lineEdit_BGX_12->setText(QString::number(ui->lineEdit_BGX_10->text().toFloat() + ui->lineEdit_BGX_11->text().toFloat())) ;
    ui->lineEdit_BGY_12->setText(QString::number(ui->lineEdit_BGY_10->text().toFloat() + ui->lineEdit_BGY_11->text().toFloat())) ;
    ui->lineEdit_BGZ_12->setText(QString::number(ui->lineEdit_BGZ_10->text().toFloat() + ui->lineEdit_BGZ_11->text().toFloat())) ;

    ui->lineEdit_KGX_12->setText(QString::number(ui->lineEdit_KGX_10->text().toDouble() *(1-ui->lineEdit_KGX_11->text().toDouble()*0.000001),'g',10)) ;
    ui->lineEdit_KGY_12->setText(QString::number(ui->lineEdit_KGY_10->text().toDouble() *(1-ui->lineEdit_KGY_11->text().toDouble()*0.000001),'g',10)) ;
    ui->lineEdit_KGZ_12->setText(QString::number(ui->lineEdit_KGZ_10->text().toDouble() *(1-ui->lineEdit_KGZ_11->text().toDouble()*0.000001),'g',10)) ;

    ui->lineEdit_UGXY_12->setText(QString::number(ui->lineEdit_UGXY_10->text().toFloat() - ui->lineEdit_UGXY_11->text().toFloat())) ;
    ui->lineEdit_UGXZ_12->setText(QString::number(ui->lineEdit_UGXZ_10->text().toFloat() + ui->lineEdit_UGXZ_11->text().toFloat())) ;
    ui->lineEdit_UGYZ_12->setText(QString::number(ui->lineEdit_UGYZ_10->text().toFloat() - ui->lineEdit_UGYZ_11->text().toFloat())) ;

    ui->lineEdit_BAX_12->setText(QString::number(ui->lineEdit_BAX_10->text().toDouble()+ui->lineEdit_BAX_11->text().toDouble()*9.79*1000/ui->lineEdit_KAX_10->text().toDouble(),'g',10));
    ui->lineEdit_BAY_12->setText(QString::number(ui->lineEdit_BAY_10->text().toDouble()+ui->lineEdit_BAY_11->text().toDouble()*9.79*1000/ui->lineEdit_KAY_10->text().toDouble(),'g',10));
    ui->lineEdit_BAZ_12->setText(QString::number(ui->lineEdit_BAZ_10->text().toDouble()+ui->lineEdit_BAZ_11->text().toDouble()*9.79*1000/ui->lineEdit_KAZ_10->text().toDouble(),'g',10));

    ui->lineEdit_KAX_12->setText(QString::number(ui->lineEdit_KAX_10->text().toDouble() *(1-ui->lineEdit_KAX_11->text().toDouble()*0.000001),'g',10)) ;
    ui->lineEdit_KAY_12->setText(QString::number(ui->lineEdit_KAY_10->text().toDouble() *(1-ui->lineEdit_KAY_11->text().toDouble()*0.000001),'g',10)) ;
    ui->lineEdit_KAZ_12->setText(QString::number(ui->lineEdit_KAZ_10->text().toDouble() *(1-ui->lineEdit_KAZ_11->text().toDouble()*0.000001),'g',10)) ;

    ui->lineEdit_UAXY_12->setText(QString::number(ui->lineEdit_UAXY_10->text().toFloat() - ui->lineEdit_UAXY_11->text().toFloat())) ;
    ui->lineEdit_UAXZ_12->setText(QString::number(ui->lineEdit_UAXZ_10->text().toFloat() + ui->lineEdit_UAXZ_11->text().toFloat())) ;
    ui->lineEdit_UAYX_12->setText(QString::number(ui->lineEdit_UAYX_10->text().toFloat() + ui->lineEdit_UAYX_11->text().toFloat())) ;


    ui->lineEdit_UAYZ_12->setText(QString::number(ui->lineEdit_UAYZ_10->text().toFloat() - ui->lineEdit_UAYZ_11->text().toFloat())) ;
    ui->lineEdit_UAZX_12->setText(QString::number(ui->lineEdit_UAZX_10->text().toFloat() - ui->lineEdit_UAZX_11->text().toFloat())) ;
    ui->lineEdit_UAZY_12->setText(QString::number(ui->lineEdit_UAZY_10->text().toFloat() + ui->lineEdit_UAZY_11->text().toFloat())) ;

}

void Widget::slotPointHoverd(const QPointF &point, bool state)
{
    if (state) {
            m_valueLabel.setText(QString::asprintf("%1.0f%", point.y()));

            QPoint curPos = mapFromGlobal(QCursor::pos());
            m_valueLabel.move(curPos.x() - m_valueLabel.width() / 2, curPos.y() - m_valueLabel.height() * 1.5);//移动数值

            m_valueLabel.show();//显示出来
        }
        else
            m_valueLabel.hide();//进行隐藏


}
void Widget::on_comboBox_currentIndexChanged(int index)
{
    ui->widget->clearGraphs();
    ui->widget_2->clearGraphs();
    ui->widget_3->clearGraphs();
    ui->widget_4->clearGraphs();
    ui->widget_5->clearGraphs();
    ui->widget_6->clearGraphs();
    if(isRun == 1)
    {

        inde1IsNew=1;
        inde2IsNew=1;
        inde3IsNew=1;
        inde4IsNew=1;
        inde5IsNew=1;
        inde6IsNew=1;
        inde7IsNew=1;
        inde8IsNew=1;
        inde9IsNew=1;
        ui->widget->xAxis->setRange(0, 1, Qt::AlignLeft);
        ui->widget->yAxis->setRange(0, 0.00001, Qt::AlignTop);

        ui->widget_2->xAxis->setRange(0, 1, Qt::AlignLeft);
        ui->widget_2->yAxis->setRange(0, 0.00001, Qt::AlignTop);

        ui->widget_3->xAxis->setRange(0, 1, Qt::AlignLeft);
        ui->widget_3->yAxis->setRange(0, 0.00001, Qt::AlignTop);

        ui->widget_4->xAxis->setRange(0, 1, Qt::AlignLeft);
        ui->widget_4->yAxis->setRange(0, 0.00001, Qt::AlignTop);

        ui->widget_5->xAxis->setRange(0, 1, Qt::AlignLeft);
        ui->widget_5->yAxis->setRange(0, 0.00001, Qt::AlignTop);

        ui->widget_6->xAxis->setRange(0, 1, Qt::AlignLeft);
        ui->widget_6->yAxis->setRange(0, 0.00001, Qt::AlignTop);
    }
    else
    {

        ui->widget->xAxis->setRange(0, 1, Qt::AlignLeft);
        ui->widget->yAxis->setRange(0, 0.00001, Qt::AlignTop);

        ui->widget_2->xAxis->setRange(0, 1, Qt::AlignLeft);
        ui->widget_2->yAxis->setRange(0, 0.00001, Qt::AlignTop);

        ui->widget_3->xAxis->setRange(0, 1, Qt::AlignLeft);
        ui->widget_3->yAxis->setRange(0, 0.00001, Qt::AlignTop);

        ui->widget_4->xAxis->setRange(0, 1, Qt::AlignLeft);
        ui->widget_4->yAxis->setRange(0, 0.00001, Qt::AlignTop);

        ui->widget_5->xAxis->setRange(0, 1, Qt::AlignLeft);
        ui->widget_5->yAxis->setRange(0, 0.00001, Qt::AlignTop);

        ui->widget_6->xAxis->setRange(0, 1, Qt::AlignLeft);
        ui->widget_6->yAxis->setRange(0, 0.00001, Qt::AlignTop);
        if(ui->comboBox->currentIndex()==0)
        {

            QPen pen;
            pen.setColor(QColor(255,0,0));
            pen.setWidthF(1.5);
            pen.setWidthF(1.5);
            ui->widget->addGraph();

            ui->widget->graph(0)->setPen(pen);
            ui->widget->graph(0)->setData(VpointNumber,VphiE);
            ui->widget->graph(0)->rescaleAxes(true);
            ui->widget->legend->setVisible(true);
            ui->widget->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget->legend->setBorderPen(Qt::NoPen);
            ui->widget->graph(0)->setName("横摇角误差phiE");
            ui->widget->replot(QCustomPlot::rpQueuedReplot);


            QPen pen3;
            pen3.setColor(QColor(0,0,255));
            pen3.setWidthF(1.5);
            ui->widget_3->addGraph();
            ui->widget_3->graph(0)->setPen(pen3);
            ui->widget_3->graph(0)->setData(VpointNumber,VphiN);
            ui->widget_3->graph(0)->rescaleAxes(true);
            ui->widget_3->legend->setVisible(true);
            ui->widget_3->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_3->legend->setBorderPen(Qt::NoPen);
            ui->widget_3->graph(0)->setName("纵摇角误差phiN");
            ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

            QPen pen5;
            pen5.setColor(QColor(53,150,181));
            pen5.setWidthF(1.5);
            ui->widget_5->addGraph();
            ui->widget_5->graph(0)->setPen(pen5);
            ui->widget_5->graph(0)->setData(VpointNumber,VphiU);
            ui->widget_5->graph(0)->rescaleAxes(true);
            ui->widget_5->legend->setVisible(true);
            ui->widget_5->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_5->legend->setBorderPen(Qt::NoPen);
            ui->widget_5->graph(0)->setName("航向角误差phiU");
            ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

            QPen pen2;
            pen2.setColor(QColor(255,0,0));
            pen2.setWidthF(1.5);
            ui->widget_2->addGraph();
            ui->widget_2->graph(0)->setPen(pen2);
            ui->widget_2->graph(0)->setData(VpointNumber,VpphiE);
            ui->widget_2->graph(0)->rescaleAxes(true);
            ui->widget_2->legend->setVisible(true);
            ui->widget_2->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_2->legend->setBorderPen(Qt::NoPen);
            ui->widget_2->graph(0)->setName("横摇角误差pphiE");
            ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

            QPen pen4;
            pen4.setColor(QColor(0,0,255));
            pen4.setWidthF(1.5);
            ui->widget_4->addGraph();
            ui->widget_4->graph(0)->setPen(pen4);
            ui->widget_4->graph(0)->setData(VpointNumber,VpphiN);
            ui->widget_4->graph(0)->rescaleAxes(true);
            ui->widget_4->legend->setVisible(true);
            ui->widget_4->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_4->legend->setBorderPen(Qt::NoPen);
            ui->widget_4->graph(0)->setName("纵摇角误差pphiN");
            ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

            QPen pen6;
            pen6.setColor(QColor(53,150,181));
            pen6.setWidthF(1.5);
            ui->widget_6->addGraph();
            ui->widget_6->graph(0)->setPen(pen6);
            ui->widget_6->graph(0)->setData(VpointNumber,VpphiU);
            ui->widget_6->graph(0)->rescaleAxes(true);
            ui->widget_6->legend->setVisible(true);
            ui->widget_6->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_6->legend->setBorderPen(Qt::NoPen);
            ui->widget_6->graph(0)->setName("航向角误差pphiU");
            ui->widget_6->replot(QCustomPlot::rpQueuedReplot);


        }

        if(ui->comboBox->currentIndex()==1)
        {

            QPen pen;
            pen.setColor(QColor(255,0,0));
            pen.setWidthF(1.5);
            ui->widget->addGraph();
            ui->widget->graph(0)->setPen(pen);
            ui->widget->graph(0)->setData(VpointNumber,VdvE);
            ui->widget->graph(0)->rescaleAxes(true);
            ui->widget->legend->setVisible(true);
            ui->widget->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget->legend->setBorderPen(Qt::NoPen);
            ui->widget->graph(0)->setName("东速误差dvE");
            ui->widget->replot(QCustomPlot::rpQueuedReplot);

            QPen pen3;
            pen3.setColor(QColor(0,0,255));
            pen3.setWidthF(1.5);
            ui->widget_3->addGraph();
            ui->widget_3->graph(0)->setPen(pen3);
            ui->widget_3->graph(0)->setData(VpointNumber,VdvN);
            ui->widget_3->graph(0)->rescaleAxes(true);
            ui->widget_3->legend->setVisible(true);
            ui->widget_3->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_3->legend->setBorderPen(Qt::NoPen);
            ui->widget_3->graph(0)->setName("北速误差dvN");
            ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

            QPen pen5;
            pen5.setColor(QColor(53,150,181));
            pen5.setWidthF(1.5);
            ui->widget_5->addGraph();
            ui->widget_5->graph(0)->setPen(pen5);
            ui->widget_5->graph(0)->setData(VpointNumber,VdvU);
            ui->widget_5->graph(0)->rescaleAxes(true);
            ui->widget_5->legend->setVisible(true);
            ui->widget_5->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_5->legend->setBorderPen(Qt::NoPen);
            ui->widget_5->graph(0)->setName("垂速误差dvU");
            ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

            QPen pen2;
            pen2.setColor(QColor(255,0,0));
            pen2.setWidthF(1.5);
            ui->widget_2->addGraph();
            ui->widget_2->graph(0)->setPen(pen2);
            ui->widget_2->graph(0)->setData(VpointNumber,VpdvE);
            ui->widget_2->graph(0)->rescaleAxes(true);
            ui->widget_2->legend->setVisible(true);
            ui->widget_2->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_2->legend->setBorderPen(Qt::NoPen);
            ui->widget_2->graph(0)->setName("东速误差pdvE");
            ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

            QPen pen4;
            pen4.setColor(QColor(0,0,255));
            pen4.setWidthF(1.5);
            ui->widget_4->addGraph();
            ui->widget_4->graph(0)->setPen(pen4);
            ui->widget_4->graph(0)->setData(VpointNumber,VpdvN);
            ui->widget_4->graph(0)->rescaleAxes(true);
            ui->widget_4->legend->setVisible(true);
            ui->widget_4->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_4->legend->setBorderPen(Qt::NoPen);
            ui->widget_4->graph(0)->setName("北速误差pdvN");
            ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

            QPen pen6;
            pen6.setColor(QColor(53,150,181));
            pen6.setWidthF(1.5);
            ui->widget_6->addGraph();
            ui->widget_6->graph(0)->setPen(pen6);
            ui->widget_6->graph(0)->setData(VpointNumber,VpdvU);
            ui->widget_6->graph(0)->rescaleAxes(true);
            ui->widget_6->legend->setVisible(true);
            ui->widget_6->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_6->legend->setBorderPen(Qt::NoPen);
            ui->widget_6->graph(0)->setName("垂速误差pdvU");
            ui->widget_6->replot(QCustomPlot::rpQueuedReplot);


        }

        if(ui->comboBox->currentIndex()==2)
        {

            QPen pen;
            pen.setColor(QColor(255,0,0));
            pen.setWidthF(1.5);
            ui->widget->addGraph();
            ui->widget->graph(0)->setPen(pen);
            ui->widget->graph(0)->setData(VpointNumber,VBGx);
            ui->widget->graph(0)->rescaleAxes(true);
            ui->widget->legend->setVisible(true);
            ui->widget->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget->legend->setBorderPen(Qt::NoPen);
            ui->widget->graph(0)->setName("x陀螺常值漂移BGx");
            ui->widget->replot(QCustomPlot::rpQueuedReplot);

            QPen pen3;
            pen3.setColor(QColor(0,0,255));
            pen3.setWidthF(1.5);
            ui->widget_3->addGraph();
            ui->widget_3->graph(0)->setPen(pen3);
            ui->widget_3->graph(0)->setData(VpointNumber,VBGy);
            ui->widget_3->graph(0)->rescaleAxes(true);
            ui->widget_3->legend->setVisible(true);
            ui->widget_3->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_3->legend->setBorderPen(Qt::NoPen);
            ui->widget_3->graph(0)->setName("y陀螺常值漂移BGy");
            ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

            QPen pen5;
            pen5.setColor(QColor(53,150,181));
            pen5.setWidthF(1.5);
            ui->widget_5->addGraph();
            ui->widget_5->graph(0)->setPen(pen5);
            ui->widget_5->graph(0)->setData(VpointNumber,VBGz);
            ui->widget_5->graph(0)->rescaleAxes(true);
            ui->widget_5->legend->setVisible(true);
            ui->widget_5->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_5->legend->setBorderPen(Qt::NoPen);
            ui->widget_5->graph(0)->setName("z陀螺常值漂移BGz");
            ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

            QPen pen2;
            pen2.setColor(QColor(255,0,0));
            pen2.setWidthF(1.5);
            ui->widget_2->addGraph();
            ui->widget_2->graph(0)->setPen(pen2);
            ui->widget_2->graph(0)->setData(VpointNumber,VpBGx);
            ui->widget_2->graph(0)->rescaleAxes(true);
            ui->widget_2->legend->setVisible(true);
            ui->widget_2->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_2->legend->setBorderPen(Qt::NoPen);
            ui->widget_2->graph(0)->setName("x陀螺常值漂移pBGx");
            ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

            QPen pen4;
            pen4.setColor(QColor(0,0,255));
            pen4.setWidthF(1.5);
            ui->widget_4->addGraph();
            ui->widget_4->graph(0)->setPen(pen4);
            ui->widget_4->graph(0)->setData(VpointNumber,VpBGy);
            ui->widget_4->graph(0)->rescaleAxes(true);
            ui->widget_4->legend->setVisible(true);
            ui->widget_4->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_4->legend->setBorderPen(Qt::NoPen);
            ui->widget_4->graph(0)->setName("y陀螺常值漂移pBGy");
            ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

            QPen pen6;
            pen6.setColor(QColor(53,150,181));
            pen6.setWidthF(1.5);
            ui->widget_6->addGraph();
            ui->widget_6->graph(0)->setPen(pen6);
            ui->widget_6->graph(0)->setData(VpointNumber,VpBGz);
            ui->widget_6->graph(0)->rescaleAxes(true);
            ui->widget_6->legend->setVisible(true);
            ui->widget_6->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_6->legend->setBorderPen(Qt::NoPen);
            ui->widget_6->graph(0)->setName("z陀螺常值漂移pBGz");
            ui->widget_6->replot(QCustomPlot::rpQueuedReplot);



        }

        if(ui->comboBox->currentIndex()==3)
        {

            QPen pen;
            pen.setColor(QColor(255,0,0));
            pen.setWidthF(1.5);
            ui->widget->addGraph();
            ui->widget->graph(0)->setPen(pen);
            ui->widget->graph(0)->setData(VpointNumber,VBAx);
            ui->widget->graph(0)->rescaleAxes(true);
            ui->widget->legend->setVisible(true);
            ui->widget->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget->legend->setBorderPen(Qt::NoPen);
            ui->widget->graph(0)->setName("x加速度计零偏BAx");
            ui->widget->replot(QCustomPlot::rpQueuedReplot);

            QPen pen3;
            pen3.setColor(QColor(0,0,255));
            pen3.setWidthF(1.5);
            ui->widget_3->addGraph();
            ui->widget_3->graph(0)->setPen(pen3);
            ui->widget_3->graph(0)->setData(VpointNumber,VBAy);
            ui->widget_3->graph(0)->rescaleAxes(true);
            ui->widget_3->legend->setVisible(true);
            ui->widget_3->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_3->legend->setBorderPen(Qt::NoPen);
            ui->widget_3->graph(0)->setName("y加速度计零偏BAy");
            ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

            QPen pen5;
            pen5.setColor(QColor(53,150,181));
            pen5.setWidthF(1.5);
            ui->widget_5->addGraph();
            ui->widget_5->graph(0)->setPen(pen5);
            ui->widget_5->graph(0)->setData(VpointNumber,VBAz);
            ui->widget_5->graph(0)->rescaleAxes(true);
            ui->widget_5->legend->setVisible(true);
            ui->widget_5->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_5->legend->setBorderPen(Qt::NoPen);
            ui->widget_5->graph(0)->setName("z加速度计零偏BAz");
            ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

            QPen pen2;
            pen2.setColor(QColor(255,0,0));
            pen2.setWidthF(1.5);
            ui->widget_2->addGraph();
            ui->widget_2->graph(0)->setPen(pen2);
            ui->widget_2->graph(0)->setData(VpointNumber,VpBAx);
            ui->widget_2->graph(0)->rescaleAxes(true);
            ui->widget_2->legend->setVisible(true);
            ui->widget_2->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_2->legend->setBorderPen(Qt::NoPen);
            ui->widget_2->graph(0)->setName("x加速度计零偏pBAx");
            ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

            QPen pen4;
            pen4.setColor(QColor(0,0,255));
            pen4.setWidthF(1.5);
            ui->widget_4->addGraph();
            ui->widget_4->graph(0)->setPen(pen4);
            ui->widget_4->graph(0)->setData(VpointNumber,VpBAy);
            ui->widget_4->graph(0)->rescaleAxes(true);
            ui->widget_4->legend->setVisible(true);
            ui->widget_4->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_4->legend->setBorderPen(Qt::NoPen);
            ui->widget_4->graph(0)->setName("y加速度计零偏pBAy");
            ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

            QPen pen6;
            pen6.setColor(QColor(53,150,181));
            pen6.setWidthF(1.5);
            ui->widget_6->addGraph();
            ui->widget_6->graph(0)->setPen(pen6);
            ui->widget_6->graph(0)->setData(VpointNumber,VpBAz);
            ui->widget_6->graph(0)->rescaleAxes(true);
            ui->widget_6->legend->setVisible(true);
            ui->widget_6->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_6->legend->setBorderPen(Qt::NoPen);
            ui->widget_6->graph(0)->setName("z加速度计零偏pBAz");
            ui->widget_6->replot(QCustomPlot::rpQueuedReplot);


        }

        if(ui->comboBox->currentIndex()==4)
        {

            QPen pen;
            pen.setColor(QColor(255,0,0));
            pen.setWidthF(1.5);
            ui->widget->addGraph();
            ui->widget->graph(0)->setPen(pen);
            ui->widget->graph(0)->setData(VpointNumber,VdKGx);
            ui->widget->graph(0)->rescaleAxes(true);
            ui->widget->legend->setVisible(true);
            ui->widget->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget->legend->setBorderPen(Qt::NoPen);
            ui->widget->graph(0)->setName("x陀螺标度误差dKGx");
            ui->widget->replot(QCustomPlot::rpQueuedReplot);

            QPen pen3;
            pen3.setColor(QColor(0,0,255));
            pen3.setWidthF(1.5);
            ui->widget_3->addGraph();
            ui->widget_3->graph(0)->setPen(pen3);
            ui->widget_3->graph(0)->setData(VpointNumber,VdKGy);
            ui->widget_3->graph(0)->rescaleAxes(true);
            ui->widget_3->legend->setVisible(true);
            ui->widget_3->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_3->legend->setBorderPen(Qt::NoPen);
            ui->widget_3->graph(0)->setName("y陀螺标度误差dKGy");
            ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

            QPen pen5;
            pen5.setColor(QColor(53,150,181));
            pen5.setWidthF(1.5);
            ui->widget_5->addGraph();
            ui->widget_5->graph(0)->setPen(pen5);
            ui->widget_5->graph(0)->setData(VpointNumber,VdKGz);
            ui->widget_5->graph(0)->rescaleAxes(true);
            ui->widget_5->legend->setVisible(true);
            ui->widget_5->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_5->legend->setBorderPen(Qt::NoPen);
            ui->widget_5->graph(0)->setName("z陀螺标度误差dKGz");
            ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

            QPen pen2;
            pen2.setColor(QColor(255,0,0));
            pen2.setWidthF(1.5);
            ui->widget_2->addGraph();
            ui->widget_2->graph(0)->setPen(pen2);
            ui->widget_2->graph(0)->setData(VpointNumber,VpdKGx);
            ui->widget_2->graph(0)->rescaleAxes(true);
            ui->widget_2->legend->setVisible(true);
            ui->widget_2->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_2->legend->setBorderPen(Qt::NoPen);
            ui->widget_2->graph(0)->setName("x陀螺标度误差pdKGx");
            ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

            QPen pen4;
            pen4.setColor(QColor(0,0,255));
            pen4.setWidthF(1.5);
            ui->widget_4->addGraph();
            ui->widget_4->graph(0)->setPen(pen4);
            ui->widget_4->graph(0)->setData(VpointNumber,VpdKGy);
            ui->widget_4->graph(0)->rescaleAxes(true);
            ui->widget_4->legend->setVisible(true);
            ui->widget_4->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_4->legend->setBorderPen(Qt::NoPen);
            ui->widget_4->graph(0)->setName("y陀螺标度误差pdKGy");
            ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

            QPen pen6;
            pen6.setColor(QColor(53,150,181));
            pen6.setWidthF(1.5);
            ui->widget_6->addGraph();
            ui->widget_6->graph(0)->setPen(pen6);
            ui->widget_6->graph(0)->setData(VpointNumber,VpdKGz);
            ui->widget_6->graph(0)->rescaleAxes(true);
            ui->widget_6->legend->setVisible(true);
            ui->widget_6->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_6->legend->setBorderPen(Qt::NoPen);
            ui->widget_6->graph(0)->setName("z陀螺标度误差pdKGz");
            ui->widget_6->replot(QCustomPlot::rpQueuedReplot);


        }

        if(ui->comboBox->currentIndex()==5)
        {

            QPen pen;
            pen.setColor(QColor(255,0,0));
            pen.setWidthF(1.5);
            ui->widget->addGraph();
            ui->widget->graph(0)->setPen(pen);
            ui->widget->graph(0)->setData(VpointNumber,VdUGxy);
            ui->widget->graph(0)->rescaleAxes(true);
            ui->widget->legend->setVisible(true);
            ui->widget->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget->legend->setBorderPen(Qt::NoPen);
            ui->widget->graph(0)->setName("y对z陀螺安装误差dUGxy");
            ui->widget->replot(QCustomPlot::rpQueuedReplot);

            QPen pen3;
            pen3.setColor(QColor(0,0,255));
            pen3.setWidthF(1.5);
            ui->widget_3->addGraph();
            ui->widget_3->graph(0)->setPen(pen3);
            ui->widget_3->graph(0)->setData(VpointNumber,VdUGxz);
            ui->widget_3->graph(0)->rescaleAxes(true);
            ui->widget_3->legend->setVisible(true);
            ui->widget_3->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_3->legend->setBorderPen(Qt::NoPen);
            ui->widget_3->graph(0)->setName("z对y陀螺安装误差dUGxz");
            ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

            QPen pen5;
            pen5.setColor(QColor(53,150,181));
            pen5.setWidthF(1.5);
            ui->widget_5->addGraph();
            ui->widget_5->graph(0)->setPen(pen5);
            ui->widget_5->graph(0)->setData(VpointNumber,VdUGyz);
            ui->widget_5->graph(0)->rescaleAxes(true);
            ui->widget_5->legend->setVisible(true);
            ui->widget_5->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_5->legend->setBorderPen(Qt::NoPen);
            ui->widget_5->graph(0)->setName("z对x陀螺安装误差dUGyz");
            ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

            QPen pen2;
            pen2.setColor(QColor(255,0,0));
            pen2.setWidthF(1.5);
            ui->widget_2->addGraph();
            ui->widget_2->graph(0)->setPen(pen2);
            ui->widget_2->graph(0)->setData(VpointNumber,VpdUGxy);
            ui->widget_2->graph(0)->rescaleAxes(true);
            ui->widget_2->legend->setVisible(true);
            ui->widget_2->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_2->legend->setBorderPen(Qt::NoPen);
            ui->widget_2->graph(0)->setName("y对z陀螺安装误差pdUGxy");
            ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

            QPen pen4;
            pen4.setColor(QColor(0,0,255));
            pen4.setWidthF(1.5);
            ui->widget_4->addGraph();
            ui->widget_4->graph(0)->setPen(pen4);
            ui->widget_4->graph(0)->setData(VpointNumber,VpdUGxz);
            ui->widget_4->graph(0)->rescaleAxes(true);
            ui->widget_4->legend->setVisible(true);
            ui->widget_4->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_4->legend->setBorderPen(Qt::NoPen);
            ui->widget_4->graph(0)->setName("z对y陀螺安装误差pdUGxz");
            ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

            QPen pen6;
            pen6.setColor(QColor(53,150,181));
            pen6.setWidthF(1.5);
            ui->widget_6->addGraph();
            ui->widget_6->graph(0)->setPen(pen6);
            ui->widget_6->graph(0)->setData(VpointNumber,VpdUGyz);
            ui->widget_6->graph(0)->rescaleAxes(true);
            ui->widget_6->legend->setVisible(true);
            ui->widget_6->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_6->legend->setBorderPen(Qt::NoPen);
            ui->widget_6->graph(0)->setName("z对x陀螺安装误差pdUGyz");
            ui->widget_6->replot(QCustomPlot::rpQueuedReplot);



        }


        if(ui->comboBox->currentIndex()==6)
        {

            QPen pen;
            pen.setColor(QColor(255,0,0));
            pen.setWidthF(1.5);
            ui->widget->addGraph();
            ui->widget->graph(0)->setPen(pen);
            ui->widget->graph(0)->setData(VpointNumber,VdKAx);
            ui->widget->graph(0)->rescaleAxes(true);
            ui->widget->legend->setVisible(true);
            ui->widget->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget->legend->setBorderPen(Qt::NoPen);
            ui->widget->graph(0)->setName("x陀螺标度误差dKAx");
            ui->widget->replot(QCustomPlot::rpQueuedReplot);

            QPen pen3;
            pen3.setColor(QColor(0,0,255));
            pen3.setWidthF(1.5);
            ui->widget_3->addGraph();
            ui->widget_3->graph(0)->setPen(pen3);
            ui->widget_3->graph(0)->setData(VpointNumber,VdKAy);
            ui->widget_3->graph(0)->rescaleAxes(true);
            ui->widget_3->legend->setVisible(true);
            ui->widget_3->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_3->legend->setBorderPen(Qt::NoPen);
            ui->widget_3->graph(0)->setName("y陀螺标度误差dKAy");
            ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

            QPen pen5;
            pen5.setColor(QColor(53,150,181));
            pen5.setWidthF(1.5);
            ui->widget_5->addGraph();
            ui->widget_5->graph(0)->setPen(pen5);
            ui->widget_5->graph(0)->setData(VpointNumber,VdKAz);
            ui->widget_5->graph(0)->rescaleAxes(true);
            ui->widget_5->legend->setVisible(true);
            ui->widget_5->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_5->legend->setBorderPen(Qt::NoPen);
            ui->widget_5->graph(0)->setName("z陀螺标度误差dKAz");
            ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

            QPen pen2;
            pen2.setColor(QColor(255,0,0));
            pen2.setWidthF(1.5);
            ui->widget_2->addGraph();
            ui->widget_2->graph(0)->setPen(pen2);
            ui->widget_2->graph(0)->setData(VpointNumber,VpdKAx);
            ui->widget_2->graph(0)->rescaleAxes(true);
            ui->widget_2->legend->setVisible(true);
            ui->widget_2->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_2->legend->setBorderPen(Qt::NoPen);
            ui->widget_2->graph(0)->setName("x陀螺标度误差pdKAx");
            ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

            QPen pen4;
            pen4.setColor(QColor(0,0,255));
            pen4.setWidthF(1.5);
            ui->widget_4->addGraph();
            ui->widget_4->graph(0)->setPen(pen4);
            ui->widget_4->graph(0)->setData(VpointNumber,VpdKAy);
            ui->widget_4->graph(0)->rescaleAxes(true);
            ui->widget_4->legend->setVisible(true);
            ui->widget_4->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_4->legend->setBorderPen(Qt::NoPen);
            ui->widget_4->graph(0)->setName("y陀螺标度误差pdKAy");
            ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

            QPen pen6;
            pen6.setColor(QColor(53,150,181));
            pen6.setWidthF(1.5);
            ui->widget_6->addGraph();
            ui->widget_6->graph(0)->setPen(pen6);
            ui->widget_6->graph(0)->setData(VpointNumber,VpdKAz);
            ui->widget_6->graph(0)->rescaleAxes(true);
            ui->widget_6->legend->setVisible(true);
            ui->widget_6->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_6->legend->setBorderPen(Qt::NoPen);
            ui->widget_6->graph(0)->setName("z陀螺标度误差pdKAz");
            ui->widget_6->replot(QCustomPlot::rpQueuedReplot);

        }

        if(ui->comboBox->currentIndex()==7)
        {
            QPen pen;
            pen.setColor(QColor(255,0,0));
            pen.setWidthF(1.5);
            ui->widget->addGraph();
            ui->widget->graph(0)->setPen(pen);
            ui->widget->graph(0)->setData(VpointNumber,VdUAxy);
            ui->widget->graph(0)->rescaleAxes(true);
            ui->widget->legend->setVisible(true);
            ui->widget->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget->legend->setBorderPen(Qt::NoPen);
            ui->widget->graph(0)->setName("x对z加速度计安装误差dUAxy");
            ui->widget->replot(QCustomPlot::rpQueuedReplot);

            QPen pen3;
            pen3.setColor(QColor(0,0,255));
            pen3.setWidthF(1.5);
            ui->widget_3->addGraph();
            ui->widget_3->graph(0)->setPen(pen3);
            ui->widget_3->graph(0)->setData(VpointNumber,VdUAxz);
            ui->widget_3->graph(0)->rescaleAxes(true);
            ui->widget_3->legend->setVisible(true);
            ui->widget_3->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_3->legend->setBorderPen(Qt::NoPen);
            ui->widget_3->graph(0)->setName("x对y加速度计安装误差dUAxz");
            ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

            QPen pen5;
            pen5.setColor(QColor(53,150,181));
            pen5.setWidthF(1.5);
            ui->widget_5->addGraph();
            ui->widget_5->graph(0)->setPen(pen5);
            ui->widget_5->graph(0)->setData(VpointNumber,VdUAyx);
            ui->widget_5->graph(0)->rescaleAxes(true);
            ui->widget_5->legend->setVisible(true);
            ui->widget_5->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_5->legend->setBorderPen(Qt::NoPen);
            ui->widget_5->graph(0)->setName("y对x加速度计安装误差dUAyx");
            ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

            QPen pen2;
            pen2.setColor(QColor(255,0,0));
            pen2.setWidthF(1.5);
            ui->widget_2->addGraph();
            ui->widget_2->graph(0)->setPen(pen2);
            ui->widget_2->graph(0)->setData(VpointNumber,VpdUAxy);
            ui->widget_2->graph(0)->rescaleAxes(true);
            ui->widget_2->legend->setVisible(true);
            ui->widget_2->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_2->legend->setBorderPen(Qt::NoPen);
            ui->widget_2->graph(0)->setName("x对z加速度计安装误差pdUAxy");
            ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

            QPen pen4;
            pen4.setColor(QColor(0,0,255));
            pen4.setWidthF(1.5);
            ui->widget_4->addGraph();
            ui->widget_4->graph(0)->setPen(pen4);
            ui->widget_4->graph(0)->setData(VpointNumber,VpdUAxz);
            ui->widget_4->graph(0)->rescaleAxes(true);
            ui->widget_4->legend->setVisible(true);
            ui->widget_4->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_4->legend->setBorderPen(Qt::NoPen);
            ui->widget_4->graph(0)->setName("x对y加速度计安装误差pdUAxz");
            ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

            QPen pen6;
            pen6.setColor(QColor(53,150,181));
            pen6.setWidthF(1.5);
            ui->widget_6->addGraph();
            ui->widget_6->graph(0)->setPen(pen6);
            ui->widget_6->graph(0)->setData(VpointNumber,VpdUAyx);
            ui->widget_6->graph(0)->rescaleAxes(true);
            ui->widget_6->legend->setVisible(true);
            ui->widget_6->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_6->legend->setBorderPen(Qt::NoPen);
            ui->widget_6->graph(0)->setName("y对x加速度计安装误差pdUAyx");
            ui->widget_6->replot(QCustomPlot::rpQueuedReplot);



        }

        if(ui->comboBox->currentIndex()==8)
        {

            QPen pen;
            pen.setColor(QColor(255,0,0));
            pen.setWidthF(1.5);
            ui->widget->addGraph();
            ui->widget->graph(0)->setPen(pen);
            ui->widget->graph(0)->setData(VpointNumber,VdUAyz);
            ui->widget->graph(0)->rescaleAxes(true);
            ui->widget->legend->setVisible(true);
            ui->widget->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget->legend->setBorderPen(Qt::NoPen);
            ui->widget->graph(0)->setName("y对z加速度计安装误差dUAyz");
            ui->widget->replot(QCustomPlot::rpQueuedReplot);

            QPen pen3;
            pen3.setColor(QColor(0,0,255));
            pen3.setWidthF(1.5);
            ui->widget_3->addGraph();
            ui->widget_3->graph(0)->setPen(pen3);
            ui->widget_3->graph(0)->setData(VpointNumber,VdUAzx);
            ui->widget_3->graph(0)->rescaleAxes(true);
            ui->widget_3->legend->setVisible(true);
            ui->widget_3->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_3->legend->setBorderPen(Qt::NoPen);
            ui->widget_3->graph(0)->setName("z对y加速度计安装误差dUAzx");
            ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

            QPen pen5;
            pen5.setColor(QColor(53,150,181));
            pen5.setWidthF(1.5);
            ui->widget_5->addGraph();
            ui->widget_5->graph(0)->setPen(pen5);
            ui->widget_5->graph(0)->setData(VpointNumber,VdUAzy);
            ui->widget_5->graph(0)->rescaleAxes(true);
            ui->widget_5->legend->setVisible(true);
            ui->widget_5->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_5->legend->setBorderPen(Qt::NoPen);
            ui->widget_5->graph(0)->setName("z对x加速度计安装误差dUAzy");
            ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

            QPen pen2;
            pen2.setColor(QColor(255,0,0));
            pen2.setWidthF(1.5);
            ui->widget_2->addGraph();
            ui->widget_2->graph(0)->setPen(pen2);
            ui->widget_2->graph(0)->setData(VpointNumber,VpdUAyz);
            ui->widget_2->graph(0)->rescaleAxes(true);
            ui->widget_2->legend->setVisible(true);
            ui->widget_2->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_2->legend->setBorderPen(Qt::NoPen);
            ui->widget_2->graph(0)->setName("y对z加速度计安装误差pdUAyz");
            ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

            QPen pen4;
            pen4.setColor(QColor(0,0,255));
            pen4.setWidthF(1.5);
            ui->widget_4->addGraph();
            ui->widget_4->graph(0)->setPen(pen4);
            ui->widget_4->graph(0)->setData(VpointNumber,VpdUAzx);
            ui->widget_4->graph(0)->rescaleAxes(true);
            ui->widget_4->legend->setVisible(true);
            ui->widget_4->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_4->legend->setBorderPen(Qt::NoPen);
            ui->widget_4->graph(0)->setName("z对y加速度计安装误差pdUAzx");
            ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

            QPen pen6;
            pen6.setColor(QColor(53,150,181));
            pen6.setWidthF(1.5);
            ui->widget_6->addGraph();
            ui->widget_6->graph(0)->setPen(pen6);
            ui->widget_6->graph(0)->setData(VpointNumber,VpdUAzy);
            ui->widget_6->graph(0)->rescaleAxes(true);
            ui->widget_6->legend->setVisible(true);
            ui->widget_6->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
            ui->widget_6->legend->setBorderPen(Qt::NoPen);
            ui->widget_6->graph(0)->setName("z对x加速度计安装误差pdUAzy");
            ui->widget_6->replot(QCustomPlot::rpQueuedReplot);



        }

    }
}

void Widget::on_pushButton_5_clicked()
{
    char fnBindFile[100] = "config.ini";
    char strAppName[100];
    mstrcat(fnBindFile, PATHNAME);

    strcpy(strAppName, "CaliPara");
    WritePrivateProfileString(strAppName,"BGX",ui->lineEdit_BGX_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BGY",ui->lineEdit_BGY_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BGZ",ui->lineEdit_BGZ_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"KGX",ui->lineEdit_KGX_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"KGY",ui->lineEdit_KGY_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"KGZ",ui->lineEdit_KGZ_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UGXY",ui->lineEdit_UGXY_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UGXZ",ui->lineEdit_UGXZ_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UGYZ",ui->lineEdit_UGYZ_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BAX",ui->lineEdit_BAX_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BAY",ui->lineEdit_BAY_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"BAZ",ui->lineEdit_BAZ_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"KAX",ui->lineEdit_KAX_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"KAY",ui->lineEdit_KAY_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"KAZ",ui->lineEdit_KAZ_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UAXY",ui->lineEdit_UAXY_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UAXZ",ui->lineEdit_UAXZ_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UAYX",ui->lineEdit_UAYX_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UAYZ",ui->lineEdit_UAYZ_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UAZX",ui->lineEdit_UAZX_12->text().toLatin1(),fnBindFile);
    WritePrivateProfileString(strAppName,"UAZY",ui->lineEdit_UAZY_12->text().toLatin1(),fnBindFile);

    on_pushButton_6_clicked();
    on_pushButton_clicked();
}

void Widget::setDafultCurve()
{

}

void Widget::initDraw()
{
    QCustomPlot* customPlot = ui->widget;

       //四边安上坐标轴
    customPlot->axisRect()->setupFullAxesBox();
    customPlot->setBackground(QColor(220,220,220));
    // 使上下轴、左右轴范围同步
    connect(customPlot->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->xAxis2, SLOT(setRange(QCPRange)));
    connect(customPlot->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot->yAxis2, SLOT(setRange(QCPRange)));

    customPlot->xAxis->setBasePen(QPen(Qt::white, 1));  // 轴线的画笔
    customPlot->xAxis->setTickPen(QPen(Qt::white, 1));  // 轴刻度线的画笔
    customPlot->xAxis->setSubTickPen(QPen(Qt::white, 1)); // 轴子刻度线的画笔
    customPlot->xAxis->setTickLabelColor(Qt::black);  // 轴刻度文字颜色
//    customPlot->xAxis->setLabel("标签");  // 只有设置了标签，轴标签的颜色才会显示
//    customPlot->xAxis->setLabelColor(Qt::white);   // 轴标签颜色
    customPlot->xAxis->setTickLengthIn(3);       // 轴线内刻度的长度
    customPlot->xAxis->setTickLengthOut(5);      // 轴线外刻度的长度
    customPlot->xAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);  // 设置轴线结束时的风格为 实角三角形但内部有凹陷的形状， setLowerEnding设置轴线开始时的风格

    customPlot->xAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));     // 网格线(对应刻度)画笔
    customPlot->yAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
    customPlot->xAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine)); // 子网格线(对应子刻度)画笔
    customPlot->yAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
    customPlot->xAxis->grid()->setSubGridVisible(true);     // 显示子网格线
    customPlot->yAxis->grid()->setSubGridVisible(true);
    customPlot->xAxis->grid()->setZeroLinePen(QPen(Qt::black));   // 设置刻度为0时的网格线的画笔
    customPlot->yAxis->grid()->setZeroLinePen(QPen(Qt::black));
    customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    customPlot->selectionRect()->setPen(QPen(Qt::black,1,Qt::DashLine));//设置选框的样式：虚线
    customPlot->selectionRect()->setBrush(QBrush(QColor(0,0,100,50)));//设置选框的样式：半透明浅蓝
    customPlot->setSelectionRectMode(QCP::SelectionRectMode::srmZoom);

    QCustomPlot* customPlot2 = ui->widget_2;
       //四边安上坐标轴
    customPlot2->axisRect()->setupFullAxesBox();
    customPlot2->setBackground(QColor(220,220,220));
    // 使上下轴、左右轴范围同步
    connect(customPlot2->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot2->xAxis2, SLOT(setRange(QCPRange)));
    connect(customPlot2->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot2->yAxis2, SLOT(setRange(QCPRange)));

    customPlot2->xAxis->setBasePen(QPen(Qt::white, 1));  // 轴线的画笔
    customPlot2->xAxis->setTickPen(QPen(Qt::white, 1));  // 轴刻度线的画笔
    customPlot2->xAxis->setSubTickPen(QPen(Qt::white, 1)); // 轴子刻度线的画笔
    customPlot2->xAxis->setTickLabelColor(Qt::black);  // 轴刻度文字颜色
//    customPlot2->xAxis->setLabel("标签");  // 只有设置了标签，轴标签的颜色才会显示
//    customPlot2->xAxis->setLabelColor(Qt::white);   // 轴标签颜色
    customPlot2->xAxis->setTickLengthIn(3);       // 轴线内刻度的长度
    customPlot2->xAxis->setTickLengthOut(5);      // 轴线外刻度的长度
    customPlot2->xAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);  // 设置轴线结束时的风格为 实角三角形但内部有凹陷的形状， setLowerEnding设置轴线开始时的风格

    customPlot2->xAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));     // 网格线(对应刻度)画笔
    customPlot2->yAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
    customPlot2->xAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine)); // 子网格线(对应子刻度)画笔
    customPlot2->yAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
    customPlot2->xAxis->grid()->setSubGridVisible(true);     // 显示子网格线
    customPlot2->yAxis->grid()->setSubGridVisible(true);
    customPlot2->xAxis->grid()->setZeroLinePen(QPen(Qt::black));   // 设置刻度为0时的网格线的画笔
    customPlot2->yAxis->grid()->setZeroLinePen(QPen(Qt::black));
    customPlot2->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    customPlot2->selectionRect()->setPen(QPen(Qt::black,1,Qt::DashLine));//设置选框的样式：虚线
    customPlot2->selectionRect()->setBrush(QBrush(QColor(0,0,100,50)));//设置选框的样式：半透明浅蓝
    customPlot2->setSelectionRectMode(QCP::SelectionRectMode::srmZoom);

    QCustomPlot* customPlot3 = ui->widget_3;
       //四边安上坐标轴
    customPlot3->axisRect()->setupFullAxesBox();
    customPlot3->setBackground(QColor(220,220,220));
    // 使上下轴、左右轴范围同步
    connect(customPlot3->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot3->xAxis2, SLOT(setRange(QCPRange)));
    connect(customPlot3->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot3->yAxis2, SLOT(setRange(QCPRange)));

    customPlot3->xAxis->setBasePen(QPen(Qt::white, 1));  // 轴线的画笔
    customPlot3->xAxis->setTickPen(QPen(Qt::white, 1));  // 轴刻度线的画笔
    customPlot3->xAxis->setSubTickPen(QPen(Qt::white, 1)); // 轴子刻度线的画笔
    customPlot3->xAxis->setTickLabelColor(Qt::black);  // 轴刻度文字颜色
//    customPlot3->xAxis->setLabel("标签");  // 只有设置了标签，轴标签的颜色才会显示
//    customPlot3->xAxis->setLabelColor(Qt::white);   // 轴标签颜色
    customPlot3->xAxis->setTickLengthIn(3);       // 轴线内刻度的长度
    customPlot3->xAxis->setTickLengthOut(5);      // 轴线外刻度的长度
    customPlot3->xAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);  // 设置轴线结束时的风格为 实角三角形但内部有凹陷的形状， setLowerEnding设置轴线开始时的风格

    customPlot3->xAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));     // 网格线(对应刻度)画笔
    customPlot3->yAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
    customPlot3->xAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine)); // 子网格线(对应子刻度)画笔
    customPlot3->yAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
    customPlot3->xAxis->grid()->setSubGridVisible(true);     // 显示子网格线
    customPlot3->yAxis->grid()->setSubGridVisible(true);
    customPlot3->xAxis->grid()->setZeroLinePen(QPen(Qt::black));   // 设置刻度为0时的网格线的画笔
    customPlot3->yAxis->grid()->setZeroLinePen(QPen(Qt::black));
    customPlot3->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    customPlot3->selectionRect()->setPen(QPen(Qt::black,1,Qt::DashLine));//设置选框的样式：虚线
    customPlot3->selectionRect()->setBrush(QBrush(QColor(0,0,100,50)));//设置选框的样式：半透明浅蓝
    customPlot3->setSelectionRectMode(QCP::SelectionRectMode::srmZoom);

    QCustomPlot* customPlot4 = ui->widget_4;
       //四边安上坐标轴
    customPlot4->axisRect()->setupFullAxesBox();
    customPlot4->setBackground(QColor(220,220,220));
    // 使上下轴、左右轴范围同步
    connect(customPlot4->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot4->xAxis2, SLOT(setRange(QCPRange)));
    connect(customPlot4->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot4->yAxis2, SLOT(setRange(QCPRange)));

    customPlot4->xAxis->setBasePen(QPen(Qt::white, 1));  // 轴线的画笔
    customPlot4->xAxis->setTickPen(QPen(Qt::white, 1));  // 轴刻度线的画笔
    customPlot4->xAxis->setSubTickPen(QPen(Qt::white, 1)); // 轴子刻度线的画笔
    customPlot4->xAxis->setTickLabelColor(Qt::black);  // 轴刻度文字颜色
//    customPlot4->xAxis->setLabel("标签");  // 只有设置了标签，轴标签的颜色才会显示
//    customPlot4->xAxis->setLabelColor(Qt::white);   // 轴标签颜色
    customPlot4->xAxis->setTickLengthIn(3);       // 轴线内刻度的长度
    customPlot4->xAxis->setTickLengthOut(5);      // 轴线外刻度的长度
    customPlot4->xAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);  // 设置轴线结束时的风格为 实角三角形但内部有凹陷的形状， setLowerEnding设置轴线开始时的风格

    customPlot4->xAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));     // 网格线(对应刻度)画笔
    customPlot4->yAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
    customPlot4->xAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine)); // 子网格线(对应子刻度)画笔
    customPlot4->yAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
    customPlot4->xAxis->grid()->setSubGridVisible(true);     // 显示子网格线
    customPlot4->yAxis->grid()->setSubGridVisible(true);
    customPlot4->xAxis->grid()->setZeroLinePen(QPen(Qt::black));   // 设置刻度为0时的网格线的画笔
    customPlot4->yAxis->grid()->setZeroLinePen(QPen(Qt::black));
    customPlot4->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    customPlot4->selectionRect()->setPen(QPen(Qt::black,1,Qt::DashLine));//设置选框的样式：虚线
    customPlot4->selectionRect()->setBrush(QBrush(QColor(0,0,100,50)));//设置选框的样式：半透明浅蓝
    customPlot4->setSelectionRectMode(QCP::SelectionRectMode::srmZoom);

    QCustomPlot* customPlot5 = ui->widget_5;
       //四边安上坐标轴
    customPlot5->axisRect()->setupFullAxesBox();
    customPlot5->setBackground(QColor(220,220,220));
    // 使上下轴、左右轴范围同步
    connect(customPlot5->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot5->xAxis2, SLOT(setRange(QCPRange)));
    connect(customPlot5->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot5->yAxis2, SLOT(setRange(QCPRange)));

    customPlot5->xAxis->setBasePen(QPen(Qt::white, 1));  // 轴线的画笔
    customPlot5->xAxis->setTickPen(QPen(Qt::white, 1));  // 轴刻度线的画笔
    customPlot5->xAxis->setSubTickPen(QPen(Qt::white, 1)); // 轴子刻度线的画笔
    customPlot5->xAxis->setTickLabelColor(Qt::black);  // 轴刻度文字颜色
//    customPlot5->xAxis->setLabel("标签");  // 只有设置了标签，轴标签的颜色才会显示
//    customPlot5->xAxis->setLabelColor(Qt::white);   // 轴标签颜色
    customPlot5->xAxis->setTickLengthIn(3);       // 轴线内刻度的长度
    customPlot5->xAxis->setTickLengthOut(5);      // 轴线外刻度的长度
    customPlot5->xAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);  // 设置轴线结束时的风格为 实角三角形但内部有凹陷的形状， setLowerEnding设置轴线开始时的风格

    customPlot5->xAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));     // 网格线(对应刻度)画笔
    customPlot5->yAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
    customPlot5->xAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine)); // 子网格线(对应子刻度)画笔
    customPlot5->yAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
    customPlot5->xAxis->grid()->setSubGridVisible(true);     // 显示子网格线
    customPlot5->yAxis->grid()->setSubGridVisible(true);
    customPlot5->xAxis->grid()->setZeroLinePen(QPen(Qt::black));   // 设置刻度为0时的网格线的画笔
    customPlot5->yAxis->grid()->setZeroLinePen(QPen(Qt::black));
    customPlot5->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    customPlot5->selectionRect()->setPen(QPen(Qt::black,1,Qt::DashLine));//设置选框的样式：虚线
    customPlot5->selectionRect()->setBrush(QBrush(QColor(0,0,100,50)));//设置选框的样式：半透明浅蓝
    customPlot5->setSelectionRectMode(QCP::SelectionRectMode::srmZoom);

    QCustomPlot* customPlot6 = ui->widget_6;
       //四边安上坐标轴
    customPlot6->axisRect()->setupFullAxesBox();
    customPlot6->setBackground(QColor(220,220,220));
    // 使上下轴、左右轴范围同步
    connect(customPlot6->xAxis, SIGNAL(rangeChanged(QCPRange)), customPlot6->xAxis2, SLOT(setRange(QCPRange)));
    connect(customPlot6->yAxis, SIGNAL(rangeChanged(QCPRange)), customPlot6->yAxis2, SLOT(setRange(QCPRange)));

    customPlot6->xAxis->setBasePen(QPen(Qt::white, 1));  // 轴线的画笔
    customPlot6->xAxis->setTickPen(QPen(Qt::white, 1));  // 轴刻度线的画笔
    customPlot6->xAxis->setSubTickPen(QPen(Qt::white, 1)); // 轴子刻度线的画笔
    customPlot6->xAxis->setTickLabelColor(Qt::black);  // 轴刻度文字颜色
//    customPlot6->xAxis->setLabel("标签");  // 只有设置了标签，轴标签的颜色才会显示
//    customPlot6->xAxis->setLabelColor(Qt::white);   // 轴标签颜色
    customPlot6->xAxis->setTickLengthIn(3);       // 轴线内刻度的长度
    customPlot6->xAxis->setTickLengthOut(5);      // 轴线外刻度的长度
    customPlot6->xAxis->setUpperEnding(QCPLineEnding::esSpikeArrow);  // 设置轴线结束时的风格为 实角三角形但内部有凹陷的形状， setLowerEnding设置轴线开始时的风格

    customPlot6->xAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));     // 网格线(对应刻度)画笔
    customPlot6->yAxis->grid()->setPen(QPen(QColor(140, 140, 140), 1, Qt::DotLine));
    customPlot6->xAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine)); // 子网格线(对应子刻度)画笔
    customPlot6->yAxis->grid()->setSubGridPen(QPen(QColor(80, 80, 80), 1, Qt::DotLine));
    customPlot6->xAxis->grid()->setSubGridVisible(true);     // 显示子网格线
    customPlot6->yAxis->grid()->setSubGridVisible(true);
    customPlot6->xAxis->grid()->setZeroLinePen(QPen(Qt::black));   // 设置刻度为0时的网格线的画笔
    customPlot6->yAxis->grid()->setZeroLinePen(QPen(Qt::black));
    customPlot6->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    customPlot6->selectionRect()->setPen(QPen(Qt::black,1,Qt::DashLine));//设置选框的样式：虚线
    customPlot6->selectionRect()->setBrush(QBrush(QColor(0,0,100,50)));//设置选框的样式：半透明浅蓝
    customPlot6->setSelectionRectMode(QCP::SelectionRectMode::srmZoom);


}

void Widget::drawCurve()
{

    if(ui->comboBox->currentIndex()==0)
    {
        if(inde1IsNew == 1)
        {
             QPen pen;
             pen.setColor(QColor(255,0,0));
             pen.setWidthF(1.5);
             pen.setWidthF(1.5);
             ui->widget->addGraph();

             ui->widget->graph(0)->setPen(pen);
             ui->widget->graph(0)->setData(VpointNumber,VphiE);
             ui->widget->graph(0)->rescaleAxes(true);
             ui->widget->legend->setVisible(true);
             ui->widget->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget->legend->setBorderPen(Qt::NoPen);
             ui->widget->graph(0)->setName("横摇角误差phiE");
             ui->widget->replot(QCustomPlot::rpQueuedReplot);

//                 ui->widget->plotLayout()->addElement(1,0, ui->widget->legend);
//                ui->widget->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignCenter);
//                 ui->widget->plotLayout()->setRowStretchFactor(1, 0.00000001);  //设置行的伸展因子

             QPen pen3;
             pen3.setColor(QColor(0,0,255));
             pen3.setWidthF(1.5);
             ui->widget_3->addGraph();
             ui->widget_3->graph(0)->setPen(pen3);
             ui->widget_3->graph(0)->setData(VpointNumber,VphiN);
             ui->widget_3->graph(0)->rescaleAxes(true);
             ui->widget_3->legend->setVisible(true);
             ui->widget_3->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_3->legend->setBorderPen(Qt::NoPen);
             ui->widget_3->graph(0)->setName("纵摇角误差phiN");
             ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

             QPen pen5;
             pen5.setColor(QColor(53,150,181));
             pen5.setWidthF(1.5);
             ui->widget_5->addGraph();
             ui->widget_5->graph(0)->setPen(pen5);
             ui->widget_5->graph(0)->setData(VpointNumber,VphiU);
             ui->widget_5->graph(0)->rescaleAxes(true);
             ui->widget_5->legend->setVisible(true);
             ui->widget_5->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_5->legend->setBorderPen(Qt::NoPen);
             ui->widget_5->graph(0)->setName("航向角误差phiU");
             ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

             QPen pen2;
             pen2.setColor(QColor(255,0,0));
             pen2.setWidthF(1.5);
             ui->widget_2->addGraph();
             ui->widget_2->graph(0)->setPen(pen2);
             ui->widget_2->graph(0)->setData(VpointNumber,VpphiE);
             ui->widget_2->graph(0)->rescaleAxes(true);
             ui->widget_2->legend->setVisible(true);
             ui->widget_2->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_2->legend->setBorderPen(Qt::NoPen);
             ui->widget_2->graph(0)->setName("横摇角误差pphiE");
             ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

             QPen pen4;
             pen4.setColor(QColor(0,0,255));
             pen4.setWidthF(1.5);
             ui->widget_4->addGraph();
             ui->widget_4->graph(0)->setPen(pen4);
             ui->widget_4->graph(0)->setData(VpointNumber,VpphiN);
             ui->widget_4->graph(0)->rescaleAxes(true);
             ui->widget_4->legend->setVisible(true);
             ui->widget_4->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_4->legend->setBorderPen(Qt::NoPen);
             ui->widget_4->graph(0)->setName("纵摇角误差pphiN");
             ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

             QPen pen6;
             pen6.setColor(QColor(53,150,181));
             pen6.setWidthF(1.5);
             ui->widget_6->addGraph();
             ui->widget_6->graph(0)->setPen(pen6);
             ui->widget_6->graph(0)->setData(VpointNumber,VpphiU);
             ui->widget_6->graph(0)->rescaleAxes(true);
             ui->widget_6->legend->setVisible(true);
             ui->widget_6->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_6->legend->setBorderPen(Qt::NoPen);
             ui->widget_6->graph(0)->setName("航向角误差pphiU");
             ui->widget_6->replot(QCustomPlot::rpQueuedReplot);

             inde1IsNew=0;
        }
        else
        {
            ui->widget->graph(0)->setData(VpointNumber,VphiE);
            ui->widget->replot(QCustomPlot::rpQueuedReplot);
            ui->widget->graph(0)->rescaleAxes(true);

            ui->widget_3->graph(0)->setData(VpointNumber,VphiN);
            ui->widget_3->graph(0)->rescaleAxes(true);
            ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_5->graph(0)->setData(VpointNumber,VphiU);
            ui->widget_5->graph(0)->rescaleAxes(true);
            ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_2->graph(0)->setData(VpointNumber,VpphiE);
            ui->widget_2->graph(0)->rescaleAxes(true);
            ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_4->graph(0)->setData(VpointNumber,VpphiN);
            ui->widget_4->graph(0)->rescaleAxes(true);
            ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_6->graph(0)->setData(VpointNumber,VpphiU);
            ui->widget_6->graph(0)->rescaleAxes(true);
            ui->widget_6->replot(QCustomPlot::rpQueuedReplot);
        }

    }

    if(ui->comboBox->currentIndex()==1)
    {
        if(inde2IsNew == 1)
        {
             QPen pen;
             pen.setColor(QColor(255,0,0));
             pen.setWidthF(1.5);
             ui->widget->addGraph();
             ui->widget->graph(0)->setPen(pen);
             ui->widget->graph(0)->setData(VpointNumber,VdvE);
             ui->widget->graph(0)->rescaleAxes(true);
             ui->widget->legend->setVisible(true);
             ui->widget->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget->legend->setBorderPen(Qt::NoPen);
             ui->widget->graph(0)->setName("东速误差dvE");
             ui->widget->replot(QCustomPlot::rpQueuedReplot);

             QPen pen3;
             pen3.setColor(QColor(0,0,255));
             pen3.setWidthF(1.5);
             ui->widget_3->addGraph();
             ui->widget_3->graph(0)->setPen(pen3);
             ui->widget_3->graph(0)->setData(VpointNumber,VdvN);
             ui->widget_3->graph(0)->rescaleAxes(true);
             ui->widget_3->legend->setVisible(true);
             ui->widget_3->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_3->legend->setBorderPen(Qt::NoPen);
             ui->widget_3->graph(0)->setName("北速误差dvN");
             ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

             QPen pen5;
             pen5.setColor(QColor(53,150,181));
             pen5.setWidthF(1.5);
             ui->widget_5->addGraph();
             ui->widget_5->graph(0)->setPen(pen5);
             ui->widget_5->graph(0)->setData(VpointNumber,VdvU);
             ui->widget_5->graph(0)->rescaleAxes(true);
             ui->widget_5->legend->setVisible(true);
             ui->widget_5->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_5->legend->setBorderPen(Qt::NoPen);
             ui->widget_5->graph(0)->setName("垂速误差dvU");
             ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

             QPen pen2;
             pen2.setColor(QColor(255,0,0));
             pen2.setWidthF(1.5);
             ui->widget_2->addGraph();
             ui->widget_2->graph(0)->setPen(pen2);
             ui->widget_2->graph(0)->setData(VpointNumber,VpdvE);
             ui->widget_2->graph(0)->rescaleAxes(true);
             ui->widget_2->legend->setVisible(true);
             ui->widget_2->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_2->legend->setBorderPen(Qt::NoPen);
             ui->widget_2->graph(0)->setName("东速误差pdvE");
             ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

             QPen pen4;
             pen4.setColor(QColor(0,0,255));
             pen4.setWidthF(1.5);
             ui->widget_4->addGraph();
             ui->widget_4->graph(0)->setPen(pen4);
             ui->widget_4->graph(0)->setData(VpointNumber,VpdvN);
             ui->widget_4->graph(0)->rescaleAxes(true);
             ui->widget_4->legend->setVisible(true);
             ui->widget_4->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_4->legend->setBorderPen(Qt::NoPen);
             ui->widget_4->graph(0)->setName("北速误差pdvN");
             ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

             QPen pen6;
             pen6.setColor(QColor(53,150,181));
             pen6.setWidthF(1.5);
             ui->widget_6->addGraph();
             ui->widget_6->graph(0)->setPen(pen6);
             ui->widget_6->graph(0)->setData(VpointNumber,VpdvU);
             ui->widget_6->graph(0)->rescaleAxes(true);
             ui->widget_6->legend->setVisible(true);
             ui->widget_6->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_6->legend->setBorderPen(Qt::NoPen);
             ui->widget_6->graph(0)->setName("垂速误差pdvU");
             ui->widget_6->replot(QCustomPlot::rpQueuedReplot);

             inde2IsNew=0;
        }
        else
        {
            ui->widget->graph(0)->setData(VpointNumber,VdvE);
            ui->widget->replot(QCustomPlot::rpQueuedReplot);
            ui->widget->graph(0)->rescaleAxes(true);

            ui->widget_3->graph(0)->setData(VpointNumber,VdvN);
            ui->widget_3->graph(0)->rescaleAxes(true);
            ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_5->graph(0)->setData(VpointNumber,VdvU);
            ui->widget_5->graph(0)->rescaleAxes(true);
            ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_2->graph(0)->setData(VpointNumber,VpdvE);
            ui->widget_2->graph(0)->rescaleAxes(true);
            ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_4->graph(0)->setData(VpointNumber,VpdvN);
            ui->widget_4->graph(0)->rescaleAxes(true);
            ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_6->graph(0)->setData(VpointNumber,VpdvU);
            ui->widget_6->graph(0)->rescaleAxes(true);
            ui->widget_6->replot(QCustomPlot::rpQueuedReplot);
        }

    }

    if(ui->comboBox->currentIndex()==2)
    {
        if(inde3IsNew == 1)
        {
             QPen pen;
             pen.setColor(QColor(255,0,0));
             pen.setWidthF(1.5);
             ui->widget->addGraph();
             ui->widget->graph(0)->setPen(pen);
             ui->widget->graph(0)->setData(VpointNumber,VBGx);
             ui->widget->graph(0)->rescaleAxes(true);
             ui->widget->legend->setVisible(true);
             ui->widget->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget->legend->setBorderPen(Qt::NoPen);
             ui->widget->graph(0)->setName("x陀螺常值漂移BGx");
             ui->widget->replot(QCustomPlot::rpQueuedReplot);

             QPen pen3;
             pen3.setColor(QColor(0,0,255));
             pen3.setWidthF(1.5);
             ui->widget_3->addGraph();
             ui->widget_3->graph(0)->setPen(pen3);
             ui->widget_3->graph(0)->setData(VpointNumber,VBGy);
             ui->widget_3->graph(0)->rescaleAxes(true);
             ui->widget_3->legend->setVisible(true);
             ui->widget_3->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_3->legend->setBorderPen(Qt::NoPen);
             ui->widget_3->graph(0)->setName("y陀螺常值漂移BGy");
             ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

             QPen pen5;
             pen5.setColor(QColor(53,150,181));
             pen5.setWidthF(1.5);
             ui->widget_5->addGraph();
             ui->widget_5->graph(0)->setPen(pen5);
             ui->widget_5->graph(0)->setData(VpointNumber,VBGz);
             ui->widget_5->graph(0)->rescaleAxes(true);
             ui->widget_5->legend->setVisible(true);
             ui->widget_5->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_5->legend->setBorderPen(Qt::NoPen);
             ui->widget_5->graph(0)->setName("z陀螺常值漂移BGz");
             ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

             QPen pen2;
             pen2.setColor(QColor(255,0,0));
             pen2.setWidthF(1.5);
             ui->widget_2->addGraph();
             ui->widget_2->graph(0)->setPen(pen2);
             ui->widget_2->graph(0)->setData(VpointNumber,VpBGx);
             ui->widget_2->graph(0)->rescaleAxes(true);
             ui->widget_2->legend->setVisible(true);
             ui->widget_2->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_2->legend->setBorderPen(Qt::NoPen);
             ui->widget_2->graph(0)->setName("x陀螺常值漂移pBGx");
             ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

             QPen pen4;
             pen4.setColor(QColor(0,0,255));
             pen4.setWidthF(1.5);
             ui->widget_4->addGraph();
             ui->widget_4->graph(0)->setPen(pen4);
             ui->widget_4->graph(0)->setData(VpointNumber,VpBGy);
             ui->widget_4->graph(0)->rescaleAxes(true);
             ui->widget_4->legend->setVisible(true);
             ui->widget_4->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_4->legend->setBorderPen(Qt::NoPen);
             ui->widget_4->graph(0)->setName("y陀螺常值漂移pBGy");
             ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

             QPen pen6;
             pen6.setColor(QColor(53,150,181));
             pen6.setWidthF(1.5);
             ui->widget_6->addGraph();
             ui->widget_6->graph(0)->setPen(pen6);
             ui->widget_6->graph(0)->setData(VpointNumber,VpBGz);
             ui->widget_6->graph(0)->rescaleAxes(true);
             ui->widget_6->legend->setVisible(true);
             ui->widget_6->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_6->legend->setBorderPen(Qt::NoPen);
             ui->widget_6->graph(0)->setName("z陀螺常值漂移pBGz");
             ui->widget_6->replot(QCustomPlot::rpQueuedReplot);

             inde3IsNew=0;
        }
        else
        {
            ui->widget->graph(0)->setData(VpointNumber,VBGx);
            ui->widget->replot(QCustomPlot::rpQueuedReplot);
            ui->widget->graph(0)->rescaleAxes(true);

            ui->widget_3->graph(0)->setData(VpointNumber,VBGy);
            ui->widget_3->graph(0)->rescaleAxes(true);
            ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_5->graph(0)->setData(VpointNumber,VBGz);
            ui->widget_5->graph(0)->rescaleAxes(true);
            ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_2->graph(0)->setData(VpointNumber,VpBGx);
            ui->widget_2->graph(0)->rescaleAxes(true);
            ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_4->graph(0)->setData(VpointNumber,VpBGy);
            ui->widget_4->graph(0)->rescaleAxes(true);
            ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_6->graph(0)->setData(VpointNumber,VpBGz);
            ui->widget_6->graph(0)->rescaleAxes(true);
            ui->widget_6->replot(QCustomPlot::rpQueuedReplot);
        }

    }

    if(ui->comboBox->currentIndex()==3)
    {
        if(inde4IsNew == 1)
        {
             QPen pen;
             pen.setColor(QColor(255,0,0));
             pen.setWidthF(1.5);
             ui->widget->addGraph();
             ui->widget->graph(0)->setPen(pen);
             ui->widget->graph(0)->setData(VpointNumber,VBAx);
             ui->widget->graph(0)->rescaleAxes(true);
             ui->widget->legend->setVisible(true);
             ui->widget->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget->legend->setBorderPen(Qt::NoPen);
             ui->widget->graph(0)->setName("x加速度计零偏BAx");
             ui->widget->replot(QCustomPlot::rpQueuedReplot);

             QPen pen3;
             pen3.setColor(QColor(0,0,255));
             pen3.setWidthF(1.5);
             ui->widget_3->addGraph();
             ui->widget_3->graph(0)->setPen(pen3);
             ui->widget_3->graph(0)->setData(VpointNumber,VBAy);
             ui->widget_3->graph(0)->rescaleAxes(true);
             ui->widget_3->legend->setVisible(true);
             ui->widget_3->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_3->legend->setBorderPen(Qt::NoPen);
             ui->widget_3->graph(0)->setName("y加速度计零偏BAy");
             ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

             QPen pen5;
             pen5.setColor(QColor(53,150,181));
             pen5.setWidthF(1.5);
             ui->widget_5->addGraph();
             ui->widget_5->graph(0)->setPen(pen5);
             ui->widget_5->graph(0)->setData(VpointNumber,VBAz);
             ui->widget_5->graph(0)->rescaleAxes(true);
             ui->widget_5->legend->setVisible(true);
             ui->widget_5->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_5->legend->setBorderPen(Qt::NoPen);
             ui->widget_5->graph(0)->setName("z加速度计零偏BAz");
             ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

             QPen pen2;
             pen2.setColor(QColor(255,0,0));
             pen2.setWidthF(1.5);
             ui->widget_2->addGraph();
             ui->widget_2->graph(0)->setPen(pen2);
             ui->widget_2->graph(0)->setData(VpointNumber,VpBAx);
             ui->widget_2->graph(0)->rescaleAxes(true);
             ui->widget_2->legend->setVisible(true);
             ui->widget_2->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_2->legend->setBorderPen(Qt::NoPen);
             ui->widget_2->graph(0)->setName("x加速度计零偏pBAx");
             ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

             QPen pen4;
             pen4.setColor(QColor(0,0,255));
             pen4.setWidthF(1.5);
             ui->widget_4->addGraph();
             ui->widget_4->graph(0)->setPen(pen4);
             ui->widget_4->graph(0)->setData(VpointNumber,VpBAy);
             ui->widget_4->graph(0)->rescaleAxes(true);
             ui->widget_4->legend->setVisible(true);
             ui->widget_4->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_4->legend->setBorderPen(Qt::NoPen);
             ui->widget_4->graph(0)->setName("y加速度计零偏pBAy");
             ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

             QPen pen6;
             pen6.setColor(QColor(53,150,181));
             pen6.setWidthF(1.5);
             ui->widget_6->addGraph();
             ui->widget_6->graph(0)->setPen(pen6);
             ui->widget_6->graph(0)->setData(VpointNumber,VpBAz);
             ui->widget_6->graph(0)->rescaleAxes(true);
             ui->widget_6->legend->setVisible(true);
             ui->widget_6->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_6->legend->setBorderPen(Qt::NoPen);
             ui->widget_6->graph(0)->setName("z加速度计零偏pBAz");
             ui->widget_6->replot(QCustomPlot::rpQueuedReplot);

             inde4IsNew=0;
        }
        else
        {
            ui->widget->graph(0)->setData(VpointNumber,VBAx);
            ui->widget->replot(QCustomPlot::rpQueuedReplot);
            ui->widget->graph(0)->rescaleAxes(true);

            ui->widget_3->graph(0)->setData(VpointNumber,VBAy);
            ui->widget_3->graph(0)->rescaleAxes(true);
            ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_5->graph(0)->setData(VpointNumber,VBAz);
            ui->widget_5->graph(0)->rescaleAxes(true);
            ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_2->graph(0)->setData(VpointNumber,VpBAx);
            ui->widget_2->graph(0)->rescaleAxes(true);
            ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_4->graph(0)->setData(VpointNumber,VpBAy);
            ui->widget_4->graph(0)->rescaleAxes(true);
            ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_6->graph(0)->setData(VpointNumber,VpBAz);
            ui->widget_6->graph(0)->rescaleAxes(true);
            ui->widget_6->replot(QCustomPlot::rpQueuedReplot);
        }

    }

    if(ui->comboBox->currentIndex()==4)
    {
        if(inde5IsNew == 1)
        {
             QPen pen;
             pen.setColor(QColor(255,0,0));
             pen.setWidthF(1.5);
             ui->widget->addGraph();
             ui->widget->graph(0)->setPen(pen);
             ui->widget->graph(0)->setData(VpointNumber,VdKGx);
             ui->widget->graph(0)->rescaleAxes(true);
             ui->widget->legend->setVisible(true);
             ui->widget->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget->legend->setBorderPen(Qt::NoPen);
             ui->widget->graph(0)->setName("x陀螺标度误差dKGx");
             ui->widget->replot(QCustomPlot::rpQueuedReplot);

             QPen pen3;
             pen3.setColor(QColor(0,0,255));
             pen3.setWidthF(1.5);
             ui->widget_3->addGraph();
             ui->widget_3->graph(0)->setPen(pen3);
             ui->widget_3->graph(0)->setData(VpointNumber,VdKGy);
             ui->widget_3->graph(0)->rescaleAxes(true);
             ui->widget_3->legend->setVisible(true);
             ui->widget_3->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_3->legend->setBorderPen(Qt::NoPen);
             ui->widget_3->graph(0)->setName("y陀螺标度误差dKGy");
             ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

             QPen pen5;
             pen5.setColor(QColor(53,150,181));
             pen5.setWidthF(1.5);
             ui->widget_5->addGraph();
             ui->widget_5->graph(0)->setPen(pen5);
             ui->widget_5->graph(0)->setData(VpointNumber,VdKGz);
             ui->widget_5->graph(0)->rescaleAxes(true);
             ui->widget_5->legend->setVisible(true);
             ui->widget_5->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_5->legend->setBorderPen(Qt::NoPen);
             ui->widget_5->graph(0)->setName("z陀螺标度误差dKGz");
             ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

             QPen pen2;
             pen2.setColor(QColor(255,0,0));
             pen2.setWidthF(1.5);
             ui->widget_2->addGraph();
             ui->widget_2->graph(0)->setPen(pen2);
             ui->widget_2->graph(0)->setData(VpointNumber,VpdKGx);
             ui->widget_2->graph(0)->rescaleAxes(true);
             ui->widget_2->legend->setVisible(true);
             ui->widget_2->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_2->legend->setBorderPen(Qt::NoPen);
             ui->widget_2->graph(0)->setName("x陀螺标度误差pdKGx");
             ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

             QPen pen4;
             pen4.setColor(QColor(0,0,255));
             pen4.setWidthF(1.5);
             ui->widget_4->addGraph();
             ui->widget_4->graph(0)->setPen(pen4);
             ui->widget_4->graph(0)->setData(VpointNumber,VpdKGy);
             ui->widget_4->graph(0)->rescaleAxes(true);
             ui->widget_4->legend->setVisible(true);
             ui->widget_4->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_4->legend->setBorderPen(Qt::NoPen);
             ui->widget_4->graph(0)->setName("y陀螺标度误差pdKGy");
             ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

             QPen pen6;
             pen6.setColor(QColor(53,150,181));
             pen6.setWidthF(1.5);
             ui->widget_6->addGraph();
             ui->widget_6->graph(0)->setPen(pen6);
             ui->widget_6->graph(0)->setData(VpointNumber,VpdKGz);
             ui->widget_6->graph(0)->rescaleAxes(true);
             ui->widget_6->legend->setVisible(true);
             ui->widget_6->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_6->legend->setBorderPen(Qt::NoPen);
             ui->widget_6->graph(0)->setName("z陀螺标度误差pdKGz");
             ui->widget_6->replot(QCustomPlot::rpQueuedReplot);

             inde5IsNew=0;
        }
        else
        {
            ui->widget->graph(0)->setData(VpointNumber,VdKGx);
            ui->widget->replot(QCustomPlot::rpQueuedReplot);
            ui->widget->graph(0)->rescaleAxes(true);

            ui->widget_3->graph(0)->setData(VpointNumber,VdKGy);
            ui->widget_3->graph(0)->rescaleAxes(true);
            ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_5->graph(0)->setData(VpointNumber,VdKGz);
            ui->widget_5->graph(0)->rescaleAxes(true);
            ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_2->graph(0)->setData(VpointNumber,VpdKGx);
            ui->widget_2->graph(0)->rescaleAxes(true);
            ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_4->graph(0)->setData(VpointNumber,VpdKGy);
            ui->widget_4->graph(0)->rescaleAxes(true);
            ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_6->graph(0)->setData(VpointNumber,VpdKGz);
            ui->widget_6->graph(0)->rescaleAxes(true);
            ui->widget_6->replot(QCustomPlot::rpQueuedReplot);
        }

    }

    if(ui->comboBox->currentIndex()==5)
    {
        if(inde6IsNew == 1)
        {
             QPen pen;
             pen.setColor(QColor(255,0,0));
             pen.setWidthF(1.5);
             ui->widget->addGraph();
             ui->widget->graph(0)->setPen(pen);
             ui->widget->graph(0)->setData(VpointNumber,VdUGxy);
             ui->widget->graph(0)->rescaleAxes(true);
             ui->widget->legend->setVisible(true);
             ui->widget->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget->legend->setBorderPen(Qt::NoPen);
             ui->widget->graph(0)->setName("y对z陀螺安装误差dUGxy");
             ui->widget->replot(QCustomPlot::rpQueuedReplot);

             QPen pen3;
             pen3.setColor(QColor(0,0,255));
             pen3.setWidthF(1.5);
             ui->widget_3->addGraph();
             ui->widget_3->graph(0)->setPen(pen3);
             ui->widget_3->graph(0)->setData(VpointNumber,VdUGxz);
             ui->widget_3->graph(0)->rescaleAxes(true);
             ui->widget_3->legend->setVisible(true);
             ui->widget_3->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_3->legend->setBorderPen(Qt::NoPen);
             ui->widget_3->graph(0)->setName("z对y陀螺安装误差dUGxz");
             ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

             QPen pen5;
             pen5.setColor(QColor(53,150,181));
             pen5.setWidthF(1.5);
             ui->widget_5->addGraph();
             ui->widget_5->graph(0)->setPen(pen5);
             ui->widget_5->graph(0)->setData(VpointNumber,VdUGyz);
             ui->widget_5->graph(0)->rescaleAxes(true);
             ui->widget_5->legend->setVisible(true);
             ui->widget_5->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_5->legend->setBorderPen(Qt::NoPen);
             ui->widget_5->graph(0)->setName("z对x陀螺安装误差dUGyz");
             ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

             QPen pen2;
             pen2.setColor(QColor(255,0,0));
             pen2.setWidthF(1.5);
             ui->widget_2->addGraph();
             ui->widget_2->graph(0)->setPen(pen2);
             ui->widget_2->graph(0)->setData(VpointNumber,VpdUGxy);
             ui->widget_2->graph(0)->rescaleAxes(true);
             ui->widget_2->legend->setVisible(true);
             ui->widget_2->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_2->legend->setBorderPen(Qt::NoPen);
             ui->widget_2->graph(0)->setName("y对z陀螺安装误差pdUGxy");
             ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

             QPen pen4;
             pen4.setColor(QColor(0,0,255));
             pen4.setWidthF(1.5);
             ui->widget_4->addGraph();
             ui->widget_4->graph(0)->setPen(pen4);
             ui->widget_4->graph(0)->setData(VpointNumber,VpdUGxz);
             ui->widget_4->graph(0)->rescaleAxes(true);
             ui->widget_4->legend->setVisible(true);
             ui->widget_4->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_4->legend->setBorderPen(Qt::NoPen);
             ui->widget_4->graph(0)->setName("z对y陀螺安装误差pdUGxz");
             ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

             QPen pen6;
             pen6.setColor(QColor(53,150,181));
             pen6.setWidthF(1.5);
             ui->widget_6->addGraph();
             ui->widget_6->graph(0)->setPen(pen6);
             ui->widget_6->graph(0)->setData(VpointNumber,VpdUGyz);
             ui->widget_6->graph(0)->rescaleAxes(true);
             ui->widget_6->legend->setVisible(true);
             ui->widget_6->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_6->legend->setBorderPen(Qt::NoPen);
             ui->widget_6->graph(0)->setName("z对x陀螺安装误差pdUGyz");
             ui->widget_6->replot(QCustomPlot::rpQueuedReplot);

             inde6IsNew=0;
        }
        else
        {
            ui->widget->graph(0)->setData(VpointNumber,VdUGxy);
            ui->widget->replot(QCustomPlot::rpQueuedReplot);
            ui->widget->graph(0)->rescaleAxes(true);

            ui->widget_3->graph(0)->setData(VpointNumber,VdUGxz);
            ui->widget_3->graph(0)->rescaleAxes(true);
            ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_5->graph(0)->setData(VpointNumber,VdUGyz);
            ui->widget_5->graph(0)->rescaleAxes(true);
            ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_2->graph(0)->setData(VpointNumber,VpdUGxy);
            ui->widget_2->graph(0)->rescaleAxes(true);
            ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_4->graph(0)->setData(VpointNumber,VpdUGxz);
            ui->widget_4->graph(0)->rescaleAxes(true);
            ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_6->graph(0)->setData(VpointNumber,VpdUGyz);
            ui->widget_6->graph(0)->rescaleAxes(true);
            ui->widget_6->replot(QCustomPlot::rpQueuedReplot);
        }

    }


    if(ui->comboBox->currentIndex()==6)
    {
        if(inde7IsNew == 1)
        {
             QPen pen;
             pen.setColor(QColor(255,0,0));
             pen.setWidthF(1.5);
             ui->widget->addGraph();
             ui->widget->graph(0)->setPen(pen);
             ui->widget->graph(0)->setData(VpointNumber,VdKAx);
             ui->widget->graph(0)->rescaleAxes(true);
             ui->widget->legend->setVisible(true);
             ui->widget->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget->legend->setBorderPen(Qt::NoPen);
             ui->widget->graph(0)->setName("x陀螺标度误差dKAx");
             ui->widget->replot(QCustomPlot::rpQueuedReplot);

             QPen pen3;
             pen3.setColor(QColor(0,0,255));
             pen3.setWidthF(1.5);
             ui->widget_3->addGraph();
             ui->widget_3->graph(0)->setPen(pen3);
             ui->widget_3->graph(0)->setData(VpointNumber,VdKAy);
             ui->widget_3->graph(0)->rescaleAxes(true);
             ui->widget_3->legend->setVisible(true);
             ui->widget_3->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_3->legend->setBorderPen(Qt::NoPen);
             ui->widget_3->graph(0)->setName("y陀螺标度误差dKAy");
             ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

             QPen pen5;
             pen5.setColor(QColor(53,150,181));
             pen5.setWidthF(1.5);
             ui->widget_5->addGraph();
             ui->widget_5->graph(0)->setPen(pen5);
             ui->widget_5->graph(0)->setData(VpointNumber,VdKAz);
             ui->widget_5->graph(0)->rescaleAxes(true);
             ui->widget_5->legend->setVisible(true);
             ui->widget_5->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_5->legend->setBorderPen(Qt::NoPen);
             ui->widget_5->graph(0)->setName("z陀螺标度误差dKAz");
             ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

             QPen pen2;
             pen2.setColor(QColor(255,0,0));
             pen2.setWidthF(1.5);
             ui->widget_2->addGraph();
             ui->widget_2->graph(0)->setPen(pen2);
             ui->widget_2->graph(0)->setData(VpointNumber,VpdKAx);
             ui->widget_2->graph(0)->rescaleAxes(true);
             ui->widget_2->legend->setVisible(true);
             ui->widget_2->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_2->legend->setBorderPen(Qt::NoPen);
             ui->widget_2->graph(0)->setName("x陀螺标度误差pdKAx");
             ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

             QPen pen4;
             pen4.setColor(QColor(0,0,255));
             pen4.setWidthF(1.5);
             ui->widget_4->addGraph();
             ui->widget_4->graph(0)->setPen(pen4);
             ui->widget_4->graph(0)->setData(VpointNumber,VpdKAy);
             ui->widget_4->graph(0)->rescaleAxes(true);
             ui->widget_4->legend->setVisible(true);
             ui->widget_4->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_4->legend->setBorderPen(Qt::NoPen);
             ui->widget_4->graph(0)->setName("y陀螺标度误差pdKAy");
             ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

             QPen pen6;
             pen6.setColor(QColor(53,150,181));
             pen6.setWidthF(1.5);
             ui->widget_6->addGraph();
             ui->widget_6->graph(0)->setPen(pen6);
             ui->widget_6->graph(0)->setData(VpointNumber,VpdKAz);
             ui->widget_6->graph(0)->rescaleAxes(true);
             ui->widget_6->legend->setVisible(true);
             ui->widget_6->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_6->legend->setBorderPen(Qt::NoPen);
             ui->widget_6->graph(0)->setName("z陀螺标度误差pdKAz");
             ui->widget_6->replot(QCustomPlot::rpQueuedReplot);

             inde7IsNew=0;
        }
        else
        {
            ui->widget->graph(0)->setData(VpointNumber,VdKAx);
            ui->widget->replot(QCustomPlot::rpQueuedReplot);
            ui->widget->graph(0)->rescaleAxes(true);

            ui->widget_3->graph(0)->setData(VpointNumber,VdKAy);
            ui->widget_3->graph(0)->rescaleAxes(true);
            ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_5->graph(0)->setData(VpointNumber,VdKAy);
            ui->widget_5->graph(0)->rescaleAxes(true);
            ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_2->graph(0)->setData(VpointNumber,VpdKAx);
            ui->widget_2->graph(0)->rescaleAxes(true);
            ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_4->graph(0)->setData(VpointNumber,VpdKAy);
            ui->widget_4->graph(0)->rescaleAxes(true);
            ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_6->graph(0)->setData(VpointNumber,VpdKAz);
            ui->widget_6->graph(0)->rescaleAxes(true);
            ui->widget_6->replot(QCustomPlot::rpQueuedReplot);
        }

    }

    if(ui->comboBox->currentIndex()==7)
    {
        if(inde8IsNew == 1)
        {
             QPen pen;
             pen.setColor(QColor(255,0,0));
             pen.setWidthF(1.5);
             ui->widget->addGraph();
             ui->widget->graph(0)->setPen(pen);
             ui->widget->graph(0)->setData(VpointNumber,VdUAxy);
             ui->widget->graph(0)->rescaleAxes(true);
             ui->widget->legend->setVisible(true);
             ui->widget->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget->legend->setBorderPen(Qt::NoPen);
             ui->widget->graph(0)->setName("x对z加速度计安装误差dUAxy");
             ui->widget->replot(QCustomPlot::rpQueuedReplot);

             QPen pen3;
             pen3.setColor(QColor(0,0,255));
             pen3.setWidthF(1.5);
             ui->widget_3->addGraph();
             ui->widget_3->graph(0)->setPen(pen3);
             ui->widget_3->graph(0)->setData(VpointNumber,VdUAxz);
             ui->widget_3->graph(0)->rescaleAxes(true);
             ui->widget_3->legend->setVisible(true);
             ui->widget_3->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_3->legend->setBorderPen(Qt::NoPen);
             ui->widget_3->graph(0)->setName("x对y加速度计安装误差dUAxz");
             ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

             QPen pen5;
             pen5.setColor(QColor(53,150,181));
             pen5.setWidthF(1.5);
             ui->widget_5->addGraph();
             ui->widget_5->graph(0)->setPen(pen5);
             ui->widget_5->graph(0)->setData(VpointNumber,VdUAyx);
             ui->widget_5->graph(0)->rescaleAxes(true);
             ui->widget_5->legend->setVisible(true);
             ui->widget_5->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_5->legend->setBorderPen(Qt::NoPen);
             ui->widget_5->graph(0)->setName("y对x加速度计安装误差dUAyx");
             ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

             QPen pen2;
             pen2.setColor(QColor(255,0,0));
             pen2.setWidthF(1.5);
             ui->widget_2->addGraph();
             ui->widget_2->graph(0)->setPen(pen2);
             ui->widget_2->graph(0)->setData(VpointNumber,VpdUAxy);
             ui->widget_2->graph(0)->rescaleAxes(true);
             ui->widget_2->legend->setVisible(true);
             ui->widget_2->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_2->legend->setBorderPen(Qt::NoPen);
             ui->widget_2->graph(0)->setName("x对z加速度计安装误差pdUAxy");
             ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

             QPen pen4;
             pen4.setColor(QColor(0,0,255));
             pen4.setWidthF(1.5);
             ui->widget_4->addGraph();
             ui->widget_4->graph(0)->setPen(pen4);
             ui->widget_4->graph(0)->setData(VpointNumber,VpdUAxz);
             ui->widget_4->graph(0)->rescaleAxes(true);
             ui->widget_4->legend->setVisible(true);
             ui->widget_4->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_4->legend->setBorderPen(Qt::NoPen);
             ui->widget_4->graph(0)->setName("x对y加速度计安装误差pdUAxz");
             ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

             QPen pen6;
             pen6.setColor(QColor(53,150,181));
             pen6.setWidthF(1.5);
             ui->widget_6->addGraph();
             ui->widget_6->graph(0)->setPen(pen6);
             ui->widget_6->graph(0)->setData(VpointNumber,VpdUAyx);
             ui->widget_6->graph(0)->rescaleAxes(true);
             ui->widget_6->legend->setVisible(true);
             ui->widget_6->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_6->legend->setBorderPen(Qt::NoPen);
             ui->widget_6->graph(0)->setName("y对x加速度计安装误差pdUAyx");
             ui->widget_6->replot(QCustomPlot::rpQueuedReplot);

             inde8IsNew=0;
        }
        else
        {
            ui->widget->graph(0)->setData(VpointNumber,VdUAxy);
            ui->widget->replot(QCustomPlot::rpQueuedReplot);
            ui->widget->graph(0)->rescaleAxes(true);

            ui->widget_3->graph(0)->setData(VpointNumber,VdUAxz);
            ui->widget_3->graph(0)->rescaleAxes(true);
            ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_5->graph(0)->setData(VpointNumber,VdUAyx);
            ui->widget_5->graph(0)->rescaleAxes(true);
            ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_2->graph(0)->setData(VpointNumber,VpdUAxy);
            ui->widget_2->graph(0)->rescaleAxes(true);
            ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_4->graph(0)->setData(VpointNumber,VpdUAxz);
            ui->widget_4->graph(0)->rescaleAxes(true);
            ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_6->graph(0)->setData(VpointNumber,VpdUAyx);
            ui->widget_6->graph(0)->rescaleAxes(true);
            ui->widget_6->replot(QCustomPlot::rpQueuedReplot);
        }

    }

    if(ui->comboBox->currentIndex()==8)
    {
        if(inde9IsNew == 1)
        {
             QPen pen;
             pen.setColor(QColor(255,0,0));
             pen.setWidthF(1.5);
             ui->widget->addGraph();
             ui->widget->graph(0)->setPen(pen);
             ui->widget->graph(0)->setData(VpointNumber,VdUAyz);
             ui->widget->graph(0)->rescaleAxes(true);
             ui->widget->legend->setVisible(true);
             ui->widget->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget->legend->setBorderPen(Qt::NoPen);
             ui->widget->graph(0)->setName("y对z加速度计安装误差dUAyz");
             ui->widget->replot(QCustomPlot::rpQueuedReplot);

             QPen pen3;
             pen3.setColor(QColor(0,0,255));
             pen3.setWidthF(1.5);
             ui->widget_3->addGraph();
             ui->widget_3->graph(0)->setPen(pen3);
             ui->widget_3->graph(0)->setData(VpointNumber,VdUAzx);
             ui->widget_3->graph(0)->rescaleAxes(true);
             ui->widget_3->legend->setVisible(true);
             ui->widget_3->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_3->legend->setBorderPen(Qt::NoPen);
             ui->widget_3->graph(0)->setName("z对y加速度计安装误差dUAzx");
             ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

             QPen pen5;
             pen5.setColor(QColor(53,150,181));
             pen5.setWidthF(1.5);
             ui->widget_5->addGraph();
             ui->widget_5->graph(0)->setPen(pen5);
             ui->widget_5->graph(0)->setData(VpointNumber,VdUAzy);
             ui->widget_5->graph(0)->rescaleAxes(true);
             ui->widget_5->legend->setVisible(true);
             ui->widget_5->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_5->legend->setBorderPen(Qt::NoPen);
             ui->widget_5->graph(0)->setName("z对x加速度计安装误差dUAzy");
             ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

             QPen pen2;
             pen2.setColor(QColor(255,0,0));
             pen2.setWidthF(1.5);
             ui->widget_2->addGraph();
             ui->widget_2->graph(0)->setPen(pen2);
             ui->widget_2->graph(0)->setData(VpointNumber,VpdUAyz);
             ui->widget_2->graph(0)->rescaleAxes(true);
             ui->widget_2->legend->setVisible(true);
             ui->widget_2->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_2->legend->setBorderPen(Qt::NoPen);
             ui->widget_2->graph(0)->setName("y对z加速度计安装误差pdUAyz");
             ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

             QPen pen4;
             pen4.setColor(QColor(0,0,255));
             pen4.setWidthF(1.5);
             ui->widget_4->addGraph();
             ui->widget_4->graph(0)->setPen(pen4);
             ui->widget_4->graph(0)->setData(VpointNumber,VpdUAzx);
             ui->widget_4->graph(0)->rescaleAxes(true);
             ui->widget_4->legend->setVisible(true);
             ui->widget_4->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_4->legend->setBorderPen(Qt::NoPen);
             ui->widget_4->graph(0)->setName("z对y加速度计安装误差pdUAzx");
             ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

             QPen pen6;
             pen6.setColor(QColor(53,150,181));
             pen6.setWidthF(1.5);
             ui->widget_6->addGraph();
             ui->widget_6->graph(0)->setPen(pen6);
             ui->widget_6->graph(0)->setData(VpointNumber,VpdUAzy);
             ui->widget_6->graph(0)->rescaleAxes(true);
             ui->widget_6->legend->setVisible(true);
             ui->widget_6->legend->setBrush(QBrush(Qt::transparent));     //设置图例透明无边框
             ui->widget_6->legend->setBorderPen(Qt::NoPen);
             ui->widget_6->graph(0)->setName("z对x加速度计安装误差pdUAzy");
             ui->widget_6->replot(QCustomPlot::rpQueuedReplot);

             inde9IsNew=0;
        }
        else
        {
            ui->widget->graph(0)->setData(VpointNumber,VdUAyz);
            ui->widget->replot(QCustomPlot::rpQueuedReplot);
            ui->widget->graph(0)->rescaleAxes(true);

            ui->widget_3->graph(0)->setData(VpointNumber,VdUAzx);
            ui->widget_3->graph(0)->rescaleAxes(true);
            ui->widget_3->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_5->graph(0)->setData(VpointNumber,VdUAzy);
            ui->widget_5->graph(0)->rescaleAxes(true);
            ui->widget_5->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_2->graph(0)->setData(VpointNumber,VpdUAyz);
            ui->widget_2->graph(0)->rescaleAxes(true);
            ui->widget_2->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_4->graph(0)->setData(VpointNumber,VpdUAzx);
            ui->widget_4->graph(0)->rescaleAxes(true);
            ui->widget_4->replot(QCustomPlot::rpQueuedReplot);

            ui->widget_6->graph(0)->setData(VpointNumber,VpdUAzy);
            ui->widget_6->graph(0)->rescaleAxes(true);
            ui->widget_6->replot(QCustomPlot::rpQueuedReplot);
        }

    }

}

void Widget::on_pushButton_9_clicked()
{
    stopRun = 1;
}

void Widget::closeEvent(QCloseEvent *e)
{

    #ifdef Q_WS_WIN
        DWORD pid = GetProcessId();
    #else
        int pid = getpid(); //获取进程id
        qDebug()<<pid;
    #endif
        QProcess p;
        QString cmd = QString("taskkill /F /PID %1 /T").arg(pid);
        p.execute(cmd);
        p.close();


}
