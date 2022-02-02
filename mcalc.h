



#ifndef MCALC_H
#define MCALC_H
#include "common.h"
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */
/******************通用算法函数区**********************/
INT32_T matinv(FLOAT64_T b[],	const FLOAT64_T a[], const INT32_T dim);
void xmult(FLOAT64_T c[], const FLOAT64_T a[], const FLOAT64_T b[]);
FLOAT64_T normpower(const FLOAT64_T x[], int dim);
FLOAT64_T norm(const FLOAT64_T x[], INT32_T dim);
void atti2quat(FLOAT64_T Qnb[], const FLOAT64_T atti[]);
void atti2dmat(FLOAT64_T Cnb[], const FLOAT64_T atti[]);
void quatmult(FLOAT64_T r[], const FLOAT64_T q[], const FLOAT64_T p[]);
void quat2dmat(FLOAT64_T Cnb[], const FLOAT64_T Qnb[]);
void dmat2atti(FLOAT64_T atti[], const FLOAT64_T Cnb[]);
void dmat2quat(FLOAT64_T Qnb[], const FLOAT64_T Cnb[]);
void dang2dquat(FLOAT64_T q[], const FLOAT64_T ang[]);
void attiTrans(FLOAT64_T atti_sys[], const FLOAT64_T Cpb[], const FLOAT64_T Cnp[]);
/******************通用算法函数区结束********************/

/******************粗对准算法函数区**********************/
void coarseAlign(FLOAT64_T Qnp_cur[], FLOAT64_T Cnp_cur[], FLOAT64_T atti_cur[], const FLOAT64_T v_ip0_t1[], const FLOAT64_T v_ip0_t2[], const FLOAT64_T u_i0_t1[], const FLOAT64_T u_i0_t2[], const FLOAT64_T Cip0p[], const FLOAT64_T Ci0n[]);
void velUpdate(FLOAT64_T Qip0p_cur[], FLOAT64_T Cip0p_cur[], FLOAT64_T Ci0n[], FLOAT64_T v_ip0_cur[], FLOAT64_T u_i0_cur[], const FLOAT64_T Qip0p_pre[], const FLOAT64_T v_ip0_pre[], const FLOAT64_T u_i0_pre[], const FLOAT64_T dtheta_ip_p[], const FLOAT64_T dv_ip_p[], const FLOAT64_T posn_init[], const FLOAT64_T posn_ref[], const FLOAT64_T vel_ref[], const UINT32_T k, const FLOAT64_T period);
/******************粗对准算法函数区结束******************/

/******************导航迭代更新算法函数区**********************/
void attiUpdate(FLOAT64_T Qnp_cur[], FLOAT64_T Cnp_cur[], FLOAT64_T atti_cur[], const FLOAT64_T Qnp_pre[], const FLOAT64_T dtheta_ip_p[], const FLOAT64_T omega_in_n[], const FLOAT64_T period);
void naviUpdateK(FLOAT64_T vel_cur[], FLOAT64_T posn_cur[], FLOAT64_T omega_in_n[], FLOAT64_T vel_damp[], FLOAT64_T posn_damp_cur[], FLOAT64_T f_d[], const FLOAT64_T vel_pre[], const FLOAT64_T posn_pre[], const FLOAT64_T posn_damp_pre[], const FLOAT64_T vel_ref[], const FLOAT64_T posn_ref[], const FLOAT64_T f_n[], const FLOAT64_T period);
void forceUpdate(FLOAT64_T f_n[], FLOAT64_T Cpn_cur[], const FLOAT64_T Cnp_cur[], const FLOAT64_T dv_p[], const FLOAT64_T period);
void shiftFilter(FLOAT64_T vel_d_cur[], FLOAT64_T shift_d_cur[], const FLOAT64_T vel_d_pre[], const FLOAT64_T shift_d_pre[], const FLOAT64_T atti_cur[], const FLOAT64_T f_d[], const FLOAT64_T period);
/******************导航迭代更新算法函数区结束******************/

void InitBDData(void);
void BDPredictPHI(FLOAT64_T PHI[], FLOAT64_T FUN[], const FLOAT64_T vel_pre[], const FLOAT64_T posn_pre[], const FLOAT64_T omega_b[], const FLOAT64_T f_b[], const FLOAT64_T Cbn[], const FLOAT64_T f_n[], const FLOAT64_T period);
void BDPredictQd(FLOAT64_T Qd[], const FLOAT64_T FUN[], const FLOAT64_T omega_b[], const FLOAT64_T f_b[], const FLOAT64_T Cbn[], const FLOAT64_T period);
void BDPredictEnd(FLOAT64_T Xkk[], FLOAT64_T Pkk[], const FLOAT64_T X_pre[], const FLOAT64_T P_pre[], const FLOAT64_T PHI[], const FLOAT64_T Qd[]);
INT32_T  BDUpdate(FLOAT64_T X_cur[], FLOAT64_T P_cur[], const FLOAT64_T Xkk[], const FLOAT64_T Pkk[], const FLOAT64_T delta_vel[]);
void BDAdjust(FLOAT64_T Qnp_out[], FLOAT64_T Cnp_out[], FLOAT64_T atti_out[], FLOAT64_T vel_out[], FLOAT64_T phi_est[], FLOAT64_T gyro_bias_est[], FLOAT64_T acc_bias_est[], const FLOAT64_T X_cur[], const FLOAT64_T Qnp_pre[], const FLOAT64_T vel_pre[]);

/******************Kalman滤波算法函数区**********************/
void KFPredictPHI(FLOAT64_T PHI[], FLOAT64_T FUN[], const FLOAT64_T vel_pre[], const FLOAT64_T posn_pre[], const FLOAT64_T Cpn[], const FLOAT64_T f_n[], const FLOAT64_T period);
void KFPredictQd(FLOAT64_T Qd[], const FLOAT64_T FUN[], const FLOAT64_T Cpn[], const FLOAT64_T period);
void KFPredictEnd(FLOAT64_T Xkk[], FLOAT64_T Pkk[], const FLOAT64_T X_pre[], const FLOAT64_T P_pre[], const FLOAT64_T PHI[], const FLOAT64_T Qd[]);
INT32_T  KFUpdate(FLOAT64_T X_cur[], FLOAT64_T P_cur[], const FLOAT64_T Xkk[], const FLOAT64_T Pkk[], const FLOAT64_T delta_vel[], const FLOAT64_T delta_posn[], UCHAR_T meas_mode);
void KFAdjust(FLOAT64_T Qnp_out[], FLOAT64_T Cnp_out[], FLOAT64_T atti_out[], FLOAT64_T vel_out[], FLOAT64_T posn_out[], FLOAT64_T phi_est[], FLOAT64_T gyro_bias_est[], FLOAT64_T acc_bias_est[], const FLOAT64_T X_cur[], const FLOAT64_T Qnp_pre[], const FLOAT64_T vel_pre[], const FLOAT64_T posn_pre[]);
void KFReset(void);
void attiFeedBack(void);
void velFeedBack(void);
void posnFeedBack(void);
/******************Kalman滤波算法函数区结束******************/
void checkMovingSta(void);
FLOAT32_T smFilter(FLOAT32_T *x, INT32_T len); // 中值滤波

/******************接口响应函数区**********************/
void SetPosnInit(void);
void SetVelInit(void);
void SetHeadingInit(void);
void SetSysAttiZero(void);
void SetExAttiZero(void);
void SetGyroPara(void);
void SetAccPara(void);
void SetBiasEqu(void);
void SetGyroBiasCoef(void);
void SetAccZeroCoef(void);
void SetLeverArm(void);
void SetSwitchCmd(void);
/******************接口响应函数区结束******************/

/******************算法数据初始化函数区**********************/
void InitNavData(void);
void InitKFData(void);

/******************算法数据初始化函数区结束******************/

/******************算法计算流程函数区**********************/
void InertTrans2(void);
void InertTrans(void);
void InputProc(void);
void CalcProc(void);
void FilterProc(void);
void CalcAttiRate(void);
/******************算法计算流程函数区结束******************/
void TurnCalibrate(void);

/******************状态切换函数区**********************/
void TurnReady(void);
void TurnCoarseAlign(void);
void TurnFineAlign(void);
void TurnNoDamp(void);
void TurnLevelDamp(void);
void TurnPosnAided(void);
void TurnCoarseAzimuth();
void TurnCompass(void);
/******************状态切换函数区结束******************/
void calcLevelCoef(FLOAT64_T K[], const FLOAT64_T varsigma, const FLOAT64_T sigma);
void calcCompassCoef(FLOAT64_T K[], const FLOAT64_T varsigma, const FLOAT64_T sigma);

#ifdef __cplusplus
}
#endif /* __cplusplus */
extern BIND_PARA_T g_stBindPara;
extern INERT_DATA_T g_stInertData;
extern NAV_DATA_T g_stNavData;
extern KF_PARA_T g_stKfPara;
extern BD_PARA_T g_stBDPara;
extern IMU_INFO_T g_stImuInfo;
extern REF_MSG_T g_stRefMsg;
extern HOST_MSG_T g_stHostMsg;
extern PULSE_MSG_T g_stPulseMsg;

extern unsigned int g_uiCntClock;
extern unsigned char g_ucFlagClock;

const FLOAT64_T g_dCoefRate[MAX_RATE_WIDTH] = {1.23215230037023,	1.08462027982689,	0.942558818557048,	0.805967916560719,	0.674847573837903,	0.549197790388595,	
0.429018566212796,	0.314309901310510,	0.205071795681732,	0.101304249326464,	0.00300726224470535,	-0.0898191655635409,	
-0.177175034098280,	-0.259060343359508,	-0.335475093347225,	-0.406419284061433,	-0.471892915502131,	-0.531895987669318,	
-0.586428500562998,	-0.635490454183166,	-0.679081848529824,	-0.717202683602972,	-0.749852959402610,	-0.777032675928738,	
-0.798741833181358,	-0.814980431160465,	-0.825748469866064,	-0.831045949298152,	-0.830872869456730,	-0.825229230341798,	
-0.814115031953356,	-0.797530274291406,	-0.775474957355944,	-0.747949081146972,	-0.714952645664491,	-0.676485650908499,	
-0.632548096878998,	-0.583139983575986,	-0.528261310999465,	-0.467912079149434,	-0.402092288025893,	-0.330801937628840,	
-0.254041027958280,	-0.171809559014208,	-0.0841075307966277,	0.00906505669446397,	0.107708203459065,	0.211821909497176,	
0.321406174808797,	0.436460999393928,	0.556986383252569,	0.682982326384721,	0.814448828790382,	0.951385890469553,	
1.09379351142223,	1.24167169164842,	1.39502043114813,	1.55383972992134,	1.71812958796806,	1.88789000528829};
const FLOAT64_T g_dCoefRate2[MAX_RATE_WIDTH2] = {5.33101045296172, 4.36076119002953, 3.44552751484719, 2.58530942741473, 1.78010692773211,
1.02992001579937, 0.334748691616481, -0.305407044816538, -0.890547193499693, -1.42067175443299,
-1.89578072761642, -2.31587411304999, -2.68095191073370, -2.99101412066754, -3.24606074285152,
-3.44609177728563, -3.59110722396989, -3.68110708290428, -3.71609135408881, -3.69606003752348,
-3.62101313320828, -3.49095064114322, -3.30587256132830, -3.06577889376351, -2.77066963844887,
-2.42054479538436, -2.01540436456998, -1.55524834600574, -1.04007673969165, -0.469889545627681,
0.155313236186143, 0.835531605749831, 1.57076556306338, 2.36101510812679, 3.20628024094007,
4.10656096150321, 5.06185726981621, 6.07216916587908, 7.13749664969181, 8.25783972125440};

#endif

