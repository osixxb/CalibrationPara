
#ifndef _COMMON_H
#define _COMMON_H
//#include <qmath.h>
#include <string.h>
//#include <qmath.h>

#define DSP_VERSION_NO	"V1.0-A-20160822"	// DSP版本号

#ifndef CONST_DEFINE
#define CONST_DEFINE
#define PI 3.1415926535897932384626433832795 // 圆周率
#define UINT32_INF 0xFFFFFFFF // 无符号整形最大值
/* unit trans const */
#define DEG2RAD (PI/180.0)			// 角度转弧度
#define RAD2DEG (180.0/PI)			// 弧度转角度
#define MIN2RAD (PI/10800.0)			// 角分转弧度
#define RAD2MIN (10800.0/PI)			// 弧度转角分
#define RAD2SEC (648000.0/PI)			// 弧度转角秒
#define SEC2RAD (PI/648000.0)			// 角秒转弧度
#define RPS2DPH (648000.0/PI)			// rad/s转deg/h
#define DPH2RPS (PI/648000.0)			// deg/h转rad/s
#define KN2MPS	(1852.0/3600.0)		// kn转m/s
#define MPS2KN	(3600.0/1852.0)		// m/s转kn
#define TRANS_LAT	(2147483648.0/90.0)
#define TRANS_LON	(2147483648.0/180.0)
#define TRANS_VEL	(2147483648.0/1000.0)
#define TRANS_HDG	(2147483648.0/180.0)
#define TRANS_ATTI	(2147483648.0/180.0)
#define TRANS_OMEGA	(2147483648.0/400.0)
#define TRANS_SHIFT	(32768.0/100.0)


/******************自定义数据类型**********************/
#ifndef TYPE_DEFINE
#define TYPE_DEFINE 
#define CHAR_T		signed char // 8bits
#define INT16_T	signed short // 16bits
#define INT32_T	signed int // 32bits
#define INT40_T	signed long // 40bits
#define UCHAR_T	unsigned char // 8bits
#define UINT16_T	unsigned short // 16bits
#define UINT32_T	unsigned int // 32bits
#define UINT40_T	unsigned long // 40bits
#define FLOAT32_T	float // 32bits
#define FLOAT64_T	double // 64bits
#define VOID_T		void
#endif


#define LEN_KEEP_GPS_POSN	1 // 1s
#define LEN_KEEP_GPS_VEL		4 // 4s
#define LEN_KEEP_LOG_VEL		4 // 4s
#define LEN_KEEP_DOP_VW		6 // 6s
#define LEN_KEEP_DOP_VE		6 // 6s


/* earth parameter const */
#define GLV_G0 9.78031811 // gravity coefficient
#define GLV_G  9.8010104 // approximate gravity
#define GLV_RE 6378137.0 // length of the semi-major axis
#define GLV_RP 6356752.0 // length of the semi-minor axis
#define GLV_R sqrt(GLV_RE*GLV_RP) // length of the semidiameter
#define GLV_F 1.0/298.257223563 // flattening of the ellipsoid
#define GLV_E 0.0818191908426 // major eccentricity of the ellipsoid
#define GLV_E2 0.00669437999013779969797476 // GLV_E^2
#define OMEGA_IE 7.2921158e-5 // earth's rate
#define local_g(lat) (GLV_G0*(1.0+5.3024e-3*(sin(lat)*sin(lat))-5.9e-6*(sin(2*lat)*sin(2*lat))))

/* NMEA 0183 */
#define LF '\n' // 0x0a
#define CR '\r' // 0x0d

#endif


/* work mode */
// ready
#define U8_WORK_READY					0x00 // 准备
// align
#define U8_WORK_SELFALIGN_COARSE		0x01 // 自对准(粗对准)
#define U8_WORK_SELFALIGN_FINE			0x11 // 自对准(精对准)

#define U8_WORK_COMPALIGN_COARSE		0x02 // 罗经对准(粗对准)
#define U8_WORK_COMPALIGN_AZIMUTH		0x12 // 罗经对准(方位对准)

#define U8_WORK_TRANSALIGN_COARSE		0x03 // 传递对准(粗对准)
#define U8_WORK_TRANSALIGN_FINE			0x13 // 传递对准(精对准)
// self navigation
#define U8_WORK_SELFNAV_NODAMP			0x04 // 自主导航(无阻尼)
#define U8_WORK_SELFNAV_LEDAMP			0x14 // 自主导航(水平阻尼)
#define U8_WORK_SELFNAV_ENDAMP			0x24 // 自主导航(全阻尼)
// compass
#define U8_WORK_COMPASS					0x05 // 罗经
// integration
#define U8_WORK_INTENAV_GPS				0x06 // 组合导航(卫导)
#define U8_WORK_INTENAV_DVL				0x16 // 组合导航(多普勒)
#define U8_WORK_INTENAV_CNS				0x26 // 组合导航(星光)
#define U8_WORK_INTENAV_USBL			0x36 // 组合导航(超短基线)
// calibration
#define U8_WORK_CALIBRATE_COARSE		0x07 // 在舰标定(粗对准)
#define U8_WORK_CALIBRATE_POSN1			0x17 // 在舰标定(位置1)
#define U8_WORK_CALIBRATE_POSN2			0x27 // 在舰标定(位置2)
#define U8_WORK_CALIBRATE_SYS			0x37 // 系统级标定
/* operate mode */
#define U8_OPER_AUTO					0x00 // 自动
#define U8_OPER_MANUAL				0x80 // 手动

/* work palce */
#define U8_SAIL_HARBOR				0x00 // 码头
#define U8_SAIL_ONSEA					0x40 // 海上

/* align time */
#define U8_ALIGN_TIME_10M			0x01 // 对准时间: 10min
#define U8_ALIGN_TIME_01H			0x02 // 对准时间: 1h
#define U8_ALIGN_TIME_12H			0x06 // 对准时间: 12h

/* Kalman filter measure mode */
#define U8_KF_MEAS_VEL				0x01 // 速度
#define U8_KF_MEAS_POSN				0x02 // 位置

/* flag ref msg */
/*
#define U16_REF_BIND_POSN			0x0001 // 装订位置
#define U16_REF_GPS_POSN			0x0002 // GPS位置
#define U16_REF_ZERO_VEL			0x0004 // 0速度
#define U16_REF_GPS_VEL				0x0008  // GPS速度 
#define U16_REF_LOG_VEL			0x0010 // 电磁计程仪/电磁速度
#define U16_REF_DVL_VW			0x0020 // 多普勒计程仪-对水速度
#define U16_REF_DVL_VE			0x0040 // 多普勒计程仪-对底速度
#define U16_REF_TIME_STAMP		0x0080 // 时码有效
*/
/* 导航信息有效性 */
#define U8_VAL_TIMSCODE		0x01
#define U8_VAL_POSN			0x02
#define U8_VAL_VEL			0x04
#define U8_VAL_HEADING		0x08
#define U8_VAL_ATTI			0x10
#define U8_VAL_OMEGA			0x20
#define U8_VAL_HEAVE			0x40
#define U8_VAL_LEVEL			0x80

/* 故障码 */
#define U16_ERR_MAT_INV			0x0001 // 故障码: 矩阵求逆奇异
#define U16_ERR_ROT_SPEED		0x0002 // 故障码: 转速过高
#define U16_ERR_TEMP_HOT			0x0004 // 故障码: 温度过高
#define U16_ERR_PULSE_R0			0x0010 // 故障码: 从惯组R0接收400Hz角增量和速度增丢数
#define U16_ERR_PULSE_R1			0x0020 // 故障码: 从惯组R1接收400Hz角增量和速度增量丢数
#define U16_ERR_ROT_R				0x0040 // 故障码: 从惯组接收400Hz旋转信息丢数


/* flag com rev */
#define U32_RECV_POSN_BIND		0x00000001  // 装订位置
#define U32_RECV_POSN_GPS		0x00000002  // GPS位置
#define U32_RECV_VEL_BIND		0x00000004  // 装订速度
#define U32_RECV_VEL_GPS			0x00000008 // GPS速度
#define U32_RECV_VEL_LOG			0x00000010 // 计程仪速度
#define U32_RECV_VEL_DVL			0x00000020 // 多普勒速度
#define U32_RECV_TIME_STAMP		0x00000040 // 收到时戳
#define U32_RECV_GYRO_PARA		0x00000080 // 陀螺标定参数
#define U32_RECV_ACC_PARA		0x00000100 // 加速度计标定参数
#define U32_RECV_GBIAS_COEF		0x00000200 // 陀螺零偏温补系数
#define U32_RECV_AZERO_COEF		0x00000400 // 加速度计零位温补系数
#define U32_RECV_HEADING_BIND	0x00000800 // 加速度计标度温补系数
#define U32_RECV_BIAS_EQU		0x00001000 // 等效零偏
#define U32_RECV_BIAS_INC			0x00002000 // 补偿零偏
#define U32_RECV_SWITCH_CMD		0x00008000 // 切换命令
#define U32_RECV_SYS_ATTI_ZERO	0x00010000 // 系统姿态零位
#define U32_RECV_EX_ATTI_ZERO	0x00020000 // 惯组姿态零位与输出姿态零位
#define U32_RECV_LEVER_ARM		0x00040000 // 杆臂
#define U32_RECV_CDU_MSG			0x00080000 // CDU参考信息
#define U32_RECV_EXT_PPS			0x00100000 // 收到GPS的pps TMin0
#define U32_RECV_SYN_SIG			0x00200000 // 收到同步信号 TMin1
#define U32_RECV_MAIN_INS		0x00400000 // 收到主惯导
#define U32_RECV_SYS_NO			0x00800000 // 收到系统编号
#define U32_RECV_SYS_ADDR		0x01000000 // 收到系统舰位
#define U32_RECV_IP_ADDR		0x02000000 // 收到IP地址
#define U32_RECV_FLASH_BURN		0x80000000 // 收到烧写命令

/* flag com valid */
#define U32_VAL_BIND_POSN		0x00000001 // 装订位置有效
#define U32_VAL_GPS_POSN		0x00000002 // GPS位置有效
#define U32_VAL_ZERO_VEL		0x00000004 // 零速有效
#define U32_VAL_GPS_VEL			0x00000008 // GPS速度有效
#define U32_VAL_LOG_VEL			0x00000010  // 电磁速度有效 
#define U32_VAL_DVL_VW			0x00000020 // 多普勒对水速度有效
#define U32_VAL_DVL_VE			0x00000040 // 多普勒对底速度有效
#define U32_VAL_TIME_STAMP		0x00000080 // 时码有效
#define U32_SYN_SND				0x00000100 // 同步发送
#define U32_ADJ_BGX				0x00000200 // 补Gx
#define U32_ADJ_BGY				0x00000400 // 补Gy 
#define U32_ADJ_BGZ				0x00000800 // 补Gz
#define U32_ADJ_BAX				0x00001000 // 补Ax
#define U32_ADJ_BAY				0x00002000 // 补Ay
#define U32_ADJ_BAZ				0x00004000 // 补Az
#define U32_ADJ_RAP				0x00008000 // 补水平姿态
#define U32_ADJ_HEADING			0x00010000 // 补航向
#define U32_ADJ_VEL				0x00020000 // 补速度
#define U32_ADJ_POSN				0x00040000 // 补位置
#define U32_DP_READY			0x00080000 // 点校状态
#define U32_HEAD_LINE			0x00100000 // 直航
#define U32_STATIC_STA		0x00200000 // 静态
#define U32_SHAKE_STA		0x00400000 // 振动
#define U32_CRASH_STA		0x00800000 // 冲击
#define U32_PPS_SND			0x01000000 // 时码发送 // 传递对准
#define U32_DRAG_SND			0x02000000 // 牵引发送 // 从惯导
#define U32_VAL_DRAG_POSN	0x10000000 // 牵引位置
#define U32_VAL_DRAG_VEL		0x20000000 // 牵引速度
#define U32_VAL_DRAG_ATTI	0x40000000 // 牵引姿态
#define U32_VAL_DRAG_OMEGA	0x80000000 // 牵引角速度


/* 阻尼速度使用优先级: 从低到高 */
#define U8_PRIOR_LOG_VEL		0x05 // 电磁计程仪
#define U8_PRIOR_DVL_VW		0x04 // 多普勒对水
#define U8_PRIOR_DVL_VE		0x03 // 多普勒对底
#define U8_PRIOR_GPS_VEL		0x02 // GPS速度
#define U8_PRIOR_ZERO_VEL	0x01 // 零速度
#define U8_PRIOR_VOID			0x00 // 没有外速度,进入无阻尼

/* update int */
#define UPDATE_FREQ		400u 	// 解算频率
#define UPDATE_PERIOD		0.0025 	// 解算周期

/* 升沉与垂向速度 */
#define TP_HEIGHT 15.0 // 截止频率
#define EXP_TP_HEIGHT (exp(-1.0/TP_HEIGHT/UPDATE_FREQ)-1.0)

#define LEN_SMOOTH_VM	10 // 电磁速度平滑滤波区间
#define LEN_SMOOTH_VW	10 // 多普勒对水速度平滑滤波区间
#define LEN_SMOOTH_VE	10 // 多普勒对底速度平滑滤波区间

#define MAX_MAT_DIM		30  // 矩阵维数阈值
#define MAX_RATE_WIDTH	60  // 角速率平滑区间
#define MAX_RATE_WIDTH2	40  // 
#define MAX_PULSE_WIDTH	4    // 脉冲数区间
#define LEN_SMOOTH_WIDTH	28 // 计程仪平滑滤波区间
#define LEN_GPS_POSN	10

#define LEN_CHECK_MOV	60 // 1min
#define LEN_CHECK_HEADLINE	11 // 11s

#define LEN_GYRO_SERIES	5 // 陀螺个数
//#define zero(x) fabs(x)<1e-21 // 判是否为零的数值阈值
#define zero2(x) (fabs(x)<0.0000000000000000000001) // 判是否为零的数值阈值

/* 运动状态 */
#define FORCE_CHANGE_STATIC		0.002*GLV_G
//#define FORCE_CHANGE_MOOR		0.01*GLV_G	// 0.01g / 120s
#define FORCE_CHANGE_MOOR		0.08*GLV_G	// 0.01g / 120s

#define ROLL_CHANGE_STATIC		0.3*DEG2RAD // 
#define ROLL_CHANGE_MOOR		1.5*DEG2RAD

#define PITCH_CHANGE_STATIC		0.1*DEG2RAD
#define PITCH_CHANGE_MOOR		0.5*DEG2RAD

#define HDG_CHANGE_STATIC		1.0*DEG2RAD	// 1deg	  / 120s
#define HDG_CHANGE_MOOR			10.0*DEG2RAD // 10deg / 120s

#define VEL_CHANGE_STATIC		0.2			// 0.2m/s / 120s
#define VEL_CHANGE_MOOR			1.0			// 1.0m/s / 120s

#define HDG_CHANG_HEADLINE		2.0*DEG2RAD

#define FRAME_XYZ		0x10
#define FRAME_ZXY		0x53
#define FRAME_YZX		0x31
#define FRAME_YXFZ		0x23
#define FRAME_XZFY		0x30


/* 定义结构体 */

typedef struct tag_HOST_MSG
{
	// 需要存储的变量
	INT32_T iPosnInit[2]; // 8bytes 初始位置
	UINT32_T uiHeadingInit; // 4bytes 初始航向
	UINT32_T uiVelInit; // 4bytes 初始速度
	UINT32_T uiSysNo; // 4bytes 系统编号
	UINT32_T uiSysAddr; // 4bytes 系统舰位
	UINT32_T uiIPAddr; // 4bytes IP地址
	INT32_T iKgBind[3]; // 12bytes 陀螺标度
	INT32_T iBgBind[3]; // 12bytes 陀螺漂移
	INT32_T iUgBind[6]; // 24bytes 陀螺安装误差
	INT32_T iKaBind[3]; // 12bytes 加速度计标度
	INT32_T iBaBind[3]; // 12bytes 加速度计零偏
	INT32_T iUaBind[6]; // 24bytes 加速度计安装误差
	INT32_T iBgEqu[3]; // 12bytes 等效陀螺漂移
	INT32_T iSysAttiZero[3]; // 12bytes 系统姿态零位
	INT32_T iImuAttiZero[3]; // 12bytes 惯组姿态零位
	INT32_T iOutAttiZero[3]; // 12bytes 输出姿态零位
	INT16_T nLeverArm[6][3]; // 36bytes 杆臂
	FLOAT32_T fBetaGyroBias[4][3]; // 48bytes 陀螺零偏温补系数
	FLOAT32_T fBetaAccZero[4][3]; // 48bytes 加速度计零位温补系数
	
	UINT32_T uiRecvSta;
	INT32_T iBgInc[3]; // 陀螺增量漂移

	UCHAR_T ucSwitchSelect; // 切换类型选择
	UCHAR_T ucSwitchCmd; // 切换命令
}HOST_MSG_T;

// bind parameter
typedef struct tag_BIND_PARA
{
	FLOAT64_T dKgBind[3]; // 陀螺标度
	FLOAT64_T dKaBind[3]; // 加速度计标度
	FLOAT64_T dCgpBind[9]; // 陀螺安装误差
	FLOAT64_T dCapBind[9]; // 加速度计安装误差
	FLOAT64_T dBgBind[3]; // 陀螺零偏
	FLOAT64_T dBaBind[3]; // 加速度计零偏
	FLOAT64_T dBgEqu[3]; // 等效陀螺漂移
	FLOAT32_T fBetaGyroBias[4][3]; // 陀螺零偏温补系数
	FLOAT32_T fBetaAccZero[4][3]; // 加速度计零偏温补系数
	FLOAT64_T dSysAttiZero[3]; // 姿态零位
	FLOAT64_T dImuAttiZero[3]; // 惯组零位
	FLOAT64_T dOutAttiZero[3]; // 输出零位
	FLOAT64_T dLeverArm[6][3]; // 杆臂
}BIND_PARA_T;


// inert data
typedef struct tag_INERT_DATA
{
	FLOAT64_T dAngIncP[3]; // 台体系下角增量
	FLOAT64_T dVelIncP[3]; // 台体系下速度增量
	FLOAT64_T dAngIncG[3]; // 陀螺系下角增量
	FLOAT64_T dVelIncA[3]; // 加速度计系下速度增量
	FLOAT64_T dPulseGyro[3]; // 陀螺脉冲数
	FLOAT64_T dPulseAcc[3]; // 加速度脉冲数
	FLOAT32_T fTempGyro[6]; // 陀螺温度
	FLOAT32_T fTempAcc[3];  // 加速度计温度
	FLOAT32_T fDeltaTempGyro[3]; // 温变速率
	FLOAT32_T fTempGyro20s[3]; // 20s温度平均值
	FLOAT32_T fTempGyroSeries[3][MAX_RATE_WIDTH];
	FLOAT64_T dAngIncMS[5][3];
	FLOAT64_T dVelIncMS[6][3];
	FLOAT64_T dOmega[3];
	FLOAT64_T dForce[3];

	FLOAT64_T dAngIncB[3]; // 台体系下角增量 rad/2.5ms
	FLOAT64_T dVelIncB[3]; // 台体系下速度增量 m/s/2.5ms
	FLOAT64_T dAngIncBSum[3]; // 台体系下角增量1和 rad/s
	FLOAT64_T dVelIncBSum[3]; // 台体系下速度增量1s和 m/s/s
	FLOAT64_T dAngIncGSum[3]; // rad/s
	FLOAT64_T dVelIncASum[3]; // m/s/s
	FLOAT64_T dOmegaIBB[3]; // 角速度 rad/s
	FLOAT64_T dDeltaOmegaIBB[3]; //  角速度的微分rad/s^2
	FLOAT64_T dOmegaIBBSeries[MAX_RATE_WIDTH2][3];
}INERT_DATA_T;

// nav calc data
typedef struct tag_NAV_DATA
{
	FLOAT64_T dOmegaInN[3];

	FLOAT64_T dAttiCh1[3]; // 通道一姿态
	FLOAT64_T dVelCh1[3]; // 通道一速度
	FLOAT64_T dPosnCh1[3]; // 通道一位置
	FLOAT64_T dFnCh1[3]; // 通道一线加速度
	FLOAT64_T dCnpCh1[9]; // 通道一n到p系方向余弦阵
	FLOAT64_T dCpnCh1[9]; // 通道一p到n系方向余弦阵
	FLOAT64_T dQnpCh1[4]; // 通道一n到p系姿态四元数
	FLOAT64_T dAttiOut[3]; // 滤波/水平阻尼输出姿态
	FLOAT64_T dVelOut[3];  // 滤波/水平阻尼输出速度
	FLOAT64_T dPosnOut[3]; // 滤波/水平阻尼输出位置
	FLOAT64_T dAttiImu1[3]; // 通道一惯组姿态
	FLOAT64_T dQnpOut[4]; // 滤波/水平阻尼输出n到p系姿态四元数
	FLOAT64_T dCnpOut[9]; // 滤波/水平阻尼输出n到p系方向余弦阵
	
	FLOAT64_T dCpb[9]; // p系到b系(旋转变换阵)
	FLOAT64_T dCb1b[9]; // 系统姿态零位阵(用于系统的)
	FLOAT64_T dCb1b2[9]; // 输出姿态零位阵(用于分解合速度的)
	FLOAT64_T dCb3b1[9]; // 惯组姿态零位阵(用于惯组坐标变换的)

	FLOAT64_T dCnb2[9]; // 
	FLOAT64_T dCb2n[9]; // 用于速度分解的姿态阵 Cnb2 = Cb1b2*CnpCh1
	
	FLOAT64_T dCnbEx[9]; // 外姿态n到b系方向余弦阵
	FLOAT64_T dCbp[9]; // b系到p系(旋转变换阵)
	
	// 计算角速度
	FLOAT64_T dAttiSeries[MAX_RATE_WIDTH][3]; // 暂存姿态
	FLOAT64_T dOmegaAtti[3]; // 姿态角速度
	// 瞬时线运动
	FLOAT64_T dFd[3];
	FLOAT64_T dVDOut[3]; // 瞬时速度:横向、纵向、垂向m/s
	FLOAT64_T dSDOut[3]; // 瞬时位移:横向、纵向、垂向m
	FLOAT64_T dVDy[3][4]; // 输出
	FLOAT64_T dVDx[3][4]; // 输入
	// 阻尼
	FLOAT64_T dKx[5]; // 东向通道阻尼系数
	FLOAT64_T dKy[5]; // 北向通道阻尼系数
	FLOAT64_T dDK[2]; // 东向通道阻尼中间参数
	FLOAT64_T dTK[2]; // 北向通道阻尼中间参数
	FLOAT64_T dVelMid[2]; // 阻尼中间速度
	FLOAT64_T dVelInt[2]; // 用于位置积分的速度
	// 惯性系粗对准
	FLOAT64_T dCip0pCalc[9]; // 从坐标系ip0到坐标系p的方向余弦阵
	FLOAT64_T dQip0pCalc[4]; // 从坐标系ip0到坐标系p的姿态四元数
	FLOAT64_T dCinCalc[9]; // 从i坐标系转换到n坐标系的方向余弦阵
	FLOAT64_T dVip0Calc[3]; // 惯性坐标系ip0的速度向量
	FLOAT64_T dUi0Calc[3]; // 惯性坐标系i0的速度向量
	FLOAT64_T dVip0T1[3]; // t1时刻，惯性坐标系ip0的速度向量
	FLOAT64_T dVip0T2[3]; // t2时刻，惯性坐标系ip0的速度向量
	FLOAT64_T dUi0T1[3]; // t1时刻，惯性坐标系i0的速度向量
	FLOAT64_T dUi0T2[3]; // t2时刻，惯性坐标系i0的速度向量
	FLOAT64_T dVip0Tk[3]; // tk时刻，惯性坐标系ip0的速度向量
	FLOAT64_T dUi0Tk[3]; // tk时刻，惯性坐标系i0的速度向量
	FLOAT64_T dQipCalc[4]; // 从坐标系i到坐标系p的姿态四元数
	FLOAT64_T dCipCalc[9]; // 从坐标系i到坐标系p的方向余弦阵

	FLOAT64_T dVelDampZero[2]; // 从无阻尼转水平阻尼时刻点的速度为阻尼速度零位
	FLOAT64_T dVelDampRef[2]; // 用于阻尼解算外参考速度 = RefMsg.dVelDampRef-NavData.dVelDampZero

	// time
	UINT32_T uiKSta;  // 子状态时间: 准备/对准(粗对准)/对准(精对准)/自主导航(无阻尼)/自主导航(水平阻尼)/综合校准/组合导航
	UINT32_T uiKPart;  // 状态时间: 准备/对准/自主导航/综合校准/组合导航
	UINT32_T uiKMax; // 最大状态运行时间
	
	UINT32_T uiKDamp; // 自主导航水平阻尼状态时间
	UINT32_T uiKNoDamp; // 自主导航无阻尼状态时间
	UINT32_T uiKKeepNoDamp; // 水平阻尼与无阻尼切换时无阻尼保持时间
	
	UINT32_T uiKCoarseAlign; // 粗对准时间
	UINT32_T uiKFineAlign; // 精对准时间
	UINT32_T uiKAlignReset1; // 精对准滤波重置点1
	UINT32_T uiKAlignReset2; // 精对准滤波重置点2
	UINT32_T uiKAlignReset3; // 精对准滤波重置点3
	UINT32_T uiKAlignReset4; // 精对准滤波重置点4
	UINT32_T uiKAlignReset5; // 精对准滤波重置点5
	UINT32_T uiKAlignReset6; // 精对准滤波重置点6
	UINT32_T uiKAlignReset7; // 精对准滤波重置点7
	UINT32_T uiKAlignReset8; // 精对准滤波重置点8

	// state
	UCHAR_T ucWorkMode; // 工作状态: 准备/对准(粗对准)/对准(精对准)/自主导航(无阻尼)/自主导航(水平阻尼)/综合校准/组合导航
	UCHAR_T ucOperMode; // 操作模式: 自动/手动
	UCHAR_T ucDampLevel; // 阻尼条件: 紧/松
	UCHAR_T ucSailPlace;  // 工作地点: 码头/海上
	UCHAR_T ucAlignTime;  // 对准时间: 30min/6h/12h
	UCHAR_T ucMeasMode; // 量测模式: 速度/位置/速度+位置
	UINT16_T ucRefValid; // 参考信息有效标示
	UINT32_T uiComRecv; // 接口信息收到标示
	UINT32_T uiComValid; // 接口信息有效标示
	UINT32_T uiErrCode; // 故障码
	UCHAR_T ucPriorVelCur; // 当前阻尼速度优先级
	UCHAR_T ucPriorVelPre; // 上周期阻尼速度优先级
	UCHAR_T ucFrameType; // 坐标系类型
	
	UCHAR_T ucForceChange; // =0-静态, b0=1-锚泊, b1=1-动态
	UCHAR_T ucVelChange[2]; // =0-静态, b0=1-锚泊, b1=1-动态
	UCHAR_T ucAttiChange[3]; // =0-静态, b0=1-锚泊, b1=1-动态
	UCHAR_T ucMoveChange; // 运动变化标识: 0-静态, 1-锚泊, 3-动态
	UCHAR_T ucHeadChange; // 航向变化: 0-直航, 1-转向
	INT32_T uiKMoor; // 锚泊时间
	INT32_T uiKStatic; // 静态时间
	INT32_T uiKHeadLine; // 直航时间
	FLOAT64_T dForce1s[3];
	FLOAT64_T dForceSeries1s[LEN_CHECK_MOV];
	FLOAT64_T dVelSeries1s[LEN_CHECK_MOV][2];
	FLOAT64_T dAttiSeries1s[LEN_CHECK_MOV][3];
	
}NAV_DATA_T;

// 系统级标定参数
typedef struct tag_BD_PARA
{
	FLOAT64_T dXCalc[27]; // 状态估计: 3个姿态误差/3个速度误差/3个陀螺漂移/3个加速度计零偏/3个陀螺安装误差/6个加速度计安装误差
	FLOAT64_T dP0Diag[27]; // 误差方差初始值
	FLOAT64_T dPCalc[27*27]; // 误差方差
	FLOAT64_T dQ[21*21]; //  过程噪声方差: 3个陀螺漂移/3个加速度计零偏/3个陀螺标度/3个陀螺安装误差/3个加速度计标度/6个加速度计安装误差
	FLOAT64_T dH[3*27]; // 量测阵3vel
	FLOAT64_T dR[3*3]; // 量测噪声阵3vel
	FLOAT64_T dXkk1[27]; // 状态一步预测
	FLOAT64_T dPkk1[27*27]; // 方差一步预测
	FLOAT64_T dDeltaVel[3];
	FLOAT64_T dPHI[27*27]; // 状态转移阵
	FLOAT64_T dQd[27*27]; // 过程噪声方差离散化
	FLOAT64_T dCbnBD[9]; // KF预测需要的b到n的方向余弦阵
	UCHAR_T ucFlagSingular; // 是否矩阵求逆奇异

}BD_PARA_T;

// kalman para
typedef struct tag_KF_PARA
{
	FLOAT64_T dXCalc[12]; // 状态估计: 3个姿态误差/2个速度误差/2个位置误差/3个陀螺漂移/2个加速度计零偏
	FLOAT64_T dP0Diag[12]; // 误差方差初始值
	FLOAT64_T dPCalc[12*12]; // 误差方差
	FLOAT64_T dQ0[5*5]; // 过程噪声方差: 3个陀螺漂移/2个加速度计零偏
	FLOAT64_T dH0[4*12]; // 量测阵: 2vel+2posn
	FLOAT64_T dR0[4*4]; // 量测噪声方差: 2vel+2posn
	FLOAT64_T dH1[2*12]; // 量测阵: 2vel
	FLOAT64_T dR1[2*2]; // 量测噪声方差: 2vel
	FLOAT64_T dH2[2*12]; // 量测阵: 2posn
	FLOAT64_T dR2[2*2]; // 量测噪声方差: 2posn
	FLOAT64_T dXkk1[12]; // 状态一步预测
	FLOAT64_T dPkk1[12*12]; // 方差一步预测

	FLOAT64_T dPhiEst[3]; // 姿态误差估计
	FLOAT64_T dGyroBiasEst[3]; // 陀螺漂移估计
	FLOAT64_T dAccBiasEst[3]; // 加速度计零偏估计
	FLOAT64_T dDeltaPosn[3]; // 量测位置
	FLOAT64_T dDeltaVel[3]; // 量测速度
	FLOAT64_T dPHI[12*12]; // 状态转移阵
	FLOAT64_T dQd[12*12]; // 过程噪声方差离散化
	FLOAT64_T dCpnKF[9]; // KF预测需要的p到n的方向余弦阵
	UCHAR_T ucFlagSingular; // 是否矩阵求逆奇异
	UCHAR_T ucFlagFilter; // 是否有量测量给滤波
}KF_PARA_T;

typedef struct tag_TIME_STAMP
{
	UINT32_T uiPPSUtcMS; // utc时间 天秒 unit:0.1ms
	UINT32_T uiPPSUtcS; // utc秒
	UINT16_T unDetaT; // 与前一个上升沿时间差
	UINT32_T uiPPSCnt; // 距前一个PPS 计数 unit: 2.5ms
	UINT32_T uiCurUtcMS; // 当前天秒 = dwPPSUtcMS+wPPSCnt*25 unit:0.1ms
}TIME_STAMP_T;

typedef struct tag_IMU_INFO
{
	FLOAT32_T fAngIncP[3]; // 1s台体系角增量和(已温补)
	FLOAT32_T fVelIncP[3]; // 1s台体系速度增量和(已温补)
	FLOAT32_T fAngIncR[3]; // 1s台体系角增量和(未温补)
	FLOAT32_T fVelIncR[3];  // 1s台体系速度增量和(未温补)
}IMU_INFO_T;

typedef struct tag_REF_MSG
{
	UCHAR_T ucComRecv; // 信息收到标识(导航通信板传)
	UCHAR_T ucComValid; // 信息有效标识(导航通信板传)
	FLOAT64_T dPosnRef[3]; // 参考位置: 纬度/经度/高度
	FLOAT64_T dVelRef[3]; //  参考速度: 东向/北向/垂向
	FLOAT64_T dVelDampRef[3]; // 参考阻尼速度: 东向/北向/对地合速度
	FLOAT32_T fGpsPosn[3]; // GPS位置: 纬度/经度/高度
	FLOAT32_T fGpsVel[3]; // GPS速度: 东向/北向/对地合速度
	FLOAT32_T fTraAng; // 航迹向
	UCHAR_T ucSVNo; // 卫星数
	FLOAT32_T fHDOP; // 水平精度因子
	FLOAT32_T fLogVel[3]; // 电磁计程仪速度: 分解东向/分解北向/电磁对水速度
	FLOAT32_T fLogVelSeries[LEN_SMOOTH_VM];
	FLOAT32_T fLogVelSm;
	FLOAT32_T fDopVw[2]; // 多普勒对水: 横向/纵向
	FLOAT32_T fDopVe[2]; // 多普勒对底: 横向/纵向
	FLOAT32_T fDopVwSeries[2][LEN_SMOOTH_VW]; // 多普勒对水暂存
	FLOAT32_T fDopVeSeries[2][LEN_SMOOTH_VE]; // 多普勒对底暂存
	FLOAT32_T fDopVwSm[2]; // 多普勒对水平滑
	FLOAT32_T fDopVeSm[2]; // 多普勒对底平滑
	UCHAR_T ucUtcHours; // UTC时间: 时
	UCHAR_T ucUtcMinutes; // UTC时间: 分
	UCHAR_T ucUtcSeconds; // UTC时间: 秒
	UCHAR_T ucUtcDay; // UTC时间: 天
	UCHAR_T ucUtcMonth; // UTC时间: 月
	UINT16_T unUtcYear; // UTC时间: 年
	UINT32_T uiUtcDataSec; // 天秒
	UINT16_T unSVNo; // 卫星数
	UINT16_T unHDOP; // 精度因子
	
	UINT16_T unStaMain;
	FLOAT64_T dPosnMain[2];
	FLOAT64_T dVelMain[2];
	FLOAT64_T dAttiMain[3];
	FLOAT64_T dOmegaMain[3];
	UINT32_T uiTimeMain;	
	
	INT32_T iCntGpsPosn; // 收到GPS位置时间
	INT32_T iCntNoGpsPosn; // 无GPS位置时间
	INT32_T iCntGpsVel; // 收到GPS速度时间
	INT32_T iCntNoGpsVel; // 无GPS速度时间
	INT32_T iCntTimeStamp; // 收到时戳时间
	INT32_T iCntNoTimeStamp; // 无时戳时间
	INT32_T iCntLogVel; // 收到电磁计程仪时间
	INT32_T iCntNoLogVel; // 无电磁计程仪时间
	INT32_T iCntDopVw; // 收到多普勒对水时间
	INT32_T iCntNoDopVw; // 无多普勒对水时间
	INT32_T iCntDopVe; // 收到多普勒对底时间
	INT32_T iCntNoDopVe; // 无多普勒对底时间
}REF_MSG_T;

typedef struct tag_PULSE_MSG
{
	unsigned int nSysTime;			// 系统时间
	unsigned int nTimeStamp;		// 时戳
	// 采样原始数据
	int nPulseGyro[3][5];			// 陀螺1ms脉冲数
	short nPulseAcc[3][5];			// 加表1ms脉冲数
	short nTGin[3],nTAin[3];		// 陀螺温度，加表温度
	short nTBrd[2];				// 采集板和加表板温度
	
	// 转换数据
	double dPulseGyro[3];			// 陀螺5ms脉冲数 为1ms脉冲数之平均
	double dPulseAcc[3];			// 加表5ms脉冲数 为1ms脉冲数之和
	// 1s数据
	double dPulseGyro1s[3];		// 1s陀螺脉冲平均
	double dPulseAcc1s[3];			// 1s加速度计脉冲和
	int iPulseAcc1s[3];				// 1s加速度计脉冲和
	unsigned short nGyroEffect[3];	// 1s内收到陀螺有效个数

	unsigned char ucGyroDataCheck[3][5];		// 标识位  1为校验正确 0为错误
	unsigned char ucGyroTempCheck[3][5];	// 标识位  1为校验正确 0为错误
	// 
	unsigned short unGyroCheck; // 标识位 bit0-bit4:GXD0-GXD4  bit5-bit9:GYD0-GYD4 bit10-bit14:GZD0-GZD4 1为校验正确 0为错误
	// unGyroCheck的最高位bit15=1表示收到1PPS信号
	unsigned char ucImuCode[1000];		
	// 200Hz信息码 
	// 0-1 TGX
	// 2-3 TGY
	// 4-5 TGZ
	// 6-7 TAX
	// 8-9 TAY
	// 10-11 TAZ
	int n_gyro_temp[6];
	int n_accl_temp[3];
}PULSE_MSG_T;

typedef struct tag_NAV_RET
{
	FLOAT64_T dPosn[2]; // 纬度 经度
	FLOAT64_T dVel[2]; // 东速 北速
	FLOAT64_T dAtti[3]; // 横摇 纵摇 航向角
	FLOAT64_T dOmega[3]; // 横摇 纵摇 航向角速率
	UINT16_T unFlagTMin; // 标识位
	UINT16_T unDetaT1; 
	UINT16_T unDetaT; // 插值时间
}NAV_RET_T;

#endif

