#include "matoper.h"
#include "mcalc.h"
#include "common.h"
#include "math.h"
unsigned int g_uiCntClock;
unsigned char g_ucFlagClock;


BIND_PARA_T g_stBindPara;
INERT_DATA_T g_stInertData;
NAV_DATA_T g_stNavData;
KF_PARA_T g_stKfPara;
BD_PARA_T g_stBDPara;
IMU_INFO_T g_stImuInfo;
REF_MSG_T g_stRefMsg;
HOST_MSG_T g_stHostMsg;
PULSE_MSG_T g_stPulseMsg;
/************************************************************************\
功能: 矩阵求逆(高斯消去法)
输入:
 * a - 矩阵，链式结构
 * dim - 矩阵维数
输出:
 * b - a的逆矩阵，链式结构
 * 返回值 - 非奇异返回 1 
            奇异返回   0
日期: 2009.7.6 
作者: FairyLan
\************************************************************************/
INT32_T matinv(FLOAT64_T b[],	const FLOAT64_T a[], const INT32_T dim) 
{
	INT32_T i,j,k,is[MAX_MAT_DIM], js[MAX_MAT_DIM], len=dim*dim;
	FLOAT64_T t;
	
	memcpy(b, a, len*sizeof(a[0]));
	for (k=0; k<dim; k++)
	{
		t = 0.0;
		for (i=k; i<dim; i++)
			for (j=k; j<dim; j++)
				if (fabs(b[i*dim+j]) > t)
					t = fabs(b[ (is[k]=i) * dim + (js[k]=j) ]);
        if (zero2(t))
			return 0;
		if (is[k] != k)
			for (j=0; j<dim; j++)
			{
				t = b[ k * dim + j ]; 
				b[ k * dim + j ] = b[ is[k] * dim + j ];
				b[ is[k] * dim + j ] = t;
			}
		if (js[k] != k)
			for (i=0; i<dim; i++)
			{
				t = b[ i * dim + k ];
				b[ i*dim + k ] = b[ i*dim + js[k] ];
				b[ i*dim + js[k] ] = t;
			}
		b[k*dim+k] = 1.0/b[k*dim+k];
		for (j=0; j<dim; j++)
			if (j != k)
				b[k*dim + j] *= b[k*dim + k];
		for (i=0; i<dim; i++)
			if (i != k)
				for (j=0; j<dim; j++)
					if (j != k)
						b[i*dim+j] -= b[i*dim+k]*b[k*dim+j];
		for (i=0; i<dim; i++)
			if (i != k)
				b[i*dim+k] *= -b[k*dim+k];
	}
	for (k=dim-1; k>=0; k--)
	{
		for (j=0; j<dim; j++)
			if (js[k] != k)
			{
				t = b[ k * dim + j ];
				b[ k * dim + j ] = b[ js[k] * dim + j ];
				b[ js[k] * dim + j ] = t;
			}
		for (i=0; i<dim; i++)
			if (is[k] != k)
			{
				t = b[ i * dim + k ];
				b[ i * dim + k ] = b[ i * dim + is[k] ];
				b[ i * dim + is[k] ] = t;
			}
	}
	return 1;
}

/************************************************************************\
功能: 求向量叉积
输入:
 * a - 左向量，链式结构，3维
 * b - 右向量，链式结构，3维
输出:
 * c - a与b的叉积，链式结构，3维
日期: 2009.7.6 
作者: FairyLan
\************************************************************************/
void xmult(FLOAT64_T c[], const FLOAT64_T a[], const FLOAT64_T b[])
{
	FLOAT64_T x[3],y[3];
	memcpy(x,a,3*sizeof(a[0]));
	memcpy(y,b,3*sizeof(b[0]));

	c[0] = x[1]*y[2]-y[1]*x[2]; // c=(x)X(y)
	c[1] = x[2]*y[0]-y[2]*x[0];
	c[2] = x[0]*y[1]-y[0]*x[1];
}

/************************************************************************\
功能: 求向量的模平方
输入:
 * x - 向量，链式结构
 * dim - 向量维数
输出:
 * 返回值 - 向量x的模平方 ret = x[1]^2+x[2]^2+...+x[dim]^2
日期: 2009.7.6 
作者: FairyLan
\************************************************************************/
FLOAT64_T normpower(const FLOAT64_T x[], INT32_T dim)
{
	FLOAT64_T ret = 0.0;
	INT32_T i;
	for (i=0; i<dim; i++)
		ret += x[i]*x[i]; // ret = x[1]^2+x[2]^2+...+x[dim]^2
	return ret;
}

/************************************************************************\
功能: 求矢量的模
输入:
 * x - 矢量，链式结构
 * dim - 矢量维数
输出:
 * 返回值 - 矢量x的模 ret = (x[1]^2+x[2]^2+...+x[dim]^2)^(1/2)
日期: 2009.7.6 
作者: FairyLan
\************************************************************************/
FLOAT64_T norm(const FLOAT64_T x[], INT32_T dim)
{
	FLOAT64_T ret = 0.0;
	INT32_T i;
	for (i=0; i<dim; i++)
		ret += x[i]*x[i]; // ret = x[1]^2+x[2]^2+...+x[dim]^2
	return sqrt(ret); // ret = (x[1]^2+x[2]^2+...+x[dim]^2)^(1/2)
}

/************************************************************************\
功能: 姿态转换成姿态四元数
输入:
 * atti - 三维向量姿态，横摇、纵摇、航向
输出:
 * Qnb - 姿态四元数，4维向量
日期: 2009.7.6 
作者: FairyLan
\************************************************************************/
void atti2quat(FLOAT64_T Qnb[], const FLOAT64_T atti[])
{
	FLOAT64_T cosR, cosP, cosH, sinR, sinP, sinH;
	
	cosR = cos(atti[0]/2.0); 
	sinR = sin(atti[0]/2.0);
	cosP = cos(atti[1]/2.0); 
	sinP = sin(atti[1]/2.0);
	cosH = cos(atti[2]/2.0); 
	sinH = sin(atti[2]/2.0);
	// Qnb = [cos(H/2)*cos(P/2)*cos(R/2)+sin(H/2)*sin(P/2)*sin(R/2);
	//			cos(H/2)*sin(P/2)*cos(R/2)+sin(H/2)*cos(P/2)*sin(R/2);
	//			cos(H/2)*cos(P/2)*sin(R/2)-sin(H/2)*sin(P/2)*cos(R/2);
	//			cos(H/2)*sin(P/2)*sin(R/2)-sin(H/2)*cos(P/2)*cos(R/2)];
	Qnb[0] = cosH*cosP*cosR+sinH*sinP*sinR;
	Qnb[1] = cosH*sinP*cosR+sinH*cosP*sinR;
	Qnb[2] = cosH*cosP*sinR-sinH*sinP*cosR;
	Qnb[3] = cosH*sinP*sinR-sinH*cosP*cosR;
}

/************************************************************************\
功能: 姿态转换成方向余弦阵
输入:
 * atti - 三维向量姿态，横摇、纵摇、航向
输出:
 * Cnb - 从n坐标系转换到b坐标系的方向余弦阵，3*3维矩阵
日期: 2009.7.5 
作者: FairyLan
\************************************************************************/
void atti2dmat(FLOAT64_T Cnb[], const FLOAT64_T atti[])
{
	FLOAT64_T cosR, cosP, cosH, sinR, sinP, sinH;
	
	cosR = cos(atti[0]); sinR = sin(atti[0]);
	cosP = cos(atti[1]); sinP = sin(atti[1]);
	cosH = cos(atti[2]); sinH = sin(atti[2]);

	Cnb[0] = cosR*cosH+sinR*sinP*sinH;
	Cnb[1] = -cosR*sinH+sinR*sinP*cosH;
	Cnb[2] = -sinR*cosP;

	Cnb[3] = cosP*sinH;
	Cnb[4] = cosP*cosH;
	Cnb[5] = sinP;

	Cnb[6] = sinR*cosH-cosR*sinP*sinH;
	Cnb[7] = -sinR*sinH-cosR*sinP*cosH;
	Cnb[8] = cosR*cosP;
}

/************************************************************************\
功能: 求四元数的积
输入:
 * q - 左转动四元数，4维向量
 * p - 右转动四元数，4维向量
输出:
 * r - q*p，4维向量
日期: 2009.7.5 
作者: FairyLan
\************************************************************************/
void quatmult(FLOAT64_T r[], const FLOAT64_T q[], const FLOAT64_T p[])
{
	FLOAT64_T x[4],y[4];
	memcpy(x,q,4*sizeof(q[0]));
	memcpy(y,p,4*sizeof(p[0]));
	r[0] = x[0]*y[0]-x[1]*y[1]-x[2]*y[2]-x[3]*y[3];
	r[1] = x[1]*y[0]+x[0]*y[1]-x[3]*y[2]+x[2]*y[3];
	r[2] = x[2]*y[0]+x[3]*y[1]+x[0]*y[2]-x[1]*y[3];
	r[3] = x[3]*y[0]+x[1]*y[2]+x[0]*y[3]-x[2]*y[1];
}

/************************************************************************\
功能: 姿态四元数转换成方向余弦阵
输入:
 * Qnb - 姿态四元数，4维向量
输出:
 * Cnb - 从n坐标系转换到b坐标系的方向余弦阵，3*3维矩阵
日期: 2009.7.5 
作者: FairyLan
\************************************************************************/
void quat2dmat(FLOAT64_T Cnb[], const FLOAT64_T Qnb[])
{
	FLOAT64_T qp0 = Qnb[0]*Qnb[0];
	FLOAT64_T qp1 = Qnb[1]*Qnb[1];
	FLOAT64_T qp2 = Qnb[2]*Qnb[2];
	FLOAT64_T qp3 = Qnb[3]*Qnb[3];

	Cnb[0] = qp0+qp1-qp2-qp3;
	Cnb[1] = 2*(Qnb[1]*Qnb[2]+Qnb[0]*Qnb[3]);
	Cnb[2] = 2*(Qnb[1]*Qnb[3]-Qnb[0]*Qnb[2]);
	
	Cnb[3] = 2*(Qnb[1]*Qnb[2]-Qnb[0]*Qnb[3]);
	Cnb[4] = qp0-qp1+qp2-qp3;
	Cnb[5] = 2*(Qnb[2]*Qnb[3]+Qnb[0]*Qnb[1]);

	Cnb[6] = 2*(Qnb[1]*Qnb[3]+Qnb[0]*Qnb[2]);
	Cnb[7] = 2*(Qnb[2]*Qnb[3]-Qnb[0]*Qnb[1]);
	Cnb[8] = qp0-qp1-qp2+qp3;
}

/************************************************************************\
功能: 方向余弦阵转换成姿态
输入:
 * Cnb - 从n坐标系转换到b坐标系的方向余弦阵，3*3维矩阵
输出:
 * atti - 三维向量姿态，横摇、纵摇、航向
日期: 2009.7.6
作者: FairyLan
\************************************************************************/
void dmat2atti(FLOAT64_T atti[], const FLOAT64_T Cnb[])
{
	FLOAT64_T roll = 0.0, pitch = 0.0, heading = 0.0;

	roll = atan(-Cnb[2]/Cnb[8]); // 横摇角
	
	pitch = atan(Cnb[5]/sqrt(Cnb[3]*Cnb[3]+Cnb[4]*Cnb[4])); // 纵摇角
	
    if (zero2(Cnb[4])) // 航向角
	{
		if (Cnb[3] > 0.0)
			heading = PI/2; // 第一象限和第二象限交界
		else 
			heading = 3*PI/2; // 第三象限和第四象限交界
	}
	else if (Cnb[4] > 0.0)
	{
        if (zero2(Cnb[3])) // 第一象限与第四象限交界
			heading = 0.0;
		else if (Cnb[3] > 0.0) // 第一象限
			heading = atan(Cnb[3]/Cnb[4]);
		else // 第四象限
			heading = 2*PI+atan(Cnb[3]/Cnb[4]);
	}
	else // 第二象限和第三象限
		heading = PI+atan(Cnb[3]/Cnb[4]);

	atti[0] = roll;
	atti[1] = pitch;
	atti[2] = heading;
}

/************************************************************************\
功能: 方向余弦阵转换成姿态四元数
输入:
 * Cnb - 从n坐标系转换到b坐标系的方向余弦阵，3*3维矩阵
输出:
 * Qnb - 姿态四元数，4维向量
日期: 2010.11.25
作者: FairyLan
\************************************************************************/
void dmat2quat(FLOAT64_T Qnb[], const FLOAT64_T Cnb[])
{
	FLOAT64_T q0,q1,q2,q3, tmp;
	q0= 0.5*sqrt(1.0+Cnb[0]+Cnb[4]+Cnb[8]);
    if (zero2(q0)) q0 = 1.0;
	q1 = (Cnb[5]-Cnb[7])/(4.0*q0);
	q2 = (Cnb[6]-Cnb[2])/(4.0*q0);
	q3 = (Cnb[1]-Cnb[3])/(4.0*q0);
	
	tmp = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);
	Qnb[0] = q0/tmp;
	Qnb[1] = q1/tmp;
	Qnb[2] = q2/tmp;
	Qnb[3] = q3/tmp;
}

/************************************************************************\
功能: 旋转矢量转换成转动四元数(泰勒展开)
输入:
 * ang - 旋转矢量，3维
输出:
 * q - 转动四元数，4维向量 q(h)  = cos(|ang|/2)+sin(|ang|/2)*ang/|ang|
日期: 2009.7.5
作者: FairyLan
\************************************************************************/
void dang2dquat(FLOAT64_T q[], const FLOAT64_T ang[])
{
	FLOAT64_T tmp = normpower(ang, 3), x;
	q[0] = 1.0-tmp/8.0+tmp*tmp/384.0-tmp*tmp*tmp/46080.0+tmp*tmp*tmp*tmp/10321920.0; // cos的Talor展开
	x = 0.5-tmp/48.0+tmp*tmp/3840.0-tmp*tmp*tmp/645120.0+tmp*tmp*tmp*tmp/185794560.0; // sin的Talor展开
	q[1] = x*ang[0];
	q[2] = x*ang[1];
	q[3] = x*ang[2];
}


/************************************************************************\
功能: 平台方向余弦阵与旋转变换阵求系统姿态
输入:
 * Cpb - 从p系到b系的旋转变换阵，3*3维矩阵
 * Cnp - 从n坐标系转换到p坐标系的方向余弦阵，3*3维矩阵
输出:
 * atti_sys - 三维向量系统姿态，横摇、纵摇、航向
日期: 2009.7.5
作者: FairyLan
\************************************************************************/
void attiTrans(FLOAT64_T atti_sys[], const FLOAT64_T Cpb[], const FLOAT64_T Cnp[])
{
	FLOAT64_T Cnb_cur[9];
	matmult(Cnb_cur, Cpb, Cnp, 3,3,3);
	dmat2atti(atti_sys, Cnb_cur);
}


/************************************************************************\
功能: 惯性系粗对准，初始对准阶段的粗对准方法
输入:
 * v_ip0_t1 - t1时刻，惯性坐标系ip0的速度向量
 * v_ip0_t2 - t2时刻，惯性坐标系ip0的速度向量
 * u_i0_t1 - t1时刻，惯性坐标系i0的速度向量
 * u_i0_t2 - t2时刻，惯性坐标系i0的速度向量
 * Cip0p - 从ip0坐标系转换到p坐标系的方向余弦阵，3*3维矩阵
 * Ci0n  - 从i0坐标系转换到n坐标系的方向余弦阵，3*3维矩阵
输出:
 * Qnp_cur - 姿态四元数，4维向量
 * Cnp_cur - 从n坐标系转换到p坐标系的方向余弦阵，3*3维矩阵
 * atti_cur - 三维向量系统姿态，横摇、纵摇、航向
日期: 2010.12.1
作者: FairyLan
\************************************************************************/
void coarseAlign(FLOAT64_T Qnp_cur[], FLOAT64_T Cnp_cur[], FLOAT64_T atti_cur[], 
				 const FLOAT64_T v_ip0_t1[], const FLOAT64_T v_ip0_t2[], 
				 const FLOAT64_T u_i0_t1[], const FLOAT64_T u_i0_t2[], 
				 const FLOAT64_T Cip0p[], const FLOAT64_T Ci0n[])
{
	FLOAT64_T tmp[3];
	FLOAT64_T r_i0[3*3], r_ip0[3*3], Cip0i0[3*3], Cip0n[3*3], Cnip0[3*3];
	// 由两个不同时间的速度向量确定矩阵[ui0(t1); ui0(t1) X ui0(t2); ui0(t1) X ui0(t2) X ui0(t1)]
	memcpy(r_i0,u_i0_t1,3*sizeof(FLOAT64_T));
	xmult(tmp,u_i0_t1,u_i0_t2);
	memcpy(&r_i0[3],tmp,3*sizeof(FLOAT64_T));
	xmult(tmp,tmp,u_i0_t1);
	memcpy(&r_i0[6],tmp,3*sizeof(FLOAT64_T));
	// 求三个向量的模
	tmp[0] = norm(r_i0,3);
	tmp[1] = norm(r_i0+3, 3);
	tmp[2] = norm(r_i0+6, 3);
	 // 归一化
	r_i0[0] /= tmp[0];
	r_i0[1] /= tmp[0];
	r_i0[2] /= tmp[0];
	r_i0[3] /= tmp[1];
	r_i0[4] /= tmp[1];
	r_i0[5] /= tmp[1];
	r_i0[6] /= tmp[2];
	r_i0[7] /= tmp[2];
	r_i0[8] /= tmp[2];

	// 由两个不同时间的速度向量确定矩阵[vip0(t1); vip0(t1) X vip0(t2); vip0(t1) X vip0(t2) X vip0(t1)]
	memcpy(r_ip0,v_ip0_t1,3*sizeof(FLOAT64_T));
	xmult(tmp,v_ip0_t1,v_ip0_t2);
	memcpy(r_ip0+3,tmp,3*sizeof(FLOAT64_T));
	xmult(tmp,tmp,v_ip0_t1);
	memcpy(r_ip0+6,tmp,3*sizeof(FLOAT64_T));
	// 求三个向量的模
	tmp[0] = norm(r_ip0,3);
	tmp[1] = norm(r_ip0+3, 3);
	tmp[2] = norm(r_ip0+6, 3);
	 // 归一化
	r_ip0[0] /= tmp[0];
	r_ip0[1] /= tmp[0];
	r_ip0[2] /= tmp[0];
	r_ip0[3] /= tmp[1];
	r_ip0[4] /= tmp[1];
	r_ip0[5] /= tmp[1];
	r_ip0[6] /= tmp[2];
	r_ip0[7] /= tmp[2];
	r_ip0[8] /= tmp[2];
	
	matinv(r_i0,r_i0,3);
	matmult(Cip0i0,r_i0,r_ip0,3,3,3);
	matmult(Cip0n,Ci0n,Cip0i0,3,3,3);
	mattran(Cnip0,Cip0n,3,3);
	matmult(Cnp_cur,Cip0p,Cnip0,3,3,3);
	dmat2quat(Qnp_cur, Cnp_cur);
	dmat2atti(atti_cur,Cnp_cur);
}

/************************************************************************\
功能:初始对准粗对准阶段，惯性坐标系下的速度更新
输入:
 * Qip0p_pre - 前一时刻，从坐标系ip0到坐标系p的姿态四元数，4维向量
 * v_ip0_pre - 前一时刻，惯性坐标系ip0的速度向量
 * u_i0_pre - 前一时刻，惯性坐标系i0的速度向量
 * dtheta_ip_p - 从前一时刻到当前时刻陀螺输出的角增量，3维向量
 * dv_ip_p - 从前一时刻到当前时刻加速度计输出的速度增量，3维向量
 * posn_init - 粗对准初始时刻的纬度、经度
 * posn_ref - 粗对准当前时刻的纬度、经度
 * vel_ref - 粗对准当前时刻的东向速度、北向速度
 * k - 粗对准开始到当前时刻计数
 * period - 更新周期
输出:
 * Qip0p_cur - 当前时刻，从坐标系ip0到坐标系p的姿态四元数，4维向量
 * Cip0p_cur - 当前时刻，从坐标系ip0到坐标系p的方向余弦阵，3*3维矩阵链式结构
 * v_ip0_cur - 当前时刻，惯性坐标系ip0的速度向量
 * u_i0_cur - 当前时刻，惯性坐标系i0的速度向量
日期:2010.12.1
作者:
\************************************************************************/
void velUpdate(FLOAT64_T Qip0p_cur[], FLOAT64_T Cip0p_cur[], FLOAT64_T Ci0n[], FLOAT64_T v_ip0_cur[], FLOAT64_T u_i0_cur[], 
			   const FLOAT64_T Qip0p_pre[], const FLOAT64_T v_ip0_pre[], const FLOAT64_T u_i0_pre[], 
			   const FLOAT64_T dtheta_ip_p[], const FLOAT64_T dv_ip_p[], const FLOAT64_T posn_init[], 
			   const FLOAT64_T posn_ref[], const FLOAT64_T vel_ref[], const UINT32_T k, const FLOAT64_T period)
{
	FLOAT64_T lat,lon, lon_init, dlon;
	FLOAT64_T g;
	FLOAT64_T qh[4];
	FLOAT64_T tmp;
	FLOAT64_T Cpip0_cur[3*3];
	FLOAT64_T f_p[3], omega_ip_p[3], x[3],y[3];
	
	lat = posn_ref[0];
	lon = posn_ref[1];
	lon_init = posn_init[1]; lon = lon-lon_init+lon_init;
	g = local_g(lat);
	
	dang2dquat(qh, dtheta_ip_p);
	quatmult(Qip0p_cur, Qip0p_pre, qh);
	tmp = norm(Qip0p_cur, 4); // 取模
	// 归一化
	Qip0p_cur[0] /= tmp;
	Qip0p_cur[1] /= tmp;
	Qip0p_cur[2] /= tmp;
	Qip0p_cur[3] /= tmp;
	
	quat2dmat(Cip0p_cur, Qip0p_cur);
	mattran(Cpip0_cur, Cip0p_cur, 3,3);
	// 惯性系下的比力
	f_p[0] = dv_ip_p[0]/period;
	f_p[1] = dv_ip_p[1]/period;
	f_p[2] = dv_ip_p[2]/period;
	// 惯性系下的载体角速度
	omega_ip_p[0] = dtheta_ip_p[0]/period;
	omega_ip_p[1] = dtheta_ip_p[1]/period;
	omega_ip_p[2] = dtheta_ip_p[2]/period;
	xmult(y,omega_ip_p,vel_ref);
	matadd(x,y,f_p,3,1);
	matmult(y,Cpip0_cur,x,3,3,1);
	// ip0系的速度更新
	v_ip0_cur[0] = v_ip0_pre[0]+y[0]*period;
	v_ip0_cur[1] = v_ip0_pre[1]+y[1]*period;
	v_ip0_cur[2] = v_ip0_pre[2]+y[2]*period;
	//dlon = lon-lon_init+OMEGA_IE*k*period;
	dlon = OMEGA_IE*k*period;
	Ci0n[0] = -sin(dlon);
	Ci0n[1] = cos(dlon);
	Ci0n[2] = 0.0;
	Ci0n[3] = -sin(lat)*cos(dlon);
	Ci0n[4] = -sin(lat)*sin(dlon);
	Ci0n[5] = cos(lat);
	Ci0n[6] = cos(lat)*cos(dlon);
	Ci0n[7] = cos(lat)*sin(dlon);
	Ci0n[8] = sin(lat);
	// i0系的速度更新
	u_i0_cur[0] = u_i0_pre[0]+g*cos(lat)*cos(dlon)*period;
	u_i0_cur[1] = u_i0_pre[1]+g*cos(lat)*sin(dlon)*period;
	u_i0_cur[2] = u_i0_pre[2]+g*sin(lat)*period;
}


/************************************************************************\
功能: 姿态更新
输入:
 * Qnp_pre - 前一时刻，从坐标系n到坐标系p的姿态四元数，4维向量
 * dtheta_ip_p - 从前一时刻到当前时刻陀螺输出的角增量，3维向量
 * omega_in_n - 导航坐标系n系相对于惯性坐标系i系的旋转角速率在n系上的投影，3维向量
 * period - 更新周期
输出:
 * Qnp_cur - 当前时刻，从坐标系n到坐标系p的姿态四元数，4维向量
 * Cnp_cur - 当前时刻，从n坐标系转换到p坐标系的方向余弦阵，3*3维矩阵
 * atti_cur - 当前时刻三维向量系统姿态，横摇、纵摇、航向
日期: 2009.7.6
作者: FairyLan
\************************************************************************/
void attiUpdate(FLOAT64_T Qnp_cur[], FLOAT64_T Cnp_cur[], FLOAT64_T atti_cur[],
				const FLOAT64_T Qnp_pre[], const FLOAT64_T dtheta_ip_p[], const FLOAT64_T omega_in_n[], const FLOAT64_T period)
{
	FLOAT64_T dtheta_in_n[3];
	FLOAT64_T tmp;
	FLOAT64_T ph[4],ph_conj[4]; // dtheta_in_n rot quat's conj
	FLOAT64_T qh[4]; // dtheta_ip_p rot quat
	
	dtheta_in_n[0] = -omega_in_n[0]*period;
	dtheta_in_n[1] = -omega_in_n[1]*period;
	dtheta_in_n[2] = -omega_in_n[2]*period;
	
	dang2dquat(ph_conj, dtheta_in_n);
	dang2dquat(qh, dtheta_ip_p);
	
	quatmult(ph, ph_conj, Qnp_pre);
	quatmult(Qnp_cur, ph, qh);
	
	tmp = norm(Qnp_cur, 4);
	Qnp_cur[0] /= tmp;
	Qnp_cur[1] /= tmp;
	Qnp_cur[2] /= tmp;
	Qnp_cur[3] /= tmp;
	
	quat2dmat(Cnp_cur, Qnp_cur);
	dmat2atti(atti_cur, Cnp_cur);
}


/************************************************************************\
功能:速度和位置更新(有阻尼)
输入:
 * vel_pre     - 前一时刻解算速度，3维向量，东向、北向、天向
 * posn_pre    - 前一时刻解算位置，3维向量，纬度、经度、高度
 * posn_damp_pre - 阻尼前的位置
 * vel_ref     - 当前参考速度
 * posn_ref    - 当前参考位置
 * f_n         - 导航坐标系n系下的加速度
 * period      - 更新周期
输出:
 * vel_cur     - 当前时刻解算速度，3维向量
 * posn_cur    - 前一时刻解算位置，3维向量
 * omega_in_n  - 导航坐标系n系相对于惯性坐标系i系的旋转角速率在n系上的投影，3维向量
 * vel_damp	 - 阻尼后的速度
 * posn_damp_cur - 阻尼后的位置
全局变量:
 * g_stNavData(dVelMid,dKx,dKy,dTK,dDK,dVelInt) - 阻尼参数
 * g_stBindPara.dBgEqu - 等效陀螺漂移补偿
日期: 2009.7.6
作者: FairyLan
\************************************************************************/
void naviUpdateK(FLOAT64_T vel_cur[], FLOAT64_T posn_cur[], FLOAT64_T omega_in_n[], 
				FLOAT64_T vel_damp[], FLOAT64_T posn_damp_cur[], FLOAT64_T f_d[],
				const FLOAT64_T vel_pre[], const FLOAT64_T posn_pre[], const FLOAT64_T posn_damp_pre[],
				const FLOAT64_T vel_ref[], const FLOAT64_T posn_ref[],
				const FLOAT64_T f_n[], const FLOAT64_T period)
{
	FLOAT64_T tmp;
	FLOAT64_T lat,lon,height;
	FLOAT64_T latd,lond;
	FLOAT64_T vE,vN,vU,vEd,vNd,vUd;
	FLOAT64_T sinL,cosL,tanL;
	FLOAT64_T g;
	FLOAT64_T RE,RN;
	FLOAT64_T omegaN,omegaE;
	FLOAT64_T dA[3];
	FLOAT64_T JK[2];
	FLOAT64_T vEm,vNm;
	FLOAT64_T omega_c = 0.0;
	
	vE = vel_pre[0]; vN = vel_pre[1]; vU = vel_pre[2];
	lat = posn_pre[0]; lon = posn_pre[1]; height = posn_pre[2];
	latd = posn_damp_pre[0]; lond = posn_damp_pre[1];
	
	sinL = sin(lat); cosL = cos(lat); tanL = tan(lat);
	// 地球参数计算
	g = local_g(lat);
	tmp = 1.0-GLV_E2*sinL*sinL;
	RN = GLV_RE*(1.0-GLV_E2)/pow(tmp,1.5)+height;
	RE = GLV_RE/pow(tmp,0.5)+height;
	// 有害加速度计算
	dA[0] = -(2.0*OMEGA_IE*sinL+vE*tanL/RE)*vN+(2.0*OMEGA_IE*cosL+vE/RE)*vU;
	dA[1] = (2.0*OMEGA_IE*sinL+vE*tanL/RE)*vE+vN*vU/RN;
	dA[2] = -(2.0*OMEGA_IE*cosL+vE/RE)*vE-vN*vN/RN+g;
	f_d[0] = f_n[0]-dA[0];
	f_d[1] = f_n[1]-dA[1];
	f_d[2] = f_n[2]-dA[2];
	// 速度更新
	vE = vE+f_d[0]*period;
	vN = vN+f_d[1]*period;
	vU = vU+f_d[2]*period;
	// 速度阻尼网络计算
	g_stNavData.dVelMid[0] = vE-g_stNavData.dTK[0];
	g_stNavData.dVelMid[1] = vN-g_stNavData.dTK[1];

	g_stNavData.dTK[0] = g_stNavData.dTK[0]+g_stNavData.dKx[0]*(g_stNavData.dVelMid[0]-vel_ref[0])*period;
	g_stNavData.dTK[1] = g_stNavData.dTK[1]+g_stNavData.dKy[0]*(g_stNavData.dVelMid[1]-vel_ref[1])*period;

	g_stNavData.dDK[0] = g_stNavData.dDK[0]+g_stNavData.dKx[2]*(g_stNavData.dVelMid[0]-vel_ref[0])*period;
	g_stNavData.dDK[1] = g_stNavData.dDK[1]+g_stNavData.dKy[2]*(g_stNavData.dVelMid[1]-vel_ref[1])*period;
	
	JK[0] = g_stNavData.dDK[0]+g_stNavData.dKx[1]*(g_stNavData.dVelMid[0]-vel_ref[0]);
	JK[1] = g_stNavData.dDK[1]+g_stNavData.dKy[1]*(g_stNavData.dVelMid[1]-vel_ref[1]);
	
	vEm = g_stNavData.dVelMid[0]+g_stNavData.dDK[0];
	vNm = g_stNavData.dVelMid[1]+g_stNavData.dDK[1];
	// 阻尼后速度计算
	vEd = g_stNavData.dVelMid[0]+JK[0];
	vNd = g_stNavData.dVelMid[1]+JK[1]; 
	g_stNavData.dVelInt[0] = vEd;
	g_stNavData.dVelInt[1] = vNd;
	if (g_stNavData.ucWorkMode == U8_WORK_CALIBRATE_SYS)
		vUd = vU; 
	else 
		vUd = 0;// 水面置0
	
	// 载体相对地球旋转角速度计算
	omegaN = vEd/RE;
	omegaE = vNd/RN;

	omega_c = (g_stNavData.dVelMid[1]-vel_ref[1])/(RN*OMEGA_IE*cosL);
	
	// 位置更新
	height += vU*period;
	//height = 0.0; // 水面置0
	lat += vNd*period/RN;
	lon += vEd*period/RE/cosL;
	if (lon > PI)
		lon -= 2*PI;
	else if(lon < -PI)
		lon += 2*PI;

	latd += vN*period/RN;
	lond += vE*period/RE/cosL;
	if (lond > PI)
		lond -= 2*PI;
	else if(lond < -PI)
		lond += 2*PI;
	// 赋值
	vel_cur[0] = vE;
	vel_cur[1] = vN;
	vel_cur[2] = vUd;
	
	posn_cur[0] = lat;
	posn_cur[1] = lon;
	posn_cur[2] = height;
	
	posn_damp_cur[0] = latd;
	posn_damp_cur[1] = lond;
	posn_damp_cur[2] = height;

	vel_damp[0] = vEm;
	vel_damp[1] = vNm;
	vel_damp[2] = vUd;

	omega_in_n[0] = -omegaE-g_stBindPara.dBgEqu[0]; // 补偿等效东向陀螺漂移
	omega_in_n[1] = OMEGA_IE*cosL+omegaN-g_stBindPara.dBgEqu[1]; // 补偿等效北向陀螺漂移
	omega_in_n[2] = OMEGA_IE*sinL+omegaN*tanL+g_stNavData.dKy[3]*omega_c;

	if (g_stNavData.ucWorkMode == U8_WORK_COMPASS || g_stNavData.ucWorkMode == U8_WORK_COMPALIGN_AZIMUTH)
	{
		posn_cur[0] = posn_ref[0];
		posn_cur[1] = posn_ref[1];
		posn_damp_cur[0] = posn_ref[0];
		posn_damp_cur[1] = posn_ref[1];
		vel_damp[0] = g_stNavData.dVelMid[0];
		vel_damp[1] = g_stNavData.dVelMid[1];
	}
	
}



/************************************************************************\
功能: 导航坐标系n系的加速度更新
输入:
 * Cnp_cur     - 当前时刻，从n坐标系转换到p坐标系的方向余弦阵，3*3维矩阵
 * dv_p        - 从前一时刻到当前时刻加速度计输出的速度增量，3维向量
 * period      - 更新周期
输出:
 * f_n         - 导航坐标系n系下的加速度
 * Cpn_cur     - 当前时刻，从p坐标系转换到n坐标系的方向余弦阵，3*3维矩阵
日期: 2009.7.6
作者: FairyLan
\************************************************************************/
void forceUpdate(FLOAT64_T f_n[], FLOAT64_T Cpn_cur[], const FLOAT64_T Cnp_cur[], const FLOAT64_T dv_p[], const FLOAT64_T period)
{
	FLOAT64_T dv_ip_n[3];
	INT32_T i;

	mattran(Cpn_cur, Cnp_cur, 3, 3);
	matmult(dv_ip_n, Cpn_cur, dv_p, 3, 3, 1);
	for (i=0; i<3; i++)
		f_n[i] = dv_ip_n[i]/period;
}

/************************************************************************\
功能: 计算瞬时位移(高通滤波)
输入:
 * atti_cur    - 当前时刻，从n坐标系转换到p坐标系的姿态角
 * period      - 更新周期
输出:
 * f_d         - 振荡坐标系n系下的加速度
日期: 2015.11.8
作者: FairyLan
b = 0.999813017920342	-2.99943905376103	2.99943905376103	-0.999813017920342
a = 1	-2.99962600087512	2.99925207168464	-0.999626070802983
       b[0]+b[1]*z^(-1)+b[2]*z^(-2)+b[3]*z^(-3)
H(z) = ----------------------------------------
       a[0]+a[1]*z^(-1)+a[2]*z^(-2)+a[3]*z^(-3)
       
\************************************************************************/
void shiftFilter(FLOAT64_T vel_d_cur[], FLOAT64_T shift_d_cur[], const FLOAT64_T vel_d_pre[], const FLOAT64_T shift_d_pre[], const FLOAT64_T atti_cur[], const FLOAT64_T f_d[], const FLOAT64_T period)
{
	FLOAT64_T f_s[3] = {0.0};
	FLOAT64_T sH,cH;
	FLOAT64_T vel_x[3];
	INT16_T i;
	
	sH = sin(atti_cur[2]);
	cH = cos(atti_cur[2]);

	f_s[0] = cH*f_d[0]-sH*f_d[1];
	f_s[1] = sH*f_d[0]+cH*f_d[1];
	f_s[2] = f_d[2];

	vel_x[0] = vel_d_pre[0]+f_s[0]*period;
	vel_x[1] = vel_d_pre[1]+f_s[1]*period;
	vel_x[2] = vel_d_pre[2]+f_s[2]*period;
	
	for (i=3; i>0; i--)
	{
		g_stNavData.dVDx[0][i] = g_stNavData.dVDx[0][i-1];
		g_stNavData.dVDx[1][i] = g_stNavData.dVDx[1][i-1];
		g_stNavData.dVDx[2][i] = g_stNavData.dVDx[2][i-1];
	}
	for (i=0; i<3; i++)
	{
		g_stNavData.dVDx[i][0] = vel_x[i];
	}
	
	for (i=3; i>0; i--)
	{
		g_stNavData.dVDy[0][i] = g_stNavData.dVDy[0][i-1];
		g_stNavData.dVDy[1][i] = g_stNavData.dVDy[1][i-1];
		g_stNavData.dVDy[2][i] = g_stNavData.dVDy[2][i-1];
	}
	for (i=0; i<3; i++)
	{
		g_stNavData.dVDy[i][0] = -g_stNavData.dVDy[i][1]*(-2.99962600087512)-g_stNavData.dVDy[i][2]*(2.99925207168464)-g_stNavData.dVDy[i][3]*(-0.999626070802983)
			+g_stNavData.dVDx[i][0]*(0.999813017920342)+g_stNavData.dVDx[i][1]*(-2.99943905376103)+g_stNavData.dVDx[i][2]*(-2.99943905376103)+g_stNavData.dVDx[i][3]*(-0.999813017920342);
		vel_d_cur[i] = g_stNavData.dVDy[i][0];
		shift_d_cur[i] = shift_d_pre[i]+g_stNavData.dVDy[i][0]*period;
	}

}

void BDPredictPHI(FLOAT64_T PHI[], FLOAT64_T FUN[],
				const FLOAT64_T vel_pre[], const FLOAT64_T posn_pre[], 
				const FLOAT64_T omega_b[], const FLOAT64_T f_b[],
				const FLOAT64_T Cbn[], const FLOAT64_T f_n[], const FLOAT64_T period)
{
	INT16_T i;
	FLOAT64_T lat,height;
	FLOAT64_T vE,vN,vU;
	FLOAT64_T sinL,cosL,tanL;
	FLOAT64_T tmp;
	FLOAT64_T RE,RN;
	FLOAT64_T F[27*27] = {0.0}, F1[27*27] = {0.0}, F2[27*27] = {0.0}, F3[27*27] = {0.0};
	
	lat = posn_pre[0]; height = posn_pre[2];
	vE = vel_pre[0]; vN = vel_pre[1]; vU = vel_pre[2];
	
	sinL = sin(lat);
	cosL = cos(lat);
	tanL = tan(lat);
	
	tmp = 1-GLV_E*GLV_E*sinL*sinL;
	RN = GLV_RE*(1.0-pow(GLV_E,2.0))/pow(tmp,1.5)+height;
	RE = GLV_RE/pow(tmp,0.5)+height;

	// state equation
	memset(F,0,729*sizeof(FLOAT64_T));
	F[1] = OMEGA_IE*sinL+vE*tanL/RE;
	F[2] = -(OMEGA_IE*cosL+vE/RE);
	F[4] = -1.0/RN;
	F[6] = -Cbn[0];
	F[7] = -Cbn[1];
	F[8] = -Cbn[2];
	F[12] = -Cbn[0]*omega_b[0];
	F[13] = -Cbn[1]*omega_b[1];
	F[14] = -Cbn[2]*omega_b[2];
	F[15] = -Cbn[1]*omega_b[0];
	F[16] = Cbn[2]*omega_b[0];
	F[17] = -Cbn[2]*omega_b[1];

	F[27+0] = -(OMEGA_IE*sinL+vE*tanL/RE);
	F[27+2] = -vN/RN;
	F[27+3] = 1.0/RE;
	F[27+6] = -Cbn[3];
	F[27+7] = -Cbn[4];
	F[27+8] = -Cbn[5];
	F[27+12] = -Cbn[3]*omega_b[0];
	F[27+13] = -Cbn[4]*omega_b[1];
	F[27+14] = -Cbn[5]*omega_b[2];
	F[27+15] = -Cbn[4]*omega_b[0];
	F[27+16] = Cbn[5]*omega_b[0];
	F[27+17] = -Cbn[5]*omega_b[1];
	
	F[54+0] = OMEGA_IE*cosL+vE/RE;
	F[54+1] = vN/RN;
	F[54+3] = tanL/RE;
	F[54+6] = -Cbn[6];
	F[54+7] = -Cbn[7];
	F[54+8] = -Cbn[8];
	F[54+12] = -Cbn[6]*omega_b[0];
	F[54+13] = -Cbn[7]*omega_b[1];
	F[54+14] = -Cbn[8]*omega_b[2];
	F[54+15] = -Cbn[7]*omega_b[0];
	F[54+16] = Cbn[8]*omega_b[0];
	F[54+17] = -Cbn[8]*omega_b[1];

	F[27*3+1] = -f_n[2];
	F[27*3+2] = f_n[1];
	F[27*3+3] = (vN*tanL-vU)/RE;
	F[27*3+4] = 2.0*OMEGA_IE*sinL+vE*tanL/RE;
	F[27*3+5] = -(2.0*OMEGA_IE*cosL+vE/RE);
	F[27*3+9] = Cbn[0];
	F[27*3+10] = Cbn[1];
	F[27*3+11] = Cbn[2];
	F[27*3+18] = Cbn[0]*f_b[0];
	F[27*3+19] = Cbn[1]*f_b[1];
	F[27*3+20] = Cbn[2]*f_b[2];
	F[27*3+21] = Cbn[0]*f_b[1];
	F[27*3+22] = -Cbn[0]*f_b[2];
	F[27*3+23] = Cbn[1]*f_b[2];
	F[27*3+24] = -Cbn[1]*f_b[0];
	F[27*3+25] = Cbn[2]*f_b[0];
	F[27*3+26] = -Cbn[2]*f_b[1];

	F[27*4+0] = f_n[2];
	F[27*4+2] = -f_n[0];
	F[27*4+3] = -2.0*(OMEGA_IE*sinL+vE*tanL/RE);
	F[27*4+4] = -vU/RN;
	F[27*4+5] = -vN/RN;
	F[27*4+9] = Cbn[3];
	F[27*4+10] = Cbn[4];
	F[27*4+11] = Cbn[5];
	F[27*4+18] = Cbn[3]*f_b[0];
	F[27*4+19] = Cbn[4]*f_b[1];
	F[27*4+20] = Cbn[5]*f_b[2];
	F[27*4+21] = Cbn[3]*f_b[1];
	F[27*4+22] = -Cbn[3]*f_b[2];
	F[27*4+23] = Cbn[4]*f_b[2];
	F[27*4+24] = -Cbn[4]*f_b[0];
	F[27*4+25] = Cbn[5]*f_b[0];
	F[27*4+26] = -Cbn[5]*f_b[1];

	F[27*5+0] = -f_n[1];
	F[27*5+1] = f_n[0];
	F[27*5+3] = 2.0*(OMEGA_IE*cosL+vE/RE);
	F[27*5+4] = -2.0*vN/RN;
	F[27*5+9] = Cbn[6];
	F[27*5+10] = Cbn[7];
	F[27*5+11] = Cbn[8];
	F[27*5+18] = Cbn[6]*f_b[0];
	F[27*5+19] = Cbn[7]*f_b[1];
	F[27*5+20] = Cbn[8]*f_b[2];
	F[27*5+21] = Cbn[6]*f_b[1];
	F[27*5+22] = -Cbn[6]*f_b[2];
	F[27*5+23] = Cbn[7]*f_b[2];
	F[27*5+24] = -Cbn[7]*f_b[0];
	F[27*5+25] = Cbn[8]*f_b[0];
	F[27*5+26] = -Cbn[8]*f_b[1];

	// discrete state matrix
	for (i=0; i<729; i++)
	{
		F1[i] = F[i]*period;
		FUN[i] = F[i];
	}
	matmult(F2,F1,F1,27,27,27);
	matmult(F3,F2,F1,27,27,27);	
	// 计算状态转移阵
	for (i=0; i<729; i++)
	{
		PHI[i] = F1[i]+F2[i]/2.0+F3[i]/6.0;
		if (0 == (i%28))
			PHI[i] += 1.0;
	}
}

void BDPredictQd(FLOAT64_T Qd[], const FLOAT64_T FUN[],
				 const FLOAT64_T omega_b[], const FLOAT64_T f_b[],
				 const FLOAT64_T Cbn[], const FLOAT64_T period)
{
	INT32_T i;
	FLOAT64_T F[729] = {0.0}, F1[729] = {0.0}, F2[729] = {0.0}, F3[729] = {0.0};
	FLOAT64_T FQ[729] = {0.0}, QF[729] = {0.0};
	FLOAT64_T Gz[567] = {0.0}, Gt[567] = {0.0};
	FLOAT64_T QG[567] = {0.0};
	
	memcpy(F, FUN, 729*sizeof(FLOAT64_T));
	// noise input matrix
	memset(Gz,0,567*sizeof(FLOAT64_T));
	Gz[0] = -Cbn[0];
	Gz[1] = -Cbn[1];
	Gz[2] = -Cbn[2];
	Gz[6] = -Cbn[0]*omega_b[0];
	Gz[7] = -Cbn[1]*omega_b[1];
	Gz[8] = -Cbn[2]*omega_b[2];
	Gz[9] = -Cbn[1]*omega_b[0];
	Gz[10] = Cbn[2]*omega_b[0];
	Gz[11] = -Cbn[2]*omega_b[1];
	//Gz[0] = -1.0;
	Gz[21+0] = -Cbn[3];
	Gz[21+1] = -Cbn[4];
	Gz[21+2] = -Cbn[5];
	Gz[21+6] = -Cbn[3]*omega_b[0];
	Gz[21+7] = -Cbn[4]*omega_b[1];
	Gz[21+8] = -Cbn[5]*omega_b[2];
	Gz[21+9] = -Cbn[4]*omega_b[0];
	Gz[21+10] = Cbn[5]*omega_b[0];
	Gz[21+11] = -Cbn[5]*omega_b[1];
	//Gz[5+1] = -1.0;
	Gz[21*2+0] = -Cbn[6];
	Gz[21*2+1] = -Cbn[7];
	Gz[21*2+2] = -Cbn[8];
	Gz[21*2+6] = -Cbn[6]*omega_b[0];
	Gz[21*2+7] = -Cbn[7]*omega_b[1];
	Gz[21*2+8] = -Cbn[8]*omega_b[2];
	Gz[21*2+9] = -Cbn[7]*omega_b[0];
	Gz[21*2+10] = Cbn[8]*omega_b[0];
	Gz[21*2+11] = -Cbn[8]*omega_b[1];
	//Gz[10+2] = -1.0;
	Gz[21*3+3] = Cbn[0];
	Gz[21*3+4] = Cbn[1];
	Gz[21*3+5] = Cbn[2];
	Gz[21*3+12] = Cbn[0]*f_b[0];
	Gz[21*3+13] = Cbn[1]*f_b[1];
	Gz[21*3+14] = Cbn[2]*f_b[2];
	Gz[21*3+15] = Cbn[0]*f_b[1];
	Gz[21*3+16] = -Cbn[0]*f_b[2];
	Gz[21*3+17] = Cbn[1]*f_b[2];
	Gz[21*3+18] = -Cbn[1]*f_b[0];
	Gz[21*3+19] = Cbn[2]*f_b[0];
	Gz[21*3+20] = -Cbn[2]*f_b[1];

	Gz[21*4+3] = Cbn[3];
	Gz[21*4+4] = Cbn[4];
	Gz[21*4+5] = Cbn[5];
	Gz[21*4+12] = Cbn[3]*f_b[0];
	Gz[21*4+13] = Cbn[4]*f_b[1];
	Gz[21*4+14] = Cbn[5]*f_b[2];
	Gz[21*4+15] = Cbn[3]*f_b[1];
	Gz[21*4+16] = -Cbn[3]*f_b[2];
	Gz[21*4+17] = Cbn[4]*f_b[2];
	Gz[21*4+18] = -Cbn[4]*f_b[0];
	Gz[21*4+19] = Cbn[5]*f_b[0];
	Gz[21*4+20] = -Cbn[5]*f_b[1];

	Gz[21*5+3] = Cbn[6];
	Gz[21*5+4] = Cbn[7];
	Gz[21*5+5] = Cbn[8];
	Gz[21*5+12] = Cbn[6]*f_b[0];
	Gz[21*5+13] = Cbn[7]*f_b[1];
	Gz[21*5+14] = Cbn[8]*f_b[2];
	Gz[21*5+15] = Cbn[6]*f_b[1];
	Gz[21*5+16] = -Cbn[6]*f_b[2];
	Gz[21*5+17] = Cbn[7]*f_b[2];
	Gz[21*5+18] = -Cbn[7]*f_b[0];
	Gz[21*5+19] = Cbn[8]*f_b[0];
	Gz[21*5+20] = -Cbn[8]*f_b[1];
	// input noise var discrete
	mattran(Gt,Gz,27,21);
	matmult(QG,Gz,g_stBDPara.dQ,27,21,21);
	matmult(F1,QG,Gt,27,21,27); // F1 = G*Q*G'
	
	matmult(FQ,F,F1,27,27,27);
	mattran(QF,FQ,27,27);
	matadd(F2,FQ,QF,27,27); // F2 = F*F1+(F*F1)'
	
	matmult(FQ,F,F2,27,27,27);
	mattran(QF,FQ,27,27);
	matadd(F3,FQ,QF,27,27); // F3 = F*F2+(F*F2)'
	
	for (i=0; i<27*27; i++)
		Qd[i] = F1[i]*period+F2[i]*period*period/2.0+F3[i]*period*period*period/6.0;
}

void BDPredictEnd(FLOAT64_T Xkk[], FLOAT64_T Pkk[], 
				  const FLOAT64_T X_pre[], const FLOAT64_T P_pre[], 
				  const FLOAT64_T PHI[], const FLOAT64_T Qd[])
{
	FLOAT64_T F1[27*27] = {0.0}, F2[27*27] = {0.0}, F3[27*27] = {0.0};
	// predict
	matmult(Xkk,PHI,X_pre,27,27,1); // state estimate extrapolation
	matmult(F1,PHI,P_pre,27,27,27);
	mattran(F2,PHI,27,27);
	matmult(F3,F1,F2,27,27,27);
	matadd(Pkk,F3,Qd,27,27);	// error covariance extrapolation
}

INT32_T  BDUpdate(FLOAT64_T X_cur[], FLOAT64_T P_cur[], 
				  const FLOAT64_T Xkk[], const FLOAT64_T Pkk[], const FLOAT64_T delta_vel[])
{
	INT32_T i;
	FLOAT64_T Z[3],r[3];
	FLOAT64_T Kk[27*3];
	FLOAT64_T Ht[3*27];
	FLOAT64_T F1[27*27], F2[27*27], F3[27*27];
	INT32_T flag = 1;
	
	Z[0] = delta_vel[0];
	Z[1] = delta_vel[1];
	Z[2] = delta_vel[2];
	// Kk
	mattran(Ht,g_stBDPara.dH,3,27);
	matmult(F1,g_stBDPara.dH,Pkk,3,27,27);
	matmult(F2,F1,Ht,3,27,3);
	matadd(F1,F2,g_stBDPara.dR,3,3);
	flag = matinv(F2,F1,3);
	matmult(F3,Pkk,Ht,27,27,3);
	matmult(Kk,F3,F2,27,3,3);
	// Xk
	matmult(F1,g_stBDPara.dH,Xkk,3,27,1);
	matsub(r,Z,F1,3,1);
	matmult(F2,Kk,r,27,3,1);
	matadd(X_cur,Xkk,F2,27,1);
	// Pk
	matmult(F1,Kk,g_stBDPara.dH,27,3,27);
	//matsub(F2,g_stKfPara.dI12,F1,12,12);
	//matmult(P_cur,F2,Pkk,12,12,12);
	for (i=0; i<27*27; i++)
	{
		F2[i] = -F1[i];
		if (0 == (i%28))
			F2[i] += 1.0;
	}
	matmult(P_cur,F2,Pkk,27,27,27);
	
	return flag;
}

/************************************************************************\
功能:卡尔曼滤波一步预测第一阶段:状态转换阵的离散化
输入:
 * vel_pre     - 前一时刻解算速度，3维向量，东向、北向、垂向
 * posn_pre    - 前一时刻解算位置，2维向量，纬度、经度、高度
 * Cpn         - 当前时刻，从坐标系p到坐标系n的方向余弦阵，3*3维矩阵链式结构
 * f_n         - 导航坐标系n系下的加速度
 * period      - 更新周期
输出:
 * PHI     	 - 前一时刻到当前时刻的状态转移阵
日期: 2009.9
作者: FairyLan
日志:
\************************************************************************/
void KFPredictPHI(FLOAT64_T PHI[], FLOAT64_T FUN[],
			   const FLOAT64_T vel_pre[], const FLOAT64_T posn_pre[], 
			   const FLOAT64_T Cpn[], const FLOAT64_T f_n[], const FLOAT64_T period)
{
	INT16_T i;
	FLOAT64_T lat,height;
	FLOAT64_T vE,vN;
	FLOAT64_T sinL,cosL,tanL;
	FLOAT64_T tmp;
	FLOAT64_T RE,RN;
	FLOAT64_T F[12*12] = {0.0}, F1[12*12] = {0.0}, F2[12*12] = {0.0}, F3[12*12] = {0.0};

	lat = posn_pre[0]; height = posn_pre[2];
	vE = vel_pre[0]; vN = vel_pre[1];

	sinL = sin(lat); cosL = cos(lat); tanL = tan(lat);
	
	tmp = 1-GLV_E2*sinL*sinL;
	RN = GLV_RE*(1.0-GLV_E2)/pow(tmp,1.5)+height;
	RE = GLV_RE/pow(tmp,0.5)+height;

	memset(F,0,144*sizeof(FLOAT64_T));
	F[1] = OMEGA_IE*sinL+vE*tanL/RE;
	F[2] = -(OMEGA_IE*cosL+vE/RE);
	F[4] = -1.0/RN;
	F[7] = -Cpn[0];
	F[8] = -Cpn[1];
	F[9] = -Cpn[2];
	//F[7] = -1.0;
	F[12+0] = -(OMEGA_IE*sinL+vE*tanL/RE);
	F[12+2] = -vN/RN;
	F[12+3] = 1.0/RE;
	F[12+5] = -OMEGA_IE*sinL;
	F[12+7] = -Cpn[3];
	F[12+8] = -Cpn[4];
	F[12+9] = -Cpn[5];
	//F[12+8] = -1.0;
	F[24+0] = OMEGA_IE*cosL+vE/RE;
	F[24+1] = vN/RN;
	F[24+3] = tanL/RE;
	F[24+5] = OMEGA_IE*cosL+vE/cosL/cosL/RE;
	F[24+7] = -Cpn[6];
	F[24+8] = -Cpn[7];
	F[24+9] = -Cpn[8];
	//F[24+9] = -1.0;
	F[36+1] = -f_n[2];
	F[36+2] = f_n[1];
	F[36+3] = vN*tanL/RE;
	F[36+4] = 2.0*OMEGA_IE*sinL+vE*tanL/RE;
	F[36+5] = (2.0*OMEGA_IE*cosL+vE/cosL/cosL/RE)*vN;
	F[36+10] = Cpn[0];
	F[36+11] = Cpn[1];
	F[48+0] = f_n[2];
	F[48+2] = -f_n[0];
	F[48+3] = -2.0*(OMEGA_IE*sinL+vE*tanL/RE);
	F[48+5] = -(2.0*OMEGA_IE*cosL+vE/cosL/cosL/RE)*vE;
	F[48+10] = Cpn[3];
	F[48+11] = Cpn[4];
	F[60+4] = 1.0/RN;
	F[72+3] = 1.0/cosL/RE;
	F[72+5] = vE*tanL/cosL/RE;
	// discrete state matrix
	for (i=0; i<144; i++)
	{
		F1[i] = F[i]*period;
		FUN[i] = F[i];
	}
	matmult(F2,F1,F1,12,12,12);
	matmult(F3,F2,F1,12,12,12);	
	// 计算状态转移阵
	for (i=0; i<144; i++)
	{
		PHI[i] = F1[i]+F2[i]/2.0+F3[i]/6.0;
		if (0 == (i%13))
			PHI[i] += 1.0;
	}
}

/************************************************************************\
功能: 卡尔曼滤波一步预测第二阶段:噪声方差阵的离散化
输入:
 * Cpn         - 当前时刻，从坐标系p到坐标系n的方向余弦阵，3*3维矩阵链式结构
 * period      - 更新周期
输出:
 * Qd    		 - 当前噪声方差阵
全局变量:
 * g_stKfPara.dQ0 - 设定噪声方差
日期: 2009.9
作者: FairyLan
日志:
\************************************************************************/
void KFPredictQd(FLOAT64_T Qd[], const FLOAT64_T FUN[],
			   const FLOAT64_T Cpn[], const FLOAT64_T period)
{
	INT32_T i;
	FLOAT64_T F[144] = {0.0}, F1[144] = {0.0}, F2[144] = {0.0}, F3[144] = {0.0};
	FLOAT64_T FQ[144] = {0.0}, QF[144] = {0.0};
	FLOAT64_T Gz[60] = {0.0}, Gt[60] = {0.0};
	FLOAT64_T QG[60] = {0.0};

	memcpy(F, FUN, 144*sizeof(FLOAT64_T));
	// noise input matrix
	memset(Gz,0,12*5*sizeof(FLOAT64_T));
	Gz[0] = -Cpn[0];
	Gz[1] = -Cpn[1];
	Gz[2] = -Cpn[2];
	//Gz[0] = -1.0;
	Gz[5+0] = -Cpn[3];
	Gz[5+1] = -Cpn[4];
	Gz[5+2] = -Cpn[5];
	//Gz[5+1] = -1.0;
	Gz[10+0] = -Cpn[6];
	Gz[10+1] = -Cpn[7];
	Gz[10+2] = -Cpn[8];
	//Gz[10+2] = -1.0;
	Gz[15+3] = Cpn[0];
	Gz[15+4] = Cpn[1];
	Gz[20+3] = Cpn[3];
	Gz[20+4] = Cpn[4];
	// input noise var discrete
	mattran(Gt,Gz,12,5);
	matmult(QG,Gz,g_stKfPara.dQ0,12,5,5);
	matmult(F1,QG,Gt,12,5,12); // F1 = G*Q*G'
	
	matmult(FQ,F,F1,12,12,12);
	mattran(QF,FQ,12,12);
	matadd(F2,FQ,QF,12,12); // F2 = F*F1+(F*F1)'
	
	matmult(FQ,F,F2,12,12,12);
	mattran(QF,FQ,12,12);
	matadd(F3,FQ,QF,12,12); // F3 = F*F2+(F*F2)'
	
	for (i=0; i<144; i++)
		Qd[i] = F1[i]*period+F2[i]*period*period/2.0+F3[i]*period*period*period/6.0;
}

/************************************************************************\
功能: 卡尔曼滤波一步预测第三阶段:状态与方差预测
输入:
 * X_pre       - 前一时刻的状态估计
 * P_pre       - 前一时刻的误差方差
 * PHI     	 - 前一时刻到当前时刻的状态转移阵
 * Qd    		 - 当前噪声方差阵
输出:
 * Xkk         - 当前状态预测值
 * Pkk         - 当前方差预测值
日期: 2009.9
作者: FairyLan
日志:
\************************************************************************/

void KFPredictEnd(FLOAT64_T Xkk[], FLOAT64_T Pkk[], 
			   const FLOAT64_T X_pre[], const FLOAT64_T P_pre[], 
			   const FLOAT64_T PHI[], const FLOAT64_T Qd[])
{
	FLOAT64_T F1[144] = {0.0}, F2[144] = {0.0}, F3[144] = {0.0};
	// predict
	matmult(Xkk,PHI,X_pre,12,12,1); // state estimate extrapolation
	matmult(F1,PHI,P_pre,12,12,12);
	mattran(F2,PHI,12,12);
	matmult(F3,F1,F2,12,12,12);
	matadd(Pkk,F3,Qd,12,12);	// error covariance extrapolation
}

/************************************************************************\
功能:卡尔曼滤波滤波更新
输入:
input
 * Xkk         - 当前状态预测值
 * Pkk         - 当前方差预测值
 * delta_vel   - 速度观测量
 * delta_posn  - 位置观测量
 * meas_mode   - 量测模式 0x01:速度 0x02:位置 0x03:速度+位置
输出:
 * X_cur       - 当前状态估计值
 * P_cur       - 当前方差估计值
 * 返回值 - 非奇异返回 1 
            奇异返回   0
全局变量:
 * g_stKfPara.dH0,dH1,dH2 - 设定滤波观测阵
 * g_stKfPara.dR0,dR1,dR2 - 设定量测噪声阵
 * g_stKfPara.dI15 - 单位阵 15*15
日期: 2009.9
作者: FairyLan
日志:
\************************************************************************/
INT32_T  KFUpdate(FLOAT64_T X_cur[], FLOAT64_T P_cur[], 
			  const FLOAT64_T Xkk[], const FLOAT64_T Pkk[],
			  const FLOAT64_T delta_vel[], const FLOAT64_T delta_posn[], UCHAR_T meas_mode)
{
	INT32_T i;
	FLOAT64_T Z[4],r[4];
	FLOAT64_T Kk[48];
	FLOAT64_T Ht[48];
	FLOAT64_T F1[144], F2[144], F3[144];
	INT32_T flag = 1;
	
	switch (meas_mode)
	{
	case (U8_KF_MEAS_VEL|U8_KF_MEAS_POSN):
		Z[0] = delta_vel[0];
		Z[1] = delta_vel[1];
		Z[2] = delta_posn[0];
		Z[3] = delta_posn[1];
		// Kk
		mattran(Ht,g_stKfPara.dH0,4,12);
		matmult(F1,g_stKfPara.dH0,Pkk,4,12,12);
		matmult(F2,F1,Ht,4,12,4);
		matadd(F1,F2,g_stKfPara.dR0,4,4);
		flag = matinv(F2,F1,4);
		matmult(F3,Pkk,Ht,12,12,4);
		matmult(Kk,F3,F2,12,4,4);
		// Xk
		matmult(F1,g_stKfPara.dH0,Xkk,4,12,1);
		matsub(r,Z,F1,4,1);
		matmult(F2,Kk,r,12,4,1);
		matadd(X_cur,Xkk,F2,12,1);
		// Pk
		matmult(F1,Kk,g_stKfPara.dH0,12,4,12);
		//matsub(F2,g_stKfPara.dI12,F1,12,12);
		//matmult(P_cur,F2,Pkk,12,12,12);
		break;
	case U8_KF_MEAS_VEL:
		Z[0] = delta_vel[0];
		Z[1] = delta_vel[1];
		// Kk
		mattran(Ht,g_stKfPara.dH1,2,12);
		matmult(F1,g_stKfPara.dH1,Pkk,2,12,12);
		matmult(F2,F1,Ht,2,12,2);
		matadd(F1,F2,g_stKfPara.dR1,2,2);
		flag = matinv(F2,F1,2);
		matmult(F3,Pkk,Ht,12,12,2);
		matmult(Kk,F3,F2,12,2,2);
		// Xk
		matmult(F1,g_stKfPara.dH1,Xkk,2,12,1);
		matsub(r,Z,F1,2,1);
		matmult(F2,Kk,r,12,2,1);
		matadd(X_cur,Xkk,F2,12,1);
		// Pk
		matmult(F1,Kk,g_stKfPara.dH1,12,2,12);
		//matsub(F2,g_stKfPara.dI12,F1,12,12);
		//matmult(P_cur,F2,Pkk,12,12,12);
		break;
	case U8_KF_MEAS_POSN:
		Z[0] = delta_posn[0];
		Z[1] = delta_posn[1];
		// Kk
		mattran(Ht,g_stKfPara.dH2,2,12);
		matmult(F1,g_stKfPara.dH2,Pkk,2,12,12);
		matmult(F2,F1,Ht,2,12,2);
		matadd(F1,F2,g_stKfPara.dR2,2,2);
		flag = matinv(F2,F1,2);
		matmult(F3,Pkk,Ht,12,12,2);
		matmult(Kk,F3,F2,12,2,2);
		// Xk
		matmult(F1,g_stKfPara.dH2,Xkk,2,12,1);
		matsub(r,Z,F1,2,1);
		matmult(F2,Kk,r,12,2,1);
		matadd(X_cur,Xkk,F2,12,1);
		// Pk
		matmult(F1,Kk,g_stKfPara.dH2,12,2,12);
		//matsub(F2,g_stKfPara.dI12,F1,12,12);
		//matmult(P_cur,F2,Pkk,12,12,12);
		break;
	default:
		break;
	}
	for (i=0; i<144; i++)
	{
		F2[i] = -F1[i];
		if (0 == (i%13))
			F2[i] += 1.0;
	}
	matmult(P_cur,F2,Pkk,12,12,12);

	return flag;
}

/************************************************************************\
功能:卡尔曼滤波修正输出
输入:
input
 * X_cur       - 当前状态估计值
 * Qnp_cur     - 当前解算姿态四元数
 * vel_pre     - 当前解算速度
 * posn_pre    - 当前解算位置
输出:
 * Qnp_out     - 修正后解算姿态四元数
 * Cnp_out     - 修正后方位余弦阵
 * atti_out    - 修正后解算姿态
 * vel_out     - 修正后解算速度
 * posn_out    - 修正后解算位置
 * phi_est     - 姿态误差估计值
 * gyro_bias_est   - 陀螺零偏估计值
 * acc_bias_est    - 加速度计零偏估计值
日期:2009.9
作者:
日志:
\************************************************************************/

void KFAdjust(FLOAT64_T Qnp_out[], FLOAT64_T Cnp_out[], FLOAT64_T atti_out[], FLOAT64_T vel_out[], FLOAT64_T posn_out[],
			  FLOAT64_T phi_est[], FLOAT64_T gyro_bias_est[], FLOAT64_T acc_bias_est[],
			  const FLOAT64_T X_cur[], const FLOAT64_T Qnp_pre[], const FLOAT64_T vel_pre[], const FLOAT64_T posn_pre[])
{
	FLOAT64_T dquat[4];
	// estimation
	phi_est[0] = X_cur[0]; // +acc_bias_est[1]/g;
	phi_est[1] = X_cur[1]; // -acc_bias_est[0]/g;
	phi_est[2] = X_cur[2]; // -gyro_bias_est[0]/OMEGA_IE/cosL
	gyro_bias_est[0] = X_cur[7];
	gyro_bias_est[1] = X_cur[8];
	gyro_bias_est[2] = X_cur[9];
	acc_bias_est[0] = X_cur[10];
	acc_bias_est[1] = X_cur[11];
	// correcting
	dang2dquat(dquat,phi_est);
	quatmult(Qnp_out,dquat,Qnp_pre);
	quat2dmat(Cnp_out,Qnp_out);
	dmat2atti(atti_out,Cnp_out);
	
	vel_out[0] = vel_pre[0]-X_cur[3];
	vel_out[1] = vel_pre[1]-X_cur[4];
	posn_out[0] = posn_pre[0]-X_cur[5];
	posn_out[1] = posn_pre[1]-X_cur[6];
}

void BDAdjust(FLOAT64_T Qnp_out[], FLOAT64_T Cnp_out[], FLOAT64_T atti_out[], FLOAT64_T vel_out[],
			  FLOAT64_T phi_est[], FLOAT64_T gyro_bias_est[], FLOAT64_T acc_bias_est[],
			  const FLOAT64_T X_cur[], const FLOAT64_T Qnp_pre[], const FLOAT64_T vel_pre[])
{
	FLOAT64_T dquat[4];
	// estimation
	phi_est[0] = X_cur[0]; // +acc_bias_est[1]/g;
	phi_est[1] = X_cur[1]; // -acc_bias_est[0]/g;
	phi_est[2] = X_cur[2]; // -gyro_bias_est[0]/OMEGA_IE/cosL
	gyro_bias_est[0] = X_cur[6];
	gyro_bias_est[1] = X_cur[7];
	gyro_bias_est[2] = X_cur[8];
	acc_bias_est[0] = X_cur[9];
	acc_bias_est[1] = X_cur[10];
	acc_bias_est[2] = X_cur[11];
	// correcting
	dang2dquat(dquat,phi_est);
	quatmult(Qnp_out,dquat,Qnp_pre);
	quat2dmat(Cnp_out,Qnp_out);
	dmat2atti(atti_out,Cnp_out);
	
	vel_out[0] = vel_pre[0]-X_cur[3];
	vel_out[1] = vel_pre[1]-X_cur[4];
	vel_out[2] = vel_pre[2]-X_cur[5];

}

/* 卡尔曼滤波器重置 */
void KFReset(void)
{
	INT32_T i;
	memset(g_stKfPara.dXCalc, 0, 12*sizeof(FLOAT64_T)); // 状态变量置零
	memset(g_stKfPara.dXkk1, 0, 12*sizeof(FLOAT64_T));   //状态中间变量置零
	memset(g_stKfPara.dPCalc, 0, 144*sizeof(FLOAT64_T));  //误差方差阵清零
	for (i=0; i<12; i++) g_stKfPara.dPCalc[i*12+i] = g_stKfPara.dP0Diag[i]; // 误差方差阵赋初始值
	memcpy(g_stKfPara.dPkk1, g_stKfPara.dPCalc, 144*sizeof(FLOAT64_T)); //误差方差阵中间变量赋值
}
void attiFeedBackBD(void)
{
	INT32_T i;  //声明计数变量
	for (i=0; i<3; i++)
	{
		g_stBDPara.dXCalc[i] = 0.0; // 姿态误差置零
		g_stBDPara.dXkk1[i] = 0.0; //姿态误差中间变零
	}
	memcpy(g_stNavData.dQnpCh1, g_stNavData.dQnpOut, 4*sizeof(FLOAT64_T)); // 输出反馈校系统姿态
	memcpy(g_stNavData.dAttiCh1, g_stNavData.dAttiOut, 3*sizeof(FLOAT64_T));   //反馈校正系统姿态输出
	memcpy(g_stNavData.dCnpCh1, g_stNavData.dCnpOut, 9*sizeof(FLOAT64_T)); //反馈校正姿态矩阵
	
	g_stNavData.uiComValid |= U32_ADJ_RAP;  //校水平标志位置1
	g_stNavData.uiComValid |= U32_ADJ_HEADING; //校航向标志位置1
}

/* 速度反馈 */
void velFeedBackBD(void)
{
	INT32_T i;
	for (i=3; i<6; i++)
	{
		g_stBDPara.dXCalc[i] = 0.0; // 速度误差置零
		g_stBDPara.dXkk1[i] = 0.0;  //速度误差中间变量置零
	}
	memcpy(g_stNavData.dVelCh1, g_stNavData.dVelOut, 2*sizeof(FLOAT64_T)); // 输出反馈校系统速度
	g_stNavData.uiComValid |= U32_ADJ_VEL;  //校正速度标志位置1
}

/* 姿态反馈 */
void attiFeedBack(void)
{
	INT32_T i;  //声明计数变量
	for (i=0; i<3; i++)
	{
		g_stKfPara.dXCalc[i] = 0.0; // 姿态误差置零
		g_stKfPara.dXkk1[i] = 0.0; //姿态误差中间变零
	}
	memcpy(g_stNavData.dQnpCh1, g_stNavData.dQnpOut, 4*sizeof(FLOAT64_T)); // 输出反馈校系统姿态
	memcpy(g_stNavData.dAttiCh1, g_stNavData.dAttiOut, 3*sizeof(FLOAT64_T));   //反馈校正系统姿态输出
	memcpy(g_stNavData.dCnpCh1, g_stNavData.dCnpOut, 9*sizeof(FLOAT64_T)); //反馈校正姿态矩阵
	
	g_stNavData.uiComValid |= U32_ADJ_RAP;  //校水平标志位置1
	g_stNavData.uiComValid |= U32_ADJ_HEADING; //校航向标志位置1
}

/* 速度反馈 */
void velFeedBack(void)
{
	INT32_T i;
	for (i=3; i<5; i++)
	{
		g_stKfPara.dXCalc[i] = 0.0; // 速度误差置零
		g_stKfPara.dXkk1[i] = 0.0;  //速度误差中间变量置零
	}
	memcpy(g_stNavData.dVelCh1, g_stNavData.dVelOut, 2*sizeof(FLOAT64_T)); // 输出反馈校系统速度
	g_stNavData.uiComValid |= U32_ADJ_VEL;  //校正速度标志位置1
}


/* 位置反馈 */
void posnFeedBack(void)
{
	INT32_T i;
	for (i=5; i<7; i++)
	{
		g_stKfPara.dXCalc[i] = 0.0;  //位置误差置零
		g_stKfPara.dXkk1[i] = 0.0;  //位置误差中间变量置零
	}
	memcpy(g_stNavData.dPosnCh1, g_stNavData.dPosnOut, 2*sizeof(FLOAT64_T));  //反馈校正系统位置
	g_stNavData.uiComValid |= U32_ADJ_POSN;  //校系统位置标志位置1
}

/************************************************************************\
功能:载体运动状态的判断(每1秒判断一次)
输入:

输出:

返回:无

日期:2015.11.11
作者:Fairylan
日志:
\************************************************************************/
void checkMovingSta(void)
{
	INT16_T i;
	g_stNavData.ucForceChange = 0x00;
	g_stNavData.ucVelChange[0] = 0x00;
	g_stNavData.ucVelChange[1] = 0x00;
	g_stNavData.ucAttiChange[0] = 0x00;
	g_stNavData.ucAttiChange[1] = 0x00;
	g_stNavData.ucAttiChange[2] = 0x00;
	g_stNavData.ucHeadChange = 0x00;
	
	g_stNavData.dForceSeries1s[0] = sqrt(g_stNavData.dForce1s[0]*g_stNavData.dForce1s[0]+g_stNavData.dForce1s[1]*g_stNavData.dForce1s[1]+g_stNavData.dForce1s[2]*g_stNavData.dForce1s[2]);
	g_stNavData.dForce1s[0] = g_stNavData.dForce1s[1] = g_stNavData.dForce1s[2] = 0.0;
	g_stNavData.dVelSeries1s[0][0] = g_stNavData.dVelOut[0]; // 东向
	g_stNavData.dVelSeries1s[0][1] = g_stNavData.dVelOut[1]; // 北向
	g_stNavData.dAttiSeries1s[0][0] = g_stNavData.dAttiOut[0]; // 横摇角
	g_stNavData.dAttiSeries1s[0][1] = g_stNavData.dAttiOut[1]; // 纵摇角
	g_stNavData.dAttiSeries1s[0][2] = g_stNavData.dAttiOut[2]; // 航向角
	for (i=LEN_CHECK_MOV-1; i>0; i--)
	{
		g_stNavData.dForceSeries1s[i] = g_stNavData.dForceSeries1s[i-1]; // 前平移
		g_stNavData.dVelSeries1s[i][0] = g_stNavData.dVelSeries1s[i-1][0];
		g_stNavData.dVelSeries1s[i][1] = g_stNavData.dVelSeries1s[i-1][1];
		g_stNavData.dAttiSeries1s[i][0] = g_stNavData.dAttiSeries1s[i-1][0];
		g_stNavData.dAttiSeries1s[i][1] = g_stNavData.dAttiSeries1s[i-1][1];
		g_stNavData.dAttiSeries1s[i][2] = g_stNavData.dAttiSeries1s[i-1][2];
		if ((g_stNavData.dAttiOut[2] > (7.0*PI/4.0)) && (g_stNavData.dAttiSeries1s[2][i] < (PI/4.0)))  //顺时针过零处理
			g_stNavData.dAttiSeries1s[2][i] = g_stNavData.dAttiSeries1s[2][i]+2*PI;
		else if ((g_stNavData.dAttiOut[2] < (PI/4.0)) && (g_stNavData.dAttiSeries1s[2][i] > (7.0*PI/4.0)))  //逆时针过零处理
			g_stNavData.dAttiSeries1s[2][i] = g_stNavData.dAttiSeries1s[2][i]-2*PI;

		if (i<LEN_CHECK_HEADLINE)
		{
			if (fabs(g_stNavData.dAttiSeries1s[0][2]-g_stNavData.dAttiSeries1s[i][2]) > HDG_CHANG_HEADLINE)
				g_stNavData.ucHeadChange = 0x01;
		}

		if (fabs(g_stNavData.dForceSeries1s[0]-g_stNavData.dForceSeries1s[i]) > FORCE_CHANGE_STATIC)
			g_stNavData.ucForceChange |= 0x01;
		if (fabs(g_stNavData.dVelSeries1s[0][0]- g_stNavData.dVelSeries1s[i][0]) > VEL_CHANGE_STATIC)
			g_stNavData.ucVelChange[0] |= 0x01;
		if (fabs(g_stNavData.dVelSeries1s[0][1]- g_stNavData.dVelSeries1s[i][1]) > VEL_CHANGE_STATIC)
			g_stNavData.ucVelChange[1] |= 0x01;
		
		if (fabs(g_stNavData.dAttiSeries1s[0][0]-g_stNavData.dAttiSeries1s[i][0]) > ROLL_CHANGE_STATIC)
			g_stNavData.ucAttiChange[0] |= 0x01;
		if (fabs(g_stNavData.dAttiSeries1s[0][1]-g_stNavData.dAttiSeries1s[i][1]) > PITCH_CHANGE_STATIC)
			g_stNavData.ucAttiChange[1] |= 0x01;
		if (fabs(g_stNavData.dAttiSeries1s[0][2]-g_stNavData.dAttiSeries1s[i][2]) > HDG_CHANGE_STATIC)
			g_stNavData.ucAttiChange[2] |= 0x01;

		if (fabs(g_stNavData.dForceSeries1s[0]-g_stNavData.dForceSeries1s[i]) > FORCE_CHANGE_MOOR)
			g_stNavData.ucForceChange |= 0x02;
		if (fabs(g_stNavData.dVelSeries1s[0][0]- g_stNavData.dVelSeries1s[i][0]) > VEL_CHANGE_MOOR)
			g_stNavData.ucVelChange[0] |= 0x02;
		if (fabs(g_stNavData.dVelSeries1s[0][1]- g_stNavData.dVelSeries1s[i][1]) > VEL_CHANGE_MOOR)
			g_stNavData.ucVelChange[1] |= 0x02;
		
		if (fabs(g_stNavData.dAttiSeries1s[0][0]-g_stNavData.dAttiSeries1s[i][0]) > ROLL_CHANGE_MOOR)
			g_stNavData.ucAttiChange[0] |= 0x02;
		if (fabs(g_stNavData.dAttiSeries1s[0][1]-g_stNavData.dAttiSeries1s[i][1]) > PITCH_CHANGE_MOOR)
			g_stNavData.ucAttiChange[1] |= 0x02;
        if (fabs(g_stNavData.dAttiSeries1s[0][2]-g_stNavData.dAttiSeries1s[i][2]) > HDG_CHANGE_MOOR)
			g_stNavData.ucAttiChange[2] |= 0x02;

	}

	g_stNavData.ucMoveChange = (g_stNavData.ucForceChange|g_stNavData.ucVelChange[0]|g_stNavData.ucVelChange[1]|g_stNavData.ucAttiChange[0]|g_stNavData.ucAttiChange[1]|g_stNavData.ucAttiChange[2]);
}

/* 中值滤波，排序剔除最大两个数和最小两个数后取均值 */ 
FLOAT32_T smFilter(FLOAT32_T *x, INT32_T len)
{
	INT32_T i,j;  //计数变量声明
	FLOAT32_T m = x[0], y[30], t; //局部变量声明
	memcpy(y, x, len*sizeof(FLOAT32_T));  //将原始值赋给局部变量

	// 冒泡排序,由小到大
	for (i=0; i<len-1; i++)  //每一个数都进行比较
	{
		for (j=i+1; j<len; j++)    //对确定出的变量进行对比
		{
			if (y[j] < y[i])  //比较出小值
			{
				t = y[j];  //小值赋给中间变量
				y[j] = y[i];  //大值后移
				y[i] = t;  //小值前移
			}
		}
	}

	// 求中值
	m = 0.0;  //求和初始化
	for (i=2; i<len-2; i++)  //求和循环
	{
		m += y[i]; // 剔除两个小数和两个大数求和
	}
	m /= (len-4); // 均值

	return m;  //返回计算值
}

// 初始位置赋值
void SetPosnInit(void)
{
	g_stRefMsg.dPosnRef[0] = ((double)g_stHostMsg.iPosnInit[0])*DEG2RAD/TRANS_LAT; // lat
	g_stRefMsg.dPosnRef[1] = ((double)g_stHostMsg.iPosnInit[1])*DEG2RAD/TRANS_LON; // lon
	g_stNavData.uiComRecv |= U32_RECV_POSN_BIND;
}

// 初始速度赋值
void SetVelInit(void)
{
	g_stRefMsg.dVelRef[2] = ((double)g_stHostMsg.uiVelInit)*KN2MPS/TRANS_VEL/2; // 合速度
	g_stNavData.uiComRecv |= U32_RECV_VEL_BIND;
}

// 初始航向赋值
void SetHeadingInit(void)
{
	FLOAT64_T atti_temp[3];
	atti_temp[0] = atti_temp[1] = 0.0;
	atti_temp[2] = ((double)g_stHostMsg.uiHeadingInit)*DEG2RAD/TRANS_HDG;
	atti2dmat(g_stNavData.dCnbEx, atti_temp);
	g_stNavData.uiComRecv |= U32_RECV_HEADING_BIND;
}

// 姿态零位赋值
void SetSysAttiZero(void)
{
	g_stBindPara.dSysAttiZero[0] = g_stHostMsg.iSysAttiZero[0]*DEG2RAD/TRANS_ATTI/2.0; // roll_zero
	g_stBindPara.dSysAttiZero[1] = g_stHostMsg.iSysAttiZero[1]*DEG2RAD/TRANS_ATTI/2.0; // pitch_zero
	g_stBindPara.dSysAttiZero[2] = g_stHostMsg.iSysAttiZero[2]*DEG2RAD/TRANS_HDG; // heading_zero
	atti2dmat(g_stNavData.dCb1b, g_stBindPara.dSysAttiZero);
	g_stNavData.uiComRecv |= U32_RECV_SYS_ATTI_ZERO;
}

void SetExAttiZero(void)
{
	g_stBindPara.dOutAttiZero[0] = g_stHostMsg.iOutAttiZero[0]*DEG2RAD/TRANS_ATTI/2.0; // roll_zero
	g_stBindPara.dOutAttiZero[1] = g_stHostMsg.iOutAttiZero[1]*DEG2RAD/TRANS_ATTI/2.0; // pitch_zero
	g_stBindPara.dOutAttiZero[2] = g_stHostMsg.iOutAttiZero[2]*DEG2RAD/TRANS_HDG; // heading_zero
	g_stBindPara.dImuAttiZero[0] = g_stHostMsg.iImuAttiZero[0]*DEG2RAD/TRANS_ATTI/2.0; // roll_zero
	g_stBindPara.dImuAttiZero[1] = g_stHostMsg.iImuAttiZero[1]*DEG2RAD/TRANS_ATTI/2.0; // pitch_zero
	g_stBindPara.dImuAttiZero[2] = g_stHostMsg.iImuAttiZero[2]*DEG2RAD/TRANS_HDG; // heading_zero
	atti2dmat(g_stNavData.dCb1b2, g_stBindPara.dOutAttiZero);
	atti2dmat(g_stNavData.dCb3b1, g_stBindPara.dImuAttiZero);
	g_stNavData.uiComRecv |= U32_RECV_EX_ATTI_ZERO;
}

// 陀螺参数赋值
void SetGyroPara(void)
{
	g_stBindPara.dKgBind[0] = g_stHostMsg.iKgBind[0]*1.0e-8*DPH2RPS*1.0e-3; // X陀螺标度
	g_stBindPara.dKgBind[1] = g_stHostMsg.iKgBind[1]*1.0e-8*DPH2RPS*1.0e-3; // Y陀螺标度
	g_stBindPara.dKgBind[2] = g_stHostMsg.iKgBind[2]*1.0e-8*DPH2RPS*1.0e-3; // Z陀螺标度
	g_stBindPara.dBgBind[0] = g_stHostMsg.iBgBind[0]*1.0e-6*DPH2RPS; // X陀螺漂移
	g_stBindPara.dBgBind[1] = g_stHostMsg.iBgBind[1]*1.0e-6*DPH2RPS; // Y陀螺漂移
	g_stBindPara.dBgBind[2] = g_stHostMsg.iBgBind[2]*1.0e-6*DPH2RPS; // Z陀螺漂移
	g_stBindPara.dCgpBind[0] = 1.0; 
	g_stBindPara.dCgpBind[1] = g_stHostMsg.iUgBind[0]*1.0e-4*MIN2RAD; // x对y安装偏角
	g_stBindPara.dCgpBind[2] = g_stHostMsg.iUgBind[1]*1.0e-4*MIN2RAD; // x对z安装偏角
	g_stBindPara.dCgpBind[3] = g_stHostMsg.iUgBind[2]*1.0e-4*MIN2RAD; // y对x安装偏角
	g_stBindPara.dCgpBind[4] = 1.0; 
	g_stBindPara.dCgpBind[5] = g_stHostMsg.iUgBind[3]*1.0e-4*MIN2RAD; // y对z安装偏角
	g_stBindPara.dCgpBind[6] = g_stHostMsg.iUgBind[4]*1.0e-4*MIN2RAD; // z对x安装偏角
	g_stBindPara.dCgpBind[7] = g_stHostMsg.iUgBind[5]*1.0e-4*MIN2RAD; // z对y安装偏角
	g_stBindPara.dCgpBind[8] = 1.0; 
	g_stNavData.uiComRecv |= U32_RECV_GYRO_PARA;
}

// 加速度计参数赋值
void SetAccPara(void)
{
	g_stBindPara.dKaBind[0] = g_stHostMsg.iKaBind[0]*1.0e-6*1.0e-6; // X加速度计标度
	g_stBindPara.dKaBind[1] = g_stHostMsg.iKaBind[1]*1.0e-6*1.0e-6; // Y加速度计标度
	g_stBindPara.dKaBind[2] = g_stHostMsg.iKaBind[2]*1.0e-6*1.0e-6; // Z加速度计标度
	g_stBindPara.dBaBind[0] = g_stHostMsg.iBaBind[0]*1.0e-6; // X加速度计零偏
	g_stBindPara.dBaBind[1] = g_stHostMsg.iBaBind[1]*1.0e-6; // Y加速度计零偏
	g_stBindPara.dBaBind[2] = g_stHostMsg.iBaBind[2]*1.0e-6; // Z加速度计零偏
	g_stBindPara.dCapBind[0] = 1.0;
	g_stBindPara.dCapBind[1] = g_stHostMsg.iUaBind[0]*1.0e-4*MIN2RAD; // x对y安装偏角
	g_stBindPara.dCapBind[2] = g_stHostMsg.iUaBind[1]*1.0e-4*MIN2RAD; // x对z安装偏角
	g_stBindPara.dCapBind[3] = g_stHostMsg.iUaBind[2]*1.0e-4*MIN2RAD; // y对x安装偏角
	g_stBindPara.dCapBind[4] = 1.0;
	g_stBindPara.dCapBind[5] = g_stHostMsg.iUaBind[3]*1.0e-4*MIN2RAD; // y对z安装偏角
	g_stBindPara.dCapBind[6] = g_stHostMsg.iUaBind[4]*1.0e-4*MIN2RAD; // z对x安装偏角
	g_stBindPara.dCapBind[7] = g_stHostMsg.iUaBind[5]*1.0e-4*MIN2RAD; // z对y安装偏角
	g_stBindPara.dCapBind[8] = 1.0;
	g_stNavData.uiComRecv |= U32_RECV_ACC_PARA;
}

void SetBiasEqu(void)
{
	g_stBindPara.dBgEqu[0] = g_stHostMsg.iBgEqu[0]*1.0e-6*DPH2RPS; // 等效E陀螺漂移
	g_stBindPara.dBgEqu[1] = g_stHostMsg.iBgEqu[1]*1.0e-6*DPH2RPS; // 等效N陀螺漂移
	g_stBindPara.dBgEqu[2] = g_stHostMsg.iBgEqu[2]*1.0e-6*DPH2RPS; // 等效U陀螺漂移
	g_stNavData.uiComRecv |= U32_RECV_BIAS_EQU;
}

void SetGyroBiasCoef(void)
{
	memcpy(g_stBindPara.fBetaGyroBias, g_stHostMsg.fBetaGyroBias, 48); // 陀螺零偏温补系数
	g_stNavData.uiComRecv |= U32_RECV_GBIAS_COEF;
}

// 加速度计零位温补系数赋值
void SetAccZeroCoef(void)
{
	memcpy(g_stBindPara.fBetaAccZero, g_stHostMsg.fBetaAccZero, 48); // 加速度计零位温补系数
	g_stNavData.uiComRecv |= U32_RECV_AZERO_COEF;
}

// 杆臂赋值
void SetLeverArm(void)
{
	INT32_T i,j;     //声明计数变量
	for (i=0; i<5; i++)       //给5种杆臂参数赋值循环
	{
		for (j=0; j<3; j++)   //给每种杆臂参数的x,y,z赋值循环
		{
			g_stBindPara.dLeverArm[i][j] = g_stHostMsg.nLeverArm[i][j]*1.0e-2;// 杆臂 m
		}
	}
	g_stNavData.uiComRecv |= U32_RECV_LEVER_ARM;	//收到杆臂参数有效标志位置1
}


// 响应切换命令
void SetSwitchCmd()
{
	switch (g_stHostMsg.ucSwitchSelect)
	{
	case 0x01: // switch work_mode
		if (U8_WORK_READY == g_stHostMsg.ucSwitchCmd)
		{
			TurnReady(); // 切换准备
		}
		else if (U8_WORK_SELFALIGN_COARSE == g_stHostMsg.ucSwitchCmd)
		{
			TurnCoarseAlign(); // 切换自对准/粗对准
		}
		else if (U8_WORK_TRANSALIGN_COARSE == g_stHostMsg.ucSwitchCmd)
		{
			TurnCoarseAlign(); // 切换传递对准/粗对准
		}
		else if (U8_WORK_COMPALIGN_COARSE == g_stHostMsg.ucSwitchCmd)
		{
			TurnCoarseAlign(); // 切换罗经/粗对准
		}
		else if (U8_WORK_SELFNAV_NODAMP == g_stHostMsg.ucSwitchCmd)
		{
			TurnNoDamp(); // 切换无阻尼
		}
		else if (U8_WORK_SELFNAV_LEDAMP == g_stHostMsg.ucSwitchCmd)
		{
			TurnLevelDamp(); // 切换水平阻尼
		}
		else if (U8_WORK_COMPASS == g_stHostMsg.ucSwitchCmd)
		{
			TurnCompass(); // 切换罗经
		}
		else if (U8_WORK_CALIBRATE_COARSE == g_stHostMsg.ucSwitchCmd)
		{
			g_stNavData.uiKCoarseAlign = 3*60*UPDATE_FREQ; // 粗对准时间
			g_stNavData.uiKFineAlign = 720*60*UPDATE_FREQ; // 卡尔曼滤波精对准时间12h
			g_stNavData.uiKAlignReset1 = 12*60*UPDATE_FREQ;
			g_stNavData.uiKAlignReset2 = 27*60*UPDATE_FREQ;
			g_stNavData.uiKAlignReset3 = 57*60*UPDATE_FREQ;
			g_stNavData.uiKAlignReset4 = 177*60*UPDATE_FREQ;
			g_stNavData.uiKAlignReset5 = 297*60*UPDATE_FREQ;
			g_stNavData.uiKAlignReset6 = 0;
			g_stNavData.uiKAlignReset7 = 0;
			g_stNavData.uiKAlignReset8 = 0;
			TurnCoarseAlign();
		}
		else
		{
		}
		break;
	case 0x02: // switch sail_place
		g_stNavData.ucSailPlace = g_stHostMsg.ucSwitchCmd; // 切换工作地点
		break;
	case 0x03: // switch oper_mode
		g_stNavData.ucOperMode = g_stHostMsg.ucSwitchCmd; // 切换操作方式
		break;
	case 0x04: // switch frame_type
		g_stNavData.ucFrameType = g_stHostMsg.ucSwitchCmd; // 切换坐标系
		break;
	default:
		break;
	}
}

// 导航更新数据初始化
void InitNavData(void)
{
	INT16_T i;
	
	// nav
	memset(&g_stNavData, 0, sizeof(NAV_DATA_T));
	g_stNavData.ucWorkMode = U8_WORK_READY;
	g_stNavData.ucSailPlace = U8_SAIL_HARBOR;
	g_stNavData.ucAlignTime = U8_ALIGN_TIME_01H;
	g_stNavData.ucOperMode = U8_OPER_MANUAL;
	//g_stNavData.ucDampLevel = U8_DAMP_LEVEL1;
	g_stNavData.ucMeasMode = (U8_KF_MEAS_POSN|U8_KF_MEAS_VEL);
	g_stNavData.ucRefValid = (U32_VAL_BIND_POSN|U32_VAL_ZERO_VEL);
	g_stNavData.ucFrameType = FRAME_XYZ;
	g_stNavData.uiComRecv = 0;
	g_stNavData.uiComValid = 0;
	g_stNavData.uiErrCode = 0;
	g_stNavData.ucPriorVelCur = U8_PRIOR_VOID;
	g_stNavData.ucPriorVelPre = U8_PRIOR_VOID;

	g_stNavData.uiKSta = 0;
	g_stNavData.uiKPart = 0;
	g_stNavData.uiKMax = UINT32_INF;

	g_stNavData.uiKCoarseAlign = 7*60*UPDATE_FREQ; // 粗对准时间
	g_stNavData.uiKFineAlign = 1157*60*UPDATE_FREQ; // 卡尔曼滤波精对准时间1h
	g_stNavData.uiKAlignReset1 = 0*60*UPDATE_FREQ;
	g_stNavData.uiKAlignReset2 = 0*60*UPDATE_FREQ;
	g_stNavData.uiKAlignReset3 = 0*60*UPDATE_FREQ;
	g_stNavData.uiKAlignReset4 = 0;
	g_stNavData.uiKAlignReset5 = 0;
	g_stNavData.uiKAlignReset6 = 0;
	g_stNavData.uiKAlignReset7 = 0;
	g_stNavData.uiKAlignReset8 = 0;
	
	for (i=0; i<3; i++)
		g_stNavData.dCip0pCalc[3*i+i] = 1.0;
	dmat2quat(g_stNavData.dQip0pCalc, g_stNavData.dCip0pCalc);
	
	// bind data
	memset(&g_stBindPara, 0, sizeof(BIND_PARA_T));  //初始化参数为0
	SetHeadingInit();	// 设置初始航向
	SetPosnInit();		// 设置初始位置
	SetVelInit();		// 置速度
	SetSysAttiZero();	// 设置姿态零位
	SetExAttiZero();	// 设置惯组姿态零位和输出姿态零位
	SetGyroPara();		// 设置陀螺标定参数
	SetAccPara();		// 设置加表标定参数
	SetBiasEqu();		// 设置陀螺等效零偏
	SetGyroBiasCoef();	// 设置陀螺零偏温补系数
	SetAccZeroCoef();	// 设置加表零偏温补系数
	SetLeverArm();		// 设置杆臂参数
	// inert data
	//memset(&g_stInertData, 0, sizeof(INERT_DATA_T));  // 初始化惯性量 为0
}

void InitBDData(void)
{
	INT16_T i;
	g_stBDPara.dP0Diag[0] = pow(1.0*PI/180.0, 2); // 1°
	g_stBDPara.dP0Diag[1] = pow(1.0*PI/180.0, 2); // 1°
	g_stBDPara.dP0Diag[2] = pow(5.0*PI/180.0, 2); // 5°
	g_stBDPara.dP0Diag[3] = pow(2.0, 2); // 2m/s
	g_stBDPara.dP0Diag[4] = pow(2.0, 2); // 2m/s
	g_stBDPara.dP0Diag[5] = pow(2.0, 2); // 2m/s
	g_stBDPara.dP0Diag[6] = pow(0.1*PI/180.0/3600.0, 2); // 0.1deg/hr
	g_stBDPara.dP0Diag[7] = pow(0.1*PI/180.0/3600.0, 2); // 0.1deg/hr
	g_stBDPara.dP0Diag[8] = pow(0.1*PI/180.0/3600.0, 2); // 0.1deg/hr
	g_stBDPara.dP0Diag[9] = pow(0.2*1e-3*GLV_G, 2); // 2mg
	g_stBDPara.dP0Diag[10] = pow(0.2*1e-3*GLV_G, 2); // 2mg
	g_stBDPara.dP0Diag[11] = pow(0.2*1e-3*GLV_G, 2); // 2mg
	g_stBDPara.dP0Diag[12] = pow(1e-4, 2); // 10ppm
	g_stBDPara.dP0Diag[13] = pow(1e-4, 2); // 10ppm
	g_stBDPara.dP0Diag[14] = pow(1e-4, 2); // 10ppm
	g_stBDPara.dP0Diag[15] = pow(2.0*MIN2RAD, 2); // 2'
	g_stBDPara.dP0Diag[16] = pow(2.0*MIN2RAD, 2); // 2'
	g_stBDPara.dP0Diag[17] = pow(2.0*MIN2RAD, 2); // 2'
	g_stBDPara.dP0Diag[18] = pow(1e-4, 2); // 100ppm
	g_stBDPara.dP0Diag[19] = pow(1e-4, 2); // 100ppm
	g_stBDPara.dP0Diag[20] = pow(1e-4, 2); // 100ppm
	g_stBDPara.dP0Diag[21] = pow(2.0*MIN2RAD, 2); // 2'
	g_stBDPara.dP0Diag[22] = pow(2.0*MIN2RAD, 2); // 2'
	g_stBDPara.dP0Diag[23] = pow(2.0*MIN2RAD, 2); // 2'
	g_stBDPara.dP0Diag[24] = pow(2.0*MIN2RAD, 2); // 2'
	g_stBDPara.dP0Diag[25] = pow(2.0*MIN2RAD, 2); // 2'
	g_stBDPara.dP0Diag[26] = pow(2.0*MIN2RAD, 2); // 2'
	memset(g_stBDPara.dPCalc, 0, 27*27*sizeof(FLOAT64_T));
	for (i=0; i<27; i++) 
	{
		g_stBDPara.dPCalc[i*27+i] = g_stBDPara.dP0Diag[i];
	}

	// input noise var
	memset(g_stBDPara.dQ, 0, 21*21*sizeof(FLOAT64_T));
	g_stBDPara.dQ[0*21+0] = pow(0.05*PI/180.0/3600.0, 2); // 0.05deg/h
	g_stBDPara.dQ[1*21+1] = pow(0.05*PI/180.0/3600.0, 2);
	g_stBDPara.dQ[2*21+2] = pow(0.05*PI/180.0/3600.0, 2);
	g_stBDPara.dQ[3*21+3] = pow(0.1e-3*GLV_G, 2); // 0.1mg
	g_stBDPara.dQ[4*21+4] = pow(0.1e-3*GLV_G, 2);
	g_stBDPara.dQ[5*21+5] = pow(0.1e-3*GLV_G, 2);
	g_stBDPara.dQ[6*21+6] = pow(1e-7, 2); // 0.1ppm
	g_stBDPara.dQ[7*21+7] = pow(1e-7, 2); // 0.1ppm
	g_stBDPara.dQ[8*21+8] = pow(1e-7, 2); // 0.1ppm
	g_stBDPara.dQ[9*21+9] = pow(2.0*SEC2RAD, 2); // 2"
	g_stBDPara.dQ[10*21+10] = pow(2.0*SEC2RAD, 2); // 2"
	g_stBDPara.dQ[11*21+11] = pow(2.0*SEC2RAD, 2); // 2"
	g_stBDPara.dQ[12*21+12] = pow(1e-6, 2); // 1ppm
	g_stBDPara.dQ[13*21+13] = pow(1e-6, 2); // 1ppm
	g_stBDPara.dQ[14*21+14] = pow(1e-6, 2); // 1ppm
	g_stBDPara.dQ[15*21+15] = pow(2.0*SEC2RAD, 2); // 2"
	g_stBDPara.dQ[16*21+16] = pow(2.0*SEC2RAD, 2); // 2"
	g_stBDPara.dQ[17*21+17] = pow(2.0*SEC2RAD, 2); // 2"
	g_stBDPara.dQ[18*21+18] = pow(2.0*SEC2RAD, 2); // 2"
	g_stBDPara.dQ[19*21+19] = pow(2.0*SEC2RAD, 2); // 2"
	g_stBDPara.dQ[20*21+20] = pow(2.0*SEC2RAD, 2); // 2"

	// state feedback matrix, vel
	memset(g_stBDPara.dH,0,3*27*sizeof(FLOAT64_T));
	g_stBDPara.dH[0*27+3] = 1.0;
	g_stBDPara.dH[1*27+4] = 1.0;
	g_stBDPara.dH[2*27+5] = 1.0;
	// measure noise var, vel
	memset(g_stBDPara.dR,0,3*3*sizeof(FLOAT64_T));
	g_stBDPara.dR[0*3+0] = pow(0.05, 2); // 0.05m/s
	g_stBDPara.dR[1*3+1] = pow(0.05, 2);
	g_stBDPara.dR[2*3+2] = pow(0.05, 2);

}

// 卡尔曼滤波数据初始化
void InitKFData(void)
{
	INT16_T i;

	g_stKfPara.dP0Diag[0] = pow(1.0*PI/180.0, 2); // 1°
	g_stKfPara.dP0Diag[1] = pow(1.0*PI/180.0, 2); // 1°
	g_stKfPara.dP0Diag[2] = pow(5.0*PI/180.0, 2); // 25°
	g_stKfPara.dP0Diag[3] = pow(5.0, 2); // 0.2m/s
	g_stKfPara.dP0Diag[4] = pow(5.0, 2); // 0.2m/s
	g_stKfPara.dP0Diag[5] = pow(6.0*PI/180.0/60.0/60.0, 2); // 0.1nmile
	g_stKfPara.dP0Diag[6] = pow(6.0*PI/180.0/60.0/60.0, 2); // 0.1nmile
	g_stKfPara.dP0Diag[7] = pow(0.1*PI/180.0/3600.0, 2); // 0.1deg/hr
	g_stKfPara.dP0Diag[8] = pow(0.1*PI/180.0/3600.0, 2); // 0.1deg/hr
	g_stKfPara.dP0Diag[9] = pow(0.1*PI/180.0/3600.0, 2); // 0.1deg/hr
	g_stKfPara.dP0Diag[10] = pow(2*1e-3*GLV_G, 2); // 2mg
	g_stKfPara.dP0Diag[11] = pow(2*1e-3*GLV_G, 2); // 2mg
	memset(g_stKfPara.dPCalc, 0, 12*12*sizeof(FLOAT64_T));
	for (i=0; i<12; i++) 
	{
		g_stKfPara.dPCalc[i*12+i] = g_stKfPara.dP0Diag[i];
	}
	memcpy(g_stKfPara.dPkk1, g_stKfPara.dPCalc, 12*12*sizeof(FLOAT64_T));
	memset(g_stKfPara.dXCalc, 0, 12*sizeof(FLOAT64_T));
	memset(g_stKfPara.dXkk1, 0, 12*sizeof(FLOAT64_T));
	memset(g_stKfPara.dGyroBiasEst, 0, 3*sizeof(FLOAT64_T));
	memset(g_stKfPara.dAccBiasEst, 0, 3*sizeof(FLOAT64_T));
	// input noise var
	memset(g_stKfPara.dQ0,0,5*5*sizeof(FLOAT64_T));
	g_stKfPara.dQ0[0*5+0] = pow(0.05*PI/180.0/3600.0, 2); // 0.05deg/h
	g_stKfPara.dQ0[1*5+1] = pow(0.05*PI/180.0/3600.0, 2);
	g_stKfPara.dQ0[2*5+2] = pow(0.05*PI/180.0/3600.0, 2);
	g_stKfPara.dQ0[3*5+3] = pow(0.1e-3*GLV_G, 2); // 0.5mg
	g_stKfPara.dQ0[4*5+4] = pow(0.1e-3*GLV_G, 2);
	
	// state feedback matrix, vel+posn
	memset(g_stKfPara.dH0,0,4*12*sizeof(FLOAT64_T));
	g_stKfPara.dH0[0*12+3] = 1.0;
	g_stKfPara.dH0[1*12+4] = 1.0;
	g_stKfPara.dH0[2*12+5] = 1.0;
	g_stKfPara.dH0[3*12+6] = 1.0;
	// measure noise var, vel+posn
	memset(g_stKfPara.dR0,0,4*4*sizeof(FLOAT64_T));
	g_stKfPara.dR0[0*4+0] = pow(0.1, 2); // 0.1m/s
	g_stKfPara.dR0[1*4+1] = pow(0.1, 2);
	g_stKfPara.dR0[2*4+2] = pow(2*PI/180.0/60.0/60.0, 2); // 2"=61.7m
	g_stKfPara.dR0[3*4+3] = pow(2*PI/180.0/60.0/60.0, 2);
	
	// state feedback matrix, vel
	memset(g_stKfPara.dH1,0,2*12*sizeof(FLOAT64_T));
	g_stKfPara.dH1[0*12+3] = 1.0;
	g_stKfPara.dH1[1*12+4] = 1.0;
	// measure noise var, vel
	memset(g_stKfPara.dR1,0,2*2*sizeof(FLOAT64_T));
	g_stKfPara.dR1[0*2+0] = pow(0.2, 2); // 0.2m/s
	g_stKfPara.dR1[1*2+1] = pow(0.2, 2);

	// state feedback matrix, posn
	memset(g_stKfPara.dH2,0,2*12*sizeof(FLOAT64_T));
	g_stKfPara.dH2[0*12+5] = 1.0;
	g_stKfPara.dH2[1*12+6] = 1.0;
	// measure noise var, posn
	memset(g_stKfPara.dR2,0,2*2*sizeof(FLOAT64_T));
	g_stKfPara.dR2[0*2+0] = pow(6*PI/180.0/60.0/60.0, 2); // 2"=61.7m
	g_stKfPara.dR2[1*2+1] = pow(6*PI/180.0/60.0/60.0, 2);

}


void InertTrans2(void)
{
	int i,j;
	double ba_temp[3],bg_temp[3];
	double temp_acc,temp_gyro;
	double ang_inc_p[3], vel_inc_p[3];
	double ang_inc_r[3], vel_inc_r[3];
	double delta_f_arm[2];

	FLOAT64_T Cpp[9];

	// 400Hz温补、零偏补偿和标度变换
	for (i=0; i<3; i++)
	{
		ba_temp[i] = 0.0;  // 加表零偏温补系数
		bg_temp[i] = 0.0; // 陀螺零偏温补系数
		temp_acc = 1.0;   // 加表温补次项
		temp_gyro = 1.0; // 陀螺温补次项
		for (j=0; j<3; j++)
		{
			ba_temp[i] += g_stBindPara.fBetaAccZero[j][i] * temp_acc;  // 加速度计温补零偏
			temp_acc   *= g_stInertData.fTempAcc[i]-25;     // 加速度计温度次项
			bg_temp[i] += g_stBindPara.fBetaGyroBias[j][i]* temp_gyro; // 陀螺温补零偏
			temp_gyro  *= g_stInertData.fTempGyro[2*i]-25; // 陀螺温度次项
		}
		ba_temp[i] += g_stBindPara.fBetaAccZero[j][i] * temp_acc;
		bg_temp[i] += g_stBindPara.fBetaGyroBias[j][i]* temp_gyro;
		ba_temp[i] *= 1.0;           // p/s
		bg_temp[i] *= DPH2RPS; // deg/h
		ang_inc_r[i] = g_stBindPara.dKgBind[i] * g_stPulseMsg.dPulseGyro[i] - (g_stBindPara.dBgBind[i])*UPDATE_PERIOD;  //无温补角增量计算
		ang_inc_p[i] = g_stBindPara.dKgBind[i] * g_stPulseMsg.dPulseGyro[i] - (g_stBindPara.dBgBind[i]+bg_temp[i])*UPDATE_PERIOD;  //有温补角增量计算
		vel_inc_r[i] = g_stBindPara.dKaBind[i] * (g_stPulseMsg.dPulseAcc[i] - g_stBindPara.dBaBind[i]*UPDATE_PERIOD);  //无温补速度增量计算
		vel_inc_p[i] = g_stBindPara.dKaBind[i] * (g_stPulseMsg.dPulseAcc[i] - (g_stBindPara.dBaBind[i]+ba_temp[i])*UPDATE_PERIOD);  //有温补速度增量计算
		g_stInertData.dAngIncG[i] = ang_inc_r[i];
		g_stInertData.dVelIncA[i] = vel_inc_r[i];
	}

	// 正交补偿
	g_stInertData.dAngIncB[0] = g_stBindPara.dCgpBind[0] * ang_inc_p[0] + g_stBindPara.dCgpBind[1] * ang_inc_p[1] + g_stBindPara.dCgpBind[2] * ang_inc_p[2];
	g_stInertData.dAngIncB[1] = g_stBindPara.dCgpBind[3] * ang_inc_p[0] + g_stBindPara.dCgpBind[4] * ang_inc_p[1] + g_stBindPara.dCgpBind[5] * ang_inc_p[2];
	g_stInertData.dAngIncB[2] = g_stBindPara.dCgpBind[6] * ang_inc_p[0] + g_stBindPara.dCgpBind[7] * ang_inc_p[1] + g_stBindPara.dCgpBind[8] * ang_inc_p[2];
	g_stInertData.dVelIncB[0] = g_stBindPara.dCapBind[0] * vel_inc_p[0] + g_stBindPara.dCapBind[1] * vel_inc_p[1] + g_stBindPara.dCapBind[2] * vel_inc_p[2];
	g_stInertData.dVelIncB[1] = g_stBindPara.dCapBind[3] * vel_inc_p[0] + g_stBindPara.dCapBind[4] * vel_inc_p[1] + g_stBindPara.dCapBind[5] * vel_inc_p[2];
	g_stInertData.dVelIncB[2] = g_stBindPara.dCapBind[6] * vel_inc_p[0] + g_stBindPara.dCapBind[7] * vel_inc_p[1] + g_stBindPara.dCapBind[8] * vel_inc_p[2];
	for (i=0; i<3; i++)
	{
		g_stInertData.dOmegaIBB[i] = g_stInertData.dAngIncB[i] * UPDATE_FREQ;
	}
	// 角加速度计算
	g_stInertData.dDeltaOmegaIBB[0] = g_stInertData.dDeltaOmegaIBB[1] = g_stInertData.dDeltaOmegaIBB[2] = 0.0;
	for (i=0; i<MAX_RATE_WIDTH2-1; i++)
	{
		g_stInertData.dOmegaIBBSeries[i][0] = g_stInertData.dOmegaIBBSeries[i+1][0];
		g_stInertData.dOmegaIBBSeries[i][1] = g_stInertData.dOmegaIBBSeries[i+1][1];
		g_stInertData.dOmegaIBBSeries[i][2] = g_stInertData.dOmegaIBBSeries[i+1][2];
		g_stInertData.dDeltaOmegaIBB[0] += g_stInertData.dOmegaIBBSeries[i][0]*g_dCoefRate2[i];
		g_stInertData.dDeltaOmegaIBB[1] += g_stInertData.dOmegaIBBSeries[i][1]*g_dCoefRate2[i];
		g_stInertData.dDeltaOmegaIBB[2] += g_stInertData.dOmegaIBBSeries[i][2]*g_dCoefRate2[i];
	}
	g_stInertData.dOmegaIBBSeries[i][0] = g_stInertData.dOmegaIBB[0];
	g_stInertData.dOmegaIBBSeries[i][1] = g_stInertData.dOmegaIBB[1];
	g_stInertData.dOmegaIBBSeries[i][2] = g_stInertData.dOmegaIBB[2];
	g_stInertData.dDeltaOmegaIBB[0] += g_stInertData.dOmegaIBBSeries[i][0]*g_dCoefRate2[i];
	g_stInertData.dDeltaOmegaIBB[1] += g_stInertData.dOmegaIBBSeries[i][1]*g_dCoefRate2[i];
	g_stInertData.dDeltaOmegaIBB[2] += g_stInertData.dOmegaIBBSeries[i][2]*g_dCoefRate2[i];
	// 杆臂补偿
	delta_f_arm[0] = (-g_stInertData.dOmegaIBB[1]*g_stInertData.dOmegaIBB[1]-g_stInertData.dOmegaIBB[2]*g_stInertData.dOmegaIBB[2]) * g_stBindPara.dLeverArm[0][0]*0.01
                                    + (g_stInertData.dOmegaIBB[0]*g_stInertData.dOmegaIBB[1] - g_stInertData.dDeltaOmegaIBB[2]) * g_stBindPara.dLeverArm[0][1]*0.01
                                    + (g_stInertData.dOmegaIBB[0]*g_stInertData.dOmegaIBB[2] + g_stInertData.dDeltaOmegaIBB[1]) * g_stBindPara.dLeverArm[0][2]*0.01;
	delta_f_arm[1] = (-g_stInertData.dOmegaIBB[0]*g_stInertData.dOmegaIBB[0]-g_stInertData.dOmegaIBB[2]*g_stInertData.dOmegaIBB[2]) * g_stBindPara.dLeverArm[1][1]*0.01
                                    + (g_stInertData.dOmegaIBB[0]*g_stInertData.dOmegaIBB[1] + g_stInertData.dDeltaOmegaIBB[2]) * g_stBindPara.dLeverArm[1][0]*0.01
                                    + (g_stInertData.dOmegaIBB[1]*g_stInertData.dOmegaIBB[2] - g_stInertData.dDeltaOmegaIBB[0]) * g_stBindPara.dLeverArm[1][2]*0.01;
	g_stInertData.dVelIncB[0] = g_stInertData.dVelIncB[0]-delta_f_arm[0]*UPDATE_PERIOD;
	g_stInertData.dVelIncB[1] = g_stInertData.dVelIncB[1]-delta_f_arm[1]*UPDATE_PERIOD;
/*	
	if (g_stNavData.ucFrameType == FRAME_YXFZ)
	{
		memcpy(ang_inc_p, g_stInertData.dAngIncB, 3*sizeof(double));
		memcpy(vel_inc_p, g_stInertData.dVelIncB, 3*sizeof(double));
		g_stInertData.dAngIncB[0] = ang_inc_p[1]; g_stInertData.dAngIncB[1] = ang_inc_p[0]; g_stInertData.dAngIncB[2] = -ang_inc_p[2];
		g_stInertData.dVelIncB[0] = vel_inc_p[1]; g_stInertData.dVelIncB[1] = vel_inc_p[0]; g_stInertData.dVelIncB[2] = -vel_inc_p[2];
	}
	else if (g_stNavData.ucFrameType == FRAME_ZXY)
	{
		memcpy(ang_inc_p, g_stInertData.dAngIncB, 3*sizeof(double));
		memcpy(vel_inc_p, g_stInertData.dVelIncB, 3*sizeof(double));
		g_stInertData.dAngIncB[0] = ang_inc_p[2]; g_stInertData.dAngIncB[1] = ang_inc_p[0]; g_stInertData.dAngIncB[2] = ang_inc_p[1];
		g_stInertData.dVelIncB[0] = vel_inc_p[2]; g_stInertData.dVelIncB[1] = vel_inc_p[0]; g_stInertData.dVelIncB[2] = vel_inc_p[1];
	}
	else if (g_stNavData.ucFrameType == FRAME_YZX)
	{
		memcpy(ang_inc_p, g_stInertData.dAngIncB, 3*sizeof(double));
		memcpy(vel_inc_p, g_stInertData.dVelIncB, 3*sizeof(double));
		g_stInertData.dAngIncB[0] = ang_inc_p[1]; g_stInertData.dAngIncB[1] = ang_inc_p[2]; g_stInertData.dAngIncB[2] = ang_inc_p[0];
		g_stInertData.dVelIncB[0] = vel_inc_p[1]; g_stInertData.dVelIncB[1] = vel_inc_p[2]; g_stInertData.dVelIncB[2] = vel_inc_p[0];
	}
	else
	{
	}

	// 累加
	for (i=0; i<3; i++)
	{
		// 坐标系
		memcpy(ang_inc_p, g_stInertData.dAngIncP, 3*sizeof(FLOAT64_T));
		memcpy(ang_inc_r, g_stInertData.dAngIncG, 3*sizeof(FLOAT64_T));
		memcpy(vel_inc_p, g_stInertData.dVelIncP, 3*sizeof(FLOAT64_T));
		memcpy(vel_inc_r, g_stInertData.dVelIncA, 3*sizeof(FLOAT64_T));
	}
	*/

//////////////////////////////////////////////////////////
	for (i=0; i<3; i++)
	{
		g_stInertData.dAngIncP[i] = g_stInertData.dAngIncB[i];
		g_stInertData.dVelIncP[i] = g_stInertData.dVelIncB[i];
	}
	// 坐标系
	memcpy(ang_inc_p, g_stInertData.dAngIncP, 3*sizeof(FLOAT64_T));
	memcpy(ang_inc_r, g_stInertData.dAngIncG, 3*sizeof(FLOAT64_T));
	memcpy(vel_inc_p, g_stInertData.dVelIncP, 3*sizeof(FLOAT64_T));
	memcpy(vel_inc_r, g_stInertData.dVelIncA, 3*sizeof(FLOAT64_T));

	if (g_stNavData.ucFrameType == FRAME_ZXY)
	{
		Cpp[0] = 0.0; Cpp[1] = 0.0; Cpp[2] = 1.0;
		Cpp[3] = 1.0; Cpp[4] = 0.0; Cpp[5] = 0.0;
		Cpp[6] = 0.0; Cpp[7] = 1.0; Cpp[8] = 0.0;
	}
	else if(g_stNavData.ucFrameType == FRAME_YZX)
	{
		
		Cpp[0] = 0.0; Cpp[1] = 1.0; Cpp[2] = 0.0;
		Cpp[3] = 0.0; Cpp[4] = 0.0; Cpp[5] = 1.0;
		Cpp[6] = 1.0; Cpp[7] = 0.0; Cpp[8] = 0.0;
	}
	else
	{
		Cpp[0] = 1.0; Cpp[1] = 0.0; Cpp[2] = 0.0;
		Cpp[3] = 0.0; Cpp[4] = 1.0; Cpp[5] = 0.0;
		Cpp[6] = 0.0; Cpp[7] = 0.0; Cpp[8] = 1.0;
	}
	matmult(g_stInertData.dAngIncP, Cpp, ang_inc_p, 3, 3, 1);
	matmult(g_stInertData.dVelIncP, Cpp, vel_inc_p, 3, 3, 1);
	matmult(g_stInertData.dAngIncG, Cpp, ang_inc_r, 3, 3, 1);
	matmult(g_stInertData.dVelIncA, Cpp, vel_inc_r, 3, 3, 1);
	
	for (i=0; i<3; i++)
	{
		g_stImuInfo.fAngIncP[i] += (FLOAT32_T)g_stInertData.dAngIncP[i];  //补偿后角增量1s和
		g_stImuInfo.fVelIncP[i] += (FLOAT32_T)g_stInertData.dVelIncP[i];   //补偿后速度增量1s和
		g_stImuInfo.fAngIncR[i] += (FLOAT32_T)g_stInertData.dAngIncG[i];  //未补偿角增量1s和
		g_stImuInfo.fVelIncR[i] += (FLOAT32_T)g_stInertData.dVelIncA[i];   //未补偿速度增量1s和
		g_stNavData.dForce1s[i] += g_stInertData.dVelIncP[i];
		g_stInertData.dOmega[i] = g_stInertData.dAngIncP[i]*UPDATE_FREQ;
		g_stInertData.dForce[i] = g_stInertData.dVelIncP[i]*UPDATE_FREQ;
	}
	
	memcpy(g_stNavData.dCpb, g_stNavData.dCb1b, 9*sizeof(FLOAT64_T));
}

// 标度变换与补偿
void InertTrans(void)
{
	INT32_T i;
	FLOAT64_T ang_inc_p[3], vel_inc_p[3];
	FLOAT64_T ang_inc_r[3], vel_inc_r[3];
	FLOAT64_T Cpp[9];

	for (i=0; i<3; i++)
	{
		ang_inc_p[i] = g_stBindPara.dKgBind[i]*g_stPulseMsg.dPulseGyro[i]-(g_stBindPara.dBgBind[i])*UPDATE_PERIOD;  //有温补角增量计算
		vel_inc_p[i] = g_stBindPara.dKaBind[i]*(g_stPulseMsg.dPulseAcc[i]-(g_stBindPara.dBaBind[i])*UPDATE_PERIOD);  //有温补速度增量计算
		ang_inc_r[i] = (g_stBindPara.dKgBind[i]*g_stPulseMsg.dPulseGyro[i]-g_stBindPara.dBgBind[i])*UPDATE_PERIOD;  //无温补角增量计算
		vel_inc_r[i] = g_stBindPara.dKaBind[i]*(g_stPulseMsg.dPulseAcc[i]-g_stBindPara.dBaBind[i]*UPDATE_PERIOD);  //无温补速度增量计算
	}
	matmult(g_stInertData.dAngIncP, g_stBindPara.dCgpBind, ang_inc_p, 3, 3, 1);  //陀螺安装误差补偿
	matmult(g_stInertData.dVelIncP, g_stBindPara.dCapBind, vel_inc_p, 3, 3, 1);   //加表安装误差补偿
	matmult(g_stInertData.dAngIncG, g_stBindPara.dCgpBind, ang_inc_r, 3, 3, 1);  //陀螺安装误差补偿
	matmult(g_stInertData.dVelIncA, g_stBindPara.dCapBind, vel_inc_r, 3, 3, 1);   //加表安装误差补偿

	// 坐标系
	memcpy(ang_inc_p, g_stInertData.dAngIncP, 3*sizeof(FLOAT64_T));
	memcpy(ang_inc_r, g_stInertData.dAngIncG, 3*sizeof(FLOAT64_T));
	memcpy(vel_inc_p, g_stInertData.dVelIncP, 3*sizeof(FLOAT64_T));
	memcpy(vel_inc_r, g_stInertData.dVelIncA, 3*sizeof(FLOAT64_T));

	if (g_stNavData.ucFrameType == FRAME_ZXY)
	{
		Cpp[0] = 0.0; Cpp[1] = 0.0; Cpp[2] = 1.0;
		Cpp[3] = 1.0; Cpp[4] = 0.0; Cpp[5] = 0.0;
		Cpp[6] = 0.0; Cpp[7] = 1.0; Cpp[8] = 0.0;
	}
	else if(g_stNavData.ucFrameType == FRAME_YZX)
	{

		Cpp[0] = 0.0; Cpp[1] = 1.0; Cpp[2] = 0.0;
		Cpp[3] = 0.0; Cpp[4] = 0.0; Cpp[5] = 1.0;
		Cpp[6] = 1.0; Cpp[7] = 0.0; Cpp[8] = 0.0;
	}
	else
	{
		Cpp[0] = 1.0; Cpp[1] = 0.0; Cpp[2] = 0.0;
		Cpp[3] = 0.0; Cpp[4] = 1.0; Cpp[5] = 0.0;
		Cpp[6] = 0.0; Cpp[7] = 0.0; Cpp[8] = 1.0;
	}

	matmult(g_stInertData.dAngIncP, Cpp, ang_inc_p, 3, 3, 1);
	matmult(g_stInertData.dVelIncP, Cpp, vel_inc_p, 3, 3, 1);
	matmult(g_stInertData.dAngIncG, Cpp, ang_inc_r, 3, 3, 1);
	matmult(g_stInertData.dVelIncA, Cpp, vel_inc_r, 3, 3, 1);

	// 1000Hz标度变换
	/*
	for (j=0; j<5; j++)
	{
		for (i=0; i<3; i++)
		{
			ang_inc_p[i] = (g_stBindPara.dKgBind[i]*g_stPulseMsg.nPulseGyro[i][j]-(g_stBindPara.dBgBind[i]+bg_temp[i]))*UPDATE_PERIOD*0.2;  //有温补角增量计算
			vel_inc_p[i] = g_stBindPara.dKaBind[i]*(g_stPulseMsg.nPulseAcc[i][j]-(g_stBindPara.dBaBind[i]+ba_temp[i])*UPDATE_PERIOD*0.2);  //有温补速度增量计算
		}
		matmult(ang_inc_r, g_stBindPara.dCgpBind, ang_inc_p, 3, 3, 1);  //陀螺安装误差补偿
		matmult(vel_inc_r, g_stBindPara.dCapBind, vel_inc_p, 3, 3, 1);   //加表安装误差补偿
		matmult(g_stInertData.dAngIncMS[j], Cpp, ang_inc_r, 3, 3, 1);
		matmult(g_stInertData.dVelIncMS[j], Cpp, vel_inc_r, 3, 3, 1);
	}
*/
	// 1s和
	for (i=0; i<3; i++)
	{
		g_stImuInfo.fAngIncP[i] += (FLOAT32_T)g_stInertData.dAngIncP[i];  //补偿后角增量1s和
		g_stImuInfo.fVelIncP[i] += (FLOAT32_T)g_stInertData.dVelIncP[i];   //补偿后速度增量1s和
		g_stImuInfo.fAngIncR[i] += (FLOAT32_T)g_stInertData.dAngIncG[i];  //未补偿角增量1s和
		g_stImuInfo.fVelIncR[i] += (FLOAT32_T)g_stInertData.dVelIncA[i];   //未补偿速度增量1s和
		g_stNavData.dForce1s[i] += g_stInertData.dVelIncP[i];
		g_stInertData.dOmega[i] = g_stInertData.dAngIncP[i]*UPDATE_FREQ;
		g_stInertData.dForce[i] = g_stInertData.dVelIncP[i]*UPDATE_FREQ;
	}
	
	memcpy(g_stNavData.dCpb, g_stNavData.dCb1b, 9*sizeof(FLOAT64_T));
}


// 处理外参考信息、装订信息、命令的流程
void InputProc(void)
{
	//// 上位机命令的处理
	if (g_stHostMsg.uiRecvSta&U32_RECV_POSN_BIND)  // 收到位置装订命令
	{
		SetPosnInit();  // 提取初始位置
		g_stHostMsg.uiRecvSta &= (~U32_RECV_POSN_BIND);
	}
	if (g_stHostMsg.uiRecvSta&U32_RECV_VEL_BIND) // 收到速度装订命令
	{
		SetVelInit(); // 提取初始速度
		g_stHostMsg.uiRecvSta &= (~U32_RECV_VEL_BIND);
	}
	if (g_stHostMsg.uiRecvSta&U32_RECV_HEADING_BIND) // 收到航向装订命令
	{
		SetHeadingInit(); // 提取初始航向
		g_stHostMsg.uiRecvSta &= (~U32_RECV_HEADING_BIND);
	}
	if (g_stHostMsg.uiRecvSta&U32_RECV_SYS_ATTI_ZERO) // 收到系统姿态零位
	{
		SetSysAttiZero(); // 提取系统姿态零位
		g_stHostMsg.uiRecvSta &= (~U32_RECV_SYS_ATTI_ZERO);
	}
	if (g_stHostMsg.uiRecvSta&U32_RECV_EX_ATTI_ZERO) // 收到惯组输出姿态零位
	{
		SetExAttiZero(); // 提取惯组姿态零位和输出姿态零位
		g_stHostMsg.uiRecvSta &= (~U32_RECV_EX_ATTI_ZERO);
	}
	if (g_stHostMsg.uiRecvSta&U32_RECV_GYRO_PARA) // 收到陀螺标定参数
	{
		SetGyroPara(); // 提取陀螺标定参数
		g_stHostMsg.uiRecvSta &= (~U32_RECV_GYRO_PARA);
	}
	if (g_stHostMsg.uiRecvSta&U32_RECV_ACC_PARA) // 收到加速度计标定参数
	{
		SetAccPara(); // 提取加速度计标定参数
		g_stHostMsg.uiRecvSta &= (~U32_RECV_ACC_PARA);
	}
	if (g_stHostMsg.uiRecvSta&U32_RECV_BIAS_EQU) // 收到陀螺等效零偏
	{
		SetBiasEqu(); // 提取陀螺等效零偏
		g_stHostMsg.uiRecvSta &= (~U32_RECV_BIAS_EQU);
	}
	if (g_stHostMsg.uiRecvSta&U32_RECV_GBIAS_COEF) // 收到陀螺零偏温补系数
	{
		SetGyroBiasCoef(); // 提取陀螺零偏温补系数
		g_stHostMsg.uiRecvSta &= (~U32_RECV_GBIAS_COEF);
	}
	if (g_stHostMsg.uiRecvSta&U32_RECV_AZERO_COEF) // 收到加速度计零位温补系数
	{
		SetAccZeroCoef(); // 提取加速度计零偏温补系数
		g_stHostMsg.uiRecvSta &= (~U32_RECV_AZERO_COEF);
	}
	if (g_stHostMsg.uiRecvSta&U32_RECV_LEVER_ARM) // 收到杆臂
	{
		SetLeverArm();  // 提取杆臂参数
		g_stHostMsg.uiRecvSta &= (~U32_RECV_LEVER_ARM);
	}

	// 切换命令
	if (g_stHostMsg.uiRecvSta&U32_RECV_SWITCH_CMD)
	{
		SetSwitchCmd();
		g_stHostMsg.uiRecvSta &= (~U32_RECV_SWITCH_CMD);
	}

	//// 运动判断/1s
	if (0 == (g_uiCntClock%UPDATE_FREQ))
	{
		checkMovingSta();
		if (0 == g_stNavData.ucHeadChange) // 直航
		{
			g_stNavData.uiKHeadLine++;
			g_stNavData.uiComValid |= U32_HEAD_LINE;
		}
		else
			g_stNavData.uiKHeadLine = 0;
		
		if (0 == g_stNavData.ucMoveChange)  // 静态
		{
			g_stNavData.uiKMoor++;
			g_stNavData.uiKStatic++;
			g_stNavData.uiComValid |= U32_STATIC_STA;
		}
		else if (1 == g_stNavData.ucMoveChange) // 摇摆/锚泊
		{
			g_stNavData.uiKMoor++;
			g_stNavData.uiKStatic = 0;
			g_stNavData.uiComValid |= U32_SHAKE_STA;
		}
		else
		{
			g_stNavData.uiKMoor = 0;
			g_stNavData.uiKStatic = 0;
			// 在导航状态下如果非静态，且处在自动模式则工作地点转到海上
			if (((g_stNavData.ucWorkMode&0x0F) >= U8_WORK_SELFNAV_NODAMP) && (U8_OPER_AUTO == g_stNavData.ucOperMode))
			{
				g_stNavData.ucSailPlace = U8_SAIL_ONSEA;
			}
		}

	}

	// 参考速度的处理
	if (U8_SAIL_HARBOR == g_stNavData.ucSailPlace) // 码头
		g_stNavData.ucRefValid |= (U32_VAL_ZERO_VEL|U32_VAL_BIND_POSN); 
	else // 海上
		g_stNavData.ucRefValid &= ~(U32_VAL_ZERO_VEL|U32_VAL_BIND_POSN);
	
	if (g_stNavData.uiKMoor > LEN_CHECK_MOV*10) // 静态/锚泊***** 10min
	{
		if ((U8_WORK_SELFNAV_LEDAMP == g_stNavData.ucWorkMode) || (U8_WORK_SELFNAV_NODAMP == g_stNavData.ucWorkMode))  //工作在水平阻尼或无阻尼
		{
			g_stRefMsg.dVelRef[2] = 0.0;
			g_stNavData.ucRefValid |= U32_VAL_ZERO_VEL; // 判断运动状态长时间在静态/锚泊使能零速有效
		}
	}

	matmult(g_stNavData.dCnb2, g_stNavData.dCb1b2, g_stNavData.dCnpCh1, 3, 3, 3);
	mattran(g_stNavData.dCb2n, g_stNavData.dCnb2, 3, 3);
	g_stNavData.ucPriorVelPre = g_stNavData.ucPriorVelCur;  //参考速度优先级赋值
	if (g_stNavData.ucRefValid & U32_VAL_GPS_VEL)
	{
		g_stRefMsg.dVelDampRef[0] = (FLOAT64_T)g_stRefMsg.fGpsVel[0]; // 参考东速
		g_stRefMsg.dVelDampRef[1] = (FLOAT64_T)g_stRefMsg.fGpsVel[1]; // 参考北速
		g_stRefMsg.dVelDampRef[2] = (FLOAT64_T)g_stRefMsg.fGpsVel[2]; // 参考合速
		g_stRefMsg.dVelRef[0] = (FLOAT64_T)g_stRefMsg.fGpsVel[0]; // 组合参考东速
		g_stRefMsg.dVelRef[1] = (FLOAT64_T)g_stRefMsg.fGpsVel[1]; // 组合参考北速
		g_stRefMsg.dVelRef[2] = (FLOAT64_T)g_stRefMsg.fGpsVel[2]; // 组合参考合速
		g_stNavData.ucPriorVelCur = U8_PRIOR_GPS_VEL;
	}
	if (g_stNavData.ucRefValid & U32_VAL_LOG_VEL)
	{
		g_stRefMsg.dVelDampRef[0] = g_stNavData.dCb2n[1]*g_stRefMsg.fLogVel[2]; // 参考东速
		g_stRefMsg.dVelDampRef[1] = g_stNavData.dCb2n[4]*g_stRefMsg.fLogVel[2]; // 参考北速
		g_stRefMsg.dVelDampRef[2] = g_stRefMsg.fLogVel[2];
		g_stRefMsg.fLogVel[0] = (FLOAT32_T)(g_stNavData.dCb2n[1]*g_stRefMsg.fLogVel[2]); // 电磁东向
		g_stRefMsg.fLogVel[1] = (FLOAT32_T)(g_stNavData.dCb2n[4]*g_stRefMsg.fLogVel[2]); // 电磁北向
		g_stNavData.ucPriorVelCur = U8_PRIOR_LOG_VEL;
	}
	if (g_stNavData.ucRefValid & U32_VAL_DVL_VW)  //多普勒对水参考速度有效
	{
		g_stRefMsg.fLogVel[0] = (FLOAT32_T)(g_stNavData.dCb2n[1]*g_stRefMsg.fDopVw[1]+g_stNavData.dCb2n[0]*g_stRefMsg.fDopVw[0]);  //多普勒速度分解东向
		g_stRefMsg.fLogVel[1] = (FLOAT32_T)(g_stNavData.dCb2n[4]*g_stRefMsg.fDopVw[1]+g_stNavData.dCb2n[3]*g_stRefMsg.fDopVw[0]);  //多普勒速度分解 北向
		g_stRefMsg.dVelDampRef[0] = g_stNavData.dCb2n[1]*g_stRefMsg.fDopVw[1]+g_stNavData.dCb2n[0]*g_stRefMsg.fDopVw[0];  //阻尼用东向参考速度
		g_stRefMsg.dVelDampRef[1] = g_stNavData.dCb2n[4]*g_stRefMsg.fDopVw[1]+g_stNavData.dCb2n[3]*g_stRefMsg.fDopVw[0];  //阻尼用北向参考速度
		g_stNavData.ucPriorVelCur = U8_PRIOR_DVL_VW; // 多普勒对水速度有效
	}
	if (g_stNavData.ucRefValid & U32_VAL_DVL_VE) // 
	{
		g_stRefMsg.fLogVel[0] = (FLOAT32_T)(g_stNavData.dCb2n[1]*g_stRefMsg.fDopVe[1]+g_stNavData.dCb2n[0]*g_stRefMsg.fDopVe[0]);  //多普勒速度分解 东向
		g_stRefMsg.fLogVel[1] = (FLOAT32_T)(g_stNavData.dCb2n[4]*g_stRefMsg.fDopVe[1]+g_stNavData.dCb2n[3]*g_stRefMsg.fDopVe[0]);  //多普勒速度分解 北向
		g_stRefMsg.dVelDampRef[0] = g_stNavData.dCb2n[1]*g_stRefMsg.fDopVe[1]+g_stNavData.dCb2n[0]*g_stRefMsg.fDopVe[0];  //阻尼用东向参考速度
		g_stRefMsg.dVelDampRef[1] = g_stNavData.dCb2n[4]*g_stRefMsg.fDopVe[1]+g_stNavData.dCb2n[3]*g_stRefMsg.fDopVe[0];  //阻尼用北向参考速度
		g_stNavData.ucPriorVelCur = U8_PRIOR_DVL_VE; // 多普勒对底速度有效
	}
	if (g_stNavData.ucRefValid & U32_VAL_ZERO_VEL)  //零速阻尼有效，零速组合有效
	{
		g_stRefMsg.dVelRef[2] = 0.0;
		g_stRefMsg.dVelDampRef[0] = g_stNavData.dCb2n[1]*g_stRefMsg.dVelRef[2];  //阻尼参考东速
		g_stRefMsg.dVelDampRef[1] = g_stNavData.dCb2n[4]*g_stRefMsg.dVelRef[2];  //阻尼参考北速
		g_stRefMsg.dVelDampRef[2] = g_stRefMsg.dVelRef[2];
		g_stRefMsg.dVelRef[0] = g_stNavData.dCb2n[1]*g_stRefMsg.dVelRef[2];          //组合参考东速
		g_stRefMsg.dVelRef[1] = g_stNavData.dCb2n[4]*g_stRefMsg.dVelRef[2];          //组合参考北速
		g_stRefMsg.dVelRef[2] = g_stRefMsg.dVelRef[2];
		g_stNavData.ucPriorVelCur = U8_PRIOR_ZERO_VEL; // 零速有效
	}
	// 阻尼/无阻尼自动切换
	if (U8_OPER_AUTO == g_stNavData.ucOperMode)
	{
		if (U8_WORK_SELFNAV_NODAMP == g_stNavData.ucWorkMode)
		{
			if (0 == g_stNavData.ucHeadChange) // 直航航向无变化
				g_stNavData.uiKKeepNoDamp++; // 累计无阻尼时间
			else 
				g_stNavData.uiKKeepNoDamp = 0; 
			if (g_stNavData.uiKKeepNoDamp > 10*UPDATE_FREQ) // 累计无阻尼时间超过10s
				TurnLevelDamp();
		}
		else if (U8_WORK_SELFNAV_LEDAMP == g_stNavData.ucWorkMode)
		{
			// 检测外速度
			if (0 == (g_stNavData.ucRefValid&(U32_VAL_ZERO_VEL|U32_VAL_DVL_VE|U32_VAL_DVL_VW|U32_VAL_LOG_VEL|U32_VAL_GPS_VEL)))
			{
				TurnNoDamp();  //转无阻尼
				g_stNavData.uiKKeepNoDamp = 0;
				g_stNavData.ucPriorVelCur = g_stNavData.ucPriorVelPre = 0;
			}
			// 有效速度改变
			if (g_stNavData.ucPriorVelCur != g_stNavData.ucPriorVelPre)  //有效速度改变
			{
				TurnNoDamp(); //转无阻尼
				TurnLevelDamp();  //再次转阻尼，计算参考速度零位
			}
		}
	}
	
	// 参考位置的处理
	g_stKfPara.ucFlagFilter = 0;
	if (U32_VAL_BIND_POSN == (g_stNavData.ucRefValid&U32_VAL_BIND_POSN)) // bind posn
	{
		g_stRefMsg.dPosnRef[0] = (FLOAT64_T)g_stHostMsg.iPosnInit[0]*DEG2RAD/TRANS_LAT;  
		g_stRefMsg.dPosnRef[1] = (FLOAT64_T)g_stHostMsg.iPosnInit[1]*DEG2RAD/TRANS_LON;
		g_stRefMsg.dPosnRef[2] = 0.0;
		g_stKfPara.ucFlagFilter = 1;
		//if ((fabs(g_stNavData.dPosnOut[0]-g_stRefMsg.dPosnRef[0])*RAD2MIN > 0.2) || (fabs(g_stNavData.dPosnOut[1]-g_stRefMsg.dPosnRef[1])*RAD2MIN > 0.2))
		//	g_stKfPara.ucFlagFilter = 0;
	}
	if (U32_VAL_GPS_POSN == (g_stNavData.ucRefValid&U32_VAL_GPS_POSN)) // gps posn
	{
		g_stRefMsg.dPosnRef[0] = g_stRefMsg.fGpsPosn[0];
		g_stRefMsg.dPosnRef[1] = g_stRefMsg.fGpsPosn[1];
		g_stRefMsg.dPosnRef[2] = 0.0;
		g_stHostMsg.iPosnInit[0] = (INT32_T)(g_stRefMsg.fGpsPosn[0]*RAD2DEG*TRANS_LAT); 
		g_stHostMsg.iPosnInit[1] = (INT32_T)(g_stRefMsg.fGpsPosn[1]*RAD2DEG*TRANS_LON);
		g_stKfPara.ucFlagFilter = 1;
		if ((fabs(g_stNavData.dPosnOut[0]-g_stRefMsg.dPosnRef[0])*RAD2MIN > 0.2) || (fabs(g_stNavData.dPosnOut[1]-g_stRefMsg.dPosnRef[1])*RAD2MIN > 0.2))
			g_stKfPara.ucFlagFilter = 0;
	}
}

// 导航解算更新流程
void CalcProc(void)
{
	// 状态时间若达到预设值则转状态
	if (g_stNavData.uiKSta >= g_stNavData.uiKMax)  //当前状态时间超过设定值
	{
		switch(g_stNavData.ucWorkMode)	//当前工作状态
		{
		case U8_WORK_SELFALIGN_COARSE:	//粗对准
			//TurnLevelDamp();
			
			//if (U8_WORK_COMPALIGN_COARSE == g_stHostMsg.ucSwitchCmd)
			//	TurnCoarseAzimuth(); 	 //切为罗经方位对准
			//else
			//	TurnFineAlign();		//切为精对准
			TurnCalibrate();
			break;
		case U8_WORK_SELFALIGN_FINE:	//精对准
			TurnLevelDamp();	// 切为无阻尼
			break;
		case U8_WORK_COMPALIGN_AZIMUTH: // 罗经方位对准
			TurnCompass();	// 切为罗经
			break;
		default:
			break;
		}
	}
	g_stNavData.uiKSta++;		//子状态计时
	g_stNavData.uiKPart++;		//状态计时

	if (U8_WORK_READY == g_stNavData.ucWorkMode)		//工作在准备状态
	{
		memcpy(g_stNavData.dPosnCh1, g_stRefMsg.dPosnRef, 2*sizeof(FLOAT64_T));  //位置用原给定位置
		memcpy(g_stNavData.dPosnOut, g_stRefMsg.dPosnRef, 2*sizeof(FLOAT64_T));

		memcpy(g_stNavData.dVelCh1, g_stRefMsg.dVelRef, 2*sizeof(FLOAT64_T));	//速度用原给定速度
		memcpy(g_stNavData.dVelOut, g_stRefMsg.dVelRef, 2*sizeof(FLOAT64_T));

		memcpy(g_stNavData.dCnpCh1, g_stNavData.dCnbEx, 9*sizeof(FLOAT64_T)); // 装订姿态
		matinv(g_stNavData.dCbp, g_stNavData.dCpb, 3);
		matmult(g_stNavData.dCnpCh1, g_stNavData.dCnbEx, g_stNavData.dCbp, 3, 3, 3);
		dmat2quat(g_stNavData.dQnpCh1, g_stNavData.dCnpCh1);
		dmat2atti(g_stNavData.dAttiImu1, g_stNavData.dCnpCh1);
		dmat2atti(g_stNavData.dAttiCh1, g_stNavData.dCnbEx);
		memcpy(g_stNavData.dAttiOut, g_stNavData.dAttiCh1, 3*sizeof(FLOAT64_T));
		return ;
	}

	// 在粗对准状态下更新惯性系的速度，利用两组惯性系速度矢量叉积求粗姿态
	if (U8_WORK_SELFALIGN_COARSE == g_stNavData.ucWorkMode) //惯性系下粗对准
	{
		velUpdate(g_stNavData.dQip0pCalc, g_stNavData.dCip0pCalc, g_stNavData.dCinCalc, g_stNavData.dVip0Calc, g_stNavData.dUi0Calc,
			g_stNavData.dQip0pCalc, g_stNavData.dVip0Calc, g_stNavData.dUi0Calc, g_stInertData.dAngIncP, g_stInertData.dVelIncP, 
			g_stNavData.dPosnCh1, g_stRefMsg.dPosnRef, g_stRefMsg.dVelRef, g_stNavData.uiKSta, UPDATE_PERIOD);  //惯性系下粗对准陀螺加表数据更新

		if (g_stNavData.uiKSta == (g_stNavData.uiKMax/4)) // Tc/4时刻取第一点
		{
			memcpy(g_stNavData.dVip0T1, g_stNavData.dVip0Calc, 3*sizeof(FLOAT64_T));
			memcpy(g_stNavData.dUi0T1, g_stNavData.dUi0Calc, 3*sizeof(FLOAT64_T));
		}
		if (g_stNavData.uiKSta > (g_stNavData.uiKMax/4))  //大于Tc/4时刻计算粗对准出的姿态角，航向角
		{
			memcpy(g_stNavData.dVip0T2, g_stNavData.dVip0Calc, 3*sizeof(FLOAT64_T));
			memcpy(g_stNavData.dUi0T2, g_stNavData.dUi0Calc, 3*sizeof(FLOAT64_T));
			coarseAlign(g_stNavData.dQnpCh1, g_stNavData.dCnpCh1, g_stNavData.dAttiImu1, g_stNavData.dVip0T1, g_stNavData.dVip0T2, g_stNavData.dUi0T1, g_stNavData.dUi0T2, g_stNavData.dCip0pCalc, g_stNavData.dCinCalc);
			attiTrans(g_stNavData.dAttiCh1, g_stNavData.dCpb, g_stNavData.dCnpCh1);
			memcpy(g_stNavData.dAttiOut, g_stNavData.dAttiCh1, 3*sizeof(FLOAT64_T));
		}
		if (g_stNavData.uiKSta == (g_stNavData.uiKMax/2)) // Tc/2时刻取第二点
		{
			memcpy(g_stNavData.dVip0Tk, g_stNavData.dVip0Calc, 3*sizeof(FLOAT64_T));
			memcpy(g_stNavData.dUi0Tk, g_stNavData.dUi0Calc, 3*sizeof(FLOAT64_T));
		}
		if (g_stNavData.uiKSta == (3*g_stNavData.uiKMax/4)) // 3Tc/4时刻取第三点
		{
			memcpy(g_stNavData.dVip0T1, g_stNavData.dVip0Tk, 3*sizeof(FLOAT64_T));
			memcpy(g_stNavData.dUi0T1, g_stNavData.dUi0Tk, 3*sizeof(FLOAT64_T));
		}
		
		memcpy(g_stNavData.dPosnCh1, g_stRefMsg.dPosnRef, 2*sizeof(FLOAT64_T));  //位置赋值
		memcpy(g_stNavData.dPosnOut, g_stRefMsg.dPosnRef, 2*sizeof(FLOAT64_T));  

		memcpy(g_stNavData.dVelCh1, g_stRefMsg.dVelRef, 2*sizeof(FLOAT64_T));  //速度赋值
		memcpy(g_stNavData.dVelOut, g_stRefMsg.dVelRef, 2*sizeof(FLOAT64_T));
		return ;
	}

	// 通道1 水平阻尼
	g_stNavData.dVelDampRef[0] = g_stRefMsg.dVelDampRef[0];  //阻尼速度东向
	g_stNavData.dVelDampRef[1] = g_stRefMsg.dVelDampRef[1];  //阻尼速度北向
	if ((g_stNavData.ucWorkMode == U8_WORK_SELFNAV_LEDAMP) || (g_stNavData.ucWorkMode == U8_WORK_SELFNAV_ENDAMP))
	{
		g_stNavData.dVelDampRef[0] = g_stNavData.dVelDampZero[0]+g_stRefMsg.dVelDampRef[0]; // 切换到阻尼时速度零位需要使用切换点的速度
		g_stNavData.dVelDampRef[1] = g_stNavData.dVelDampZero[1]+g_stRefMsg.dVelDampRef[1];  //北向
	}

	forceUpdate(g_stNavData.dFnCh1, g_stNavData.dCpnCh1, g_stNavData.dCnpCh1, g_stInertData.dVelIncP, UPDATE_PERIOD);  //比力更新
	naviUpdateK(g_stNavData.dVelCh1, g_stNavData.dPosnCh1, g_stNavData.dOmegaInN, g_stNavData.dVelOut, g_stNavData.dPosnOut, g_stNavData.dFd,
		g_stNavData.dVelCh1, g_stNavData.dPosnCh1, g_stNavData.dPosnOut, g_stNavData.dVelDampRef, g_stRefMsg.dPosnRef, g_stNavData.dFnCh1, UPDATE_PERIOD);  //速度，位置更新
	attiUpdate(g_stNavData.dQnpCh1, g_stNavData.dCnpCh1, g_stNavData.dAttiImu1, g_stNavData.dQnpCh1, g_stInertData.dAngIncP, g_stNavData.dOmegaInN, UPDATE_PERIOD); //姿态，航向更新

	attiTrans(g_stNavData.dAttiCh1, g_stNavData.dCpb, g_stNavData.dCnpCh1);
	attiTrans(g_stNavData.dAttiOut, g_stNavData.dCpb, g_stNavData.dCnpCh1); //提取姿态角
	memcpy(g_stNavData.dQnpOut, g_stNavData.dQnpCh1, 4*sizeof(FLOAT64_T));  //通道1值赋给输出
	memcpy(g_stNavData.dPosnOut, g_stNavData.dPosnCh1, 3*sizeof(FLOAT64_T));

	shiftFilter(g_stNavData.dVDOut, g_stNavData.dSDOut, g_stNavData.dVDOut, g_stNavData.dSDOut, g_stNavData.dAttiCh1, g_stNavData.dFd, UPDATE_PERIOD); // 瞬时线运动滤波
}


void FilterProc(void)
{
	INT32_T iResetCnt;

	//// 量测模式的处理
	if (U8_WORK_INTENAV_DVL == g_stNavData.ucWorkMode)
		g_stNavData.ucMeasMode = U8_KF_MEAS_VEL;
	else if (U8_WORK_INTENAV_GPS == g_stNavData.ucWorkMode)
		g_stNavData.ucMeasMode = U8_KF_MEAS_POSN;
	else if (U8_WORK_SELFALIGN_FINE == g_stNavData.ucWorkMode)
	{
		g_stNavData.ucMeasMode = U8_KF_MEAS_POSN;
		if (U8_SAIL_HARBOR == g_stNavData.ucSailPlace)
			g_stNavData.ucMeasMode |= U8_KF_MEAS_VEL;
	}

	if (U8_WORK_CALIBRATE_SYS == g_stNavData.ucWorkMode)
	{
		if (1 == (g_stNavData.uiKSta%4)) // 2.5ms一步预测
		{
			memcpy(g_stBDPara.dCbnBD, g_stNavData.dCpnCh1, 9*sizeof(FLOAT64_T));  //暂存姿态变化阵
			BDPredictPHI(g_stBDPara.dPHI, g_stBDPara.dQd, g_stNavData.dVelCh1, g_stNavData.dPosnCh1, g_stInertData.dOmega, g_stInertData.dForce, g_stBDPara.dCbnBD, g_stNavData.dFnCh1, UPDATE_PERIOD*4.0);
			//KFPredictPHI(g_stKfPara.dPHI, g_stKfPara.dQd, g_stNavData.dVelCh1, g_stNavData.dPosnCh1, g_stKfPara.dCpnKF, g_stNavData.dFnCh1, UPDATE_PERIOD*4.0); // 离散化，计算状态转移阵
		}
		else if (2 == (g_stNavData.uiKSta%4)) // 5ms一步预测
		{
			BDPredictQd(g_stBDPara.dQd, g_stBDPara.dQd, g_stInertData.dOmega, g_stInertData.dForce, g_stBDPara.dCbnBD, UPDATE_PERIOD*4.0);
			//KFPredictQd(g_stKfPara.dQd, g_stKfPara.dQd, g_stKfPara.dCpnKF, UPDATE_PERIOD*4.0);  //计算Qd
		}
		else if (3 == (g_stNavData.uiKSta%4)) // 7.5ms一步预测
		{
			BDPredictEnd(g_stBDPara.dXkk1, g_stBDPara.dPkk1, g_stBDPara.dXCalc, g_stBDPara.dPCalc, g_stBDPara.dPHI, g_stBDPara.dQd);
			//KFPredictEnd(g_stKfPara.dXkk1, g_stKfPara.dPkk1, g_stKfPara.dXCalc, g_stKfPara.dPCalc, g_stKfPara.dPHI, g_stKfPara.dQd);
			memcpy(g_stBDPara.dXCalc, g_stBDPara.dXkk1, 27*sizeof(FLOAT64_T)); // 将一步预测状态赋给估计值
			memcpy(g_stBDPara.dPCalc, g_stBDPara.dPkk1, 27*27*sizeof(FLOAT64_T)); // 将一步预测方差赋给误差方差
		}
		else if (0 == (g_stNavData.uiKSta%UPDATE_FREQ)) // 1s进行量测
		{
			g_stBDPara.dDeltaVel[0] = g_stNavData.dVelCh1[0]-g_stRefMsg.dVelRef[0]; // 东速误差
			g_stBDPara.dDeltaVel[1] = g_stNavData.dVelCh1[1]-g_stRefMsg.dVelRef[1]; // 北速误差
			g_stBDPara.dDeltaVel[2] = g_stNavData.dVelCh1[2]-g_stRefMsg.dVelRef[2];
			g_stBDPara.ucFlagSingular = BDUpdate(g_stBDPara.dXCalc, g_stBDPara.dPCalc, g_stBDPara.dXkk1, g_stBDPara.dPkk1, g_stBDPara.dDeltaVel);
			if (0 == g_stBDPara.ucFlagSingular)  //求逆出现奇异
				g_stNavData.uiErrCode |= U16_ERR_MAT_INV;  //报解算奇异故障
		}
		BDAdjust(g_stNavData.dQnpOut, g_stNavData.dCnpOut, g_stNavData.dAttiImu1, g_stNavData.dVelOut, g_stKfPara.dPhiEst, g_stKfPara.dGyroBiasEst, g_stKfPara.dAccBiasEst, g_stBDPara.dXCalc, g_stNavData.dQnpCh1, g_stNavData.dVelCh1); //滤波值涑计算
		attiTrans(g_stNavData.dAttiOut, g_stNavData.dCpb, g_stNavData.dCnpOut);  //提取导航系姿态角，航向角

		//if (0 == g_stNavData.uiKSta%(UPDATE_FREQ))
		{
		//	attiFeedBackBD();  //通道1姿态，航向反馈
		//	velFeedBackBD();  //通道1速度反馈
		}
		g_stNavData.dPosnCh1[0] = g_stRefMsg.dPosnRef[0];
		g_stNavData.dPosnCh1[1] = g_stRefMsg.dPosnRef[1];
		g_stNavData.dPosnCh1[2] = 0;
	}
	//// 精对准/位置组合/速度组合使用卡尔曼滤波
	if ((U8_WORK_SELFALIGN_FINE == g_stNavData.ucWorkMode) || (U8_WORK_INTENAV_GPS== g_stNavData.ucWorkMode) || (U8_WORK_INTENAV_DVL == g_stNavData.ucWorkMode) )
	{
		if (1 == (g_stNavData.uiKSta%4)) // 2.5ms一步预测
		{
			memcpy(g_stKfPara.dCpnKF, g_stNavData.dCpnCh1, 9*sizeof(FLOAT64_T));  //暂存姿态变化阵
			KFPredictPHI(g_stKfPara.dPHI, g_stKfPara.dQd, g_stNavData.dVelCh1, g_stNavData.dPosnCh1, g_stKfPara.dCpnKF, g_stNavData.dFnCh1, UPDATE_PERIOD*4.0); // 离散化，计算状态转移阵
		}
		else if (2 == (g_stNavData.uiKSta%4)) // 5ms一步预测
		{
			KFPredictQd(g_stKfPara.dQd, g_stKfPara.dQd, g_stKfPara.dCpnKF, UPDATE_PERIOD*4.0);  //计算Qd
		}
		else if (3 == (g_stNavData.uiKSta%4)) // 7.5ms一步预测
		{
			KFPredictEnd(g_stKfPara.dXkk1, g_stKfPara.dPkk1, g_stKfPara.dXCalc, g_stKfPara.dPCalc, g_stKfPara.dPHI, g_stKfPara.dQd);
			memcpy(g_stKfPara.dXCalc, g_stKfPara.dXkk1, 12*sizeof(FLOAT64_T)); // 将一步预测状态赋给估计值
			memcpy(g_stKfPara.dPCalc, g_stKfPara.dPkk1, 144*sizeof(FLOAT64_T)); // 将一步预测方差赋给误差方差
		}
		else if ((0 == (g_stNavData.uiKSta%UPDATE_FREQ)) && (1 == g_stKfPara.ucFlagFilter)) // 1s进行量测
		{
			g_stKfPara.dDeltaPosn[0] = g_stNavData.dPosnCh1[0]-g_stRefMsg.dPosnRef[0]; // 纬度误差
			g_stKfPara.dDeltaPosn[1] = g_stNavData.dPosnCh1[1]-g_stRefMsg.dPosnRef[1]; // 经度误差
			g_stKfPara.dDeltaPosn[2] = 0.0;
			g_stKfPara.dDeltaVel[0] = g_stNavData.dVelCh1[0]-g_stRefMsg.dVelRef[0]; // 东速误差
			g_stKfPara.dDeltaVel[1] = g_stNavData.dVelCh1[1]-g_stRefMsg.dVelRef[1]; // 北速误差
			g_stKfPara.dDeltaVel[2] = 0.0;
			g_stKfPara.ucFlagSingular = KFUpdate(g_stKfPara.dXCalc, g_stKfPara.dPCalc, g_stKfPara.dXkk1, g_stKfPara.dPkk1, g_stKfPara.dDeltaVel, g_stKfPara.dDeltaPosn, g_stNavData.ucMeasMode);
			if (0 == g_stKfPara.ucFlagSingular)  //求逆出现奇异
				g_stNavData.uiErrCode |= U16_ERR_MAT_INV;  //报解算奇异故障
			g_stKfPara.ucFlagFilter = 0;  //滤波标志位清零
		}
		KFAdjust(g_stNavData.dQnpOut, g_stNavData.dCnpOut, g_stNavData.dAttiImu1, g_stNavData.dVelOut, g_stNavData.dPosnOut, g_stKfPara.dPhiEst, g_stKfPara.dGyroBiasEst, g_stKfPara.dAccBiasEst, g_stKfPara.dXCalc, g_stNavData.dQnpCh1, g_stNavData.dVelCh1, g_stNavData.dPosnCh1); //滤波值涑计算
		attiTrans(g_stNavData.dAttiOut, g_stNavData.dCpb, g_stNavData.dCnpOut);  //提取导航系姿态角，航向角
		
		if (U8_WORK_INTENAV_GPS == g_stNavData.ucWorkMode) // 位置组合
		{
			if (0 == g_stNavData.uiKSta%(60*UPDATE_FREQ))
			{
				attiFeedBack();  //通道1姿态，航向反馈
				velFeedBack();  //通道1速度反馈
				posnFeedBack(); //通道1位置反馈
			}
			iResetCnt = g_stNavData.uiKSta%(16*60*60*UPDATE_FREQ);
			if (0 == iResetCnt)  // 16h 重置滤波
			{
				g_stBindPara.dBgBind[2] += g_stKfPara.dGyroBiasEst[2]; // 补偿垂向陀螺漂移
				g_stKfPara.dXCalc[9] = g_stKfPara.dXkk1[9] = 0.0; // 重置垂向陀螺漂移状态估计
				g_stHostMsg.iBgBind[2] = (INT32_T)(g_stBindPara.dBgBind[2]*RPS2DPH*1.0e6);
				g_stNavData.uiComValid |= U32_ADJ_BGZ;
				KFReset();
			}
		}

		if (U8_WORK_SELFALIGN_FINE == g_stNavData.ucWorkMode) // 精对准阶段
		{
			if ((g_stNavData.uiKAlignReset1 == g_stNavData.uiKSta) || (g_stNavData.uiKAlignReset2 == g_stNavData.uiKSta) || (g_stNavData.uiKAlignReset3 == g_stNavData.uiKSta)
				|| (g_stNavData.uiKAlignReset4 == g_stNavData.uiKSta) || (g_stNavData.uiKAlignReset4 == g_stNavData.uiKSta) || (g_stNavData.uiKAlignReset6 == g_stNavData.uiKSta)
				|| (g_stNavData.uiKAlignReset7 == g_stNavData.uiKSta) || (g_stNavData.uiKAlignReset8 == g_stNavData.uiKSta) || (g_stNavData.uiKFineAlign == g_stNavData.uiKSta))  // 重置滤波器
			{
				attiFeedBack();   //通道1姿态，航向角反馈
				velFeedBack();   //通道1速度反馈
				posnFeedBack();  //通道1位置反馈
				KFReset();
			}

		}
	}
}

// 最小二程平滑由80个姿态角求当前姿态角速率
void CalcAttiRate(void)
{
	UINT16_T i;
	// 赋初值为0
	g_stNavData.dOmegaAtti[0] = 0.0;
	g_stNavData.dOmegaAtti[1] = 0.0;
	g_stNavData.dOmegaAtti[2] = 0.0;
	// 滤波窗平移
	for (i=0; i<MAX_RATE_WIDTH-1; i++)
	{
		g_stNavData.dAttiSeries[i][0] = g_stNavData.dAttiSeries[i+1][0];  //横摇
		g_stNavData.dAttiSeries[i][1] = g_stNavData.dAttiSeries[i+1][1];  //纵摇
		g_stNavData.dAttiSeries[i][2] = g_stNavData.dAttiSeries[i+1][2];  //方位
		// 航向过一、四象限处理
		if ((g_stNavData.dAttiOut[2]>(7.0*PI/4.0)) && (g_stNavData.dAttiSeries[i][2]<(PI/4.0)))  //顺时针过零
			g_stNavData.dAttiSeries[i][2] = g_stNavData.dAttiSeries[i][2]+2*PI;
		else if ((g_stNavData.dAttiOut[2]<(PI/4.0)) && (g_stNavData.dAttiSeries[i][2]>(7.0*PI/4.0)))  //逆时针过零
			g_stNavData.dAttiSeries[i][2] = g_stNavData.dAttiSeries[i][2]-2*PI; 
		// 加权滤波系数
		g_stNavData.dOmegaAtti[0] += g_stNavData.dAttiSeries[i][0]*g_dCoefRate[i];  //横摇
		g_stNavData.dOmegaAtti[1] += g_stNavData.dAttiSeries[i][1]*g_dCoefRate[i];  //纵摇
		g_stNavData.dOmegaAtti[2] += g_stNavData.dAttiSeries[i][2]*g_dCoefRate[i];  //方位
	}
	// 最新的姿态放队列尾
	g_stNavData.dAttiSeries[i][0] = g_stNavData.dAttiOut[0];  //横摇
	g_stNavData.dAttiSeries[i][1] = g_stNavData.dAttiOut[1];  //纵摇
	g_stNavData.dAttiSeries[i][2] = g_stNavData.dAttiOut[2];  //方位
	g_stNavData.dOmegaAtti[0] += g_stNavData.dAttiSeries[i][0]*g_dCoefRate[i];  //计算队尾横摇
	g_stNavData.dOmegaAtti[1] += g_stNavData.dAttiSeries[i][1]*g_dCoefRate[i];  //扑愣游沧菀?
	g_stNavData.dOmegaAtti[2] += g_stNavData.dAttiSeries[i][2]*g_dCoefRate[i];  //计算队尾方位
}

void TurnReady(void)
{
	// 从新初始化数据
	InitNavData();
	InitKFData();
	InitBDData();
}

void TurnCoarseAlign(void)
{
	UINT16_T i;
	// 姿态变换阵、惯性系速度初始化
	memset(g_stNavData.dCip0pCalc, 0, 9*sizeof(FLOAT64_T));
	for (i=0; i<3; i++)
		g_stNavData.dCip0pCalc[i*3+i] = 1.0;
	dmat2quat(g_stNavData.dQip0pCalc, g_stNavData.dCip0pCalc);
	memset(g_stNavData.dVip0Calc, 0, 3*sizeof(FLOAT64_T));
	memset(g_stNavData.dUi0Calc, 0, 3*sizeof(FLOAT64_T));
	g_stNavData.ucWorkMode = U8_WORK_SELFALIGN_COARSE; // 转状态
	g_stNavData.uiKSta = 0; // 状态时间清零
	g_stNavData.uiKPart = 0;
	g_stNavData.uiKMax = g_stNavData.uiKCoarseAlign; // 粗对准时间设置
}

void TurnFineAlign(void)
{
	if (U8_WORK_SELFALIGN_FINE == g_stNavData.ucWorkMode)
		return;
	// 阻尼系数和中间参数清零
	memset(g_stNavData.dKx, 0, 5*sizeof(FLOAT64_T));
	memset(g_stNavData.dKy, 0, 5*sizeof(FLOAT64_T));
	memset(g_stNavData.dDK, 0, 2*sizeof(FLOAT64_T));
	memset(g_stNavData.dTK, 0, 2*sizeof(FLOAT64_T));
	KFReset(); // 卡尔曼滤波重置
	// 速度位置的初始值用外参考赋值
	memcpy(g_stNavData.dVelCh1, g_stRefMsg.dVelRef, 2*sizeof(FLOAT64_T));
	memcpy(g_stNavData.dVelOut, g_stRefMsg.dVelRef, 2*sizeof(FLOAT64_T));
	memcpy(g_stNavData.dPosnCh1, g_stRefMsg.dPosnRef, 2*sizeof(FLOAT64_T));
	memcpy(g_stNavData.dPosnOut, g_stRefMsg.dPosnRef, 2*sizeof(FLOAT64_T));
	g_stNavData.ucWorkMode = U8_WORK_SELFALIGN_FINE; // 转状态
	g_stNavData.uiKSta = 0; // 状态时间清零, 子状态时间 KPart不清零
	g_stNavData.uiKMax = g_stNavData.uiKFineAlign; // 精对准时间设置
}

void TurnCalibrate(void)
{

	// 阻尼系数和中间参数清零
	memset(g_stNavData.dKx, 0, 5*sizeof(FLOAT64_T));
	memset(g_stNavData.dKy, 0, 5*sizeof(FLOAT64_T));
	memset(g_stNavData.dDK, 0, 2*sizeof(FLOAT64_T));
	memset(g_stNavData.dTK, 0, 2*sizeof(FLOAT64_T));
	KFReset(); // 卡尔曼滤波重置
	// 速度位置的初始值用外参考赋值
	memcpy(g_stNavData.dVelCh1, g_stRefMsg.dVelRef, 2*sizeof(FLOAT64_T));
	memcpy(g_stNavData.dVelOut, g_stRefMsg.dVelRef, 2*sizeof(FLOAT64_T));
	memcpy(g_stNavData.dPosnCh1, g_stRefMsg.dPosnRef, 2*sizeof(FLOAT64_T));
	memcpy(g_stNavData.dPosnOut, g_stRefMsg.dPosnRef, 2*sizeof(FLOAT64_T));
	g_stNavData.ucWorkMode = U8_WORK_CALIBRATE_SYS; // 转状态
	g_stNavData.uiKSta = 0; // 状态时间清零, 子状态时间 KPart不清零
	g_stNavData.uiKMax = g_stNavData.uiKFineAlign; // 精对准时间设置
}

void TurnNoDamp(void)
{
	if (U8_WORK_SELFNAV_NODAMP == g_stNavData.ucWorkMode)  //无阻尼状态反回
		return;
	// 阻尼系数清零,中间参数未变
	memset(g_stNavData.dKx, 0, 5*sizeof(FLOAT64_T));  //x通道阻尼系数清零
	memset(g_stNavData.dKy, 0, 5*sizeof(FLOAT64_T));  //y通道阻尼系数清零
	// 若前状态为精对准或综合校准或组合则对姿态、速度、位置赋值,子状态计数清零
	if ((U8_WORK_SELFALIGN_FINE == g_stNavData.ucWorkMode) || (U8_WORK_INTENAV_DVL== g_stNavData.ucWorkMode) || (U8_WORK_INTENAV_GPS == g_stNavData.ucWorkMode))
	{
		memcpy(g_stNavData.dQnpCh1, g_stNavData.dQnpOut, 4*sizeof(FLOAT64_T));  //反馈校正姿态，航向
		memcpy(g_stNavData.dAttiCh1, g_stNavData.dAttiOut, 3*sizeof(FLOAT64_T));
		memcpy(g_stNavData.dCnpCh1, g_stNavData.dCnpOut, 9*sizeof(FLOAT64_T));

		memcpy(g_stNavData.dVelCh1, g_stNavData.dVelOut, 2*sizeof(FLOAT64_T));     //反馈校正速度
		memcpy(g_stNavData.dPosnCh1, g_stNavData.dPosnOut, 2*sizeof(FLOAT64_T));  //反馈校正位置
		memcpy(g_stNavData.dVelMid, g_stNavData.dVelOut, 2*sizeof(FLOAT64_T));      //重置中间速度
		g_stNavData.uiKPart = 0;
	}
	g_stNavData.ucWorkMode = U8_WORK_SELFNAV_NODAMP; // 转状态
	g_stNavData.uiKSta = 0; // 状态时间清零
	g_stNavData.uiKMax = UINT32_INF; // 时间设置
}

void TurnLevelDamp(void)
{
	if (U8_WORK_SELFNAV_LEDAMP == g_stNavData.ucWorkMode)  //阻尼状态返回
		return;
	if (0 == (g_stNavData.ucRefValid&(U32_VAL_ZERO_VEL|U32_VAL_DVL_VE|U32_VAL_DVL_VW|U32_VAL_LOG_VEL|U32_VAL_GPS_VEL))) // no ref vel
	{
		TurnNoDamp(); // 没有参考速度信息则入无阻尼
		return;
	}

	// 设置阻尼系数,中间参数未变
	g_stNavData.dKx[0] = g_stNavData.dKy[0] = 0.002482;  //k1系数初始化
	g_stNavData.dKx[1] = g_stNavData.dKy[1] = 2.1955;     //k2系数初始化
	g_stNavData.dKx[2] = g_stNavData.dKy[2] = 0.002482;  //k3系数初始化
	// 若当前状态为精对准或综合校准或组合则对姿态、速度、位置赋值,子状态计数清零
	if ((U8_WORK_SELFALIGN_FINE == g_stNavData.ucWorkMode) || (U8_WORK_INTENAV_DVL== g_stNavData.ucWorkMode) || (U8_WORK_INTENAV_GPS == g_stNavData.ucWorkMode))
	{
		memcpy(g_stNavData.dQnpCh1, g_stNavData.dQnpOut, 4*sizeof(FLOAT64_T));  //反馈校正姿态航向
		memcpy(g_stNavData.dAttiCh1, g_stNavData.dAttiOut, 3*sizeof(FLOAT64_T));
		memcpy(g_stNavData.dCnpCh1, g_stNavData.dCnpOut, 9*sizeof(FLOAT64_T));

		memcpy(g_stNavData.dVelCh1, g_stNavData.dVelOut, 2*sizeof(FLOAT64_T));  //反馈校正速度
		memcpy(g_stNavData.dPosnCh1, g_stNavData.dPosnOut, 2*sizeof(FLOAT64_T));  //反馈校正位置
		memcpy(g_stNavData.dVelMid, g_stNavData.dVelOut, 2*sizeof(FLOAT64_T));    //重置中间速度
		g_stNavData.uiKPart = 0;  //自状态时间清零
	}
	// 进入水平阻尼初始时刻计算阻尼参考速度零位
	g_stNavData.dVelDampZero[0] = g_stNavData.dVelMid[0]-g_stRefMsg.dVelDampRef[0];  //计算东速零位
	g_stNavData.dVelDampZero[1] = g_stNavData.dVelMid[1]-g_stRefMsg.dVelDampRef[1];  //计算北速零位
	g_stNavData.ucWorkMode = U8_WORK_SELFNAV_LEDAMP; // 转状态
	g_stNavData.uiKSta = 0; // 状态时间清零
	g_stNavData.uiKMax = UINT32_INF; // 时间设置
}

void TurnPosnAided(void)
{
	if (U8_WORK_INTENAV_GPS == g_stNavData.ucWorkMode)  //位置组合状态返回
		return;
	if (0 == (g_stNavData.ucRefValid&(U32_VAL_BIND_POSN|U32_VAL_GPS_POSN))) // no ref posn
		return;

	// 阻尼系数和中间参数清零
	memset(g_stNavData.dKx, 0, 5*sizeof(FLOAT64_T));  //x通道阻尼系数清零
	memset(g_stNavData.dKy, 0, 5*sizeof(FLOAT64_T));  //y通道阻尼系数清零
	memset(g_stNavData.dDK, 0, 2*sizeof(FLOAT64_T));  //k2中间变量清零
	memset(g_stNavData.dTK, 0, 2*sizeof(FLOAT64_T));  //k1中间变量清零
	// 若前状态为精对准则对速度、位置赋值
	if (U8_WORK_SELFALIGN_FINE != g_stNavData.ucWorkMode)
	{
		memcpy(g_stNavData.dVelCh1, g_stRefMsg.dVelRef, 2*sizeof(FLOAT64_T));
		memcpy(g_stNavData.dVelOut, g_stRefMsg.dVelRef, 2*sizeof(FLOAT64_T));
		memcpy(g_stNavData.dPosnCh1, g_stRefMsg.dPosnRef, 2*sizeof(FLOAT64_T));
		memcpy(g_stNavData.dPosnOut, g_stRefMsg.dPosnRef, 2*sizeof(FLOAT64_T));
	}
	g_stNavData.ucWorkMode = U8_WORK_INTENAV_GPS; // 转状态
	g_stNavData.uiKSta = 0; // 状态时间清零
	g_stNavData.uiKPart = 0;  //系统时间清零
	g_stNavData.uiKMax = UINT32_INF; // 时间设置
}

void TurnCoarseAzimuth()
{
	memset(g_stNavData.dDK, 0, 2*sizeof(FLOAT64_T));
	memset(g_stNavData.dTK, 0, 2*sizeof(FLOAT64_T));
	calcLevelCoef(g_stNavData.dKx, 0.8, 0.0164);
	calcCompassCoef(g_stNavData.dKy, 0.8, 0.0164);
	g_stNavData.ucWorkMode = U8_WORK_COMPALIGN_AZIMUTH; // 转状态
	g_stNavData.uiKSta = 0; // 状态时间清零
	g_stNavData.uiKPart = 0;  //系统时间清零
	g_stNavData.uiKMax = g_stNavData.uiKFineAlign; // 时间设置
}

void TurnCompass(void)
{
	if (U8_WORK_COMPASS == g_stNavData.ucWorkMode) // 罗经
		return;
	memset(g_stNavData.dDK, 0, 2*sizeof(FLOAT64_T));
	memset(g_stNavData.dTK, 0, 2*sizeof(FLOAT64_T));
	calcLevelCoef(g_stNavData.dKx, 0.8, 0.0023);
	calcCompassCoef(g_stNavData.dKy, 0.8, 0.0023);
	g_stNavData.ucWorkMode = U8_WORK_COMPASS; // 转状态
	g_stNavData.uiKSta = 0; // 状态时间清零
	g_stNavData.uiKPart = 0;  //系统时间清零
	g_stNavData.uiKMax = UINT32_INF; // 时间设置
}

/************************************************************************\
 * calculate level damp coefficient
 * Td        - oscillation cycle
 * varsigma  - damp ratio
 output
 * K	     - aim coefficient
2012.6.14 LuQuancong
\************************************************************************/
void calcLevelCoef(FLOAT64_T K[], const FLOAT64_T varsigma, const FLOAT64_T sigma)
{
	FLOAT64_T tmp;
    if (zero2(varsigma) || zero2(sigma))
	{
		memset(K, 0, 5*sizeof(FLOAT64_T));
		return ;
	}

	tmp = sigma/varsigma;
	K[0] = 3*sigma;
	K[1] = (2*sigma*sigma+tmp*tmp)*GLV_R/GLV_G-1.0;
	K[2] = tmp*tmp*sigma*GLV_R/GLV_G;
	K[3] = 0.0;
	K[4] = 0.0;
}

/************************************************************************\
 * calculate azimuth damp coefficient
 * Td        - oscillation cycle
 * varsigma  - damp ratio
 output
 * K	     - aim coefficient
2012.6.14 LuQuancong
\************************************************************************/
void calcCompassCoef(FLOAT64_T K[], const FLOAT64_T varsigma, const FLOAT64_T sigma)
{
	double tmp;
    if (zero2(varsigma) || zero2(sigma))
	{
		memset(K, 0, 5*sizeof(FLOAT64_T));
		return ;
	}
	tmp = sigma/varsigma;

	// 3阶
	K[0] = 3*sigma;
	K[1] = (2*sigma*sigma+tmp*tmp)*GLV_R/GLV_G-1.0;
	K[2] = 0.0;
	K[3] = tmp*tmp*sigma*GLV_R/GLV_G;
	K[4] = 0.0;
}




