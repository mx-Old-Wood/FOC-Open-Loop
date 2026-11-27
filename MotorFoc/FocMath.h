#ifndef __FOCMATH_H__
#define __FOCMATH_H__

#include <stdint.h>

typedef struct
{
    float Iu;           // 三相电流A
    float Iv;           // 三相电流B
    float Iw;           // 三相电路C
    float Alpha;        // 二相静止坐标系 Alpha 轴
    float Beta;         // 二相静止坐标系 Beta 轴
} Clarke, *M_Clarke;            

typedef struct
{
    float Alpha;        // 二相静止坐标系 Alpha 轴
    float Beta;         // 二相静止坐标系 Beta 轴
    float Theta;        // 电机电角度
    float Ds;           // 电机二相旋转坐标系下的d轴电压
    float Qs;           // 电机二相旋转坐标系下的q轴电压
} Park, *M_Park;

typedef struct
{
	float Alpha;        // 二相静止坐标系 Alpha 轴
	float Beta;	        // 二相静止坐标系 Beta 轴
	float Theta;        // 电机电角度
	float Ds;	        // 电机二相旋转坐标系下的d轴电流
	float Qs;	        // 电机二相旋转坐标系下的q轴电流
} IPark, *M_IPark;

typedef struct 
{
	float Ualpha;	    // 二相静止坐标系alpha-轴
	float Ubeta;	    // 二相静止坐标系beta-轴
	float Ta;		    // 三相矢量占空比Ta
	float Tb;		    // 三相矢量占空比Tb
	float Tc;		    // 三相矢量占空比Tc
} SVPWM, *M_SVPWM;

void ClarkeCale(M_Clarke value);
void ParkCale(M_Park value);
void IParkCale(M_IPark value);
int SvpwmCale(M_SVPWM value, float Vbus);

#endif
