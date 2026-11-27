#include "stm32f4xx.h"                  // Device header
#include "arm_math.h"
#include "FocMath.h"

void ClarkeCale(M_Clarke value)
{
    value -> Alpha = value -> Iu;                                                           // IA = 2/3(Iu - 0.5Iv - 0.5Iw)
    value -> Beta = (value->Iu + 2.0f * value->Iv) * 0.577350269f;                          // IB = 2/3(√3/2 * Iv - √3/2 * Iw)
}

void ParkCale(M_Park value)
{
    value -> Ds = arm_cos_f32(value -> Theta) * value -> Alpha + arm_sin_f32(value -> Theta) * value -> Beta;
    value -> Qs = arm_cos_f32(value -> Theta) * value -> Beta - arm_sin_f32(value -> Theta) * value -> Alpha;
}

void IParkCale(M_IPark value)
{
    value -> Alpha = arm_cos_f32(value -> Theta) * value -> Ds - arm_sin_f32(value -> Theta) * value -> Qs;
    value -> Beta = arm_sin_f32(value -> Theta) * value -> Ds + arm_cos_f32(value -> Theta) * value -> Qs;
}

int SvpwmCale(M_SVPWM value, float Vbus)
{
    const float Ua = value -> Ualpha;                                                       // U1 = Uα，取 α 轴分量 
    const float Ub = -0.5f * value -> Ualpha + 0.866025404f * value -> Ubeta;               // U2 = -0.5·Uα + (√3/2)·Uβ
    const float Uc = -Ua - Ub;                                                              // U3 = -U1 - U2

    float Umax = (Ua > Ub) ? Ua : Ub;
    float Umin = (Ua < Ub) ? Ua : Ub;

    Umax = (Uc > Umax) ? Uc : Umax;
    Umin = (Uc < Umin) ? Uc : Umin;
    
    const float Uspan = Umax - Umin;
    if (!(Vbus > 0.0f) || !(Uspan <= Vbus))
        return 0;
    
    const float Umid = Umin + 0.5f * Uspan;                                                 // 计算极值中点，即注入的零序分量 (Umax + Umin) / 2
    value -> Ta = 0.5f + (Ua -Umid) / Vbus;                                                 // 平移中心点 + 归一化 + 0.5偏置，算出A相占空比 (0~1)
    value -> Tb = 0.5f + (Ub -Umid) / Vbus;                                                 // 算出B相占空比
    value -> Tc = 0.5f + (Uc -Umid) / Vbus;                                                 // 算出C相占空比

    return 1;
}
