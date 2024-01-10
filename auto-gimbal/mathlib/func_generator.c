/**
  *************************************************************************
  * @brief  �����źŷ�����
  * @author ZZJ
  * @date   2021/2/10
  * @note   1. ����ɢ���������źţ�
  *                 ���ң����������ǲ������ݲ����źţ������ź�
  *         2. ʱ�䶼����msΪ��λ
  *         3. ��ʼ��ʱ�ı���˳��Ϊ��
  *             ������ָ�룬�������ڣ�����޷�����С�޷���
  *             ֱ�����������������������������������
  *         4. �ɲ�����������͸�˹����������ǰ����Ҫ����Ӳ��RNG
  *************************************************************************
  */
#define __FUNC_GENERATOR_GLOBALS__
#include "func_generator.h"


/*------------------------------------�����źŽ�ֱ����������-----------------------------------*/
/**
  * @Brief  �����źŷ�������ʼ��
  * @note   һ��������������
  */
void FGT_sin_init(
    FGT_sin_t*  sin,
    uint16_t    Td,
    float       max,
    float       min,
    float       dc,
    float       T,
    float       A,
    float       phi
)
{
    sin->Td = Td;
    sin->time = 0;

    sin->max = max;
    sin->min = min;

    sin->dc = dc;

    sin->A = A;
    sin->T = T;
    sin->phi = phi;

    sin->out = 0;
}

/**
  * @Brief  �����źŷ��������³�ʼ������
  * @note   ֻ���ڷ��������й����иı䲿�ֲ���
  */
void FGT_sin_reinit(
    FGT_sin_t*  sin,
    float       max,
    float       min,
    float       dc,
    float       T,
    float       A
)
{
    sin->max = max;
    sin->min = min;

    sin->dc = dc;

    sin->A = A;
    sin->T = T;
}


/**
  * @Brief  ���������ź�
  */
float FGT_sin_cal(FGT_sin_t* sin)
{
    float ac  = 0;
    float res = 0;

    /*���㽻����*/
    ac = (sin->A) * sinf( (2*pi / sin->T) * (sin->time) + sin->phi );

    /*���㽻ֱ����*/
    res = ac + sin->dc;
    Output_Limit(res, sin->max, sin->min);

    /*����ʱ�䣨������ټ��㣬ʹ��0��ʼ��*/
    sin->time = fmodf(sin->time+sin->Td, sin->T);

    /*���������¼*/
    sin->out = res;

    /*����ǲ���ֱ����*/
    return sin->out;
}


/*------------------------------------�����źŽ�ֱ����������-----------------------------------*/
/**
  * @Brief  �����źŷ�������ʼ��
  * @note   ����������������
  */
void FGT_sqr_init(
    FGT_sqr_t* sqr,

    uint16_t Td,

    float max,
    float min,

    float dc,

    float Th,
    float Tl,
    float high,
    float low
)
{
    sqr->Td   = Td;
    sqr->time = 0;

    sqr->max = max;
    sqr->min = min;

    sqr->dc  = dc;

    sqr->high = high;
    sqr->low  = low;
    sqr->Th   = Th;
    sqr->Tl   = Tl;

    sqr->out  = 0;
}


/**
  * @Brief  ���������ź�
  */
float FGT_sqr_cal(FGT_sqr_t* sqr)
{
    float ac  = 0;
    float res = 0;

    /*��������*/
    float T   = sqr->Th + sqr->Tl;

    /*���㽻����*/
    if( sqr->time > sqr->Th )
        ac = sqr->low;
    else
        ac = sqr->high;

    /*���㽻ֱ����*/
    res = ac + sqr->dc;
    Output_Limit(res, sqr->max, sqr->min);

    /*����ʱ�䣨������ټ��㣬ʹ��0��ʼ��*/
    sqr->time = fmodf(sqr->time+sqr->Td, T);

    /*���������¼*/
    sqr->out = res;

    /*���ط�����ֱ����*/
    return sqr->out;
}


/*------------------------------------�ǲ��źŽ�ֱ����������-----------------------------------*/
/**
  * @Brief  �ǲ��źŷ�������ʼ��
  * @note   ����������������
  */
void FGT_agl_init(
    FGT_agl_t* agl,

    uint16_t Td,
    float max,
    float min,

    float dc,

    float T1,
    float T2,
    float high,
    float low
)
{
    agl->Td   = Td;
    agl->time = 0;

    agl->max = max;
    agl->min = min;

    agl->dc  = dc;

    agl->T1 = T1;
    agl->T2 = T2;

    agl->high = high;
    agl->low  = low;

    agl->out = 0;
}


/**
  * @Brief  �����ǲ��ź�
  */
float FGT_agl_cal(FGT_agl_t* agl)
{
    float ac  = 0;
    float res = 0;

    /*��������*/
    float T   = agl->T1 + agl->T2;

    /*ȷ������,�������ʱ�䣬���㽻����*/
    if( agl->time < agl->T1 )  //�ڵ�һ���յ�֮ǰ
    {
        ac = ( (agl->low-agl->high)/agl->T1 ) * agl->time + agl->high;
    }
    else if( agl->time >= agl->T1 )  //�������յ�֮��
    {
        ac = ( (agl->high-agl->low)/agl->T2 ) * ( agl->time - agl->T1 ) + agl->low;
    }

    /*���㽻ֱ����*/
    res = ac + agl->dc;
    Output_Limit(res, agl->max, agl->min);

    /*����ʱ��*/
    agl->time = fmodf(agl->time+agl->Td,T);

    /*���������¼*/
    agl->out = res;

    /*����ǲ���ֱ����*/
    return agl->out;
}


/*------------------------------------���ź���-----------------------------------*/
/**
  * @Brief  ������������ʼ��
  */
void FGT_npz_init(
    FGT_npz_t* npz,
    uint16_t Td,
    float T1,
    float T2,
    float T3
)
{
    npz->Td   = Td;
    npz->time = 0;

    npz->T1 = T1;
    npz->T2 = T2;
    npz->T3 = T3;

    npz->out = 0;
}


/**
  * @Brief  ���������ź�
  * @func   ���β���T1ʱ���1��T2ʱ���0��T3ʱ���-1�ź�
  */
float FGT_npz_cal(FGT_npz_t* npz)
{
    /*��������*/
    float T   = npz->T1 + npz->T2 + npz->T3;

    /*ȷ�����*/
    if( npz->time <= npz->T1 )  //�ڵ�һ���յ�֮ǰ
        npz->out = 1;
    else if( (npz->T1 < npz->time) && (npz->time < npz->T1 + npz->T2) )  //�������յ�֮��
        npz->out = 0;
    else if( (npz->T1 + npz->T2 <= npz->time) && (npz->time <= T) )  //�ڵڶ����յ�֮��
        npz->out = -1;

    /*����ʱ�䣨������ټ��㣬ʹ��0��ʼ��*/
    npz->time = fmodf(npz->time+npz->Td,T);

    /*�������*/
    return npz->out;
}

/*------------------------------------�������������-----------------------------------*/
/**
  * @Brief  ��ȡ��ǰ���ɵ�32λ�޷��������
  */
//HAL_StatusTypeDef HAL_RNG_GenerateRandomNumber(RNG_HandleTypeDef *hrng, uint32_t *random32bit);


/**
  * @Brief  ����[min,max]��Χ�������
  */
int RNG_Get_RandomRange(int min,int max)
{
    uint32_t temp_32_rng;
//    HAL_RNG_GenerateRandomNumber(&hrng,&temp_32_rng);
    return temp_32_rng%(max-min+1) +min;
}


/*------------------------------------��˹����-----------------------------------*/
/**
  * @Brief  ����һ����˹������
  */
float GaussGenerate(float mu, float sigma_f)
{
    float t1,t2,a,r;
    float x;
    /*�����������ȷֲ���0~1���������*/
    t1 = (float)rand()/(RAND_MAX);
    t2 = (float)rand()/(RAND_MAX);
    /*�������������������ֲ�����*/
    a = 2*pi*t1;            //a�Ǽ�����ĽǶȣ������0~2*pi�ľ��ȷֲ�
    r = sqrt(-2*log2f(t2));   //r�Ǽ�����ľ��룺�����Ȼ���������ŵ�һ�ֲַ�
    /*�ü�����(a,r)ת���ɵѿ�������(x,y)������ǲ����ĸ�˹������*/
    x = r*cos(a);

    return mu+sigma_f*x;
}


/**
  * @Brief  ����һ����˹��������
  */
void Gauss(float gs[], int lengh, float mu, float sigma_f)
{
    for(int i=0; i<lengh; i++)
        gs[i]=GaussGenerate(mu,sigma_f);
}


/*------------------------------------һ�㺯���źŷ�����-----------------------------------*/
/**
  * @Brief  һ�㺯���źŷ�����
  * @param  �����ṹ��ָ��
  * @note   ��Ҫ��������������ṹ������ڣ�΢����֮��Ķ�Ӧ��ϵ
  */
float FGT_f_generator(FGT_f_t* pf)
{
    float temp_out;
    /*����ʱ�䣨������ټ��㣬ʹ��0��ʼ��*/
    pf->time = fmodf(pf->time+pf->Td,pf->T);

    /*������� �޷�*/
    temp_out = pf->f(pf->time);
    Output_Limit(temp_out, pf->max, pf-> min);
    pf->out = temp_out;

    /*�������*/
    return pf->out;
}



/*------------------------------------�ܳ�ʼ��-----------------------------------*/
#include "control_def.h"

static float f_test(float x)
{
    return 0.1f * x;
}


/*
    ��ִ������ 10ms��T = 6000�� Td = 1����ζ��
    ������̽�ִ�� ��6000 /1.��* 10 = 60000ms = 1min
    ������ǰ�����޷�
*/
FGT_f_t test_f =
{
    .f = f_test,
    .time = 0,
    .T = 6000,
    .Td = 10,
    .max = 7000,
    .min = 0,
    .out = 0
};


void FGT_init(void)
{

    /* һ�㲨���źŷ����� */
    FGT_sin_init(&test_sin, 2, 3670, 2400, 3035, 300,        600,0);
    FGT_sin_init(&test_cos, 1, 300, 0, 150, 4000,        100, pi/2);
    FGT_sqr_init(&test_sqr, 1, 3500, 2500, 0, 1000, 1000,    3500,2500);
    FGT_agl_init(&test_agl, 5, 500, -500, 200, 100,200,     100, -50);

    FGT_npz_init(&test_npz_101, 5, 3000, 1000, 3000);
    FGT_npz_init(&test_npz_10, 5, 3000, 1000, 0);

    /* ���⹦���źŷ����� */
    //FGT_sin_init(&chassis_spin_sin, 2, 3670, 2400, 3035, 800,        600,0);
    FGT_sqr_init(&test_pit, 1, (GIMBAL_PIT_MAX-150), (GIMBAL_PIT_MIN+150), 0, 800, 800,  (GIMBAL_PIT_MAX-150),(GIMBAL_PIT_MIN+150));
}

//FGT_sin_t test_s =
//{
//    .Td = 1,
//    .time = 0,
//    .max = 1000,
//    .min = -1000,
//    .dc = 0,
//    .T = 100,
//    .A = 50,
//    .phi = 0,
//    .out = 0
//};



