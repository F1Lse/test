#include "ubf.h"

#include "string.h"
#include "stdlib.h"

#define ASSERT(x) do {while(!(x));} while(0)

struct _ubf_t {
    void* pbuff;//���������׵�ַ
    uint32_t num;//num��size�ֽڵ����ݵ�Ԫ
    uint32_t size;
    uint32_t write;//��д�뵥Ԫ������(write-1)ָ����������
    uint8_t isfull;//����������־
};

ubf_t ubf_create(uint32_t num, uint32_t size) {
    ubf_t pubf = (ubf_t)malloc(sizeof(struct _ubf_t));
    void* pbuff = NULL;
    memset(pubf, 0, sizeof(struct _ubf_t));
    if (pubf) {
        pubf->num = num;
        pubf->size = size;
        pbuff = calloc(num, size);//�������ڴ�ʵ�壬ͬʱ����
        ASSERT(pbuff);
        pubf->pbuff = pbuff;
        return pubf;
    }
    return NULL;
}

void ubf_delete(ubf_t* ubf) {
    free((*ubf)->pbuff);
    free(*ubf);
    *ubf = NULL;//��ָֹ�뱻��һ��ʹ��
}

void ubf_push(ubf_t ubf, const void* pdata) {
    ASSERT(ubf);
    memcpy((uint8_t*)ubf->pbuff+ubf->write*ubf->size, pdata, ubf->size);
    if (++ubf->write >= ubf->num){
        ubf->write = 0;
        ubf->isfull = 1;
    }
}

void* ubf_pop(ubf_t ubf, uint32_t k) {
    void* res;
    if (k < ubf->write)//��ǰ��ȡ
        res = (uint8_t*)ubf->pbuff + (ubf->write-1-k)*ubf->size;
    else if (k < ubf->num)//�ۻ�β����ȡ
        res = (uint8_t*)ubf->pbuff + ((ubf->num - 1) - (k - ubf->write)) * ubf->size;
    else//������������
        res = NULL;
    return res;
}

uint32_t ubf_get_stock(ubf_t ubf) {
    uint32_t res;
    if (ubf == NULL)
        res = 0;
    else if (ubf->isfull)//����������
        res = ubf->num;
    else
        res = ubf->write;
    return res;
}

void ubf_clear(ubf_t ubf) {
    ASSERT(ubf);
    memset(ubf->pbuff, 0, ubf->num * ubf->size);
    ubf->write = 0;
    ubf->isfull = 0;
}

/* ��Ԫ��Ϊ���µ����� */
// ע�⴫�������������ָ��һ��Ҫ��ǰ�����ã�
uint32_t ubf_pop_into_array_new2old(ubf_t ubf, void* pdata, uint32_t k, uint32_t num) {
    ASSERT(NULL != ubf && k <= ubf_get_stock(ubf)-1); //������������ �� ��ȡû�г�����
    void* ptr;
    uint32_t real_res_num = 0;
    for (uint32_t cnt = 0; cnt < num; ++cnt) {  //cnt��[0, num-1]������num-1-0+1=num������Ȼѭ��
        ptr = ubf_pop(ubf, k + cnt);
        if (ptr) {
            memcpy((uint8_t*)pdata+cnt*ubf->size, ptr, ubf->size);
            ++real_res_num;
        }
        else break;
    }
    return real_res_num;
}

/* ��Ԫ��Ϊ��ɵ����� */
uint32_t ubf_pop_into_array_old2new(ubf_t ubf, void* pdata, uint32_t k, uint32_t num) {
    ASSERT(NULL != ubf && k <= ubf_get_stock(ubf)-1); //������������ �� ��ȡ����û���������
    void* ptr;
    uint32_t real_num = 0;

    for (uint32_t cnt = k+num-1; cnt+1 > k; --cnt) {
        ptr = ubf_pop(ubf, cnt);
        if (ptr) {
            memcpy((uint8_t*)pdata+real_num*ubf->size, ptr, ubf->size);
            ++real_num;
        }
    }
    return real_num;
}

/* ʵʱ����ƽ���˲� */
float ubf_avg_filter(ubf_t ubf, float* pdata, uint32_t num) {
    ASSERT(ubf && pdata && num);//���������� ���������������0
    ubf_push(ubf, pdata);//�������µ�����
    if (ubf_get_stock(ubf) <= num) {  //��������Ч���ݲ���
        num = ubf_get_stock(ubf);//����ƽ����ĸ
    }
    
    float temp_res = 0.0f, *temp_array = NULL;
    temp_array = (float*)calloc(num, sizeof(float));//������ʱ�ռ� ͬʱ����
    if (temp_array) {
        ubf_pop_into_array_new2old(ubf, temp_array, 0, num);
        for (uint8_t i = 0; i < num; ++i) {
            temp_res += temp_array[i];
        }
    }
    free(temp_array);
    
    return temp_res / num;
}

float ubf_lin_wei_filter(ubf_t ubf, float* pdata, float* pweight, uint32_t num) {
    
    return 0;
}

/* ʵʱ�������Լ�Ȩ�� */
float ubf_lin_wei_gradual_deduction(ubf_t ubf, float* pdata, float* pweight, uint32_t num) {
    ASSERT(NULL != ubf && num !=0 && num%2 == 0 && num <= ubf_get_stock(ubf));//���������� ���������������0 ��num��ż�� ��������С������

    ubf_push(ubf, pdata);//�������µ�����   
    if (ubf->isfull == 0 && num > ubf->write) {  //�������������ڵ�ǰ����
        return 0;
    }
    float temp_res = 0;
    float* temp_array = (float*)calloc(num, sizeof(float));//������ʱ�ռ� ͬʱ����
    if (temp_array) {
        ubf_pop_into_array_new2old(ubf, temp_array, 0, num);//��ȡ���µ�num������
        for (uint8_t i = 0; i < num/2; ++i) {
            temp_res += pweight[i] * (temp_array[i] - temp_array[i + num/2]);
        }
    }
    free(temp_array);
    return temp_res;
}

/* ʵʱ�������Լ�ȨԤ������ */
float ubf_lin_wei_predict(ubf_t ubf, float* pdata, float* pweight, uint32_t num, uint8_t k, uint8_t flag) {
    ASSERT(ubf && pdata && pweight && num);
    if (flag && ubf_get_stock(ubf) >= num + k) {  //����Ԥ��
        float res = 0;
        float* temp_array = (float*)calloc(num, sizeof(float));//������ʱ�ռ� ͬʱ����
        if (temp_array) { //�������ڴ�
            ubf_pop_into_array_new2old(ubf, temp_array, k, num);//��ȡkʱ��ǰnum������
            for (uint8_t i = 0; i < num-1; ++i) {
                res += pweight[i] * (temp_array[i] - temp_array[i+1]);
            }
            res += *(float*)ubf_pop(ubf, 0);
            ubf_push(ubf, &res);
            *pdata = res;
        }
        free(temp_array);
    } else {
        ubf_push(ubf, pdata);
    }
    return *pdata;
}


/**
 * @brief ģ����Դ���,��ע�ͺ����в鿴�ڴ�ֵ��֤
 */
// #define TEST_ENABLE

#ifdef TEST_ENABLE
#include "stdio.h"

int main()
{
    uint32_t i;
    /* ����1����������������ȡ���ݡ�ɾ�������� */
    struct _test_t
    {
        float a;
        int b;
        char c;
    }test;
    struct _test_t* tt;
    test.a = 0.1;
    test.b = 0;
    test.c = 124;

    ubf_t ubf1 = ubf_create(5, sizeof(struct _test_t));// �������ѻ�����ָ��

    for (i=0; i<8; i++)
    {
        ubf_push(ubf1, &test);// ��������
        test.b++;
    }
    for (i=0; i<8; i++)
    {
        tt = ubf_pop(ubf1, i);// ��ȡ��K-iʱ�̵�����
    }
    ubf_delete(ubf1);// �ͷŻ�����


    /* ����2����ȡ���ݽ�һά���� */
    uint32_t real_res, array_new[20]={}, array_old[20]={},
        num_indeed = 15, num_inneed = 21;//num_indeed��������С����20����num_inneed��Ҫȡ��������
    ubf_t ubf = ubf_create(num_indeed, sizeof(uint32_t));// �������ѻ�����ָ��
    if (ubf)
    {
        for (i=0; i<num_indeed; ++i)
        {
            ubf_push(ubf, &i);
        }
        real_res = ubf_pop_into_array_new2old(ubf, array_new, 4, num_inneed);
        printf("the res new array is as follow:\n");
        for (i=0; i<20; ++i)
        {
            printf("%d ", array_new[i]);
        }
        printf("\nthe real num of the res = %d", real_res);

        printf("\n-------------------------------------\n");

        real_res = ubf_pop_into_array_old2new(ubf, array_old, 4, num_inneed);
        printf("the res old array is as follow:\n");
        for (i=0; i<20; ++i)
        {
            printf("%d ", array_old[i]);
        }
        printf("\nthe real num of the res = %d", real_res);

        ubf_delete(ubf);// �ͷŻ�����
    }
    return 0;
}
#endif

