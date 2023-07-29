/**
  Laser project by dem1305

  @2018
  
  ������ ������
*/
//----------------------------------------------------------------------------//
#ifndef __MPU_H
#define __MPU_H
//----------------------------------------------------------------------------//
#include "core.h"
//----------------------------------------------------------------------------//
#define PRIVDEFENA  4 //���������� ������� � ����� ������ (���) � �������r��������� ������. 
#define HFNMIENA    2 //��������� ������������� MPU � ������������ NMI � ���������� Hard Fault. 
#define MPUENABLE   1

#define MPUVALID    16 //����� ������� MPU ����� ������������ ����� REGION �������� RBAR

//�������� ������
#define MPU_S              (((uint32_t)1)<<18) //SHARED    MEMORY  �����������
#define MPU_C              (((uint32_t)1)<<17) //CACHEABLE MEMORY  ����������
#define MPU_B              (((uint32_t)1)<<16) //BUFFERED  MEMORY  ��������������
#define MPU_REGION_ENABLE  ((uint32_t)1) //������ ��������
#define MPU_REGION_DISABLE (~MPU_REGION_ENABLE)

// AP_�����������������_������������������� �������
typedef enum
{
  AT_X=                    0x000,
  AT_RWX=                  0x001,
  AT_RWRO=                 0x010,
  AT_RWRW=                 0x011,
  AT_ROX=                  0x101,
  AP_RORO=                 0x110,
  AP_RO_RO2=               0x111
}MPU_AP_Type;
/*
��  �������r���������   ����������������   �������� 
   
000 ������ ��������     ������ ��������   ������ �������� 
001 ������/������       ������ ��������   ������ ������ �� �������r��������� ������ 
010 ������/������       ������ ������     ������ � ���������������� ���r����� ������� ����� 
011 ������/������       ������/������     ������ ������ 
100 ��������������      ��������������    �������������� 
101 ������ ������       ������ ��������   ������ ������ �� �������r��������� ������ 
110 ������ ������       ������ ������     ������ ������ 
111 ������ ������       ������ ������     ������ ������ 
*/

/* ������ ������� ������,��� ������������ MPU
0x00000 ��������������� 
0x00001 ��������������� 
0x00010 ��������������� 
0x00011 /��������������� 
*/
typedef enum{
MPU_SIZE_32=0x00100 ,//32 ����� 
MPU_SIZE_64=0x00101 ,//64 ����� 
MPU_SIZE_128=0x00110 ,//128 ���� 
MPU_SIZE_256=0x00111 ,//256 ���� 
MPU_SIZE_512=0x01000 ,//512 ���� 
MPU_SIZE_1K=0x01001 ,//1 ����� 
MPU_SIZE_2K=0x01010 ,//2 ����� 
MPU_SIZE_4K=0x01011 ,//4 ����� 
MPU_SIZE_8K=0x01100 ,//8 ����� 
MPU_SIZE_16K=0x01101 ,//16 ����� 
MPU_SIZE_32K=0x01110 ,//32 ����� 
MPU_SIZE_64K=0x01111 ,//64 ����� 
MPU_SIZE_128K=0x10000 ,//128 ����� 
MPU_SIZE_256K=0x10001 ,//256 ����� 
MPU_SIZE_512K=0x10010 ,//512 ����� 
MPU_SIZE_1M=0x10011 ,//1 ����� 
MPU_SIZE_2M=0x10100 ,//2 ����� 
MPU_SIZE_4M=0x10101 ,//4 ����� 
MPU_SIZE_8M=0x10110 ,//8 ����� 
MPU_SIZE_16M=0x10111 ,//16 ����� 
MPU_SIZE_32M=0x11000 ,//32 ����� 
MPU_SIZE_64M=0x11001 ,//64 ����� 
MPU_SIZE_128M=0x11010 ,//128 ����� 
MPU_SIZE_256M=0x11011 ,//256 ����� 
MPU_SIZE_512M=0x11100 ,//512 ����� 
MPU_SIZE_1G=0x11101 ,//1 ����� 
MPU_SIZE_2G=0x11110 ,//2 ����� 
MPU_SIZE_4G=0x11111  //4 ����� 
}MPU_size_type;
//----------------------------------------------------------------------------//
//��������� �������� �������� ������
void MPU_set_region(uint8_t numRegion,
                    void* mem_region_base_adr,
                    MPU_size_type sizeType,
                    MPU_AP_Type accessType,
                    uint32_t MPUCTRL_MASK,
                    bool bEnableRegion);

//---------------------------------------
void MPU_update_region(uint8_t numRegion,
                        void* mem_region_base_adr,
                        MPU_size_type sizeType,
                        MPU_AP_Type accessType,
                        bool bEnableRegion);

//---------------------------------------        
void MPU_set_ENABLE_region(uint8_t numRegion,bool bEnableRegion);
//---------------------------------------  
uint32_t MPU_get_ENABLE_region(uint8_t numRegion);
//---------------------------------------        
void MPU_protect_region(uint8_t numRegion,
                         void* mem_region_base_adr,
                         MPU_size_type sizeType,
                         MPU_AP_Type accessType);
//----------------------------------------------------------------------------//

//----------------------------------------------------------------------------//
#endif