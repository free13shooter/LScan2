/**
  Laser project by dem1305

  @2018
  
  защита памяти
*/
//----------------------------------------------------------------------------//
#ifndef __MPU_H
#define __MPU_H
//----------------------------------------------------------------------------//
#include "core.h"
//----------------------------------------------------------------------------//
#define PRIVDEFENA  4 //Разрешение доступа к карте памяти (фон) в привилеrированном режиме. 
#define HFNMIENA    2 //разрешает использование MPU в обработчиках NMI и исключения Hard Fault. 
#define MPUENABLE   1

#define MPUVALID    16 //номер области MPU будет определяться полем REGION регистра RBAR

//атрибуты памяти
#define MPU_S              (((uint32_t)1)<<18) //SHARED    MEMORY  разделяемая
#define MPU_C              (((uint32_t)1)<<17) //CACHEABLE MEMORY  кешируемая
#define MPU_B              (((uint32_t)1)<<16) //BUFFERED  MEMORY  буферизируемая
#define MPU_REGION_ENABLE  ((uint32_t)1) //регион разрешен
#define MPU_REGION_DISABLE (~MPU_REGION_ENABLE)

// AP_привилегированный_непривилегированный доступы
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
АР  Привилеrированный   Пользовательский   Описание 
   
000 Доступ запрещён     Доступ запрещён   Доступ запрещён 
001 Чтение/запись       Доступ запрещён   Доступ только на привилеrированном уровне 
010 Чтение/запись       Только чтение     Запись в пользовательской проrрамме вызовет отказ 
011 Чтение/запись       Чтение/запись     Полный доступ 
100 Непредсказуемо      Непредсказуемо    Непредсказуемо 
101 Только чтение       Доступ запрещён   Только чтение на привилеrированном уровне 
110 Только чтение       Только чтение     Только чтение 
111 Только чтение       Только чтение     Только чтение 
*/

/* Размер области памяти,для конфигурации MPU
0x00000 Зарезервировано 
0x00001 Зарезервировано 
0x00010 Зарезервировано 
0x00011 /Зарезервировано 
*/
typedef enum{
MPU_SIZE_32=0x00100 ,//32 байта 
MPU_SIZE_64=0x00101 ,//64 байта 
MPU_SIZE_128=0x00110 ,//128 байт 
MPU_SIZE_256=0x00111 ,//256 байт 
MPU_SIZE_512=0x01000 ,//512 байт 
MPU_SIZE_1K=0x01001 ,//1 Кбайт 
MPU_SIZE_2K=0x01010 ,//2 Кбайт 
MPU_SIZE_4K=0x01011 ,//4 Кбайт 
MPU_SIZE_8K=0x01100 ,//8 Кбайт 
MPU_SIZE_16K=0x01101 ,//16 Кбайт 
MPU_SIZE_32K=0x01110 ,//32 Кбайт 
MPU_SIZE_64K=0x01111 ,//64 Кбайт 
MPU_SIZE_128K=0x10000 ,//128 Кбайт 
MPU_SIZE_256K=0x10001 ,//256 Кбайт 
MPU_SIZE_512K=0x10010 ,//512 Кбайт 
MPU_SIZE_1M=0x10011 ,//1 Мбайт 
MPU_SIZE_2M=0x10100 ,//2 Мбайт 
MPU_SIZE_4M=0x10101 ,//4 Мбайт 
MPU_SIZE_8M=0x10110 ,//8 Мбайт 
MPU_SIZE_16M=0x10111 ,//16 Мбайт 
MPU_SIZE_32M=0x11000 ,//32 Мбайт 
MPU_SIZE_64M=0x11001 ,//64 Мбайт 
MPU_SIZE_128M=0x11010 ,//128 Мбайт 
MPU_SIZE_256M=0x11011 ,//256 Мбайт 
MPU_SIZE_512M=0x11100 ,//512 Мбайт 
MPU_SIZE_1G=0x11101 ,//1 Гбайт 
MPU_SIZE_2G=0x11110 ,//2 Гбайт 
MPU_SIZE_4G=0x11111  //4 Гбайт 
}MPU_size_type;
//----------------------------------------------------------------------------//
//настройка доступов регионов памяти
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