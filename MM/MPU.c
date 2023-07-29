/**
  Laser project by dem1305

  @2017
  
  OS core
*/
//----------------------------------------------------------------------------//
#include "MPU.h"
//----------------------------------------------------------------------------//
//настройка доступов регионов памяти
void MPU_set_region(uint8_t numRegion,
                    void* mem_region_base_adr,
                    MPU_size_type sizeType,
                    MPU_AP_Type accessType,
                    uint32_t MPUCTRL_MASK,
                    bool bEnableRegion)
{
  MPU->CTRL = 0; 
  MPU->RNR  = (uint32_t)(numRegion & 7);
  MPU->RBAR = ( ((uint32_t)mem_region_base_adr)&((uint32_t)~0x1F) );
  MPU->RASR = (((uint32_t)accessType)<<24) |  (MPU_S) | (MPU_B) | (((uint32_t)sizeType)<<1)
    | (bEnableRegion?MPU_REGION_ENABLE:0);
  MPU->CTRL = (uint32_t)MPUCTRL_MASK; // Реrистр управления MPU 
  
  __asm("   ISB \n \
            DSB \n");
}
//---------------------------------------
void MPU_update_region( uint8_t numRegion,
                         void* mem_region_base_adr,
                         MPU_size_type sizeType,
                         MPU_AP_Type accessType,
                         bool bEnableRegion)
{
  //uint32_t ctrl=MPU->CTRL;
  //MPU->CTRL=0;
  MPU->RNR  = (uint32_t)(numRegion & 7);
  MPU->RASR&=~MPU_REGION_ENABLE;
  MPU->RBAR = ( ((uint32_t)mem_region_base_adr)&((uint32_t)~0x1F) );
  MPU->RASR = (((uint32_t)accessType)<<24) | (MPU_S) | (MPU_B) | (((uint32_t)sizeType)<<1)
    | (bEnableRegion?MPU_REGION_ENABLE:0);
  //MPU->CTRL=ctrl;
  __asm("   ISB \n \
            DSB \n");
}
//---------------------------------------        
void MPU_set_ENABLE_region( uint8_t numRegion,bool bEnableRegion)
{
  MPU->RNR  = (uint32_t)(numRegion & 7);
  if(bEnableRegion)MPU->RASR|=MPU_REGION_ENABLE;
  else MPU->RASR&=~MPU_REGION_ENABLE;
  
  __asm("   ISB \n \
            DSB \n");
}
//---------------------------------------  
uint32_t MPU_get_ENABLE_region( uint8_t numRegion)
{
  MPU->RNR  = (uint32_t)(numRegion & 7);
  return (MPU->RASR & MPU_REGION_ENABLE);
}
      
