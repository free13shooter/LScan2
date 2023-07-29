//----------------------------------------------------------------------------//
#ifndef __SPIWLAN_H
#define __SPIWLAN_H

//----------------------------------------------------------------------------//
#include "LScan.h"
#include "stm32f4xx_spi.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
//----------------------------------------------------------------------------//
#define WCMD_FREE             1         //������ ���������� ����� � �������� ������ 
#define WCMD_SAMPLESIZE       2         //����������� ������ ������ ��������
#define WCMD_STREAM_BLOCKSIZE 4         //����������� ������ ������ �������� DMA
#define WCMD_PING	      8		//����
#define WCMD_BUFSIZE	      16	//�������� ������ ��������� ������
#define WCMD_PREP_DMA	      32	//����������� DMA ���������� � �����
#define WCMD_STREAM	      64	//����������� � ����� ������
#define WCMD_STA_SSID         128       //��������� STA SSID
#define WCMD_STA_PASS         129       //��������� STA PASS
#define WCMD_STA_RECONNECT    130       //��������������� STA SSID
#define WCMD_STA_GET_PAR      131       //������ ���������� IP STA SSID (������) //{IP(32 bit) GATEWAY(32 bit) MASK(32 bit) PORT(16 bit)}
#define WCMD_STA_BRO_PAR      132       //������ ���������� IP ����� broadcast
//----------------------------------------------------------------------------//

//#define SPI1_ENABLE_CRC 

#define SPI1_DMA_HUNG_DETECT_TICKS  -1 //30*1000 // ������������ ����������� �� �����������������

#define SPI1_CS_PIN               GPIO_Pin_7
#define SPI1_CS_PORT              GPIOE
#define SPI1_CS_EXTI_LINE         EXTI_Line7
#define SPI1_CS_EXTI_PORT         EXTI_PortSourceGPIOE 
#define SPI1_CS_EXTI_PIN          EXTI_PinSource7
#define SPI1_CS_EXTI_IRQ          EXTI9_5_IRQn //EXTI15_10_IRQn
#define SPI1_CS_EXTI_IRQHandler   EXTI9_5_IRQHandler //EXTI15_10_IRQHandler

#define SPI1_HANDSHAKE_PIN   GPIO_Pin_11
#define SPI1_HANDSHAKE_PORT  GPIOB

#define	SPI1_HAND_LOW 	SPI1_HANDSHAKE_PORT->BSRRH = SPI1_HANDSHAKE_PIN
#define SPI1_HAND_HIGH 	SPI1_HANDSHAKE_PORT->BSRRL = SPI1_HANDSHAKE_PIN
//--------------------------------------------------------------
#define SPI1_DMA_IRQn_PRIO      1 //���������(������)
#define SPI1_DMA_IRQn_PRIO_SUB  3 //������������ > playtimer

#define SPI1_DMA_RX_IRQHandler   DMA2_Stream2_IRQHandler
#define SPI1_DMA_TX_IRQHandler   DMA2_Stream3_IRQHandler
//----------------------------------------------------------------------------//
void SPI_WLAN_init(bool newThread);
void SPI_WLAN_deinit();
//----------------------------------------------------------------------------//
#endif /* __SPIWLAN_H */

