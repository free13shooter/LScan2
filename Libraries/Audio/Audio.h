/**
  ******************************************************************************
  * @file    Audio.h 
  * @author  MW dem1305
  * @version V1.0.0
  * @date    8-Nov-2016
  ******************************************************************************
  */ 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef  __AUDIO_H 
#define  __AUDIO_H 
/* Includes ------------------------------------------------------------------*/
#include "utils.h"
#include "mp3dec.h"
#include "assembly.h"
#include "diskio.h"
#include "LScan.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

//----------------------------------------------------------------------------//
typedef bool Aud_ty_cb(short* buffer,void* context);//callback for audio timer
//----------------------------------------------------------------------------//
extern bool disableAudio;
extern float SoundGain;

#define _AudioOn()  TIM_Cmd(TIM5,ENABLE)   // ��������� ���
#define _AudioOff() TIM_Cmd(TIM5,DISABLE)  //���������� ���
#define _SetAudioVolume(int_volume) SetSoundGain((float)(int_volume/255.0f))//0-255

//----------------------------------------------------------------------------//
void OutputAudioSample(int16_t sample);
void OutputAudioSampleWithoutBlocking(int16_t sample);
void StopAudio();

FRESULT PlayMP3File(TCHAR* filename);//������������� ���� (���������� ����� ����������)
//������ �� ����� [� ��������������� ���������,���� �� 0]
FRESULT ReadFromFileWithAbsOffset(FIL* fp,uint8_t* outBuffer,UINT abs_offset,UINT* rd_bytes);
//----------------------------------------------------------------------------//
void TIM4_IRQHandler(); //���������� ������� ������� �����/��������� PCM 
//----------------------------------------------------------------------------//
#define SOUND_PWM_LEN     875           //����� ��� (��� 96 ��� 84���/96���=875)
#define SOUND_PWM_CENTER  (SOUND_PWM_LEN/2) //����� ���
#define AUDIO_16BIT_DIV   (32768/SOUND_PWM_CENTER+1) //�������� ����� ��� 16-������ ������
#define SetSoundGain(fgain) SoundGain=(fgain) //������ ��������(�������� ��������� ��� ������������ ���)
void SetSoundVal(float val);//������ ������� ��������� ������� 0.0f- +/-1.0f �� ���������
void Sound_Config(void);//������������ ����� , PA3,TIM5 CH4 PWM
void Beep(uint16_t tone,uint16_t timeMs);
void BeepPause(uint16_t timeMs);//������� �������� ����� ������������� timeMks �����������
void BeepMelody(uint16_t* MelodyArray,uint16_t notesTotal);//��������� �������
//----------------------------------------------------------------------------//
#endif /* __AUDIO_H */
/******************* (C) COPYRIGHT 2016 dem1305 *****END OF FILE****/