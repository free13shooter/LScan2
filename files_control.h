/**
  ******************************************************************************
  * @file    ILDA_player.c 
  * @author  MW dem1305
  * @version V1.0.0
  * @date    15-sept-2015
  * @brief   LScan ilda format player with FATFS 
  ******************************************************************************
  */ 

#ifndef __FILES_CTRL
#define __FILES_CTRL
//_________________________________________________________________________________________________

#include "ff.h"   //FATFS
#include "LScan_types.h"

//�������� ���������� �����
bool getFileExtension(TCHAR* fname,TCHAR* out);
//��������� �� ���������� ������ � ����� �� ���������� 
bool is_ext_matches(TCHAR* fname,TCHAR* pExt,...);
//�������� ���������� ������/����� � ���������(�����).
//���� pExtension==NULL-���������� �����.
//���� pExt=="DIR" - ������� �����
int get_numFiles(DIR* cd,TCHAR* pExt,...);
//�������� ���������� ������/����� � ���������(�����)� ������ ���������� �� ����� TYPEMODEMASK
int get_numFilesForTypeMask(DIR* cd);
           
int is_root();//�������� �� ������� �����(������ 1,���� ��, ����� 0. -1==error)
TCHAR* fs_obj_name(const TCHAR* path);//�������� ��� �� ����
FRESULT cfld_name(const TCHAR* outbuff);//�������� ��� ������� �����

//�������� ��������� ���� .������������ �� ������� ������� ��������� � ����������� � ��������.circular-�� �����
FRESULT get_next_file(DIR* cd,TCHAR* pbuff,bool circular);
//�������� ���������� ���� .������������ �� ������� ������� ��������� � ������� �� ��������.circular-�� �����
FRESULT get_prev_file(DIR* cd,const TCHAR* curobj,TCHAR* pbuff,bool circular);
//��������� �� ������ ���������� �� �������� ������� � �������� ��� ����������� �������,���� outbuf �� NULL
FRESULT rewind_and_get_prev(DIR* cd,const TCHAR* inbuf,TCHAR* outbuf);
//��������� �� ������ ���������� �� ����������� ������� � �������� ��� ����������� �������,���� buff �� NULL
FRESULT rewind_and_set_prev(DIR* cd,const TCHAR* curobj, TCHAR* buff);
int is_dir(const TCHAR* curobj);//-1 ��� ������
//���������� ��������� �� ��������� ������ ����� � �������� ��� ���������� �������,���� buff �� NULL
FRESULT set_to_lst_obj(DIR* cd,TCHAR* buff);

// -----------------------------------------------------------------------------------------------//
/* Copy memory to memory */
void _memcpy (void* dst, const void* src, UINT cnt);
/* Fill memory */
void _memset (void* dst, int val, UINT cnt);
/* Compare memory to memory */
int _memcmp (const void* dst, const void* src, UINT cnt);
/* string len in TCHAR */
int _strlen (const TCHAR* src);
/* string compare */
int _strcmp (const TCHAR* src1,const TCHAR* src2);
// -----------------------------------------------------------------------------------------------//
//_________________________________________________________________________________________________

#endif //__FILES_CTRL