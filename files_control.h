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

//получить расширение файла
bool getFileExtension(TCHAR* fname,TCHAR* out);
//совпадают ли расширения файлов с одним из расширений 
bool is_ext_matches(TCHAR* fname,TCHAR* pExt,...);
//получить количество файлов/папок в директори(папке).
//Если pExtension==NULL-расширение любое.
//если pExt=="DIR" - считать папки
int get_numFiles(DIR* cd,TCHAR* pExt,...);
//получить количество файлов/папок в директори(папке)с учетом расширения по маске TYPEMODEMASK
int get_numFilesForTypeMask(DIR* cd);
           
int is_root();//корневая ли текущая папка(вернет 1,если да, иначе 0. -1==error)
TCHAR* fs_obj_name(const TCHAR* path);//получить имя по пути
FRESULT cfld_name(const TCHAR* outbuff);//получить имя текущей папки

//получить следующий файл .сканирование от текущей позиции указателя с углублением в подпапки.circular-по кругу
FRESULT get_next_file(DIR* cd,TCHAR* pbuff,bool circular);
//получить предыдущий файл .сканирование от текущей позиции указателя с выходом из подпапки.circular-по кругу
FRESULT get_prev_file(DIR* cd,const TCHAR* curobj,TCHAR* pbuff,bool circular);
//перемотка от начала директории до текущего объекта и получить имя предыдущего объекта,если outbuf не NULL
FRESULT rewind_and_get_prev(DIR* cd,const TCHAR* inbuf,TCHAR* outbuf);
//перемотка от начала директории до предыдущего объекта и получить имя предыдущего объекта,если buff не NULL
FRESULT rewind_and_set_prev(DIR* cd,const TCHAR* curobj, TCHAR* buff);
int is_dir(const TCHAR* curobj);//-1 при ошибке
//установить указатель на последний объект папки и получить имя последнего объекта,если buff не NULL
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