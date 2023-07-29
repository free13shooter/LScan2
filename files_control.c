/**
  ******************************************************************************
  * @file    ILDA_player.c 
  * @author  MW dem1305
  * @version V1.0.0
  * @date    24-sept-2015
  * @brief   LScan ilda format player with FATFS 
  ******************************************************************************
  */ 

#include "files_control.h"
#include <string.h>
#include "LScan.h"

#pragma data_alignment = 4
FILINFO finf;//fileinfo
TCHAR lfnam[255];
TCHAR strbuff[255];

extern volatile TYPEMODEMASK TMMask;    //маска настроек режимов воспроизведения файлов,LScan.c
// ---------------------------------------------------------------------------------------------------------//
//получить расширение файла
bool getFileExtension(TCHAR* fname,TCHAR* out)
{
  int len=strlen(fname);
  if (!len)return false;
  //вхождение точки
  int p=len-1;
  while(p>=0 && fname[p]!='.'){--p;};
  if(p<0)return false;
  _memcpy(out,fname+p+1,len-p-1);
  return true;
}
// ---------------------------------------------------------------------------------------------------------//
//совпадают ли расширения файлов с одним из расширений 
bool is_ext_matches(TCHAR* fname,TCHAR* pExt,...)
{
  TCHAR ext[8];ZeroMemory(ext,sizeof(TCHAR)*8);//[8];
  TCHAR** ptr=&pExt;
  if(ptr==0 || !getFileExtension(fname,ext))return false;
  while(ptr!=0 && *ptr!=0 && *ptr!="")
  {
    if(strcmp(ext,*ptr)==0)return true;
    ptr++;
  }
  return false;
}
// ---------------------------------------------------------------------------------------------------------//
//получить количество файлов/папок в директори(папке).
//Если pExtension==NULL-расширение любое.
//если pExt=="DIR" - считать папки
int get_numFiles(DIR* cd,TCHAR* pExt,...)
{
  if(!cd)return 0;
  int n=0;
  FILINFO fi;//fileinfo
  fi.fsize=sizeof(fi.fname)/sizeof(TCHAR);
  if(FR_OK!=f_readdir(cd, NULL))return 0;//на начало
    
  while(f_readdir(cd, &fi)==FR_OK)
  {
    if(fi.fname[0]==0)return n;//нет больше объектов
    if(fi.fname[0] == '.')continue;//в родительскую или папка
    if(pExt!=0 && pExt=="DIR" && finf.fattrib & AM_DIR) {n++;continue;}
    else if(pExt==0)n++;//любое
            else{ //проверить,совпадают ли расширения файлов с одним из расширений 
              if(is_ext_matches(fi.fname,pExt))n++; //функция сама пройдет по всем расширениям
            }
  };
  
  return n;
}
//получить количество файлов/папок в директори(папке)с учетом расширения по маске TYPEMODEMASK
int get_numFilesForTypeMask(DIR* cd)
{
  return (TMMask & TM_ILDA)==0?get_numFiles(cd,_T("ilda"),_T("ild"),_T("ILDA"),_T("ILD"),_T("")):get_numFiles(cd,_T("mp3"),_T("MP3"),_T(""));
}
// ---------------------------------------------------------------------------------------------------------//
//получить следующий файл .сканирование от текущей позиции указателя с углублением в подпапки
//circular-по кругу
FRESULT get_next_file(DIR* cd,TCHAR* pbuff,bool circular)
{
  FRESULT fr;
  TCHAR* objname;
  int nfiles;
  goto loop_scan_files;    
  //-------------------FILES--------------------//
beg_scan_files:
  nfiles=get_numFilesForTypeMask(cd);
  if(nfiles==0 && (TMMask & TM_FOLDER_ONE))return FR_NO_FILE;//файлов больше не найдено
  fr=f_readdir(cd, NULL);
  if(fr!=FR_OK)return fr;
//---------------LOOP
loop_scan_files://по файлам
  fr=f_readdir(cd, &finf);
  if(fr!=FR_OK)return fr;
    
  if(finf.fname[0]==0 || ( !(TMMask & TM_FOLDER_ONE) && (TMMask & TM_FOLDER_ANY) && bRnd )) //объекты закончились или не внутри папки и любой файл и вероятность
  {
    if(TMMask & TM_FOLDER_ONE)
    {
      if(circular)goto beg_scan_files;
      else return FR_NO_FILE;//файлов больше не найдено
    }
    
    goto beg_scan_folder;//сканировать по папкам
  }
  
  if (finf.fname[0] == '.')goto loop_scan_files;//текущая
  
  if((finf.fattrib & AM_DIR) || ((TMMask & TM_FILE_RANDOM) && bRnd)/*пропуск по рандом*/) goto loop_scan_files;//папка
  else
  {   
    objname=finf.fname;
    strcpy((void*)pbuff,objname);
    return FR_OK;
  }

//-------------------FOLDERS--------------------//
beg_scan_folder:
  fr=f_readdir(cd, NULL);//на начало
  if(fr!=FR_OK)return fr;
//---------------LOOP 
loop_scan_folder:
  
  fr=f_readdir(cd, &finf);if(fr!=FR_OK)return fr;
    
  if(finf.fname[0]==0)//нет объектов.необходимо перейти в родительскую
  {
    int r=is_root();if(r==-1)return FR_INT_ERR;  
    
    if(r==0)//не корневая,переход в родительскую
    {
      static TCHAR cfld[_MAX_LFN + 1]=_T("");
      fr=cfld_name(cfld);if(fr!=FR_OK)return fr;
      
      fr=f_chdir(_T("1:.."));
      if(fr!=FR_OK)return fr;
      //перемотка до тек.папки
      fr=f_opendir(cd,_T("1:."));if(fr!=FR_OK)return fr;
      fr=rewind_and_get_prev(cd,cfld,NULL);if(fr!=FR_OK)return fr;
     
      goto loop_scan_folder;//продолжить со следующей папки
    }
    else//IS ROOT
    {
      if(circular)
      {
        fr=f_opendir(cd,_T("/"));if(fr!=FR_OK)return fr;
        fr=f_readdir(cd, NULL);if(fr!=FR_OK)return fr;//на начало
        goto beg_scan_files;//начать все заново
      }
      
      return FR_NO_FILE;//сканирование полностью завершено(root)/файлов больше не найдено
    }//if(r==0){...}else{...}
  }// if(finf.fname[0]==0)
  
  //проверка объекта
  if (finf.fname[0] == '.')goto loop_scan_folder;
  
  objname=finf.fname;
  
  if((finf.fattrib & AM_DIR) && !((TMMask & TM_FILE_RANDOM) && bRnd)/*рандом * вероятность == пропуск*/) 
  {   
    //strcat(fpath,objname);
    fr=f_opendir(cd,objname);//открыть вложенную папку
    if(fr!=FR_OK)return fr;
    
    fr=f_chdir(objname);//сменить текущую папку
    if(fr!=FR_OK)return fr;
    
    goto beg_scan_files;//сканируем вложенную папку на файлы 
  }
  //файл,ищем папку
  goto loop_scan_folder;
//-----------------------------------------------  
}
// ---------------------------------------------------------------------------------------------------------//
int is_root()//корневая ли текущая папка(вернет 1,если да, иначе 0. -1==error)
{
  FRESULT fr=f_getcwd (strbuff,255 );
  if(fr!=FR_OK )return -1;//ошибка откр. директория
  
  const TCHAR *tt;
  const TCHAR* path=strbuff;
  
  for (tt = path; (UINT)*tt >= (_USE_LFN ? ' ' : '!') && *tt != ':'; tt++) ;	/* Find ':' in the path */
  if (*tt == ':')path=++tt;
  
  if(_strcmp(path,_T("/"))==0)return 1;//+terminator
  return 0;
}
// ---------------------------------------------------------------------------------------------------------//
TCHAR* fs_obj_name(const TCHAR* path)//получить имя по пути
{
  char* s=strrchr(path,'/') ; 
  if(s==NULL)return (TCHAR*)((UINT)path+(UINT)_strlen(path));//==""==terminator
  ++s;
  return (TCHAR*)s;
}
// ---------------------------------------------------------------------------------------------------------//
FRESULT cfld_name(const TCHAR* outbuff)//получить имя текущей папки
{
  TCHAR* outbf=(TCHAR*)outbuff;//продлить время жизни
  FRESULT fr=f_getcwd (strbuff,_MAX_LFN + 1);
  if(fr!=FR_OK )return fr;//ошибка откр. директория
  TCHAR* nm=fs_obj_name(strbuff);//получить имя объекта по пути
  strcpy(outbf,nm);
  return FR_OK;
}
// ---------------------------------------------------------------------------------------------------------//
/* Copy memory to memory */

void _memcpy (void* dst, const void* src, UINT cnt) {
	BYTE *d = (BYTE*)dst;
	const BYTE *s = (const BYTE*)src;

#if _WORD_ACCESS == 1
	while (cnt >= sizeof (int)) {
		*(int*)d = *(int*)s;
		d += sizeof (int); s += sizeof (int);
		cnt -= sizeof (int);
	}
#endif
	while (cnt--)
		*d++ = *s++;
}

/* Fill memory */

void _memset (void* dst, int val, UINT cnt) {
	BYTE *d = (BYTE*)dst;

	while (cnt--)
		*d++ = (BYTE)val;
}

/* Compare memory to memory */

int _memcmp (const void* dst, const void* src, UINT cnt) {
	const BYTE *d = (const BYTE *)dst, *s = (const BYTE *)src;
	int r = 0;

	while (cnt-- && (r = *d++ - *s++) == 0) ;
	return r;
}

/* string len in TCHAR */

int _strlen (const TCHAR* src)
{
  const TCHAR *s = src;
  int cnt=0;
  
  while ( 0 != (const BYTE *)*s++){cnt++;} ;
  return cnt;
}  
/* string compare */

int _strcmp (const TCHAR* src1,const TCHAR* src2)
{
        const BYTE *s1 =(const BYTE *) src1, *s2 =(const BYTE *) src2;
	int r = 0;
        
	while ((0 != *s1)&&(0 != *s2)) 
        {
          for(int n=0;n<sizeof(TCHAR);n++)
          {
            r=*s1++ - *s2++;
            if(r!=0)return r;
          }
        };
        
        if( ((0 == *s1)&&(0 != *s2))||((0 == *s2)&&(0 != *s1)) )return 1;//разная длина
	return r;
}
// ---------------------------------------------------------------------------------------------------------//
//получить предыдущий файл .сканирование от текущей позиции указателя с выходом из подпапки.circular-по кругу
FRESULT get_prev_file(DIR* cd,const TCHAR* curobj,TCHAR* pbuff,bool circular)
{
  static TCHAR tbuf[_MAX_LFN + 1]=_T("");
  int d,nfiles;
  FRESULT fr;
  TCHAR* outbf=(TCHAR*)pbuff;
  strcpy(tbuf,curobj);
  if(_strcmp(tbuf,_T("."))==0 || _strcmp(tbuf,_T(".."))==0 || _strcmp(tbuf,_T(""))==0)goto lp_scan_fldr_check;
  d=is_dir(tbuf);if(d==-1)return FR_INT_ERR;
  if(d==0)goto lp_scan_fls;//не папка,сканировать на файлы дальше
  if(!(TMMask & TM_FOLDER_ONE))goto lp_scan_fldr;//скан папок
//---------------FILES---------------
beg_scan_fls: //в обратном порядке,от конца
  nfiles=get_numFilesForTypeMask(cd);
  if(nfiles==0 && (TMMask & TM_FOLDER_ONE))return FR_NO_FILE;//файлов больше не найдено
  //установить на последний объект
  set_to_lst_obj(cd,tbuf);if(fr!=FR_OK)return fr;
  
lp_scan_fls:
  strcpy(strbuff,tbuf);
  fr=rewind_and_set_prev(cd,strbuff,tbuf);//установить указатель на предыдущий объект
  if(fr!=FR_OK)return fr;
  
  if(_strcmp(tbuf,_T("."))==0 || _strcmp(tbuf,_T(".."))==0 || _strcmp(tbuf,_T(""))==0 || //больше нет.по папкам 
     ( !(TMMask & TM_FOLDER_ONE) && (TMMask & TM_FOLDER_ANY) && bRnd ) ) //или не внутри папки и любой файл и вероятность
  {
    if(TMMask & TM_FOLDER_ONE)//только внутри папки
    {
      if(circular)goto beg_scan_fls;
        else return FR_NO_FILE;//файлов больше не найдено
    }
    
    goto lp_scan_fldr_check;
  }
  
  d=is_dir(tbuf);if(d==-1)return FR_INT_ERR;
  
  if(d==0 && (!((TMMask & TM_FILE_RANDOM) && bRnd))/*не (рандом и пропуск)*/ ){strcpy(outbf,tbuf); return FR_OK;}//файл найден
  
  goto lp_scan_fls;//папка,сканировать на файлы дальше

//---------------FOLDERS---------------
beg_lp_scan_fldr:
  fr=cfld_name(tbuf);if(fr!=FR_OK)return fr;
  fr=f_opendir(cd,_T("."));if(fr!=FR_OK)return fr;
  //установить на последний объект
  fr=set_to_lst_obj(cd,tbuf);if(fr!=FR_OK)return fr;
    
lp_scan_fldr:
  strcpy(strbuff,tbuf);
  fr=rewind_and_set_prev(cd,strbuff,tbuf);//установить указатель на предыдущий объект
  if(fr!=FR_OK)return fr;
  if(_strcmp(tbuf,_T("."))==0 || _strcmp(tbuf,_T(".."))==0 || _strcmp(tbuf,_T(""))==0)
  {
    goto beg_scan_fls;//по файлам
  }
  d=is_dir(tbuf);if(d==-1)return FR_INT_ERR;
  if(d==0)goto lp_scan_fldr;//дальше
  goto beg_lp_scan_fldr;//найдена папка,открыть
  
lp_scan_fldr_check:    
  if(_strcmp(tbuf,_T("."))==0 || _strcmp(tbuf,_T(".."))==0 || _strcmp(tbuf,_T(""))==0)//предыдущего нет в текущей папке
  {
    int r=is_root();if(r==-1)return FR_INT_ERR;  
    if(r==0)//не корневая,переход в родительскую
    {
      TCHAR cfld[255]=_T("");
      fr=cfld_name(cfld);if(fr!=FR_OK)return fr;
      fr=f_chdir(_T(".."));if(fr!=FR_OK)return fr;
      fr=f_opendir(cd,_T("."));if(fr!=FR_OK)return fr;
      //установить указатель на предыдущий объект папки
      fr=rewind_and_set_prev(cd,cfld,tbuf);if(fr!=FR_OK)return fr;
      if(_strcmp(tbuf,_T("."))==0 || _strcmp(tbuf,_T(".."))==0 || _strcmp(tbuf,_T(""))==0)
      {
        goto beg_scan_fls;
      }
      d=is_dir(tbuf);if(d==-1)return FR_INT_ERR;
      if(d==0)goto lp_scan_fldr;//продолжить поиск папок
      goto beg_lp_scan_fldr;//вход в папку
    }
    else//IS ROOT
    {
      if(circular)
      {
        FRESULT fr=f_readdir(cd, NULL);//на начало
        if(fr!=FR_OK)return fr;
        return get_next_file(cd,(TCHAR*)pbuff,circular);
      }
      
      return FR_NO_FILE;//сканирование полностью завершено(root)/файлов больше не найдено
    }//if(r==0){...}else{...}
  }// if(tbuf==_T(".")....)
  else
  {
    //войти и сканировать на паки
    goto beg_lp_scan_fldr;
  }
}
// ---------------------------------------------------------------------------------------------------------//
//перемотка от начала директории до текущего объекта и получить имя предыдущего объекта,если outbuf не NULL
FRESULT rewind_and_get_prev(DIR* cd,const TCHAR* inbuf,TCHAR* outbuf) 
{
  FRESULT fr=f_readdir(cd, NULL);if(fr!=FR_OK)return fr;//на начало
  TCHAR* outbf=outbuf;
  finf.fname[0]=0;
  TCHAR* objname=_T("");
  if(outbuf!=NULL)outbuf[0]=0;
        
  do
  {
    if(outbuf)//скопировать последний 
    {
      strcpy(outbf,objname);
    }
    fr=f_readdir(cd, &finf);if(fr!=FR_OK)return fr;
    if(finf.fname[0]==0)break;
    objname=finf.fname;
  }
  while(_strcmp(objname,inbuf)!=0);
  
  return fr;
}
// ---------------------------------------------------------------------------------------------------------//
//перемотка от начала директории до предыдущего объекта и получить имя предыдущего объекта,если buff не NULL
FRESULT rewind_and_set_prev(DIR* cd,const TCHAR* curobj,TCHAR* buff) 
{
  const TCHAR* outbf=(const TCHAR*)buff;
  const TCHAR* cbf=(const TCHAR*)curobj;
  TCHAR btemp[_MAX_LFN + 1];
  
  FRESULT fr=rewind_and_get_prev(cd,cbf,btemp);if(fr!=FR_OK)return fr;//перемотка до текущего и получение имени
  fr=rewind_and_get_prev(cd,btemp,NULL);if(fr!=FR_OK)return fr;//перемотка до предыдущего
  
  if(buff)strcpy((void*)outbf,btemp);
  
  return fr;
}
// ---------------------------------------------------------------------------------------------------------//
int is_dir(const TCHAR* curobj)//-1 при ошибке
{
  if(_strcmp(curobj,_T("."))==0 || _strcmp(curobj,_T(".."))==0)return 1;
  if(_strcmp(curobj,_T(""))==0)return 0;
  FRESULT fr=f_stat(curobj,&finf);if(fr!=FR_OK)return -1;
  if(finf.fattrib & AM_DIR)return 1;
 
  return 0;
}
// ---------------------------------------------------------------------------------------------------------//
//установить указатель на последний объект папки и получить имя последнего объекта,если buff не NULL
FRESULT set_to_lst_obj(DIR* cd,TCHAR* buff)
{
  TCHAR* outbf=buff;
  FRESULT fr=f_readdir(cd, NULL);//на начало
  if(fr!=FR_OK)return fr;
  finf.fname[0]=0;
  TCHAR* objname=_T("");
  if(buff!=NULL)buff[0]=0;
  
  
  //пролистать все
  do
  {
    if(buff)//скопировать последний 
    {
      objname=finf.fname;
      strcpy(outbf,objname);
    }
    fr=f_readdir(cd, &finf);if(fr!=FR_OK)return fr;
  }
  while(finf.fname[0]!=0);
        
  fr=rewind_and_get_prev(cd,_T(""),NULL);if(fr!=FR_OK)return fr;
    
  return fr;
}
// ---------------------------------------------------------------------------------------------------------//