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

extern volatile TYPEMODEMASK TMMask;    //����� �������� ������� ��������������� ������,LScan.c
// ---------------------------------------------------------------------------------------------------------//
//�������� ���������� �����
bool getFileExtension(TCHAR* fname,TCHAR* out)
{
  int len=strlen(fname);
  if (!len)return false;
  //��������� �����
  int p=len-1;
  while(p>=0 && fname[p]!='.'){--p;};
  if(p<0)return false;
  _memcpy(out,fname+p+1,len-p-1);
  return true;
}
// ---------------------------------------------------------------------------------------------------------//
//��������� �� ���������� ������ � ����� �� ���������� 
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
//�������� ���������� ������/����� � ���������(�����).
//���� pExtension==NULL-���������� �����.
//���� pExt=="DIR" - ������� �����
int get_numFiles(DIR* cd,TCHAR* pExt,...)
{
  if(!cd)return 0;
  int n=0;
  FILINFO fi;//fileinfo
  fi.fsize=sizeof(fi.fname)/sizeof(TCHAR);
  if(FR_OK!=f_readdir(cd, NULL))return 0;//�� ������
    
  while(f_readdir(cd, &fi)==FR_OK)
  {
    if(fi.fname[0]==0)return n;//��� ������ ��������
    if(fi.fname[0] == '.')continue;//� ������������ ��� �����
    if(pExt!=0 && pExt=="DIR" && finf.fattrib & AM_DIR) {n++;continue;}
    else if(pExt==0)n++;//�����
            else{ //���������,��������� �� ���������� ������ � ����� �� ���������� 
              if(is_ext_matches(fi.fname,pExt))n++; //������� ���� ������� �� ���� �����������
            }
  };
  
  return n;
}
//�������� ���������� ������/����� � ���������(�����)� ������ ���������� �� ����� TYPEMODEMASK
int get_numFilesForTypeMask(DIR* cd)
{
  return (TMMask & TM_ILDA)==0?get_numFiles(cd,_T("ilda"),_T("ild"),_T("ILDA"),_T("ILD"),_T("")):get_numFiles(cd,_T("mp3"),_T("MP3"),_T(""));
}
// ---------------------------------------------------------------------------------------------------------//
//�������� ��������� ���� .������������ �� ������� ������� ��������� � ����������� � ��������
//circular-�� �����
FRESULT get_next_file(DIR* cd,TCHAR* pbuff,bool circular)
{
  FRESULT fr;
  TCHAR* objname;
  int nfiles;
  goto loop_scan_files;    
  //-------------------FILES--------------------//
beg_scan_files:
  nfiles=get_numFilesForTypeMask(cd);
  if(nfiles==0 && (TMMask & TM_FOLDER_ONE))return FR_NO_FILE;//������ ������ �� �������
  fr=f_readdir(cd, NULL);
  if(fr!=FR_OK)return fr;
//---------------LOOP
loop_scan_files://�� ������
  fr=f_readdir(cd, &finf);
  if(fr!=FR_OK)return fr;
    
  if(finf.fname[0]==0 || ( !(TMMask & TM_FOLDER_ONE) && (TMMask & TM_FOLDER_ANY) && bRnd )) //������� ����������� ��� �� ������ ����� � ����� ���� � �����������
  {
    if(TMMask & TM_FOLDER_ONE)
    {
      if(circular)goto beg_scan_files;
      else return FR_NO_FILE;//������ ������ �� �������
    }
    
    goto beg_scan_folder;//����������� �� ������
  }
  
  if (finf.fname[0] == '.')goto loop_scan_files;//�������
  
  if((finf.fattrib & AM_DIR) || ((TMMask & TM_FILE_RANDOM) && bRnd)/*������� �� ������*/) goto loop_scan_files;//�����
  else
  {   
    objname=finf.fname;
    strcpy((void*)pbuff,objname);
    return FR_OK;
  }

//-------------------FOLDERS--------------------//
beg_scan_folder:
  fr=f_readdir(cd, NULL);//�� ������
  if(fr!=FR_OK)return fr;
//---------------LOOP 
loop_scan_folder:
  
  fr=f_readdir(cd, &finf);if(fr!=FR_OK)return fr;
    
  if(finf.fname[0]==0)//��� ��������.���������� ������� � ������������
  {
    int r=is_root();if(r==-1)return FR_INT_ERR;  
    
    if(r==0)//�� ��������,������� � ������������
    {
      static TCHAR cfld[_MAX_LFN + 1]=_T("");
      fr=cfld_name(cfld);if(fr!=FR_OK)return fr;
      
      fr=f_chdir(_T("1:.."));
      if(fr!=FR_OK)return fr;
      //��������� �� ���.�����
      fr=f_opendir(cd,_T("1:."));if(fr!=FR_OK)return fr;
      fr=rewind_and_get_prev(cd,cfld,NULL);if(fr!=FR_OK)return fr;
     
      goto loop_scan_folder;//���������� �� ��������� �����
    }
    else//IS ROOT
    {
      if(circular)
      {
        fr=f_opendir(cd,_T("/"));if(fr!=FR_OK)return fr;
        fr=f_readdir(cd, NULL);if(fr!=FR_OK)return fr;//�� ������
        goto beg_scan_files;//������ ��� ������
      }
      
      return FR_NO_FILE;//������������ ��������� ���������(root)/������ ������ �� �������
    }//if(r==0){...}else{...}
  }// if(finf.fname[0]==0)
  
  //�������� �������
  if (finf.fname[0] == '.')goto loop_scan_folder;
  
  objname=finf.fname;
  
  if((finf.fattrib & AM_DIR) && !((TMMask & TM_FILE_RANDOM) && bRnd)/*������ * ����������� == �������*/) 
  {   
    //strcat(fpath,objname);
    fr=f_opendir(cd,objname);//������� ��������� �����
    if(fr!=FR_OK)return fr;
    
    fr=f_chdir(objname);//������� ������� �����
    if(fr!=FR_OK)return fr;
    
    goto beg_scan_files;//��������� ��������� ����� �� ����� 
  }
  //����,���� �����
  goto loop_scan_folder;
//-----------------------------------------------  
}
// ---------------------------------------------------------------------------------------------------------//
int is_root()//�������� �� ������� �����(������ 1,���� ��, ����� 0. -1==error)
{
  FRESULT fr=f_getcwd (strbuff,255 );
  if(fr!=FR_OK )return -1;//������ ����. ����������
  
  const TCHAR *tt;
  const TCHAR* path=strbuff;
  
  for (tt = path; (UINT)*tt >= (_USE_LFN ? ' ' : '!') && *tt != ':'; tt++) ;	/* Find ':' in the path */
  if (*tt == ':')path=++tt;
  
  if(_strcmp(path,_T("/"))==0)return 1;//+terminator
  return 0;
}
// ---------------------------------------------------------------------------------------------------------//
TCHAR* fs_obj_name(const TCHAR* path)//�������� ��� �� ����
{
  char* s=strrchr(path,'/') ; 
  if(s==NULL)return (TCHAR*)((UINT)path+(UINT)_strlen(path));//==""==terminator
  ++s;
  return (TCHAR*)s;
}
// ---------------------------------------------------------------------------------------------------------//
FRESULT cfld_name(const TCHAR* outbuff)//�������� ��� ������� �����
{
  TCHAR* outbf=(TCHAR*)outbuff;//�������� ����� �����
  FRESULT fr=f_getcwd (strbuff,_MAX_LFN + 1);
  if(fr!=FR_OK )return fr;//������ ����. ����������
  TCHAR* nm=fs_obj_name(strbuff);//�������� ��� ������� �� ����
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
        
        if( ((0 == *s1)&&(0 != *s2))||((0 == *s2)&&(0 != *s1)) )return 1;//������ �����
	return r;
}
// ---------------------------------------------------------------------------------------------------------//
//�������� ���������� ���� .������������ �� ������� ������� ��������� � ������� �� ��������.circular-�� �����
FRESULT get_prev_file(DIR* cd,const TCHAR* curobj,TCHAR* pbuff,bool circular)
{
  static TCHAR tbuf[_MAX_LFN + 1]=_T("");
  int d,nfiles;
  FRESULT fr;
  TCHAR* outbf=(TCHAR*)pbuff;
  strcpy(tbuf,curobj);
  if(_strcmp(tbuf,_T("."))==0 || _strcmp(tbuf,_T(".."))==0 || _strcmp(tbuf,_T(""))==0)goto lp_scan_fldr_check;
  d=is_dir(tbuf);if(d==-1)return FR_INT_ERR;
  if(d==0)goto lp_scan_fls;//�� �����,����������� �� ����� ������
  if(!(TMMask & TM_FOLDER_ONE))goto lp_scan_fldr;//���� �����
//---------------FILES---------------
beg_scan_fls: //� �������� �������,�� �����
  nfiles=get_numFilesForTypeMask(cd);
  if(nfiles==0 && (TMMask & TM_FOLDER_ONE))return FR_NO_FILE;//������ ������ �� �������
  //���������� �� ��������� ������
  set_to_lst_obj(cd,tbuf);if(fr!=FR_OK)return fr;
  
lp_scan_fls:
  strcpy(strbuff,tbuf);
  fr=rewind_and_set_prev(cd,strbuff,tbuf);//���������� ��������� �� ���������� ������
  if(fr!=FR_OK)return fr;
  
  if(_strcmp(tbuf,_T("."))==0 || _strcmp(tbuf,_T(".."))==0 || _strcmp(tbuf,_T(""))==0 || //������ ���.�� ������ 
     ( !(TMMask & TM_FOLDER_ONE) && (TMMask & TM_FOLDER_ANY) && bRnd ) ) //��� �� ������ ����� � ����� ���� � �����������
  {
    if(TMMask & TM_FOLDER_ONE)//������ ������ �����
    {
      if(circular)goto beg_scan_fls;
        else return FR_NO_FILE;//������ ������ �� �������
    }
    
    goto lp_scan_fldr_check;
  }
  
  d=is_dir(tbuf);if(d==-1)return FR_INT_ERR;
  
  if(d==0 && (!((TMMask & TM_FILE_RANDOM) && bRnd))/*�� (������ � �������)*/ ){strcpy(outbf,tbuf); return FR_OK;}//���� ������
  
  goto lp_scan_fls;//�����,����������� �� ����� ������

//---------------FOLDERS---------------
beg_lp_scan_fldr:
  fr=cfld_name(tbuf);if(fr!=FR_OK)return fr;
  fr=f_opendir(cd,_T("."));if(fr!=FR_OK)return fr;
  //���������� �� ��������� ������
  fr=set_to_lst_obj(cd,tbuf);if(fr!=FR_OK)return fr;
    
lp_scan_fldr:
  strcpy(strbuff,tbuf);
  fr=rewind_and_set_prev(cd,strbuff,tbuf);//���������� ��������� �� ���������� ������
  if(fr!=FR_OK)return fr;
  if(_strcmp(tbuf,_T("."))==0 || _strcmp(tbuf,_T(".."))==0 || _strcmp(tbuf,_T(""))==0)
  {
    goto beg_scan_fls;//�� ������
  }
  d=is_dir(tbuf);if(d==-1)return FR_INT_ERR;
  if(d==0)goto lp_scan_fldr;//������
  goto beg_lp_scan_fldr;//������� �����,�������
  
lp_scan_fldr_check:    
  if(_strcmp(tbuf,_T("."))==0 || _strcmp(tbuf,_T(".."))==0 || _strcmp(tbuf,_T(""))==0)//����������� ��� � ������� �����
  {
    int r=is_root();if(r==-1)return FR_INT_ERR;  
    if(r==0)//�� ��������,������� � ������������
    {
      TCHAR cfld[255]=_T("");
      fr=cfld_name(cfld);if(fr!=FR_OK)return fr;
      fr=f_chdir(_T(".."));if(fr!=FR_OK)return fr;
      fr=f_opendir(cd,_T("."));if(fr!=FR_OK)return fr;
      //���������� ��������� �� ���������� ������ �����
      fr=rewind_and_set_prev(cd,cfld,tbuf);if(fr!=FR_OK)return fr;
      if(_strcmp(tbuf,_T("."))==0 || _strcmp(tbuf,_T(".."))==0 || _strcmp(tbuf,_T(""))==0)
      {
        goto beg_scan_fls;
      }
      d=is_dir(tbuf);if(d==-1)return FR_INT_ERR;
      if(d==0)goto lp_scan_fldr;//���������� ����� �����
      goto beg_lp_scan_fldr;//���� � �����
    }
    else//IS ROOT
    {
      if(circular)
      {
        FRESULT fr=f_readdir(cd, NULL);//�� ������
        if(fr!=FR_OK)return fr;
        return get_next_file(cd,(TCHAR*)pbuff,circular);
      }
      
      return FR_NO_FILE;//������������ ��������� ���������(root)/������ ������ �� �������
    }//if(r==0){...}else{...}
  }// if(tbuf==_T(".")....)
  else
  {
    //����� � ����������� �� ����
    goto beg_lp_scan_fldr;
  }
}
// ---------------------------------------------------------------------------------------------------------//
//��������� �� ������ ���������� �� �������� ������� � �������� ��� ����������� �������,���� outbuf �� NULL
FRESULT rewind_and_get_prev(DIR* cd,const TCHAR* inbuf,TCHAR* outbuf) 
{
  FRESULT fr=f_readdir(cd, NULL);if(fr!=FR_OK)return fr;//�� ������
  TCHAR* outbf=outbuf;
  finf.fname[0]=0;
  TCHAR* objname=_T("");
  if(outbuf!=NULL)outbuf[0]=0;
        
  do
  {
    if(outbuf)//����������� ��������� 
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
//��������� �� ������ ���������� �� ����������� ������� � �������� ��� ����������� �������,���� buff �� NULL
FRESULT rewind_and_set_prev(DIR* cd,const TCHAR* curobj,TCHAR* buff) 
{
  const TCHAR* outbf=(const TCHAR*)buff;
  const TCHAR* cbf=(const TCHAR*)curobj;
  TCHAR btemp[_MAX_LFN + 1];
  
  FRESULT fr=rewind_and_get_prev(cd,cbf,btemp);if(fr!=FR_OK)return fr;//��������� �� �������� � ��������� �����
  fr=rewind_and_get_prev(cd,btemp,NULL);if(fr!=FR_OK)return fr;//��������� �� �����������
  
  if(buff)strcpy((void*)outbf,btemp);
  
  return fr;
}
// ---------------------------------------------------------------------------------------------------------//
int is_dir(const TCHAR* curobj)//-1 ��� ������
{
  if(_strcmp(curobj,_T("."))==0 || _strcmp(curobj,_T(".."))==0)return 1;
  if(_strcmp(curobj,_T(""))==0)return 0;
  FRESULT fr=f_stat(curobj,&finf);if(fr!=FR_OK)return -1;
  if(finf.fattrib & AM_DIR)return 1;
 
  return 0;
}
// ---------------------------------------------------------------------------------------------------------//
//���������� ��������� �� ��������� ������ ����� � �������� ��� ���������� �������,���� buff �� NULL
FRESULT set_to_lst_obj(DIR* cd,TCHAR* buff)
{
  TCHAR* outbf=buff;
  FRESULT fr=f_readdir(cd, NULL);//�� ������
  if(fr!=FR_OK)return fr;
  finf.fname[0]=0;
  TCHAR* objname=_T("");
  if(buff!=NULL)buff[0]=0;
  
  
  //���������� ���
  do
  {
    if(buff)//����������� ��������� 
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