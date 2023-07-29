/**
  ******************************************************************************
  * @file    ILDA_player.c 
  * @author  MW dem1305
  * @version V1.0.0
  * @date    15-sept-2015
  * @brief   LScan ilda format player with FATFS 
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------------------------------------*/
#include "ILDA_player.h"
//#include "LScan_protocol.h"
#include <string.h>
#include "Pangolin8bitPalette.h"
//#include "atomic.h"
#include "sdcard.h"

//----------------------------------------------------------------------------------------------------------//
#define SwapWord(sVal) sVal=HIBYTE(sVal)+LOBYTE(sVal)*256;//перевернуть байты
#define SwapCoord3DWords(coord3D) SwapWord(coord3D.x);SwapWord(coord3D.y);SwapWord(coord3D.z);SwapWord(coord3D.status);
#define SwapCoord3DF4Words(coord3DF4) SwapWord(coord3DF4.x);SwapWord(coord3DF4.y);SwapWord(coord3DF4.z);SwapWord(coord3DF4.status1);SwapWord(coord3DF4.status2);
#define SwapCoord2DWords(coord2D) SwapWord(coord2D.x);SwapWord(coord2D.y);SwapWord(coord2D.status);
#define SwapCoordHdrWords(_coordIn) SwapWord(_coordIn.pointsTotal);SwapWord(_coordIn.frameNum);SwapWord(_coordIn.framesTotal);
#define SwapColorsHdrWords(colors) SwapWord(colors.colorsTotal);SwapWord(colors.paletteNum);SwapWord(colors.future);

#define pSwapCoord3DWords(coord3D) SwapWord(coord3D->x);SwapWord(coord3D->y);SwapWord(coord3D->z);SwapWord(coord3D->status);
#define pSwapCoord3DF4Words(coord3DF4) SwapWord(coord3DF4->x);SwapWord(coord3DF4->y);SwapWord(coord3DF4->z);SwapWord(coord3DF4->status1);SwapWord(coord3DF4->status2);
#define pSwapCoord2DWords(coord2D) SwapWord(coord2D->x);SwapWord(coord2D->y);SwapWord(coord2D->status);
#define pSwapCoordHdrWords(_coordIn) SwapWord(_coordIn->pointsTotal);SwapWord(_coordIn->frameNum);SwapWord(_coordIn->framesTotal);
#define pSwapColorsHdrWords(colors) SwapWord(colors->colorsTotal);SwapWord(colors->paletteNum);SwapWord(colors->future);


#define _voiceenable (!(TMMask & TM_NOVOICE))
//----------------------------------------------------------------------------------------------------------//
extern uint8_t StreamBuf[];;
//extern uint8_t  discr_SPD_proc;
extern volatile int freesize;

extern volatile IMODE ICmode;
//extern int sampleBytes;//количество принятых байтов
//extern uint16_t mel8notes[16];
CLR colorTable[256];//текущая цветовая таблица

extern uint16_t X,Y; //целевые координаты 
extern uint8_t R,G,B;//целевые цвета 
//----------------------------------------------------------------------------------------------------------//
static bool CheckILDAHeaderFormat(FIL* pfile,ILDA_COLOR_HDR* pclr_hdr,ILDA_COORD_HDR* pcrd_hdr,BYTE* pfrmt);//return format.false=FS error
static inline bool GetFileCurrentSPOINT(FIL* pfile,UINT* ptr,UINT pClrTable,BYTE format,BYTE pt_size,SPOINT* pspt);
static uint8_t Get_Pts_Ms(USHORT pointsTotal);
//запись в поток воспроизведения,вернет -1 при ошибке или количество выведенных точек
//формат 255=запись из массива SPOINT
int FillStreamFromFileSource(FIL* pfile,UINT* ptr,UINT pointsTotal,BYTE format,BYTE pt_size,ULONG pColorTable,uint8_t discrMs);
/* Variables -----------------------------------------------------------------------------------------------*/
extern __IO int sampleSpeed;//адаптация скорости,проценты
extern uint8_t LScanStatus;
//extern uint32_t FullSysReset;
//extern uint32_t USB_Disconnect;
extern bool FSMounted;

//#pragma data_alignment = 4
//FATFS* FileSysObj;
//FIL* pILDA_file=NULL;  //
//DIR* cdir;//directory
//FILINFO finfo;//fileinfo

//char rxtxbuff[32];


//FATFS* FileSysObj;
//FIL* pILDA_file;  /* File object */
//DIR* cdir;//directory
//FILINFO* finfo;//fileinfo

//char rxtxbuff[32];

extern volatile uint8_t  PlayFlag;//флаг воспроизведения
extern volatile TYPEMODEMASK TMMask;    //маска настроек режимов воспроизведения файлов,LScan.c
#include "files_control.h"
// ---------------------------------------------------------------------------------------------------------//
__IO uint8_t stream_busy=(uint8_t)0;//volatile
__IO IS_RES  PlayState= IS_OK;//volatile
__IO bool repeatMode=true;
__IO float opt_level=1.3f;//уровень оптимизации 0-..... 0=отключено
extern OPT_V opt;

extern HANDLE PlayerThread;

extern bool OnSoundNotifications;//включены оповещения?
volatile bool Exit_Proccess_Player=false;
// ---------------------------------------------------------------------------------------------------------//
//поиск ILDA и запуск проигрывания.
IS_RES Proccess_Player()
{
  
  FRESULT fr;
  Exit_Proccess_Player=false;
  PlayState= IS_OK;
  
  UINT errcnt=0;
  
  while(PlayState==IS_BUSY && !butEventClick(but_play))
  {
    Sleep(100);
  }
  
  //pILDA_file=mmmalloc(sizeof(FIL));//ZeroMemory(pILDA_file,sizeof(FIL));
  
  DIR* cdir=ccmalloc(sizeof(DIR));
  
  TCHAR fnm[_MAX_LFN + 1];ZeroMemory(fnm,sizeof(TCHAR)*(_MAX_LFN + 1));//[_MAX_LFN + 1];//текущий файл
  //TCHAR ext[8];ZeroMemory(ext,sizeof(TCHAR)*8);//[8];
  //change current drive
  fr=f_chdrive (_T("1:"));  
  if(fr!=FR_OK )goto ret_error_file_system;//ошибка смены диска
  //open root
  fr=f_opendir(cdir,_T("/"));
  if(fr!=FR_OK )goto ret_error_file_system;//ошибка откр. директория
  
loop_play_files: 
  fr=get_next_file(cdir,fnm,false);
  
ch_fresult: 
  if(fr==FR_NO_FILE)
  {
    //Beep(2000,100);
    //if(circular){
    //  Sound_notification(1,"0://SOUNDS/repeat_cycle.mp3");
    //}
    //else
    //{
      Sound_notification(1,"0://SOUNDS/files_no_more_found.mp3");//no more files.mp3");
      for(int no=0;no<20;++no)OutText(0,"no files",0xFF0000);
    //}
    PlayState=IS_OK;//плеер остановлен
    Atom32Set((unsigned long*)&PlayerThread,0);//PlayerThread=NULL;
    return IS_EODIR;
  }
  
  if(fr!=FR_OK )goto ret_error_file_system;
  
  //ZeroMemory(ext,sizeof(ext));
  //if(!getFileExtension(fnm,ext))goto loop_play_files;
  //if(strcmp(ext,_T("ild"))==0||strcmp(ext,_T("ilda"))==0||
  //       strcmp(ext,_T("ILD"))==0||strcmp(ext,_T("ILDA"))==0)
  if(is_ext_matches(fnm,_T("ILD"),_T("ild"),_T("ILDA"),_T("ilda"),0))
  {
        
    //fr=Sound_notification("0://SOUNDS/found next file. Playing.mp3");
    Beep(3000,50);
    PlayState=playILDAFile(fnm);
    if(Exit_Proccess_Player)
    {
      PlayState=IS_OK;
      goto ret_process_func;
    }
    
    if(PlayState & IS_EFS)
    {
      errcnt++;
      if(errcnt>=10)goto ret_error_file_system;
    }
    else errcnt=0;
    
    //----------проверка кнопок------------
    if(butEventClick(but_ff))
    {
      //Sound_notification("0://SOUNDS/next file.mp3");
      Beep(2500,80);
      goto loop_play_files;
    }
    
    if(butEventClick(but_rew))
    {
      //Sound_notification("0://SOUNDS/previous file.mp3");
      Beep(2200,80);
      fr=get_prev_file(cdir,fnm,fnm,false);
      goto ch_fresult;
    }
    
 /*   if(butEventClick(but_stop))
    {
      PlayState= IS_BUSY;
      Sound_notification(1,"0://SOUNDS/stop playing.mp3");
      goto loop_play_files;
    } */
    //--------------------------------------
  }
  
  goto loop_play_files;
  //return PlayState;
  
  //-----------------------------------------------
ret_error_file_system:
  //Beep(200,300);//error
  Sound_notification(1,"0://SOUNDS/error.mp3");
  for(int no=0;no<20;no++){RECT rloc={-4000,0,200,200};OutText(&rloc,"FILE SYSTEM ERROR...",0xFF0000);}
  Sleep((uint32_t)400);
  
  PlayState=IS_ERR|IS_EFS;
  
ret_process_func:
  msafe_free(cdir);
  //msafe_free(fnm);
  //msafe_free(ext);
  //msafe_free(pILDA_file);
  //PlayerThread=NULL;
  
  Atom32Set((unsigned long*)&PlayerThread,0);// (unsigned long* ptr,unsigned long newValue)
  Exit_Proccess_Player=false;
  return PlayState;
}
//----------------------------------------------------------------------------//
//проверка заголовка,загрузка координатного и цветового заголовка.false=FS error
static bool CheckILDAHeaderFormat(FIL* pfile,ILDA_COLOR_HDR* pclr_hdr,ILDA_COORD_HDR* pcrd_hdr,BYTE* pfrmt)//return format.
{
  UINT rdd;
  LSCAN2_LEDOff(LEDBLUE);//BLUE 
  FRESULT res=f_read (pfile,pcrd_hdr,32,&rdd);
  
  if(FR_OK!=res || 32!=rdd)return false;//error
  
  if(pcrd_hdr->headerSym[0]!='I'||pcrd_hdr->headerSym[1]!='L'||pcrd_hdr->headerSym[2]!='D'||pcrd_hdr->headerSym[3]!='A')
  {
    pcrd_hdr->format=255;//error
  }
  
  if(pcrd_hdr->format==2)memcpy(pclr_hdr,pcrd_hdr,32);
    
  *pfrmt=pcrd_hdr->format;
  return true;
}
//----------------------------------------------------------------------------//
//ошибки формата файловых данных не считаются сбоем файловой системы
//для запуска нового воспроизведения необходимо обнулить PlayState
IS_RES playILDAFile(TCHAR* filename)//NULL==продолжить воспроизведение
{
  ILDA_COORD_HDR CoordHeader;
  ILDA_COLOR_HDR ColorHeader;
  ULONG pColorTable=0;//указатели на палитры
  
  bool pause=false;
  bool even=false;
  
  UINT rdd;
  UINT clrsTot=0;
  FRESULT fr;
  USHORT discretisation=0;
  BYTE format=0;
  BYTE dimension=0;
  UINT   frameMem=0;
  int pts_saved=0;
  UINT pMem=0;
  
  FIL* pILDA_file;
  ULONG fileSize=0;
  
  //if(pILDA_file) f_close(pILDA_file);
  pILDA_file=mmmalloc(sizeof(FIL));
  
  //ZeroMemory(pILDA_file,sizeof(FIL));
  fr=f_open(pILDA_file,filename,FA_READ);
  if(fr!=FR_OK)goto close_ret_IS_ERR;//FS error
  //openned=true;
  fileSize=f_size(pILDA_file);
  PlayState=IS_PLAY;

  while (pMem<fileSize)
  {
        
    if(Exit_Proccess_Player)goto close_ret_IS_EOF;
    
    if(pause)
    {
      pMem=frameMem;
      if(f_lseek(pILDA_file,pMem)!=FR_OK)goto close_ret_IS_ERR;//FS error
    }
    
    if(!CheckILDAHeaderFormat(pILDA_file,&ColorHeader,&CoordHeader,&format))goto close_ret_IS_ERR;//FS error
    frameMem=pMem;
    pMem+=32;
    //--------------------------------------------------
    switch(format)
    {
    case 2://color table
      SwapColorsHdrWords(ColorHeader);
      //добавить цветовую таблицу.Последняя-действующая.Таблица становится действительной для текущего фрейма.
      pColorTable=pMem;
      clrsTot=(UINT)(ColorHeader.colorsTotal>255?255:ColorHeader.colorsTotal);
      
      fr=f_read (pILDA_file,colorTable,clrsTot*3,&rdd);
      if(FR_OK!=fr|| clrsTot*3!=rdd)goto close_ret_IS_ERR;//FS error 
      pMem+=clrsTot*3;//size of тable RGB
      if((UINT)ColorHeader.colorsTotal-clrsTot==0)break;
      clrsTot=(UINT)ColorHeader.colorsTotal-clrsTot;
      pMem+=clrsTot*3;//size of color RGB
      if(f_lseek(pILDA_file,pMem)!=FR_OK)goto close_ret_IS_ERR;//FS error
      break;
      //+++++++++++++++++++++++++error
    case 255://error
      RED_LED_ON();//RED
      goto close_ret_IS_EOF;//не считать ошибку данных
      //+++++++++++++++++++++++++new frame
    default://points 
      SwapCoordHdrWords(CoordHeader);
      BLUE_LED_ON();
      
      if(CoordHeader.pointsTotal==0)goto close_ret_IS_EOF;//EOF
      //---------check buttons--------
      if(isButtonPressedShort(but_play))
      {
        pause=!pause;
        if(pause)Sound_notification(1,"0://SOUNDS/pause.mp3");
          else Sound_notification(1,"0://SOUNDS/play.mp3");
      }
      else 
      {
        if(/*isButtonPressedShort(but_stop) || */isButtonPressedShort(but_ff)||isButtonPressedShort(but_rew))
        {
          goto close_ret_IS_EOF;//EOF
        }
        else
        {
          if(isButtonPressedLong(but_ff)||isButtonPressedLong(but_rew))
          {
            even=!even;
            if(even)goto close_ret_IS_EOF;//EOF
          }
          else even=false;
        }
      }
      //------------------------------
      switch(format)
      {
        case 1:dimension=6;break;//2D
        case 4:dimension=10;break;//??
        default:dimension=8;break;//3D
      }
      //заполнение точками
      if(pMem+(UINT)dimension*(UINT)CoordHeader.pointsTotal>(UINT)fileSize)//бывают некорректные данные
      {
        CoordHeader.pointsTotal=(USHORT)((fileSize-pMem)/dimension);
      }
      
      discretisation=Get_Pts_Ms(CoordHeader.pointsTotal);//для фрейма
      
    continue_FillStreamFromFileSource: 
      
      if(CoordHeader.pointsTotal==0)continue;
      
      //ЧТЕНИЕ И ЗАПИСЬ В ПОТОК ВЫВОДА
      pts_saved=FillStreamFromFileSource(pILDA_file,&pMem,(UINT)CoordHeader.pointsTotal,
                                         format,dimension,pColorTable,discretisation);
      
      
      if(pts_saved==-1)//FS ERROR
      {
        RED_LED_ON();
        //while(1);
        goto close_ret_IS_ERR;
      }
      else 
      {
        CoordHeader.pointsTotal-=(USHORT)pts_saved;
        if(CoordHeader.pointsTotal)goto continue_FillStreamFromFileSource;
        //return PlayState;
      }
      
      Sleep(0);//отдать квант
      
   }//switch
}//while 
  
close_ret_IS_EOF:
  f_close(pILDA_file);
  msafe_free(pILDA_file);
  return PlayState=IS_EOF;
  
close_ret_IS_ERR:
  f_close(pILDA_file);
  msafe_free(pILDA_file);
  RED_LED_ON();
  return PlayState=IS_ERR|IS_EFS;
}
//----------------------------------------------------------------------------//
//получить точку фрейма в формате SPOINT. false при ошибке файловой системы
static inline bool GetFileCurrentSPOINT(FIL* pfile,UINT* ptr,UINT pClrTable,BYTE format,BYTE pt_size,SPOINT* pspt)
{
  short x,y;
  UINT rdd;
  FRESULT res;
  char rxtxbuff[32];
  
  res=f_read (pfile,rxtxbuff,(UINT)pt_size,&rdd);
  if(FR_OK!=res|| (UINT)pt_size!=rdd)return false;//FS error
  *ptr+=pt_size;

  /*if(FullSysReset || USB_Disconnect)
  {
    RED_LED_ON();
    while(1);
  }*/
  
  //return true;//debug

  static COORD3D c3d;
  static COORD2D c2d;
  static COORD3D_FORMAT4 c3df4;
  static USHORT status;
  static BYTE rclr,gclr,bclr;
  static CLR colorRGB;//temp
  
  switch(format)
    {
    case 1://2D dim=6
      memcpy(&c2d,rxtxbuff,(size_t)6);
      SwapCoord2DWords(c2d);
      x=c2d.x;y=c2d.y;
      status=c2d.status;//index
      break;
    case 4://3D format 4 dim=10
      memcpy(&c3df4,rxtxbuff,(size_t)10);
      SwapCoord3DF4Words(c3df4);
      x=c3df4.x;y=c3df4.y;
      status=c3df4.status1;//index
      break;
    default://3D dim=8
      memcpy(&c3d,rxtxbuff,(size_t)8);
      SwapCoord3DWords(c3d);
      x=c3d.x;y=c3d.y;
      status=c3d.status;//index
      break;
    }

  //-----------color-----------------
  if(status&0x4000&&format!=5)//Blanking set,Lasers off
  {
    rclr=gclr=bclr=0;
  }
  else//no blanking
  {
    if(pClrTable==0)
    {
      switch(format)
      {
      case 5:
        rclr=status&0xFF;//red
        gclr=(status&0xFF00)>>8;//green
        bclr=LOBYTE(c3d.z);//blue
        status=c3d.z;
        if(status&0x4000)rclr=gclr=bclr=0;
        break;
      case 4:
        bclr=LOBYTE(c3df4.status1);
        rclr=LOBYTE(c3df4.status2);
        gclr=HIBYTE(c3df4.status2);
        break;
      default:
        //цвета из таблицы Pangolin:
        status&=0xFF;//index
        rclr=PangolinPalette[status].r;//red
        gclr=PangolinPalette[status].g;//green
        bclr=PangolinPalette[status].b;//blue
        break;
      }
    }
    else//index in color table
    {
      status&=0xFF;//index
      
      colorRGB=colorTable[status];
      rclr=colorRGB.r;
      gclr=colorRGB.g;
      bclr=colorRGB.b;
    }
  }//else no blanking
  

  pspt->x=(int)x;pspt->y=(int)y;
  pspt->color=(DWORD)((((DWORD)rclr)<<16)|(((DWORD)gclr)<<8)|((DWORD)bclr));
  return true;
}
//----------------------------------------------------------------------------//
//вычисление дискретизации в точках в миллисекунду для фрейма
static uint8_t Get_Pts_Ms(USHORT pointsTotal)
{
 #ifdef DISCR_CONST
  //debug
  return DISCR_CONST_VALUE;
#endif
  
  //----------------calculate frames frequency-----------------------
  int discr;
  int packetSize=MAX_DISCR*sizeof(ICP)/1000;
  if(packetSize>MAX_ISOPACKET)packetSize=MAX_ISOPACKET;

  int frames=(packetSize*1000)/sizeof(ICP)/(int)pointsTotal;

  if(frames<MIN_FRAMES_PER_SEC)frames=MIN_FRAMES_PER_SEC;
  else if(frames>MAX_FRAMES_PER_SEC) frames=MAX_FRAMES_PER_SEC;

  discr=( pointsTotal*frames/sizeof(ICP) )*sizeof(ICP) ; //больше или равно,кратно 8

  if(discr>MAX_DISCR)discr=MAX_DISCR;else if(discr<MIN_DISCR)discr=MIN_DISCR;


  int packetDSS=discr/**discr_SPD_proc/100*//1000*sizeof(ICP);
    
  if(packetDSS>MAX_ISOPACKET)packetDSS=MAX_ISOPACKET;else if(packetDSS<MIN_ISOPACKET)packetDSS=MIN_ISOPACKET;
  
  discr=packetDSS/sizeof(ICP);
  if(discr<1)discr=1;
  return (uint8_t)discr;
}
//----------------------------------------------------------------------------//
//запись в поток воспроизведения,вернет -1 при ошибке или количество выведенных точек
//формат 255=запись из массива SPOINT
int FillStreamFromFileSource(FIL* pfile,UINT* ptr,UINT pointsTotal,BYTE format,BYTE pt_size,ULONG pColorTable,uint8_t discrMs)
{
  static uint8_t stage_out=0;
  static uint8_t discr=(uint8_t)OPT_DISCR_MS;
  static uint8_t pointsInSample=0;
  if(discrMs!=0 )
  {
    if(discr!=discrMs)pointsInSample=0;
    discr=discrMs;
  }
  
  static SPOINT* pPt;//for array source
  pPt=(SPOINT*)ptr;
  
  UINT p=pointsTotal;
    
  static float ax,ay, t,/*t_quad,*/ k,dx,dy;//ускорения по осям,время периода,коэффициент пропорциональности превышения лимита
  bool anchor=false;//якорь установлен
   
  static SPOINT A={0,0,0};static SPOINT B={0,0,0};static SPOINT InsP={0,0,0};//предыдущая/текущая/вставка
  //критически возможное ускорение   
  static float critAcc;//рассчет в зависимости от глубины оптимизации
  critAcc=opt_level==0.0f?65536.0f:65536.0f/opt_level;//ускорение в отношении расстояний на оптимальной частоте 
  t=(1.0f/((float)discr));//время периода дискретизации,в миллисекундах
  //t_quad=t*t; //t^2 -not need
  static int pp;
  
  if(!(TMMask & TM_OPT))//без оптимизации
  {
    if(stage_out)goto stage_out_no_opt;else stage_out=1;
    
    while( p )
    {
      if(format!=0xFF)
      {
        if(GetFileCurrentSPOINT(pfile,ptr,pColorTable,format,pt_size,&B)==false)return -1;//error
      }
      else
      {
        B=*pPt;
        ++pPt;
      }
      
    stage_out_no_opt:
      if(!insert_SPOINT_to_stream(&B,discr,pointsInSample,1)){return pointsTotal-p;}
      --pointsInSample;
      --p;
    }
    
    //------end frame--------------
    BLUE_LED_OFF();  
    stage_out=0;
    return pointsTotal-p;
  }
  //с оптимизацией
  switch(stage_out)
  {
  case 1:goto stage_out_1;
  case 2:goto stage_out_2;
  case 3:goto stage_out_3;
  case 4:goto stage_out_4;
  }
  //-----------------------
  while( p )///*&& sampleBytes<=STREAMBUF_SIZE*/ && SD_Detect())
  {
    if(format!=0xFF)
    {
      if(GetFileCurrentSPOINT(pfile,ptr,pColorTable,format,pt_size,&B)==false)return -1;//error
    }
    else
    {
      B=*pPt;
      ++pPt;
    }
    
    //вставка гасящей в начале фрейма
    if(p==pointsTotal)
    {
      InsP.x=B.x;InsP.y=B.y;
      InsP.color=0x0;
      A=InsP;
      for (pp=0;pp<(discr*1000/OPT_DISCR+1)*5;++pp)
      {
        if(pointsInSample==0)pointsInSample=(uint8_t)(p > (uint32_t)discr ? discr : p); 
      stage_out_1:
        if(!insert_SPOINT_to_stream(&A,discr,pointsInSample,1)){stage_out=1;return pointsTotal-p;}//stop?
        
        --pointsInSample;
        anchor=true;
      }
    }
    //---------------------------OPT--------------->>>>>>
    //v=v0+at  a=((c2-c1) - v0*t)*2/t^2 , s=(c2-c1) (c1,c2-координаты,t-время дискретизации)
    //расчет необходимой скорости и ускорения
  check_acceleration:
    if(opt_level==0)goto next_point;
    dx=(float)(B.x-A.x);
    dy=(float)(B.y-A.y);
    ax=(dx - opt.v0x*t)*2.0f/t;//необходимое ускорение по оси X
    ay=(dy - opt.v0y*t)*2.0f/t;//необходимое ускорение по оси Y
    
        
    k=max(fabs(ax),fabs(ay))/critAcc;//во сколько раз превышает критическое
    
    if(k>2.0f && !anchor && ((ax>0&&opt.v0x<=0 || ax<0&&opt.v0x>=0)||(ay>0&&opt.v0y<=0 || ay<0&&opt.v0y>=0)) )
    {
      if(pointsInSample==0)pointsInSample=(uint8_t)(p > (uint32_t)discr ? discr : p);
    stage_out_2:
      if(!insert_SPOINT_to_stream(&A,discr,pointsInSample,1)){stage_out=2;return pointsTotal-p;}//stop?
      
      --pointsInSample;
      anchor=true;
      ax=(- opt.v0x)*2.0f/t;//необходимое ускорение торможения по оси X (S=0)
      ay=(- opt.v0y)*2.0f/t;//необходимое ускорение торможения по оси Y (S=0)
      opt.v0x+=ax*t;
      opt.v0y+=ay*t;
      goto check_acceleration;
    }
    else if(k>1.0f)
    {
      InsP.x=(int)_crd((float)A.x,(float)B.x,k*0.9f);
      InsP.y=(int)_crd((float)A.y,(float)B.y,k*0.9f);
      InsP.color=B.color;
      if(pointsInSample==0)pointsInSample=(uint8_t)(p > (uint32_t)discr ? discr : p); 
      
    stage_out_3:
      if(!insert_SPOINT_to_stream(&InsP,discr,pointsInSample,1)){stage_out=3;return pointsTotal-p;}//stop?
      
      ax=((float)(InsP.x-A.x) - opt.v0x*t)*2.0f/t;//необходимое ускорение по оси X
      ay=((float)(InsP.y-A.y) - opt.v0y*t)*2.0f/t;//необходимое ускорение по оси Y
      A=InsP;
      --pointsInSample;
      anchor=false;
    }
    
    opt.v0x+=ax*t;
    opt.v0y+=ay*t;
    
    anchor=false;
    //---------------------------OPT---------------<<<<<<
   next_point:
    //============inserting_point=============
    if(pointsInSample==0)pointsInSample=(uint8_t)(p > (uint32_t)discr ? discr : p); 
  stage_out_4:
    if(!insert_SPOINT_to_stream(&B,discr,pointsInSample,1)){stage_out=4;return pointsTotal-p;}//stop?
    
    stage_out=0;
    
    A=B;
    //========================================  
    --p;
    --pointsInSample;
    //-------------
  }
  //------end frame--------------
  BLUE_LED_OFF();  
  
  return pointsTotal-p;
}
//-----------------------------------------------------------------------------------------//
bool insert_SPOINT_to_stream(SPOINT* pt,uint8_t discr_in_MS,uint8_t points_in_sample,uint32_t wait_num)
{
  static uint32_t stotal=0;
  //static BYTE oldG=0;
  //static uint16_t oldLx=2048;static uint16_t oldLy=2048;
  //дистанция до указателя чтения  
  //int wrdist=bytesCircularBufferFromTo(wrPtr,rdPtr,STREAMBUF_SIZE);
  //uint32_t wt=wait_num;//циклов ожидать
  
  if(ICmode!=IM_SD || Exit_Proccess_Player)return false;
  
  if(stotal>=SPI_DMA_BLOCK_SIZE && rdy_ilda==1)rdy_ilda=2;
  
  while((!rdy_ilda || rdy_ilda==2)  && !Exit_Proccess_Player && ICmode==IM_SD);
  
  if(ICmode!=IM_SD || Exit_Proccess_Player){
    stotal=0;return false;
  }
  
  LScanStatus=STATUS_PLAY;
  
  //1-bufer ready to fill (it->process)
  //2-block copyed to stream(process->it)
  if(stotal<SPI_DMA_BLOCK_SIZE){
    uint16_t Lx=(uint16_t)((pt->x)/LSCAN_SIZE_PROP_DIV+(pt->x<0?2048:2047)); ///16 norm = 0-4095
    uint16_t Ly=(uint16_t)(4095-((pt->y)/LSCAN_SIZE_PROP_DIV+(pt->y<0?2048:2047)));
    //------------------------------
    DWORD ncolor=pt->color;
    //BYTE newG=(BYTE)((ncolor&0xFF00)>>8); //новое значение яркости
    //oldLx=Lx;oldLy=Ly;oldG=newG;
    
    wrPtr[stotal]=0; //cmd
    wrPtr[stotal+1]=discr_in_MS;//points_in_sample;//discr in points
    wrPtr[stotal+2]=LOBYTE(Lx);
    wrPtr[stotal+3]=LOBYTE(Ly);
    wrPtr[stotal+4]=(HIBYTE(Lx)|(HIBYTE(Ly)<<4));
    //colors x 3
    wrPtr[stotal+5]=(BYTE)((ncolor&0xFF0000)>>16);//B 
    wrPtr[stotal+6]=(BYTE)((ncolor&0xFF00)>>8);//G
    wrPtr[stotal+7]=(BYTE)(ncolor&0xFF);//R
    stotal+=sizeof(ICP);
    if(stotal>=SPI_DMA_BLOCK_SIZE){
      rdy_ilda=2;stotal=0;
    }
  }
  
  /*
  //-------минимум 1 точка,ожидание освобождения места в буфере +1-----------
  while((PlayFlag && wrdist > 0 && wrdist<=sizeof(ICP)))//||(!PlayFlag && wrdist<=sizeof(ICP)))//(1+1+insertNum)*sizeof(ICP))
  {
    if(wt==0)return false;//BUSY
    Sleep(0);//отдать время другому потоку
    wt--;
    wrdist=bytesCircularBufferFromTo(wrPtr,rdPtr,STREAMBUF_SIZE);
  }
    
  //-------место свободно-------->>>
    //[0,4095]
  uint16_t Lx=(uint16_t)((pt->x)/LSCAN_SIZE_PROP_DIV+(pt->x<0?2048:2047)); ///16 norm = 0-4095
  uint16_t Ly=(uint16_t)(4095-((pt->y)/LSCAN_SIZE_PROP_DIV+(pt->y<0?2048:2047)));
  //------------------------------
  oldLx=Lx;oldLy=Ly;oldG=newG;
  uint8_t* IsoWrPtr=(uint8_t*)Atom32Get((unsigned long*)&wrPtr);
  
  IsoWrPtr[0]=0; //cmd
  IsoWrPtr[1]=discr_in_MS;//points_in_sample;//discr in points
  IsoWrPtr[2]=LOBYTE(Lx);
  IsoWrPtr[3]=LOBYTE(Ly);
  IsoWrPtr[4]=(HIBYTE(Lx)|(HIBYTE(Ly)<<4));
  //colors x 3
  IsoWrPtr[5]=(BYTE)((ncolor&0xFF0000)>>16);//B 
  IsoWrPtr[6]=newG;//G
  IsoWrPtr[7]=(BYTE)(ncolor&0xFF);//R
  //------------
  
  if( IsoWrPtr+sizeof(ICP) >= StreamOverbuf )//wrPtr=StreamBuf;else wrPtr+=sizeof(ICP);
    Atom32Set((unsigned long*)&wrPtr,(unsigned long)StreamBuf);//wrPtr=StreamBuf;
        else Atom32Set((unsigned long*)&wrPtr,(unsigned long)(IsoWrPtr+sizeof(ICP)));//wrPtr+=sizeof(ICP);
   */     

  return true;
}
//----------------------------------------------------------------------------//
void SetXYPos(int x,int y)//перемещение текущей координаты (лучи гасятся) в ILDA/uint16_t значениях
{
  int cx,cy;//Y
  GetDAC(&cx,&cy,true);
  if(x!=cx || y!=cy)
  {
    x=(int)((short)x/LSCAN_SIZE_PROP_DIV+2048);
    y=4095-(int)((short)y/LSCAN_SIZE_PROP_DIV+2048);
    DAC->DHR12R1=((uint16_t)x)&((uint16_t)0xFFF);
    DAC->DHR12R2=((uint16_t)y)&((uint16_t)0xFFF);
  }
}
//----------------------------------------------------------------------------//
void GetDAC(int* x,int* y,bool ILDA_format)//получить текущее значение координат в ILDA/uint16_t значениях
{
  int cx=(int)((DAC->DHR12R1)&((uint16_t)0xFFF));//X
  int cy=(int)((DAC->DHR12R2)&((uint16_t)0xFFF));//Y
  if(!ILDA_format){*x=cx;*y=cy;return;}
  cx=(cx-2048)*(int)LSCAN_SIZE_PROP_DIV;cy=(cy-2048)*(int)LSCAN_SIZE_PROP_DIV;
  *x=cx;*y=cy;
}
//----------------------------------------------------------------------------//

