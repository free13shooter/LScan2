/**
  ******************************************************************************
  * @file    font_func.c 
  * @author  MW dem1305
  * @version V1.0.0
  * @date    3-okt-2015
  * @brief   font vector drawing functions
  ******************************************************************************
*/ 

/* Includes ------------------------------------------------------------------------------------------------*/

#include "FontMicrosoftSansSerif_14_1.h"
#include "ILDA_player.h"
//----------------------------------------------------------------------------------------------------------//

//направление вектора
#define V_U   ((U8)1)
#define V_D   ((U8)2)
#define V_L   ((U8)4)
#define V_R   ((U8)8)
#define V_UL  ((U8)V_U|V_L)
#define V_UR  ((U8)V_U|V_R)
#define V_DL  ((U8)V_D|V_L)
#define V_DR  ((U8)V_D|V_R)

unsigned char chArr[16][14];//x,y
extern OPT_V opt;
bool vect_symbols=true;//в каком виде выводить символы (матричном/векторном)

extern uint32_t SamplePeriod;
extern uint16_t X,Y; //целевые координаты 
extern uint8_t R,G,B;//целевые цвета 
//----------------------------------------------------------------------------------------------------------//
//вывод знака в позиции SPOINT,если не NULL,иначе в текущей,с заданным цветом.scale-множитель размера.
void OutVectorChar(unsigned short chIndex,RECT* rpos_size,DWORD SymColor,int loops,uint8_t discrMS,bool vector_mode )
{
  GUI_FONT_PROP* fntProp=GUI_FontMicrosoftSansSerif_14_1.pGUI_FONT_PROP;
  unsigned short first=fntProp->First;
  unsigned short last=fntProp->Last;
  if(chIndex<first||chIndex>last)return;
  //GUI_CHARINFO CharInfo=fntProp->paCharInfo[chIndex];
  const unsigned char * pData=fntProp->paCharInfo[chIndex - fntProp->First].pData;//указатель на битовый набор
  unsigned char XSize=fntProp->paCharInfo[chIndex - fntProp->First].XSize;//ширина символа в битах
  unsigned char XDist=fntProp->paCharInfo[chIndex - fntProp->First].XDist;//дистанция до следующего символа в битах
  
  int chVBytes=GUI_FontMicrosoftSansSerif_14_1.YSize;//количество байт по вертикали
  int chHBytes=fntProp->paCharInfo[chIndex - fntProp->First].BytesPerLine;//ширина битовой маски в байтах для линии растра
  
  //вывод в цикле
  int lp=loops==0?1:loops;
  while(lp)
  {
    for(int yb=0;yb<chVBytes;yb++)//копирование побайтно во временный массив
    {
      U16* pt16=(U16*)pData;
      U16 bitline=chHBytes>1?(((U16)LOBYTE(pt16[yb]))<<8)|(HIBYTE(pt16[yb])):(U16)(pData[yb]);//преобразование в 16 бит
      for(int xb=0;xb<8*chHBytes;xb++)
      {
        chArr[xb] [13-yb]=( ((U16)1<<(8*chHBytes-1-xb))& (bitline) )==(U16)0?0:1;
      }
    }
    //сканирование и вывод
    OutMatrix(chHBytes*8,chVBytes,rpos_size,SymColor,discrMS,vector_mode);
    --lp;
  }
  
  if(rpos_size!=NULL)
  {
    //заполнение размеров знака и локаций по осям величиной дистанций в единицах измерения
    rpos_size->w=(int)XSize;rpos_size->h=(int)(GUI_FontMicrosoftSansSerif_14_1.YSize);
    rpos_size->x=(int)XDist;rpos_size->y=(int)(GUI_FontMicrosoftSansSerif_14_1.YDist);
  }
  return ;
}
//----------------------------------------------------------------------------------------------------------//
//сканирование битовой картинки для нахождения точки.
//если не обнаружено-вернет 0
bool searchPoint(int H,int V,SPOINT* A,bool around)//вокруг?
{
  int ix,iy;
  int x0,x1,y0,y1;
  
  int xb=0,yb=0,xe=H-1,ye=V-1;
  if(around)
  {
    xb= A->x>0? A->x-1:0;
    xe= A->x<H-1? A->x+1:H-1;
    yb= A->y>0? A->y-1:0;
    ye= A->y<V-1? A->y+1:V-1;
  }
  
  int mins=8;//количество соседей
  int s=0;
  
  int xm=-1,ym=-1;
  //поиск точки с минимальным количеством соседей
  for(iy=yb;iy<=ye;iy++)
  {
    for(ix=xb;ix<=xe;ix++)
    {
      if( chArr[ix][iy] ==1 )
      {
        x0=ix>0? ix-1:0;
        x1=ix<H-1? ix+1:H-1;
        y0=iy>0? iy-1:0;
        y1=iy<V-1? iy+1:V-1;
        s=0;
        for(int yy=y0;yy<=y1;yy++)
        {
          for(int xx=x0;xx<=x1;xx++)
          {
            if(chArr[xx][yy]==1)s++;
          }
        }
        
        if(s<mins){xm=ix;ym=iy;mins=s;};
      }
    }
  }
  
  if(xm==-1)return 0;//не найдено
  A->x=xm;A->y=ym;
  
  return true;//не обнаружено
}


//----------------------------------------------------------------------------------------------------------//
bool getVector(int H,int V,SPOINT* A)//получить вектор от текущей точки
{
  chArr[A->x][A->y]=2;
  
  if(searchPoint(H,V,A,true)==false)return false;
  return true;
}
//----------------------------------------------------------------------------------------------------------//
//сканирование и вывод матричного изображения.vector_mode-флаг прорисовки сплошным(не растр)вектором
void OutMatrix(int H,int V,RECT* rpos_size,DWORD SymColor,uint8_t discr,bool vector_mode )
{
  int scaleX=rpos_size!=NULL?rpos_size->w:400;int scaleY=rpos_size!=NULL?rpos_size->h:400;
  int posX=rpos_size!=NULL?rpos_size->x:0,posY=rpos_size!=NULL?rpos_size->y:0;
  DWORD color=SymColor==0?0xFFFFFF:SymColor;
  bool cont=false;
  static SPOINT str[128];int t=0;uint8_t pointsInSample=0;
  int total=0; bool new_vector=true;
  SPOINT A,B, S;
  
  GetDAC(&A.x,&A.y,true);
  //A.x=X;A.y=Y;
  
loop_point_search:
  if(searchPointFast(H, V,&S,cont)==false)//поиск первой точки
  {
    if(!cont)
    {
      if(t>0)startStreamArray(str,128,discr,&t,&pointsInSample);
      return;
    }
    
    //новый вектор?
    new_vector=true;
    cont=false;goto loop_point_search;
  }
  else
  {
    cont=true;
    chArr[S.x][S.y]=2;
    //if(!pushToStreamArray(str,128,1,&t,&pointsInSample,&O))return;//stop?
    B.x=S.x*scaleX+posX;B.y=S.y*scaleY+posY;B.color=color;
    if((total==0|| new_vector || vector_mode==false) &&( B.x!=A.x || B.y!=A.y))
    {
      B.color=0;
      while(OptimizeVector(&A,&B,&A, discr)==true)
      {
        if(!pushToStreamArray(str,128,discr,&t,&pointsInSample,&A))return;//stop?
        total++;
      }
      if(!pushToStreamArray(str,128,discr,&t,&pointsInSample,&B))return;//stop?
      total++;
      A=B;
      B.color=color;
    }
    new_vector=false;
    
    while(OptimizeVector(&A,&B,&A, discr)==true)
    {
      if(!pushToStreamArray(str,128,discr,&t,&pointsInSample,&A))return;//stop?
      total++;
    }
    
    if(!pushToStreamArray(str,128,discr,&t,&pointsInSample,&B))return;//stop?
    total++;
    if(discr>20 && vector_mode==true)
    {
      if(!pushToStreamArray(str,128,discr,&t,&pointsInSample,&B))return;//stop?
      total++;
    }
    A=B;    
    goto loop_point_search;
  }
   
}
//----------------------------------------------------------------------------------------------------------//

bool searchPointFast(int H,int V,SPOINT* A,bool around)//вокруг?
{
  int ix,iy;
  
  int xb=0,yb=0,xe=H-1,ye=V-1;
  if(around)
  {
    xb= A->x>0? A->x-1:0;
    xe= A->x<H-1? A->x+1:H-1;
    yb= A->y>0? A->y-1:0;
    ye= A->y<V-1? A->y+1:V-1;
  }
  
  for(iy=yb;iy<=ye;iy++)
  {
    for(ix=xb;ix<=xe;ix++)
    {
      if( chArr[ix][iy] ==1 )
      {
        A->x=ix;A->y=iy;
        return true;
      }
    }
  }
  
  return false;//не найдено
}

//----------------------------------------------------------------------------------------------------------//
inline bool startStreamArray(SPOINT* arr,int len,uint8_t discr,int* cnt,uint8_t* pts_pack)//принудительный вывод
{
 return pushToStreamArray(arr,len,discr,cnt,pts_pack,NULL);//запись в массив и вывод в поток при заполнении
}
//----------------------------------------------------------------------------------------------------------//
inline bool pushToStreamArray(SPOINT* arr,int len,uint8_t discr,int* cnt,uint8_t* pts_pack,SPOINT* pt)//запись в массив и вывод в поток при заполнении
{
  if(pt!=NULL)//иначе старт вывода
  {
    arr[*cnt]=*pt;
    (*cnt)++;
  }
  
  if(*cnt>=len || pt==NULL)
  {
    int p=*cnt;
    while(p>0)
    {
      if(*pts_pack==0)*pts_pack=(uint8_t)(p > (uint32_t)discr ? discr : p); 
      if(!insert_SPOINT_to_stream(&arr[*cnt-p],discr,*pts_pack,100000))return false;//stop?
      --(*pts_pack);
      --p;
    }
    *cnt=0;
  }
  
  return true;
}
//----------------------------------------------------------------------------------------------------------//
//оптимизация вектора для текущей дискретизации.Вернет true,если необходимо вставить точку I
bool OptimizeSpeed(SPOINT* A,SPOINT* B,SPOINT* I, uint8_t discr)
{
  static float vx,vy;
  static int dx,dy;
  int optV=1000;//units of ILDA per millisecond
  //(int)( (float)(OPT_DISCR_MS*OPT_DISCR_MS)/(float)((int)discr*(int)discr) *10.0f);
  dx=B->x-A->x;dy=B->y-A->y;
  int max_dx=max(abs(dx),abs(dy));
  float t=1.0f/(float)discr;
  
  vx=(float)dx/t-opt.v0x;vy=(float)dy/t-opt.v0y;//целевая скорость
  
  static float k_Prop;k_Prop=(float)max_dx/(float)optV;
  
  if(k_Prop<=1.0f)//оптимизация не требуется
  {
    I->x=B->x;
    I->y=B->y;
    I->color=B->color;
    return false;
  }
  
  if(k_Prop>2.0f &&((vx>0&&opt.v0x<=0 || vx<0&&opt.v0x>=0)||(vy>0&&opt.v0y<=0 || vy<0&&opt.v0y>=0))&&
     ((A->x!=B->x)||(A->y!=B->y)))//SET ANCHOR
  {
    I->x=A->x;
    I->y=A->y;
    I->color=A->color;
    //vx=-opt.v0x;vy=-opt.v0y;
    vx/=2.0f;vy/=2.0f;
  }
  else
  {
    if(k_Prop>=2.0f)k_Prop*=0.9f;
    I->x=(int)_crd((float)A->x,(float)B->x,k_Prop);
    I->y=(int)(int)_crd((float)A->y,(float)B->y,k_Prop);
    I->color=B->color;
    dx=B->x-I->x,dy=B->y-I->y;
    vx=(float)dx/t-opt.v0x;vy=(float)dy/t-opt.v0y;//полученная ограниченная скорость
  }
  
  opt.v0x+=vx;
  opt.v0y+=vy;
  
  return true;
}
//----------------------------------------------------------------------------------------------------------//
//оптимизация вектора для текущей дискретизации.Вернет true,если необходимо вставить точку I
bool OptimizeVector(SPOINT* A,SPOINT* B,SPOINT* I, uint8_t discr)
{
  int optDist=(int)( (float)(OPT_DISCR_MS*OPT_DISCR_MS)/(float)((int)discr*(int)discr) *10.0f);
  int dx=B->x-A->x,dy=B->y-A->y;
  int max_dx=max(abs(dx),abs(dy));
  float t=1.0f/(float)discr;
  static float vx,vy;
  static float k_Prop;k_Prop=(float)max_dx/(float)optDist;
  
  if(k_Prop<=1.0f)//оптимизация не требуется
  {
    I->x=B->x;
    I->y=B->y;
    I->color=B->color;
    vx=(float)dx/t;vy=(float)dy/t;//скорость
    opt.v0x+=vx;opt.v0y+=vy;
    return false;
  }
  
  SPOINT L=*A;
  if(k_Prop>2.0f)k_Prop=2.0f;
  I->x=(int)_crd((float)A->x,(float)B->x,k_Prop);
  I->y=(int)(int)_crd((float)A->y,(float)B->y,k_Prop);
  I->color=B->color;
  
  dx=I->x-L.x;dy=B->y-L.y;
  vx=(float)dx/t;vy=(float)dy/t;//скорость
  opt.v0x+=vx;opt.v0y+=vy;
  return true;
}
//----------------------------------------------------------------------------------------------------------//
//вывод текста с текущей дискретизацией в позиции и с размером RECT.возврат количества выведенных символов
UINT OutText(RECT* rps,TCHAR* ptx,DWORD color)
{
  RECT rp=rps==NULL?(RECT){0,0,300,300}:*rps;
  int sx=rp.w;int sy=rp.h;
  RECT rin=rp;
  
  int xl=rps!=NULL?rps->x:-32767;
  int yl=rps!=NULL?rps->y:32767;
  
  int cntr=0;
  while(ptx[cntr]!=0){++cntr;}
    
  uint8_t discr=(uint8_t)((cntr*2/10)+10);
  if(discr>MAX_DISCR_MS)discr=MAX_DISCR_MS;
    
  int n=0;
  int W,H;
 
  while(ptx[n]!=0)
  {
    OutVectorChar((unsigned short)ptx[n],&rp,color,1,discr,vect_symbols );
    W=rp.x*sx;H=rp.y*sy;
    
    rin.x+=W;
    if(rin.x>32767-W)
    {
      rin.x=xl;//вправо
      rin.y-=H;//вниз
      if(rin.y<-32767+H)rin.y=yl;
    }
    
    rp=rin;
    ++n;
  }
  
   
  return n;
}
//----------------------------------------------------------------------------------------------------------//