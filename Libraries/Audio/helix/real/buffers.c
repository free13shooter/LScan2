/**************************************************************************************
 * Fixed-point MP3 decoder
 * Jon Recker (jrecker@real.com), Ken Cooke (kenc@real.com)
 * June 2003
 *
 * buffers.c - allocation and freeing of internal MP3 decoder buffers
 *
 * All memory allocation for the codec is done in this file, so if you don't want 
 *  to use other the default system malloc() and free() for heap management this is 
 *  the only file you'll need to change.
 **************************************************************************************/

#include <stdlib.h>		/* for malloc, free */
#include "coder.h"


#define USE_STAT_BUF 0
#define USE_RAM_CCM  1

#if USE_STAT_BUF==0
#include "CCM_MM_RAM.h"
#endif
/**************************************************************************************
 * Function:    ClearBuffer
 *
 * Description: fill buffer with 0's
 *
 * Inputs:      pointer to buffer
 *              number of bytes to fill with 0
 *
 * Outputs:     cleared buffer
 *
 * Return:      none
 *
 * Notes:       slow, platform-independent equivalent to memset(buf, 0, nBytes)
 **************************************************************************************/
static void ClearBuffer(void *buf, int nBytes)
{
	int i;
	unsigned char *cbuf = (unsigned char *)buf;

	for (i = 0; i < nBytes; i++)
		cbuf[i] = 0;

	return;
}

/**************************************************************************************
 * Function:    AllocateBuffers
 *
 * Description: allocate all the memory needed for the MP3 decoder
 *
 * Inputs:      none
 *
 * Outputs:     none
 *
 * Return:      pointer to MP3DecInfo structure (initialized with pointers to all 
 *                the internal buffers needed for decoding, all other members of 
 *                MP3DecInfo structure set to 0)
 *
 * Notes:       if one or more mallocs fail, function frees any buffers already
 *                allocated before returning
 **************************************************************************************/
MP3DecInfo *AllocateBuffers(void)
{
	MP3DecInfo *mp3DecInfo;
	FrameHeader *fh;
	SideInfo *si;
	ScaleFactorInfo *sfi;
	HuffmanInfo *hi;
	DequantInfo *di;
	IMDCTInfo *mi;
	SubbandInfo *sbi;
	
#if USE_STAT_BUF==1
  /*
	 * Use static buffers to make the RAM usage
	 * known at compile time.
	 */
	static MP3DecInfo s_mp3DecInfo;
	static FrameHeader s_fh;
	static SideInfo s_si;
	static ScaleFactorInfo s_sfi;
	static HuffmanInfo s_hi;
	static DequantInfo s_di;
	static IMDCTInfo s_mi;
	static SubbandInfo s_sbi;
  
	mp3DecInfo = &s_mp3DecInfo;
	fh = &s_fh;
	si = &s_si;
	sfi = &s_sfi;
	hi = &s_hi;
	di = &s_di;
	mi = &s_mi;
	sbi = &s_sbi;
#else  
	
#if USE_RAM_CCM==1
  mp3DecInfo = (MP3DecInfo *)ccmalloc(sizeof(MP3DecInfo));
	if (!mp3DecInfo) {
		return 0;
	}
	fh =  (FrameHeader *)     ccmalloc(sizeof(FrameHeader));
	si =  (SideInfo *)        ccmalloc(sizeof(SideInfo));
	sfi = (ScaleFactorInfo *) ccmalloc(sizeof(ScaleFactorInfo));
	hi =  (HuffmanInfo *)     ccmalloc(sizeof(HuffmanInfo));
	di =  (DequantInfo *)     ccmalloc(sizeof(DequantInfo));
	mi =  (IMDCTInfo *)       ccmalloc(sizeof(IMDCTInfo));
	sbi = (SubbandInfo *)     ccmalloc(sizeof(SubbandInfo));
#else
  mp3DecInfo = (MP3DecInfo *)mmmalloc(sizeof(MP3DecInfo));
	if (!mp3DecInfo) {
		return 0;
	}
  fh =  (FrameHeader *)     mmmalloc(sizeof(FrameHeader));
	si =  (SideInfo *)        mmmalloc(sizeof(SideInfo));
	sfi = (ScaleFactorInfo *) mmmalloc(sizeof(ScaleFactorInfo));
	hi =  (HuffmanInfo *)     mmmalloc(sizeof(HuffmanInfo));
	di =  (DequantInfo *)     mmmalloc(sizeof(DequantInfo));
	mi =  (IMDCTInfo *)       mmmalloc(sizeof(IMDCTInfo));
	sbi = (SubbandInfo *)     mmmalloc(sizeof(SubbandInfo));
#endif //#if USE_RAM_CCM==1
  ClearBuffer(mp3DecInfo, sizeof(MP3DecInfo));
#endif
	mp3DecInfo->FrameHeaderPS =     (void *)fh;
	mp3DecInfo->SideInfoPS =        (void *)si;
	mp3DecInfo->ScaleFactorInfoPS = (void *)sfi;
	mp3DecInfo->HuffmanInfoPS =     (void *)hi;
	mp3DecInfo->DequantInfoPS =     (void *)di;
	mp3DecInfo->IMDCTInfoPS =       (void *)mi;
	mp3DecInfo->SubbandInfoPS =     (void *)sbi;

	if (!fh || !si || !sfi || !hi || !di || !mi || !sbi) {
		FreeBuffers(mp3DecInfo);	/* safe to call - only frees memory that was successfully allocated */
		return 0;
	}

	/* important to do this - DSP primitives assume a bunch of state variables are 0 on first use */
	ClearBuffer(fh,  sizeof(FrameHeader));
	ClearBuffer(si,  sizeof(SideInfo));
	ClearBuffer(sfi, sizeof(ScaleFactorInfo));
	ClearBuffer(hi,  sizeof(HuffmanInfo));
	ClearBuffer(di,  sizeof(DequantInfo));
	ClearBuffer(mi,  sizeof(IMDCTInfo));
	ClearBuffer(sbi, sizeof(SubbandInfo));

	return mp3DecInfo;
}

#define SAFE_FREE(x)	{if (x)	mfree(x);	(x) = 0;}	/* helper macro */

/**************************************************************************************
 * Function:    FreeBuffers
 *
 * Description: frees all the memory used by the MP3 decoder
 *
 * Inputs:      pointer to initialized MP3DecInfo structure
 *
 * Outputs:     none
 *
 * Return:      none
 *
 * Notes:       safe to call even if some buffers were not allocated (uses SAFE_FREE)
 **************************************************************************************/
void FreeBuffers(MP3DecInfo *mp3DecInfo)
{
	if (!mp3DecInfo)
		return;
#if USE_STAT_BUF==0
	// Malloc not used, nothing to do
	SAFE_FREE(mp3DecInfo->FrameHeaderPS);
	SAFE_FREE(mp3DecInfo->SideInfoPS);
	SAFE_FREE(mp3DecInfo->ScaleFactorInfoPS);
	SAFE_FREE(mp3DecInfo->HuffmanInfoPS);
	SAFE_FREE(mp3DecInfo->DequantInfoPS);
	SAFE_FREE(mp3DecInfo->IMDCTInfoPS);
	SAFE_FREE(mp3DecInfo->SubbandInfoPS);

	SAFE_FREE(mp3DecInfo)
#endif
}
