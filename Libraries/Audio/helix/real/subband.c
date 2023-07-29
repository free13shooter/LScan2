/**************************************************************************************
 * Fixed-point MP3 decoder
 * Jon Recker (jrecker@real.com), Ken Cooke (kenc@real.com)
 * June 2003
 *
 * subband.c - subband transform (synthesis filterbank implemented via 32-point DCT
 *               followed by polyphase filter)
 **************************************************************************************/

#include "coder.h"
#include "assembly.h"

/**************************************************************************************
 * Function:    Subband
 *
 * Description: do subband transform on all the blocks in one granule, all channels
 *
 * Inputs:      filled MP3DecInfo structure, after calling IMDCT for all channels
 *              vbuf[ch] and vindex[ch] must be preserved between calls
 *
 * Outputs:     decoded PCM data, interleaved LRLRLR... if stereo
 *
 * Return:      0 on success,  -1 if null input pointers
 **************************************************************************************/
int Subband(MP3DecInfo *mp3DecInfo, short *pcmBuf)
{
	int b;
	HuffmanInfo *hi;
	IMDCTInfo *mi;
	SubbandInfo *sbi;

	/* validate pointers */
	if (!mp3DecInfo || !mp3DecInfo->HuffmanInfoPS || !mp3DecInfo->IMDCTInfoPS || !mp3DecInfo->SubbandInfoPS)
		return -1;

	hi = (HuffmanInfo *)mp3DecInfo->HuffmanInfoPS;
	mi = (IMDCTInfo *)(mp3DecInfo->IMDCTInfoPS);
	sbi = (SubbandInfo*)(mp3DecInfo->SubbandInfoPS);

	if (mp3DecInfo->nChans == 2) {
		/* stereo */
		for (b = 0; b < BLOCK_SIZE; b++) {
			FDCT32(mi->outBuf[0][b], sbi->vbuf + 0*32, sbi->vindex, (b & 0x01), mi->gb[0]);
			FDCT32(mi->outBuf[1][b], sbi->vbuf + 1*32, sbi->vindex, (b & 0x01), mi->gb[1]);
			PolyphaseStereo(pcmBuf, sbi->vbuf + sbi->vindex + VBUF_LENGTH * (b & 0x01), polyCoef);
			sbi->vindex = (sbi->vindex - (b & 0x01)) & 7;
			pcmBuf += (2 * NBANDS);
		}
	} else {
		/* mono */
		for (b = 0; b < BLOCK_SIZE; b++) {
			FDCT32(mi->outBuf[0][b], sbi->vbuf + 0*32, sbi->vindex, (b & 0x01), mi->gb[0]);
			PolyphaseMono(pcmBuf, sbi->vbuf + sbi->vindex + VBUF_LENGTH * (b & 0x01), polyCoef);
			sbi->vindex = (sbi->vindex - (b & 0x01)) & 7;
			pcmBuf += NBANDS;
		}
	}

	return 0;
}

