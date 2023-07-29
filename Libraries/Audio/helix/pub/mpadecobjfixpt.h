#ifndef _MPADECOBJFIXPT_H_
#define _MPADECOBJFIXPT_H_

#include "mp3dec.h"		/* public C API for new MP3 decoder */

class CMpaDecObj
{
public:
    CMpaDecObj();
    ~CMpaDecObj();

    ///////////////////////////////////////////////////////////////////////////
    // Function:    Init_n
    // Purpose:     Initialize the mp3 decoder.
    // Parameters:  pSync       a pointer to a syncword
    //              ulSize      the size of the buffer pSync points to
    //              bUseSize    this tells the decoder to use the input frame
    //                          size on the decode instead of calculating
    //                          the frame size.  This is necessary when
    //                          our formatted mp3 data (main_data_begin always
    //                          equal to 0).
    //
    // Returns:     returns 1 on success, 0 on failure
    ///////////////////////////////////////////////////////////////////////////
    int     Init_n(unsigned char *pSync,
                   unsigned long ulSize,
                   unsigned char bUseSize=0);

    ///////////////////////////////////////////////////////////////////////////
    // Function:    DecodeFrame_v
    // Purpose:     Decodes one mp3 frame
    // Parameters:  pSource     pointer to an mp3 frame (at a syncword)
    //              pulSize     size of the buffer pSource points to.  It will
    //                          contain the number of mp3 bytes decoded upon
    //                          return.
    //              pPCM        pointer to a buffer to decode into
    //              pulPCMSize  size of the PCM buffer.  It will contain the
    //                          number of PCM bytes prodced upon return.
    ///////////////////////////////////////////////////////////////////////////
    void    DecodeFrame_v(unsigned char *pSource,
              	          unsigned long *pulSize,
                   	      unsigned char *pPCM,
                       	  unsigned long *pulPCMSize);

	// overloaded new version that returns error code in errCode
    void    DecodeFrame_v(unsigned char *pSource,
              	          unsigned long *pulSize,
                   	      unsigned char *pPCM,
                       	  unsigned long *pulPCMSize, 
						  int *errCode);

    void    GetPCMInfo_v(unsigned long &ulSampRate,
                         int &nChannels,
                         int &nBitsPerSample);

	// return number of samples per frame, PER CHANNEL (renderer multiplies this result by nChannels)
    int     GetSamplesPerFrame_n();

    void    SetTrustPackets(unsigned char bTrust) { m_bTrustPackets = bTrust; }

private:
	void *              m_pDec;		// generic void ptr

	void *				m_pDecL1;	// not implemented (could use old Xing mpadecl1.cpp)
	void *				m_pDecL2;	// not implemented (could use old Xing mpadecl2.cpp)
	HMP3Decoder			m_pDecL3;

	MP3FrameInfo		m_lastMP3FrameInfo;
	unsigned char		m_bUseFrameSize;
        unsigned char           m_bTrustPackets;
        int                     decoder_state;
};

#endif	/* _MPADECOBJFIXPT_H_ */
