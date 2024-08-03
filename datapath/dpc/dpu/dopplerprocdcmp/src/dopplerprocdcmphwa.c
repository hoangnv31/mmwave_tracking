/**
 *   @file  dopplerprocdcmphwa.c
 *
 *   @brief
 *      Implements Data path Doppler processing Unit using HWA.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2018 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* mmWave SDK driver/common Include Files */
#include <ti/common/sys_common.h>
#include <ti/drivers/osal/SemaphoreP.h>
#include <ti/drivers/osal/MemoryP.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/esm/esm.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/hwa/hwa.h>

/* Utils */
#include <ti/utils/mathutils/mathutils.h>

/* Compression constants. */
#include <ti/demo/xwr64xx_compression/mmw/compressionConsts.h>

/* Data Path Include files */
#include <ti/datapath/dpedma/dpedma.h>
#include <ti/datapath/dpedma/dpedmahwa.h>
#include <ti/datapath/dpc/dpu/dopplerprocdcmp/dopplerprocdcmphwa.h>
#include <ti/datapath/dpc/dpu/dopplerprocdcmp/include/dopplerprocdcmphwainternal.h>


/* Flag to check input parameters */
#define DEBUG_CHECK_PARAMS   1

/* HWA ping/pong buffers offset */
#define DPU_DOPPLERPROCDCMPHWA_SRC_PING_EDMA_OFFSET   (obj->hwaMemBankAddr[0])
#define DPU_DOPPLERPROCDCMPHWA_SRC_PONG_EDMA_OFFSET   (obj->hwaMemBankAddr[1])
#define DPU_DOPPLERPROCDCMPHWA_DCMP_DST_PING_EDMA_OFFSET   (obj->hwaMemBankAddr[2])
#define DPU_DOPPLERPROCDCMPHWA_FFT_DST_PING_EDMA_OFFSET   (obj->hwaMemBankAddr[3])
#define DPU_DOPPLERPROCDCMPHWA_DCMP_DST_PONG_EDMA_OFFSET   (obj->hwaMemBankAddr[2])
#define DPU_DOPPLERPROCDCMPHWA_FFT_DST_PONG_EDMA_OFFSET   (obj->hwaMemBankAddr[3])

#define DPU_DOPPLERPROCDCMPHWA_SRC_PING_OFFSET   ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCDCMPHWA_SRC_PING_EDMA_OFFSET)
#define DPU_DOPPLERPROCDCMPHWA_SRC_PONG_OFFSET   ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCDCMPHWA_SRC_PONG_EDMA_OFFSET)
#define DPU_DOPPLERPROCDCMPHWA_DCMP_DST_PING_OFFSET   ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCDCMPHWA_DCMP_DST_PING_EDMA_OFFSET)
#define DPU_DOPPLERPROCDCMPHWA_FFT_DST_PING_OFFSET   ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCDCMPHWA_FFT_DST_PING_EDMA_OFFSET)
#define DPU_DOPPLERPROCDCMPHWA_DCMP_DST_PONG_OFFSET   ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCDCMPHWA_DCMP_DST_PONG_EDMA_OFFSET)
#define DPU_DOPPLERPROCDCMPHWA_FFT_DST_PONG_OFFSET   ADDR_TRANSLATE_CPU_TO_HWA(DPU_DOPPLERPROCDCMPHWA_FFT_DST_PONG_EDMA_OFFSET)
#define DPU_DOPPLERPROCDCMPHWA_SUMABS_DST_PING_OFFSET	DPU_DOPPLERPROCDCMPHWA_SRC_PING_OFFSET
#define DPU_DOPPLERPROCDCMPHWA_SUMABS_DST_PONG_OFFSET	DPU_DOPPLERPROCDCMPHWA_SRC_PONG_OFFSET

#define DPU_DOPPLERPROCDCMPHWA_SUMABS1_DST_PING_OFFSET	DPU_DOPPLERPROCDCMPHWA_DCMP_DST_PING_OFFSET
#define DPU_DOPPLERPROCDCMPHWA_SUMABS1_DST_PONG_OFFSET	DPU_DOPPLERPROCDCMPHWA_DCMP_DST_PONG_OFFSET
#define DPU_DOPPLERPROCDCMPHWA_SUMABS2_DST_PING_OFFSET	DPU_DOPPLERPROCDCMPHWA_SRC_PING_OFFSET
#define DPU_DOPPLERPROCDCMPHWA_SUMABS2_DST_PONG_OFFSET	DPU_DOPPLERPROCDCMPHWA_SRC_PONG_OFFSET


/*===========================================================
 *                    Internal Functions
 *===========================================================*/
void  cfgEGEParamListDopplerProc (uint8_t * cmpEGEArr, uint16_t compressionRatio, uint16_t srcWidth);

/**
 *  @b Description
 *  @n
 *      HWA processing completion call back function.
 *  \ingroup    DPU_DOPPLERPROCDCMP_INTERNAL_FUNCTION
 */
static void DPU_DopplerProcDcmpHWA_hwaDoneIsrCallback(void * arg)
{
    if (arg != NULL) {
        SemaphoreP_post((SemaphoreP_Handle)arg);
    }
}

/**
 *  @b Description
 *  @n
 *      EDMA completion call back function.
 *  \ingroup    DPU_DOPPLERPROCDCMP_INTERNAL_FUNCTION
 */
static void DPU_DopplerProcDcmpHWA_edmaDoneIsrCallback(uintptr_t arg,
    uint8_t transferCompletionCode)
{
    if (arg != NULL) {
        SemaphoreP_post((SemaphoreP_Handle)arg);
    }
}

/**
 *  @b Description
 *  @n
 *      Configures HWA for Doppler processing.
 *
 *  @param[in] obj    - DPU obj
 *  @param[in] cfg    - DPU configuration
 *
 *  \ingroup    DPU_DOPPLERPROCDCMP_INTERNAL_FUNCTION
 *
 *  @retval error code.
 */ 
	
static inline int32_t DPU_DopplerProcDcmpHWA_configHwa
(
    DPU_DopplerProcDcmpHWA_Obj      *obj,
    DPU_DopplerProcDcmpHWA_Config   *cfg
)
{
    HWA_InterruptConfig     paramISRConfig;
    uint32_t                paramsetIdx = 0;;
	uint32_t                pingDCMPparamsetIdx, pingFFTparamsetIdx, pingSumparamsetIdx,pingSumparamsetIdx_1,pingSumparamsetIdx_2;
	uint16_t numSamplesPerBlockOut;
	uint16_t numSamplesPerBlockIn ; 
    int32_t                 retVal = 0U;
    uint8_t                 destChan;
	uint16_t 				compressionRatio  = cfg->staticCfg.compressCfg.ratio;
	uint16_t 				rangeBinIdx;
	HWA_ParamConfig         hwaDopParamCfg[DPU_DOPPLERPROCDCMPHWA_MAX_NUM_HWA_PARAMSET];

    numSamplesPerBlockOut = cfg->staticCfg.numRxAntennas * cfg->staticCfg.compressCfg.numRangeBinsPerBlock;
    numSamplesPerBlockIn = (uint16_t) ((numSamplesPerBlockOut * (uint32_t)compressionRatio) >> HWA_CMP_RATIO_BW);

    uint16_t numBlocks = cfg->staticCfg.numDopplerChirps*cfg->staticCfg.numTxAntennas;
	/* Check if we have the correct number of paramsets.*/
   
  
    memset((void*) &hwaDopParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    /************* Decompression for all TX antennas *********************************/
	pingDCMPparamsetIdx = paramsetIdx;
    hwaDopParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_DMA; 
    hwaDopParamCfg[paramsetIdx].dmaTriggerSrc = obj->hwaDmaTriggerSourcePing; 
    hwaDopParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_COMPRESS; 
    
    hwaDopParamCfg[paramsetIdx].source.srcAddr = DPU_DOPPLERPROCDCMPHWA_SRC_PING_OFFSET;
    hwaDopParamCfg[paramsetIdx].source.srcShift = 0;
    hwaDopParamCfg[paramsetIdx].source.srcCircShiftWrap = 0;
    hwaDopParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaDopParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
    hwaDopParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_UNSIGNED;
    hwaDopParamCfg[paramsetIdx].source.srcConjugate = 0;
    hwaDopParamCfg[paramsetIdx].source.srcScale = 0;
    hwaDopParamCfg[paramsetIdx].source.bpmEnable = 0;
    hwaDopParamCfg[paramsetIdx].source.bpmPhase = 0;

    hwaDopParamCfg[paramsetIdx].dest.dstAddr = DPU_DOPPLERPROCDCMPHWA_DCMP_DST_PING_OFFSET;
    hwaDopParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaDopParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
    hwaDopParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED; 
    hwaDopParamCfg[paramsetIdx].dest.dstConjugate = 0; 
    hwaDopParamCfg[paramsetIdx].dest.dstScale = 0;
    hwaDopParamCfg[paramsetIdx].dest.dstSkipInit = 0; 

    hwaDopParamCfg[paramsetIdx].accelModeArgs.compressMode.compressDecompress  = HWA_CMP_DCMP_COMPRESS;
    hwaDopParamCfg[paramsetIdx].accelModeArgs.compressMode.method   = HWA_COMPRESS_METHOD_EGE;
    hwaDopParamCfg[paramsetIdx].accelModeArgs.compressMode.ditherEnable = HWA_FEATURE_BIT_ENABLE;
    hwaDopParamCfg[paramsetIdx].accelModeArgs.compressMode.passSelect  = HWA_COMPRESS_PATHSELECT_BOTHPASSES;
    hwaDopParamCfg[paramsetIdx].accelModeArgs.compressMode.headerEnable = HWA_FEATURE_BIT_ENABLE;
    hwaDopParamCfg[paramsetIdx].accelModeArgs.compressMode.scaleFactorBW = 4;
    hwaDopParamCfg[paramsetIdx].accelModeArgs.compressMode.EGEKarrayLength = 3;

    /* HWA range CMP src/dst configuration*/
    hwaDopParamCfg[paramsetIdx].source.srcAcnt = numSamplesPerBlockIn- 1; 
    hwaDopParamCfg[paramsetIdx].source.srcAIdx = sizeof(uint32_t);
    hwaDopParamCfg[paramsetIdx].source.srcBcnt = numBlocks-1;
    hwaDopParamCfg[paramsetIdx].source.srcBIdx = numSamplesPerBlockIn*hwaDopParamCfg[paramsetIdx].source.srcAIdx;

    hwaDopParamCfg[paramsetIdx].dest.dstAcnt = numSamplesPerBlockOut-1; 
    hwaDopParamCfg[paramsetIdx].dest.dstAIdx = sizeof(cmplx16ImRe_t); 
    hwaDopParamCfg[paramsetIdx].dest.dstBIdx = numSamplesPerBlockOut*hwaDopParamCfg[paramsetIdx].dest.dstAIdx; 

	retVal = HWA_configParamSet(obj->hwaHandle, 
                                cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx, 
                                &hwaDopParamCfg[paramsetIdx], NULL);

	if (retVal != 0)
    {
		retVal = -10100;
        goto exit;
    }
	/* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(obj->hwaHandle, 
                                          cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
		retVal = -10100+1;
        goto exit;
    }
	
    /************* Doppler FFT for all TX antennas *********************************/
    paramsetIdx++;
	pingFFTparamsetIdx = paramsetIdx;
    memset( (void*) &hwaDopParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
    hwaDopParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE; 
    hwaDopParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT; 

    hwaDopParamCfg[paramsetIdx].source.srcAddr = DPU_DOPPLERPROCDCMPHWA_DCMP_DST_PING_OFFSET; // address is relative to start of MEM0
    hwaDopParamCfg[paramsetIdx].source.srcAcnt = cfg->staticCfg.numDopplerChirps - 1; //size in samples - 1

    hwaDopParamCfg[paramsetIdx].source.srcAIdx = cfg->staticCfg.numVirtualAntennas * cfg->staticCfg.compressCfg.numRangeBinsPerBlock * sizeof(cmplx16ImRe_t); 
    hwaDopParamCfg[paramsetIdx].source.srcBcnt = (cfg->staticCfg.numVirtualAntennas * cfg->staticCfg.compressCfg.numRangeBinsPerBlock) - 1; 
    hwaDopParamCfg[paramsetIdx].source.srcBIdx = sizeof(cmplx16ImRe_t); 
    hwaDopParamCfg[paramsetIdx].source.srcShift = 0; 
    hwaDopParamCfg[paramsetIdx].source.srcCircShiftWrap = 0; 
    hwaDopParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
    hwaDopParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; 
    hwaDopParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED; 
    hwaDopParamCfg[paramsetIdx].source.srcConjugate = 0; //no conjugate
    hwaDopParamCfg[paramsetIdx].source.srcScale = 0;
    hwaDopParamCfg[paramsetIdx].source.bpmEnable = 0; 
    hwaDopParamCfg[paramsetIdx].source.bpmPhase = 0; 

    hwaDopParamCfg[paramsetIdx].dest.dstAddr = DPU_DOPPLERPROCDCMPHWA_FFT_DST_PING_OFFSET; // address is relative to start of MEM0
    hwaDopParamCfg[paramsetIdx].dest.dstAcnt = cfg->staticCfg.numDopplerBins - 1; //this is samples - 1
    hwaDopParamCfg[paramsetIdx].dest.dstAIdx = cfg->staticCfg.numVirtualAntennas * cfg->staticCfg.compressCfg.numRangeBinsPerBlock * sizeof(uint16_t); 
    hwaDopParamCfg[paramsetIdx].dest.dstBIdx = sizeof(uint16_t);
    hwaDopParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL; //Real
    hwaDopParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT; 
    hwaDopParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_UNSIGNED; 
    hwaDopParamCfg[paramsetIdx].dest.dstConjugate = 0; //no conjugate
    hwaDopParamCfg[paramsetIdx].dest.dstScale = 0;
    hwaDopParamCfg[paramsetIdx].dest.dstSkipInit = 0; 

    hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 1;
    hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = cfg->staticCfg.log2NumDopplerBins;

    /* scaling is enabled in all stages except for the first stage which is defined by user*/
    if(cfg->hwRes.hwaCfg.firstStageScaling == DPU_DOPPLERPROCDCMPHWA_FIRST_SCALING_DISABLED)
    {
        /* Enable scaling on all stages except first one.*/
        hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = (cfg->staticCfg.numDopplerBins - 1) >> 1;
    }    
    else
    {
        /* Enable scaling on all stages.*/
        hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = (cfg->staticCfg.numDopplerBins - 1);
    }    

    hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.interfZeroOutEn = 0; //disabled
    hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.windowEn = 1; //enabled
    hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.windowStart = cfg->hwRes.hwaCfg.winRamOffset; 
    hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.winSymm = cfg->hwRes.hwaCfg.winSym; 
    hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.winInterpolateMode = 0; //fftsize is less than 1K
    hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_ENABLED;
    hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;

    hwaDopParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
	
	retVal = HWA_configParamSet(obj->hwaHandle, 
                                cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx, 
                                &hwaDopParamCfg[paramsetIdx], NULL);
    if (retVal != 0)
    {
		retVal = -10100+2;
        goto exit;
    }

    /* Make sure DMA interrupt/trigger is disabled for this paramset*/
    retVal = HWA_disableParamSetInterrupt(obj->hwaHandle, 
                                          cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                          HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
    if (retVal != 0)
    {
		retVal = -10100+4;
        goto exit;
    }
	
	if (cfg->staticCfg.compressCfg.numRangeBinsPerBlock > 1)
	{
		/* Integrate across Rx Antennas. */
		paramsetIdx++;
		pingSumparamsetIdx_1 = paramsetIdx;
		
		memset( (void*) &hwaDopParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
		hwaDopParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE; 
		hwaDopParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT; 

		hwaDopParamCfg[paramsetIdx].source.srcAddr = DPU_DOPPLERPROCDCMPHWA_FFT_DST_PING_OFFSET;
		hwaDopParamCfg[paramsetIdx].source.srcAcnt = (cfg->staticCfg.numRxAntennas)-1; //size in samples - 1

		hwaDopParamCfg[paramsetIdx].source.srcAIdx = sizeof(uint16_t); 
		hwaDopParamCfg[paramsetIdx].source.srcBcnt = (cfg->staticCfg.numTxAntennas*cfg->staticCfg.numDopplerBins* cfg->staticCfg.compressCfg.numRangeBinsPerBlock) - 1; 
		hwaDopParamCfg[paramsetIdx].source.srcBIdx = cfg->staticCfg.numRxAntennas * sizeof(uint16_t) ; 
		hwaDopParamCfg[paramsetIdx].source.srcShift = 0; 
		hwaDopParamCfg[paramsetIdx].source.srcCircShiftWrap = 0; 
		hwaDopParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL; //real data
		hwaDopParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; 
		hwaDopParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_UNSIGNED; 
		hwaDopParamCfg[paramsetIdx].source.srcConjugate = 0; //no conjugate
		hwaDopParamCfg[paramsetIdx].source.srcScale = 3;
		hwaDopParamCfg[paramsetIdx].source.bpmEnable = 0; 
		hwaDopParamCfg[paramsetIdx].source.bpmPhase = 0; 

		hwaDopParamCfg[paramsetIdx].dest.dstAddr = DPU_DOPPLERPROCDCMPHWA_SUMABS1_DST_PING_OFFSET; // address is relative to start of MEM0
			
		hwaDopParamCfg[paramsetIdx].dest.dstAcnt = 1 - 1; //get only bin zero
		hwaDopParamCfg[paramsetIdx].dest.dstAIdx = sizeof(uint16_t); 
		hwaDopParamCfg[paramsetIdx].dest.dstBIdx = sizeof(uint16_t); // size of one output sample
		hwaDopParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL; 
		hwaDopParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT; 
		hwaDopParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_UNSIGNED; 
		hwaDopParamCfg[paramsetIdx].dest.dstConjugate = 0; //no conjugate
		hwaDopParamCfg[paramsetIdx].dest.dstScale = 7;
		hwaDopParamCfg[paramsetIdx].dest.dstSkipInit = 0; // no skipping

		if(cfg->staticCfg.numRxAntennas == 1)
		{
			/*If number of virtual antennas is 1, do not use FFT to compute sum magnitude.*/
			hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 0;
			hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = 1;
		}
		else
		{
			hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 1;
			hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(cfg->staticCfg.numRxAntennas);
		}
		hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 0x3FF; 
		hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.interfZeroOutEn = 0; 
		hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.windowEn = 0; 
		hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.windowStart = 0; 
		hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.winSymm = cfg->hwRes.hwaCfg.winSym; 
		hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.winInterpolateMode = 0; 
		hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED; 
		hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT; 

		hwaDopParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
		retVal = HWA_configParamSet(obj->hwaHandle, 
									cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
									&hwaDopParamCfg[paramsetIdx], 
									NULL);
		if (retVal != 0)
		{
			retVal = -10100+50 ;
			goto exit;
		}

		/* First, make sure all DMA interrupt/trigger are disabled for this paramset*/
		retVal = HWA_disableParamSetInterrupt(obj->hwaHandle, 
											  cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
											  HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
		if (retVal != 0)
		{
			retVal = -10100+60 ;
			goto exit;
		}
	
		/* Integrate across Tx Antennas. */
		for (rangeBinIdx = 0; rangeBinIdx < cfg->staticCfg.compressCfg.numRangeBinsPerBlock; rangeBinIdx++)
		{
			paramsetIdx++;
			if (rangeBinIdx == 0)
			{
				pingSumparamsetIdx_2 = paramsetIdx;
			}
			memset( (void*) &hwaDopParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
		
			hwaDopParamCfg[paramsetIdx] = hwaDopParamCfg[pingSumparamsetIdx_1];

			hwaDopParamCfg[paramsetIdx].source.srcAddr = DPU_DOPPLERPROCDCMPHWA_SUMABS1_DST_PING_OFFSET + (sizeof(uint16_t)*rangeBinIdx);
			hwaDopParamCfg[paramsetIdx].source.srcAcnt = (cfg->staticCfg.numTxAntennas)-1; //size in samples - 1

			hwaDopParamCfg[paramsetIdx].source.srcAIdx = sizeof(uint16_t)*cfg->staticCfg.compressCfg.numRangeBinsPerBlock; 
			hwaDopParamCfg[paramsetIdx].source.srcBcnt = (cfg->staticCfg.numDopplerBins) - 1; 
			hwaDopParamCfg[paramsetIdx].source.srcBIdx = sizeof(uint16_t)*cfg->staticCfg.compressCfg.numRangeBinsPerBlock*cfg->staticCfg.numTxAntennas; 
			hwaDopParamCfg[paramsetIdx].source.srcScale = 3;
			
			hwaDopParamCfg[paramsetIdx].dest.dstAddr = DPU_DOPPLERPROCDCMPHWA_SUMABS2_DST_PING_OFFSET + (sizeof(uint16_t)*cfg->staticCfg.numDopplerBins*rangeBinIdx); // address is relative to start of MEM0
				
			hwaDopParamCfg[paramsetIdx].dest.dstAcnt = 1 - 1; //get only bin zero
			hwaDopParamCfg[paramsetIdx].dest.dstAIdx = sizeof(uint16_t); 
			hwaDopParamCfg[paramsetIdx].dest.dstBIdx = sizeof(uint16_t); // size of one output sample
			hwaDopParamCfg[paramsetIdx].dest.dstScale = 6;

			if(cfg->staticCfg.numTxAntennas == 1)
			{
				/*If number of virtual antennas is 1, do not use FFT to compute sum magnitude.*/
				hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 0;
				hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = 1;
			}
			else
			{
				hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 1;
				hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(cfg->staticCfg.numTxAntennas);
			}
			
			retVal = HWA_configParamSet(obj->hwaHandle, 
										cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
										&hwaDopParamCfg[paramsetIdx], 
										NULL);
			if (retVal != 0)
			{
				retVal = -10100+50 + rangeBinIdx;
				goto exit;
			}

			/* First, make sure all DMA interrupt/trigger are disabled for this paramset*/
			retVal = HWA_disableParamSetInterrupt(obj->hwaHandle, 
												  cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
												  HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
			if (retVal != 0)
			{
				retVal = -10100+60 + rangeBinIdx;
				goto exit;
			}
		}
	}
	else
    {
		/* Absolute sum across all Tx Antennas. */
		paramsetIdx++;
		if (rangeBinIdx == 0)
		{
			pingSumparamsetIdx = paramsetIdx;
		}

		memset( (void*) &hwaDopParamCfg[paramsetIdx], 0, sizeof(HWA_ParamConfig));
		hwaDopParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE; 
		hwaDopParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT; 

		hwaDopParamCfg[paramsetIdx].source.srcAddr = DPU_DOPPLERPROCDCMPHWA_FFT_DST_PING_OFFSET; // address is relative to start of MEM0
		hwaDopParamCfg[paramsetIdx].source.srcAcnt = (cfg->staticCfg.numVirtualAntennas)-1; //size in samples - 1

		hwaDopParamCfg[paramsetIdx].source.srcAIdx = sizeof(uint16_t); 
		hwaDopParamCfg[paramsetIdx].source.srcBcnt = (cfg->staticCfg.numDopplerBins) - 1; 
		hwaDopParamCfg[paramsetIdx].source.srcBIdx = cfg->staticCfg.numVirtualAntennas * sizeof(uint16_t); 
		hwaDopParamCfg[paramsetIdx].source.srcShift = 0; 
		hwaDopParamCfg[paramsetIdx].source.srcCircShiftWrap = 0; 
		hwaDopParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_REAL; //real data
		hwaDopParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; 
		hwaDopParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_UNSIGNED; 
		hwaDopParamCfg[paramsetIdx].source.srcConjugate = 0; //no conjugate
		hwaDopParamCfg[paramsetIdx].source.srcScale = 3;
		hwaDopParamCfg[paramsetIdx].source.bpmEnable = 0; 
		hwaDopParamCfg[paramsetIdx].source.bpmPhase = 0; 

		hwaDopParamCfg[paramsetIdx].dest.dstAddr = DPU_DOPPLERPROCDCMPHWA_SUMABS_DST_PING_OFFSET; // address is relative to start of MEM0
			
		hwaDopParamCfg[paramsetIdx].dest.dstAcnt = 1 - 1; //get only bin zero
		hwaDopParamCfg[paramsetIdx].dest.dstAIdx = sizeof(uint16_t); 
		hwaDopParamCfg[paramsetIdx].dest.dstBIdx = sizeof(uint16_t); // size of one output sample
		hwaDopParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL; 
		hwaDopParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT; 
		hwaDopParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_UNSIGNED; 
		hwaDopParamCfg[paramsetIdx].dest.dstConjugate = 0; //no conjugate
		hwaDopParamCfg[paramsetIdx].dest.dstScale = 8;
		hwaDopParamCfg[paramsetIdx].dest.dstSkipInit = 0; // no skipping

		if(cfg->staticCfg.numVirtualAntennas == 1)
		{
			/*If number of virtual antennas is 1, do not use FFT to compute sum magnitude.*/
			hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 0;
			hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = 1;
		}
		else
		{
			hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 1;
			hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(cfg->staticCfg.numVirtualAntennas);
		}
		hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 0x3FF; 
		hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.interfZeroOutEn = 0; 
		hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.windowEn = 0; 
		hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.windowStart = 0; 
		hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.winSymm = cfg->hwRes.hwaCfg.winSym; 
		hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.winInterpolateMode = 0; 
		hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED; 
		hwaDopParamCfg[paramsetIdx].accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT; 

		hwaDopParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
		retVal = HWA_configParamSet(obj->hwaHandle, 
									cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
									&hwaDopParamCfg[paramsetIdx], 
									NULL);
		if (retVal != 0)
		{
			retVal = -10100+50 + rangeBinIdx;
			goto exit;
		}

		/* First, make sure all DMA interrupt/trigger are disabled for this paramset*/
		retVal = HWA_disableParamSetInterrupt(obj->hwaHandle, 
											  cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
											  HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
		if (retVal != 0)
		{
			retVal = -10100+60 + rangeBinIdx;
			goto exit;
		}
	}	
    /************ Enable the DMA hookup to this paramset so that data gets copied out ***********/
    

    retVal = HWA_getDMAChanIndex(obj->hwaHandle, 
                                 cfg->hwRes.edmaCfg.edmaOut.ping.channel,
                                 &destChan);
    if (retVal != 0)
    {
        retVal = -10100+7;
		goto exit;
    }
    /* Now enable interrupt */
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
    paramISRConfig.dma.dstChannel = destChan;
    paramISRConfig.cpu.callbackArg = NULL;
    retVal = HWA_enableParamSetInterrupt(obj->hwaHandle, cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx, &paramISRConfig);
    if (retVal != 0)
    {
        retVal = -1009;
        goto exit;
    }
    paramsetIdx++;

    /******************** programming HWACC for the pong buffer ****************************/
    /************* Doppler DCMP for all TX antennas on pong *********************************/
	hwaDopParamCfg[paramsetIdx] = hwaDopParamCfg[pingDCMPparamsetIdx]; // Decompression param-set
	hwaDopParamCfg[paramsetIdx].dmaTriggerSrc = obj->hwaDmaTriggerSourcePong;
	
	hwaDopParamCfg[paramsetIdx].source.srcAddr = DPU_DOPPLERPROCDCMPHWA_SRC_PONG_OFFSET;
	hwaDopParamCfg[paramsetIdx].dest.dstAddr = DPU_DOPPLERPROCDCMPHWA_DCMP_DST_PONG_OFFSET;

	retVal = HWA_configParamSet(obj->hwaHandle,
							cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
							&hwaDopParamCfg[paramsetIdx],
							NULL);
	if (retVal != 0)
	{
		retVal = -1009;
        goto exit;
	}
			
	/* Make sure DMA interrupt/trigger is disabled for this paramset*/
	retVal = HWA_disableParamSetInterrupt(obj->hwaHandle, 
										  cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
										  HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
	if (retVal != 0)
	{
		retVal = -1009;
        goto exit;
	}
	
	/************* Doppler FFT for all TX antennas on pong *********************************/
	paramsetIdx	++;
	hwaDopParamCfg[paramsetIdx] = hwaDopParamCfg[pingFFTparamsetIdx]; // FFT param-set
	hwaDopParamCfg[paramsetIdx].source.srcAddr   = DPU_DOPPLERPROCDCMPHWA_DCMP_DST_PONG_OFFSET;
	hwaDopParamCfg[paramsetIdx].dest.dstAddr   = DPU_DOPPLERPROCDCMPHWA_FFT_DST_PONG_OFFSET;

  
	retVal = HWA_configParamSet(obj->hwaHandle,
								cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
								&hwaDopParamCfg[paramsetIdx],
								NULL);
	if (retVal != 0)
	{
		retVal = -1010;
        
		goto exit;
	}

	/* Make sure DMA interrupt/trigger is disabled for this paramset*/
	retVal = HWA_disableParamSetInterrupt(obj->hwaHandle, 
										  cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx ,
										  HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
	if (retVal != 0)
	{
		retVal = -1011;
        goto exit;
	}
        
    /************* Sum of magnitudes for pong*********************************/
    if (cfg->staticCfg.compressCfg.numRangeBinsPerBlock > 1)
	{
		paramsetIdx	++;
		
		hwaDopParamCfg[paramsetIdx] = hwaDopParamCfg[pingSumparamsetIdx_1];
		hwaDopParamCfg[paramsetIdx].source.srcAddr = DPU_DOPPLERPROCDCMPHWA_FFT_DST_PONG_OFFSET;
		hwaDopParamCfg[paramsetIdx].dest.dstAddr   = DPU_DOPPLERPROCDCMPHWA_SUMABS1_DST_PONG_OFFSET;

		retVal = HWA_configParamSet(obj->hwaHandle,
									cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
									&hwaDopParamCfg[paramsetIdx],
									NULL);
		if (retVal != 0)
		{
			retVal = -1012;
			goto exit;
		}

		/* First, make sure all DMA interrupt/trigger are disabled for this paramset*/
		retVal = HWA_disableParamSetInterrupt(obj->hwaHandle, 
											  cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
											  HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
		if (retVal != 0)
		{
			retVal = -1013;
			goto exit;
		}

		for (rangeBinIdx = 0; rangeBinIdx < cfg->staticCfg.compressCfg.numRangeBinsPerBlock; rangeBinIdx++)
		{
			paramsetIdx	++;
			
			hwaDopParamCfg[paramsetIdx] = hwaDopParamCfg[pingSumparamsetIdx_2];
			hwaDopParamCfg[paramsetIdx].source.srcAddr = DPU_DOPPLERPROCDCMPHWA_SUMABS1_DST_PONG_OFFSET + (sizeof(uint16_t)*rangeBinIdx);
			hwaDopParamCfg[paramsetIdx].dest.dstAddr   = 
			DPU_DOPPLERPROCDCMPHWA_SUMABS2_DST_PONG_OFFSET + (sizeof(uint16_t)*cfg->staticCfg.numDopplerBins*rangeBinIdx);
			// address is relative to start of MEM0
				
			retVal = HWA_configParamSet(obj->hwaHandle,
									cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
									&hwaDopParamCfg[paramsetIdx],
									NULL);
			if (retVal != 0)
			{
				retVal = -1012;
				goto exit;
			}

			/* First, make sure all DMA interrupt/trigger are disabled for this paramset*/
			retVal = HWA_disableParamSetInterrupt(obj->hwaHandle, 
												  cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
												  HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
			if (retVal != 0)
			{
				retVal = -1013;
				goto exit;
			}

		}
		retVal = HWA_configParamSet(obj->hwaHandle,
									cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
									&hwaDopParamCfg[paramsetIdx],
									NULL);
		if (retVal != 0)
		{
			retVal = -1012;
			goto exit;
		}

		/* First, make sure all DMA interrupt/trigger are disabled for this paramset*/
		retVal = HWA_disableParamSetInterrupt(obj->hwaHandle, 
											  cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
											  HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
		if (retVal != 0)
		{
			retVal = -1013;
			goto exit;
		}

		if (paramsetIdx != 13)
		{
			retVal = -543;
			goto exit;
		}
	}
	else
	{
		paramsetIdx	++;
		hwaDopParamCfg[paramsetIdx] = hwaDopParamCfg[pingSumparamsetIdx];
		hwaDopParamCfg[paramsetIdx].source.srcAddr = DPU_DOPPLERPROCDCMPHWA_FFT_DST_PONG_OFFSET;
		hwaDopParamCfg[paramsetIdx].dest.dstAddr   = DPU_DOPPLERPROCDCMPHWA_SUMABS_DST_PONG_OFFSET;
		
		retVal = HWA_configParamSet(obj->hwaHandle,
									cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
									&hwaDopParamCfg[paramsetIdx],
									NULL);
		if (retVal != 0)
		{
			retVal = -1012;
			goto exit;
		}

		/* First, make sure all DMA interrupt/trigger are disabled for this paramset*/
		retVal = HWA_disableParamSetInterrupt(obj->hwaHandle, 
											  cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
											  HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
		if (retVal != 0)
		{
			retVal = -1013;
			goto exit;
		}
	}
  
    /************ Enable the DMA hookup to this paramset so that data gets copied out ***********/
    retVal = HWA_getDMAChanIndex(obj->hwaHandle, 
                                 cfg->hwRes.edmaCfg.edmaOut.pong.channel,
                                 &destChan);
    if (retVal != 0)
    {
        retVal = -1014;
        goto exit;
    }
    
    /*Now enable interrupt*/
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
    paramISRConfig.dma.dstChannel = destChan; //EDMA channel to trigger to copy the data out
    paramISRConfig.cpu.callbackArg = NULL;
    retVal = HWA_enableParamSetInterrupt(obj->hwaHandle,
                                        cfg->hwRes.hwaCfg.paramSetStartIdx + paramsetIdx,
                                        &paramISRConfig);
    if (retVal != 0)
    {
        retVal = -1015;
        goto exit;
    }
    
exit:
    return(retVal);
 }

/**
 *  @b Description
 *  @n
 *  Doppler DPU EDMA configuration.
 *  This implementation of doppler processing involves Ping/Pong 
 *  Mechanism, hence there are two sets of EDMA transfer.
 *
 *  @param[in] obj    - DPU obj
 *  @param[in] cfg    - DPU configuration
 *
 *  \ingroup    DPU_DOPPLERPROCDCMP_INTERNAL_FUNCTION
 *
 *  @retval EDMA error code, see EDMA API.
 */
    
static inline int32_t DPU_DopplerProcDcmpHWA_configEdma
(
    DPU_DopplerProcDcmpHWA_Obj      *obj,
    DPU_DopplerProcDcmpHWA_Config   *cfg

)
{
    int32_t             retVal = EDMA_NO_ERROR;
    cmplx16ImRe_t       *radarCubeBase = (cmplx16ImRe_t *)cfg->hwRes.radarCube.data;
    uint16_t            *detMatrixBase = (uint16_t *)cfg->hwRes.detMatrix.data;
    int16_t             sampleLenInBytes = sizeof(cmplx16ImRe_t);
    uint32_t            sizeOfAbsTransfer = cfg->staticCfg.numDopplerBins * cfg->staticCfg.compressCfg.numRangeBinsPerBlock;
    uint32_t            sizeOfAbsTransferBytes = sizeOfAbsTransfer * sizeof(uint16_t);
    DPEDMA_ChainingCfg  chainingCfg;
	DPEDMA_syncACfg     doppSyncACfg;
	DPEDMA_syncABCfg    doppSyncABCfg;
    uint16_t compressionRatio = cfg->staticCfg.compressCfg.ratio;
	uint16_t numRangeBinsPerBlock = cfg->staticCfg.compressCfg.numRangeBinsPerBlock;
	uint16_t numSamplesPerBlockIn = (((uint32_t)compressionRatio) * cfg->staticCfg.numRxAntennas * numRangeBinsPerBlock) >> HWA_CMP_RATIO_BW;
	if(obj == NULL)
    {
        retVal = DPU_DOPPLERPROCDCMPHWA_EINVAL;
        goto exit;
    }
    
    /**************PROGRAM DMA'S FOR PING**************************************/
        
    /***************************************************************************
     *  PROGRAM DMA channel  to transfer 2D abs data from accelerator output
     *  buffer (ping) to L3
     **************************************************************************/
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaIn.ping.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = false;

    doppSyncACfg.srcAddress  = (uint32_t)DPU_DOPPLERPROCDCMPHWA_SRC_PING_EDMA_OFFSET;
    doppSyncACfg.destAddress = (uint32_t)(&detMatrixBase[0]);
    doppSyncACfg.aCount      = sizeOfAbsTransferBytes;
    doppSyncACfg.bCount      = cfg->staticCfg.numRangeBins / (2U * cfg->staticCfg.compressCfg.numRangeBinsPerBlock); //factor of 2 due to ping/pong
    doppSyncACfg.srcBIdx     = 0;
    doppSyncACfg.dstBIdx     = sizeOfAbsTransferBytes * 2U;//factor of 2 due to ping/pong
         
    retVal = DPEDMA_configSyncA_singleFrame(cfg->hwRes.edmaCfg.edmaHandle,
                                &cfg->hwRes.edmaCfg.edmaOut.ping,
                                &chainingCfg,
                                &doppSyncACfg,
                                true, //isEventTriggered
                                false,//isIntermediateTransferInterruptEnabled
                                false,//isTransferCompletionEnabled
                                NULL, //transferCompletionCallbackFxn
                                NULL);//transferCompletionCallbackFxnArg

    if (retVal != EDMA_NO_ERROR)
    {
        goto exit;
    }

    /*********************************************************************************************************
    *  PROGRAM DMA channel  to transfer data from the compressed Radar cube to accelerator input buffer (ping)
    *********************************************************************************************************/    
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaHotSig.ping.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = true;

    doppSyncABCfg.srcAddress  = (uint32_t)(&radarCubeBase[0]);
    doppSyncABCfg.destAddress = (uint32_t)(DPU_DOPPLERPROCDCMPHWA_SRC_PING_EDMA_OFFSET);
    doppSyncABCfg.aCount      = sampleLenInBytes * numSamplesPerBlockIn  * cfg->staticCfg.numDopplerChirps *  cfg->staticCfg.numTxAntennas;
    doppSyncABCfg.bCount      = 1;
    doppSyncABCfg.cCount      = cfg->staticCfg.numRangeBins / (2U * cfg->staticCfg.compressCfg.numRangeBinsPerBlock);//factor of 2 due to ping/pong
    doppSyncABCfg.srcBIdx     = 0;
    doppSyncABCfg.dstBIdx     = 0;
    doppSyncABCfg.srcCIdx     = 2 * doppSyncABCfg.aCount;//factor of 2 due to ping/pong
    doppSyncABCfg.dstCIdx     = 0;

    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                 &cfg->hwRes.edmaCfg.edmaIn.ping,
                                 &chainingCfg,
                                 &doppSyncABCfg,
                                 false,//isEventTriggered
                                 true, //isIntermediateTransferCompletionEnabled
                                 false,//isTransferCompletionEnabled
                                 NULL, //transferCompletionCallbackFxn
                                 NULL);//transferCompletionCallbackFxnArg

    if (retVal != EDMA_NO_ERROR)
    {
        goto exit;
    }
    
    /******************************************************************************************
    *  PROGRAM DMA channel to trigger HWA for processing of input (ping)
    ******************************************************************************************/            
    retVal = DPEDMAHWA_configOneHotSignature(cfg->hwRes.edmaCfg.edmaHandle,
                                             &cfg->hwRes.edmaCfg.edmaHotSig.ping,
                                             obj->hwaHandle,
                                             obj->hwaDmaTriggerSourcePing,
                                             false);

    if (retVal != EDMA_NO_ERROR)
    {
        goto exit;
    }

    /**************PROGRAM DMA'S FOR PONG********************************************************/

    /******************************************************************************************
    *  PROGRAM DMA channel  to transfer 2D abs data from accelerator output buffer (pong) to L3
    ******************************************************************************************/
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaIn.pong.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = false;

    /* Transfer parameters are the same as ping, except for src/dst addresses */
	doppSyncACfg.srcAddress  = (uint32_t)DPU_DOPPLERPROCDCMPHWA_SRC_PONG_EDMA_OFFSET;
    doppSyncACfg.destAddress = (uint32_t)(&detMatrixBase[sizeOfAbsTransfer]);
    
    retVal = DPEDMA_configSyncA_singleFrame(cfg->hwRes.edmaCfg.edmaHandle,
                                &cfg->hwRes.edmaCfg.edmaOut.pong,
                                &chainingCfg,
                                &doppSyncACfg,
                                true, //isEventTriggered
                                false,//isIntermediateTransferInterruptEnabled
                                true, //isTransferCompletionEnabled
                                DPU_DopplerProcDcmpHWA_edmaDoneIsrCallback, //transferCompletionCallbackFxn
                                (uintptr_t)obj->edmaDoneSemaHandle); //transferCompletionCallbackFxnArg
    
    if (retVal != EDMA_NO_ERROR)
    {
        goto exit;
    }

    /******************************************************************************************
     *  PROGRAM DMA channel  to transfer data from L3 to accelerator input buffer (pong)
     ******************************************************************************************/ 
    chainingCfg.chainingChan                  = cfg->hwRes.edmaCfg.edmaHotSig.pong.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled        = true;

    /* Transfer parameters are the same as ping, except for src/dst addresses */
	doppSyncABCfg.srcAddress  = ((uint32_t)(&radarCubeBase[0])) + (uint32_t)doppSyncABCfg.aCount;
    doppSyncABCfg.destAddress = (uint32_t)(DPU_DOPPLERPROCDCMPHWA_SRC_PONG_EDMA_OFFSET);

    retVal = DPEDMA_configSyncAB(cfg->hwRes.edmaCfg.edmaHandle,
                                 &cfg->hwRes.edmaCfg.edmaIn.pong,
                                 &chainingCfg,
                                 &doppSyncABCfg,
                                 false,//isEventTriggered
                                 true, //isIntermediateTransferCompletionEnabled
                                 false,//isTransferCompletionEnabled
                                 NULL, //transferCompletionCallbackFxn
                                 NULL);//transferCompletionCallbackFxnArg
    
    if (retVal != EDMA_NO_ERROR)
    {
        goto exit;
    }

    /******************************************************************************************
    *  PROGRAM DMA channel to trigger HWA for processing of input (pong)
    ******************************************************************************************/  
    retVal = DPEDMAHWA_configOneHotSignature(cfg->hwRes.edmaCfg.edmaHandle,
                                             &cfg->hwRes.edmaCfg.edmaHotSig.pong,
                                             obj->hwaHandle,
                                             obj->hwaDmaTriggerSourcePong,
                                             false);

    if (retVal != EDMA_NO_ERROR)
    {
        goto exit;
    }

exit:
    return(retVal);
} 

/*===========================================================
 *                    Doppler Proc External APIs
 *===========================================================*/

/**
 *  @b Description
 *  @n
 *      dopplerProcDcmp DPU init function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @param[in]   initCfg Pointer to initial configuration parameters
 *  @param[out]  errCode Pointer to errCode generates by the API
 *
 *  \ingroup    DPU_DOPPLERPROCDCMP_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid handle
 *  @retval
 *      Error       - NULL
 */
DPU_DopplerProcDcmpHWA_Handle DPU_DopplerProcDcmpHWA_init
(
    DPU_DopplerProcDcmpHWA_InitParams *initCfg,
    int32_t                    *errCode
)
{
    DPU_DopplerProcDcmpHWA_Obj  *obj = NULL;
    SemaphoreP_Params       semParams;
    HWA_MemInfo             hwaMemInfo;
    uint32_t                i;

    *errCode       = 0;
    
    if((initCfg == NULL) || (initCfg->hwaHandle == NULL))
    {
        *errCode = DPU_DOPPLERPROCDCMPHWA_EINVAL;
        goto exit;
    }    

    /* Allocate memory */
    obj = MemoryP_ctrlAlloc(sizeof(DPU_DopplerProcDcmpHWA_Obj), 0U);
    if(obj == NULL)
    {
        *errCode = DPU_DOPPLERPROCDCMPHWA_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)obj, 0U, sizeof(DPU_DopplerProcDcmpHWA_Obj));
    
    /* Save init config params */
    obj->hwaHandle   = initCfg->hwaHandle;

    /* Create DPU semaphores */
    SemaphoreP_Params_init(&semParams);
    semParams.mode = SemaphoreP_Mode_BINARY;
    obj->edmaDoneSemaHandle = SemaphoreP_create(0, &semParams);
    if(obj->edmaDoneSemaHandle == NULL)
    {
        *errCode = DPU_DOPPLERPROCDCMPHWA_ESEMA;
        goto exit;
    }

    SemaphoreP_Params_init(&semParams);
    semParams.mode = SemaphoreP_Mode_BINARY;
    obj->hwaDoneSemaHandle = SemaphoreP_create(0, &semParams);
    if(obj->hwaDoneSemaHandle == NULL)
    {
        *errCode = DPU_DOPPLERPROCDCMPHWA_ESEMA;
        goto exit;
    }    
    
    /* Populate HWA base addresses and offsets. This is done only once, at init time.*/
    *errCode =  HWA_getHWAMemInfo(obj->hwaHandle, &hwaMemInfo);
    if (*errCode < 0)
    {       
        goto exit;
    }
    
    /* check if we have enough memory banks*/
    if(hwaMemInfo.numBanks < DPU_DOPPLERPROCDCMPHWA_NUM_HWA_MEMBANKS)
    {    
        *errCode = DPU_DOPPLERPROCDCMPHWA_EHWARES;
        goto exit;
    }
    
    for (i = 0; i < DPU_DOPPLERPROCDCMPHWA_NUM_HWA_MEMBANKS; i++)
    {
        obj->hwaMemBankAddr[i] = hwaMemInfo.baseAddress + i * hwaMemInfo.bankSize;
    }
    
exit:    

    if(*errCode < 0)
    {
        if(obj != NULL)
        {
            MemoryP_ctrlFree(obj, sizeof(DPU_DopplerProcDcmpHWA_Obj));
            obj = NULL;
        }
    }
   return ((DPU_DopplerProcDcmpHWA_Handle)obj);
}

/**
  *  @b Description
  *  @n
  *   Doppler DPU configuration 
  *
  *  @param[in]   handle     DPU handle.
  *  @param[in]   cfg        Pointer to configuration parameters.
  *
  *  \ingroup    DPU_DOPPLERPROCDCMP_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success      = 0
  *  @retval
  *      Error       != 0 @ref DPU_DOPPLERPROCDCMP_ERROR_CODE
  */

int32_t DPU_DopplerProcDcmpHWA_config
(
    DPU_DopplerProcDcmpHWA_Handle    handle,
    DPU_DopplerProcDcmpHWA_Config    *cfg
)
{
    DPU_DopplerProcDcmpHWA_Obj   *obj;
    int32_t                  retVal = 0;
    uint16_t                 expectedWinSamples;
	uint16_t				 numParamSetsPerPing;


    obj = (DPU_DopplerProcDcmpHWA_Obj *)handle;
    if(obj == NULL)
    {
        retVal = DPU_DOPPLERPROCDCMPHWA_EINVAL;
        goto exit;
    }
    
#if DEBUG_CHECK_PARAMS
    /* Validate params */
    if(!cfg ||
       !cfg->hwRes.edmaCfg.edmaHandle ||
       !cfg->hwRes.hwaCfg.window || 
       !cfg->hwRes.radarCube.data ||
       !cfg->hwRes.detMatrix.data
      )
    {
        retVal = DPU_DOPPLERPROCDCMPHWA_EINVAL;
        goto exit;
    }

    /* Check if radar cube format is supported by DPU*/
    if(cfg->hwRes.radarCube.datafmt != DPIF_RADARCUBE_FORMAT_1)
    {
        retVal = DPU_DOPPLERPROCDCMPHWA_ECUBEFORMAT;
        goto exit;
    }

    /* Check if detection matrix format is supported by DPU*/
    if(cfg->hwRes.detMatrix.datafmt != DPIF_DETMATRIX_FORMAT_1)
    {
		retVal = DPU_DOPPLERPROCDCMPHWA_EDETMFORMAT;
        goto exit;
    }
    
    /* Check if radar cube column fits into one HWA memory bank */
    if((cfg->staticCfg.numTxAntennas * cfg->staticCfg.numRxAntennas * 
        cfg->staticCfg.numDopplerChirps * sizeof(cmplx16ImRe_t)) > (SOC_HWA_MEM_SIZE/SOC_HWA_NUM_MEM_BANKS))
    {
        retVal = DPU_DOPPLERPROCDCMPHWA_EEXCEEDHWAMEM;
        goto exit;
    }

    /* Check if abs value of log2 of 2D FFT fits in one HWA memory bank */
    if((cfg->staticCfg.numTxAntennas * cfg->staticCfg.numRxAntennas * 
        cfg->staticCfg.numDopplerBins * sizeof(uint16_t)) > (SOC_HWA_MEM_SIZE/SOC_HWA_NUM_MEM_BANKS))
    {
        retVal = DPU_DOPPLERPROCDCMPHWA_EEXCEEDHWAMEM;
        goto exit;
    }

    /* Check if number of range bins is even*/
    if((cfg->staticCfg.numRangeBins % 2) != 0)
    {
        retVal = DPU_DOPPLERPROCDCMPHWA_EINVAL;
        goto exit;
    }
    
    /* Check if detection matrix size is sufficient*/
    if(cfg->hwRes.detMatrix.dataSize < (cfg->staticCfg.numRangeBins *
                                        cfg->staticCfg.numDopplerBins * sizeof(uint16_t)))
    {
        retVal = DPU_DOPPLERPROCDCMPHWA_EDETMSIZE;
        goto exit;
    }

    /* Check window Size */
    if(cfg->hwRes.hwaCfg.winSym == HWA_FFT_WINDOW_NONSYMMETRIC)
    {
        expectedWinSamples = cfg->staticCfg.numDopplerChirps;
    }
    else
    {
        /* odd samples have to be rounded up per HWA */
        expectedWinSamples = (cfg->staticCfg.numDopplerChirps + 1) / 2;
    }

    if (cfg->hwRes.hwaCfg.windowSize != expectedWinSamples * sizeof(int32_t)) 
    {
			retVal = DPU_DOPPLERPROCDCMPHWA_EWINDSIZE;
            goto exit;
    }        
#endif

    /* Save necessary parameters to DPU object that will be used during Process time */
    
    /* EDMA parameters needed to trigger first EDMA transfer*/
    obj->edmaHandle  = cfg->hwRes.edmaCfg.edmaHandle;
    memcpy((void*)(&obj->edmaIn), (void *)(&cfg->hwRes.edmaCfg.edmaIn), sizeof(DPU_DopplerProcDcmp_Edma));
    
	numParamSetsPerPing = (2 + cfg->staticCfg.compressCfg.numRangeBinsPerBlock + (cfg->staticCfg.compressCfg.numRangeBinsPerBlock != 1));
    /*HWA parameters needed for the HWA common configuration*/
    obj->hwaNumLoops      = cfg->staticCfg.numRangeBins / (2U * cfg->staticCfg.compressCfg.numRangeBinsPerBlock);
    obj->hwaParamStartIdx = cfg->hwRes.hwaCfg.paramSetStartIdx;    
    obj->hwaParamStopIdx  = cfg->hwRes.hwaCfg.paramSetStartIdx + (2*numParamSetsPerPing) - 1;
	obj->compressionRatio  = cfg->staticCfg.compressCfg.ratio;
    
    /* Disable the HWA */
    retVal = HWA_enable(obj->hwaHandle, 0); 
    if (retVal != 0)
    {
		goto exit;
    }
    
    /* HWA window configuration */
    retVal = HWA_configRam(obj->hwaHandle,
                           HWA_RAM_TYPE_WINDOW_RAM,
                           (uint8_t *)cfg->hwRes.hwaCfg.window,
                           cfg->hwRes.hwaCfg.windowSize, //size in bytes
                           cfg->hwRes.hwaCfg.winRamOffset * sizeof(int32_t)); 
    if (retVal != 0)
    {
		goto exit;
    }
    
    /*******************************/
    /**  Configure HWA            **/
    /*******************************/
    /*Compute source DMA channels that will be programmed in both HWA and EDMA.   
      The DMA channels are set to be equal to the paramsetIdx used by HWA*/
    /* Ping DMA channel (Ping uses the first paramset)*/  
    obj->hwaDmaTriggerSourcePing = cfg->hwRes.hwaCfg.paramSetStartIdx;
    /* Pong DMA channel*/  
    obj->hwaDmaTriggerSourcePong = cfg->hwRes.hwaCfg.paramSetStartIdx + numParamSetsPerPing;
    retVal = DPU_DopplerProcDcmpHWA_configHwa(obj, cfg);
    if (retVal != 0)
    {
		
        goto exit;
    }
                    
    /*******************************/
    /**  Configure EDMA           **/
    /*******************************/    
    retVal = DPU_DopplerProcDcmpHWA_configEdma(obj, cfg);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return retVal;
}


 /**
  *  @b Description
  *  @n Doppler DPU process function. 
  *   
  *  @param[in]   handle     DPU handle.
  *  @param[out]  outParams  Output parameters.
  *
  *  \ingroup    DPU_DOPPLERPROCDCMP_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success     =0
  *  @retval
  *      Error      !=0 @ref DPU_DOPPLERPROCDCMP_ERROR_CODE
  */
	
int32_t DPU_DopplerProcDcmpHWA_process
(
    DPU_DopplerProcDcmpHWA_Handle    handle,
    DPU_DopplerProcDcmpHWA_OutParams *outParams
)
{
    volatile uint32_t   startTime;
    DPU_DopplerProcDcmpHWA_Obj *obj;
    int32_t             retVal = 0;
    bool                status;
    obj = (DPU_DopplerProcDcmpHWA_Obj *)handle;
    uint16_t compressionRatio = obj->compressionRatio;
	HWA_CommonConfig    doppHwaCommonConfig;

    if (obj == NULL)
    {
        retVal = DPU_DOPPLERPROCDCMPHWA_EINVAL;
        goto exit;
    }
    /* Set inProgress state */
    obj->inProgress = true;

    startTime = Cycleprofiler_getTimeStamp();
    
    /**********************************************/
    /* ENABLE NUMLOOPS DONE INTERRUPT FROM HWA */
    /**********************************************/
    retVal = HWA_enableDoneInterrupt(obj->hwaHandle,
                                       DPU_DopplerProcDcmpHWA_hwaDoneIsrCallback,
                                       obj->hwaDoneSemaHandle);
    if (retVal != 0)
    {
        goto exit;
    }
    
    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    memset((void*) &doppHwaCommonConfig, 0, sizeof(HWA_CommonConfig));

    /* Config Common Registers */
    doppHwaCommonConfig.configMask =
        HWA_COMMONCONFIG_MASK_NUMLOOPS |
        HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
        HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
        HWA_COMMONCONFIG_MASK_FFT1DENABLE |
        HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD |
		HWA_COMMONCONFIG_MASK_EGECOMRESS_KPARAM;

    doppHwaCommonConfig.numLoops      = obj->hwaNumLoops;     
    doppHwaCommonConfig.paramStartIdx = obj->hwaParamStartIdx;
    doppHwaCommonConfig.paramStopIdx  = obj->hwaParamStopIdx; 
    doppHwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    doppHwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;
	cfgEGEParamListDopplerProc(&doppHwaCommonConfig.compressMode.EGEKparam[0], compressionRatio,HWA_SAMPLES_WIDTH_16BIT);
    
    retVal = HWA_configCommon(obj->hwaHandle, &doppHwaCommonConfig);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Enable the HWA */
    retVal = HWA_enable(obj->hwaHandle,1); 
    if (retVal != 0)
    {
        goto exit;
    }

    EDMA_startTransfer(obj->edmaHandle, obj->edmaIn.ping.channel, EDMA3_CHANNEL_TYPE_DMA);
    EDMA_startTransfer(obj->edmaHandle, obj->edmaIn.pong.channel, EDMA3_CHANNEL_TYPE_DMA);
    
    /**********************************************/
    /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
    /**********************************************/
    status = SemaphoreP_pend(obj->hwaDoneSemaHandle, SemaphoreP_WAIT_FOREVER);
    
    if (status != SemaphoreP_OK)
    {
        retVal = DPU_DOPPLERPROCDCMPHWA_ESEMASTATUS;
        goto exit;
    }

    HWA_disableDoneInterrupt(obj->hwaHandle);

    /* Disable the HWA */
    retVal = HWA_enable(obj->hwaHandle, 0); 
    if (retVal != 0)
    {
        goto exit;
    }
    
    /**********************************************/
    /* WAIT FOR EDMA DONE INTERRUPT            */
    /**********************************************/
    status = SemaphoreP_pend(obj->edmaDoneSemaHandle, SemaphoreP_WAIT_FOREVER);
    if (status != SemaphoreP_OK)
    {
        retVal = DPU_DOPPLERPROCDCMPHWA_ESEMASTATUS;
        goto exit;
    }
    
    outParams->stats.numProcess++;
    outParams->stats.processingTime = Cycleprofiler_getTimeStamp() - startTime;
    
exit:
    if (obj != NULL)
    {
        obj->inProgress = false;
    }    
    
    return retVal;
}

/**
  *  @b Description
  *  @n
  *  Doppler DPU deinit 
  *
  *  @param[in]   handle   DPU handle.
  *
  *  \ingroup    DPU_DOPPLERPROCDCMP_EXTERNAL_FUNCTION
  *
  *  @retval
  *      Success      =0
  *  @retval
  *      Error       !=0 @ref DPU_DOPPLERPROCDCMP_ERROR_CODE
  */
int32_t DPU_DopplerProcDcmpHWA_deinit(DPU_DopplerProcDcmpHWA_Handle handle)
{
    int32_t     retVal = 0;
    
    /* Free memory */
    if(handle == NULL)
    {
        retVal = DPU_DOPPLERPROCDCMPHWA_EINVAL;
    }
    else
    {
        MemoryP_ctrlFree(handle, sizeof(DPU_DopplerProcDcmpHWA_Obj));
    }
    
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function creates the k-array list for EGE compression. 
 *
 *  @param[in, out]  cmpEGEArr 		pointer to array for the k-array list. 
 *  @param[in]  compressionRatio 	the desired compression ratio.  
 *  @param[in]  bitwidth 		the input formatter bitwidth (16/32-bit)
 *
 *  \ingroup    DPU_RANGEPROCCMP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
void  cfgEGEParamListDopplerProc (uint8_t * cmpEGEArr, uint16_t compressionRatio, uint16_t srcWidth)
{ 
	uint32_t ik;

	if (srcWidth == HWA_SAMPLES_WIDTH_16BIT)
	{
		if (compressionRatio == HWA_CMP_50P_RATIO)
		{
			cmpEGEArr[0] = 6; 
			cmpEGEArr[1] = 7; 
			cmpEGEArr[2] = 8; 
			cmpEGEArr[3] = 9; 
			cmpEGEArr[4] = 10; 
			cmpEGEArr[5] = 11; 
			cmpEGEArr[6] = 13; 
			cmpEGEArr[7] = 15; 
		}
		else
		{
			for (ik = 0; ik < HWA_CMP_K_ARR_LEN; ik ++)
			{
				cmpEGEArr[ik] = 2 * ik + 1;
			}

		}
	}
	else
	{
		for (ik = 0; ik < HWA_CMP_K_ARR_LEN; ik ++)
		{
			cmpEGEArr[ik] = 4 * ik + 1;
		}

	}
}
