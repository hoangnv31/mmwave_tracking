/*
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
/**
 *   @file  rangeproccmphwa.c
 *
 *   @brief
 *      Implements Range FFT data processing Unit using HWA.
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


/* mmWave SDK common/driver Include files */
#include <ti/drivers/osal/SemaphoreP.h>
#include <ti/drivers/osal/MemoryP.h>
#include <ti/drivers/edma/edma.h>
#include <ti/drivers/hwa/hwa.h>

/* Compression constants. */
#include <ti/demo/xwr64xx_compression/mmw/compressionConsts.h>

/* Data Path Include files */
#include <ti/datapath/dpu/rangeproccmp/rangeproccmphwa.h>

/* MATH utils library Include files */
#include <ti/utils/mathutils/mathutils.h>

/* Internal include Files */
#include <ti/datapath/dpu/rangeproccmp/include/rangeproccmphwa_internal.h>

/* Compression constants. */
#include <ti/demo/xwr64xx_compression/mmw/compressionConsts.h>

/* Flag to check input parameters */
#define DEBUG_CHECK_PARAMS   1



/**************************************************************************
 ************************ Internal Functions Prototype       **********************
 **************************************************************************/
static void rangeProcCmpHWADoneIsrCallback(void * arg);
static void rangeProcCmpHWA_EDMA_transferCompletionCallbackFxn(uintptr_t arg,
    uint8_t transferCompletionCode);
void  cfgEGEParamListRangeProc (uint8_t * cmpEGEArr, uint16_t compressionRatio, uint16_t srcWidth);
static int32_t rangeProcCmpHWA_ConfigEDMATranspose
(
    rangeProcCmp_dpParams      *dpParams,
    EDMA_Handle             handle,
    DPEDMA_ChanCfg          *chanCfg,
    DPEDMA_ChainingCfg      *chainingCfg,
    uint32_t                srcAddress,
    uint32_t                destAddress,
    bool                    isTransferCompletionEnabled,
    EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
    uintptr_t               transferCompletionCallbackFxnArg
);

static int32_t rangeFFTandCmpProcHWA_ConfigHWA
(
    rangeProcCmpHWAObj     *rangeProcCmpObj,
    uint8_t     destChanPing,
    uint8_t     destChanPong,
    uint16_t    hwaMemSrcPingOffset,
    uint16_t    hwaMemSrcPongOffset,
    uint16_t    hwaMemFFTDestPingOffset,
    uint16_t    hwaMemFFTDestPongOffset,
    uint16_t    hwaMemCmpDestPingOffset,
    uint16_t    hwaMemCmpDestPongOffset
);

static int32_t rangeProcCmpHWA_TriggerHWA
(
    rangeProcCmpHWAObj     *rangeProcCmpObj
);
static int32_t rangeProcCmpHWA_ConfigEDMA_DataOut_interleave
(
    rangeProcCmpHWAObj     *rangeProcCmpObj,
    rangeProcCmp_dpParams  *DPParams,
    DPU_RangeProcCmpHWA_HW_Resources *pHwConfig,
    uint32_t            hwaOutPingOffset,
    uint32_t            hwaOutPongOffset
);	
static int32_t rangeProcCmpHWA_ConfigEDMA_DataIn
(
    rangeProcCmpHWAObj         *rangeProcCmpObj,
    rangeProcCmp_dpParams      *DPParams,
    DPU_RangeProcCmpHWA_HW_Resources *pHwConfig
);
static int32_t rangeProcCmpHWA_ConfigInterleaveMode
(
    rangeProcCmpHWAObj         *rangeProcCmpObj,
    rangeProcCmp_dpParams      *DPParams,
    DPU_RangeProcCmpHWA_HW_Resources *pHwConfig
);
static int32_t rangeProcCmpHWA_dcRangeSignatureCompensation_init
(
    rangeProcCmpHWAObj     *rangeProcCmpObj,
    DPU_RangeProcCmp_CalibDcRangeSigCfg *calibDcRangeSigCfg,
    uint8_t             resetMeanBuffer
);






/**************************************************************************
 ************************RangeProcCmpHWA_ Internal Functions **********************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      HWA processing completion call back function as per HWA API.
 *      Depending on the programmed transfer completion codes,
 *      posts HWA done semaphore.
 *
 *  @param[in]  arg                 Argument to the callback function
 *
 *  \ingroup    DPU_RANGEPROCCMP_INTERNAL_FUNCTION
 *
 *  @retVal     N/A
 */
static void rangeProcCmpHWADoneIsrCallback(void * arg)
{
    if (arg != NULL) 
    {
        SemaphoreP_post((SemaphoreP_Handle)arg);
    }
}

/**
 *  @b Description
 *  @n
 *      EDMA processing completion call back function as per EDMA API.
 *
 *  @param[in]  arg                     Argument to the callback function
 *  @param[in]  transferCompletionCode  EDMA transfer complete code
 *
 *  \ingroup    DPU_RANGEPROCCMP_INTERNAL_FUNCTION
 *
 *  @retVal     N/A
 */
uint32_t frameTxCompleteCnt = 0;
static void rangeProcCmpHWA_EDMA_transferCompletionCallbackFxn(uintptr_t arg,
    uint8_t transferCompletionCode)
{
    rangeProcCmpHWAObj     *rangeProcCmpObj;

    /* Get rangeProcCmp object */
    rangeProcCmpObj = (rangeProcCmpHWAObj *)arg;

    if (transferCompletionCode == rangeProcCmpObj->dataOutSignatureChan)
    {
		frameTxCompleteCnt++;
        rangeProcCmpObj->numEdmaDataOutCnt++;
        SemaphoreP_post(rangeProcCmpObj->edmaDoneSemaHandle);
    }
}


/**
 *  @b Description
 *  @n
 *      Function to config a dummy channel with 3 linked paramset. Each paramset is linked
 *   to a EDMA data copy channel
 *
 *  @param[in]  dpParams                Pointer to data path parameters
 *  @param[in]  handle                  EDMA handle
 *  @param[in]  chanCfg                 EDMA channel configuraton
 *  @param[in]  chainingCfg             EDMA chaining configuration
 *  @param[in]  srcAddress              EDMA copy source address
 *  @param[in]  destAddress             EDMA copy destination address
 *  @param[in]  isTransferCompletionEnabled Number of iterations the dummy channel will be excuted.
 *  @param[in]  transferCompletionCallbackFxn Transfer completion call back function.
 *  @param[in]  transferCompletionCallbackFxnArg Argument for transfer completion call back function.
 *
 *  \ingroup    DPU_RANGEPROCCMP_INTERNAL_FUNCTION
 *
 *  @retVal     N/A
 */
static int32_t rangeProcCmpHWA_ConfigEDMATranspose
(
    rangeProcCmp_dpParams      *dpParams,
    EDMA_Handle             handle,
    DPEDMA_ChanCfg          *chanCfg,
    DPEDMA_ChainingCfg      *chainingCfg,
    uint32_t                srcAddress,
    uint32_t                destAddress,
    bool                    isTransferCompletionEnabled,
    EDMA_transferCompletionCallbackFxn_t transferCompletionCallbackFxn,
    uintptr_t               transferCompletionCallbackFxnArg
)
{
    int32_t                 retVal;
	uint16_t compressionRatio = dpParams->compressCfg.ratio;
	DPEDMA_syncABCfg        syncABCfg;

    /* dpedma configuration */
    syncABCfg.aCount = (dpParams->numRxAntennas * dpParams->compressCfg.numRangeBinsPerBlock * sizeof(cmplx16ImRe_t) * compressionRatio) >> HWA_CMP_RATIO_BW;
    syncABCfg.bCount = dpParams->numRangeBins/dpParams->compressCfg.numRangeBinsPerBlock;
    syncABCfg.cCount = dpParams->numChirpsPerFrame/2U;
    syncABCfg.srcBIdx = syncABCfg.aCount;
    syncABCfg.srcCIdx = 0U;
    syncABCfg.dstBIdx = syncABCfg.aCount * dpParams->numChirpsPerFrame;
    syncABCfg.dstCIdx = syncABCfg.aCount * 2;

    syncABCfg.srcAddress = srcAddress;
    syncABCfg.destAddress = destAddress;

    retVal = DPEDMA_configSyncAB(handle,
            chanCfg,
            chainingCfg,
            &syncABCfg,
            true,    /* isEventTriggered */
            false,   /* isIntermediateTransferCompletionEnabled */
            isTransferCompletionEnabled,   /* isTransferCompletionEnabled */
            transferCompletionCallbackFxn,
            transferCompletionCallbackFxnArg);

    if (retVal != EDMA_NO_ERROR)
    {
        goto exit;
    }

exit:
    return (retVal);
}

/**
 *  @b Description
 *  @n
 *      Internal function to config HWA to perform range FFT
 *
 *  @param[in]  rangeProcCmpObj                  Pointer to rangeProcCmp object
 *
 *  \ingroup    DPU_RANGEPROCCMP_INTERNAL_FUNCTION
 *
 *  @retVal
 *      Success     - 0
 *  @retVal
 *      Error       - <0
 */\
static int32_t rangeProcCmpHWA_ConfigHWACommon
(
    rangeProcCmpHWAObj     *rangeProcCmpObj
)
{
	HWA_CommonConfig    hwaCommonConfig;
    rangeProcCmp_dpParams  *DPParams;
    int32_t             retVal;
	
    DPParams = &rangeProcCmpObj->params;
	uint16_t compressionRatio = DPParams->compressCfg.ratio;
    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    /* Config Common Registers */
    hwaCommonConfig.configMask = HWA_COMMONCONFIG_MASK_NUMLOOPS |
                               HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
                               HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
                               HWA_COMMONCONFIG_MASK_FFT1DENABLE |
                               HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD |
                               HWA_COMMONCONFIG_MASK_TWIDDITHERENABLE |
                               HWA_COMMONCONFIG_MASK_LFSRSEED |
							   HWA_COMMONCONFIG_MASK_EGECOMRESS_KPARAM;

    hwaCommonConfig.fftConfig.twidDitherEnable = HWA_FEATURE_BIT_ENABLE;
    hwaCommonConfig.fftConfig.lfsrSeed = 0x1234567; /*Some non-zero value*/
    hwaCommonConfig.numLoops = DPParams->numChirpsPerFrame/2U;
    hwaCommonConfig.paramStartIdx = rangeProcCmpObj->hwaCfg.paramSetStartIdx;
    hwaCommonConfig.paramStopIdx = rangeProcCmpObj->hwaCfg.paramSetStartIdx + rangeProcCmpObj->hwaCfg.numParamSet - 1U;

    if (rangeProcCmpObj->hwaCfg.dataInputMode == DPU_RangeProcCmpHWA_InputMode_ISOLATED)
    {
        /* HWA will input data from M0 memory*/
        hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    }
    else
    {
		retVal = -123;
		goto exit;
        /* HWA will input data from ADC buffer memory*/
        //hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_ENABLE;
    }
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;

    /* Compression configuration. */
    cfgEGEParamListRangeProc(&hwaCommonConfig.compressMode.EGEKparam[0], compressionRatio,HWA_SAMPLES_WIDTH_16BIT);

    retVal = HWA_configCommon(rangeProcCmpObj->initParms.hwaHandle, &hwaCommonConfig);
    if (retVal != 0)
    {
        goto exit;
    }

    /**********************************************/
    /* ENABLE NUMLOOPS DONE INTERRUPT FROM HWA */
    /**********************************************/
    retVal = HWA_enableDoneInterrupt(rangeProcCmpObj->initParms.hwaHandle,
                                        rangeProcCmpHWADoneIsrCallback,
                                        rangeProcCmpObj->hwaDoneSemaHandle);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return(retVal);
}
/**
 *  @b Description
 *  @n
 *      Internal function to config HWA to perform range FFT and then compress it
 *  by 50 %. 
 *
 *  @param[in]  rangeProcCmpObj               Pointer to rangeProcCmp object
 *  @param[in]  destChanPing                  Destination channel id for PING
 *  @param[in]  destChanPong                  Destination channel id for PONG
 *  @param[in]  hwaMemSrcPingOffset           Source Address offset for Ping input
 *  @param[in]  hwaMemSrcPongOffset           Source Address offset for Pong input
 *  @param[in]  hwaMemFFTDestPingOffset       Destination address offset for FFT for Ping output
 *  @param[in]  hwaMemFFTDestPongOffset       Destination address offset for FFT for Pong output
 *  @param[in]  hwaMemCmpDestPingOffset       Destination address offset for Compression for Ping output
 *  @param[in]  hwaMemCmpDestPongOffset       Destination address offset for Compression for Pong output
 *
 *  \ingroup    DPU_RANGEPROCCMP_INTERNAL_FUNCTION
 *
 *  @retVal
 *      Success     - 0
 *  @retVal
 *      Error       - <0
 */
static int32_t rangeFFTandCmpProcHWA_ConfigHWA
(
    rangeProcCmpHWAObj     *rangeProcCmpObj,
    uint8_t     destChanPing,
    uint8_t     destChanPong,
    uint16_t    hwaMemSrcPingOffset,
    uint16_t    hwaMemSrcPongOffset,
    uint16_t    hwaMemFFTDestPingOffset,
    uint16_t    hwaMemFFTDestPongOffset,
    uint16_t    hwaMemCmpDestPingOffset,
    uint16_t    hwaMemCmpDestPongOffset
)
{

    int32_t                 errCode = 0;
    HWA_Handle                      hwaHandle;
    rangeProcCmp_dpParams             *pDPParams;
    uint8_t                         index;
	uint16_t compressionRatio;
	HWA_ParamConfig         hwaParamCfg[DPU_RANGEPROCCMPHWA_NUM_HWA_PARAM_SETS];
	uint32_t                paramsetIdx = 0;
	uint32_t                hwParamsetIdx = 0;
	uint32_t                pingFFTParamSetIdx = 0;
	uint32_t                pingCMPParamSetIdx = 0;
	HWA_InterruptConfig     paramISRConfig;

    hwaHandle = rangeProcCmpObj->initParms.hwaHandle;
    pDPParams = &rangeProcCmpObj->params;
	compressionRatio = pDPParams->compressCfg.ratio;
    uint16_t numSamplesPerBlockIn = pDPParams->numRxAntennas * pDPParams->compressCfg.numRangeBinsPerBlock;
    uint16_t numSamplesPerBlockOut = (uint16_t) ((numSamplesPerBlockIn * (uint32_t)compressionRatio) >> HWA_CMP_RATIO_BW);
    uint16_t numBlocks = pDPParams->numAdcSamples/pDPParams->compressCfg.numRangeBinsPerBlock;
    memset(hwaParamCfg,0,sizeof(hwaParamCfg));
	
	paramsetIdx = 0;
	hwParamsetIdx = 0;
	pingFFTParamSetIdx = 0;
	pingCMPParamSetIdx = 0;
    hwParamsetIdx = rangeProcCmpObj->hwaCfg.paramSetStartIdx;
    for(index = 0; index < DPU_RANGEPROCCMPHWA_NUM_HWA_PARAM_SETS; index++)
    {
        errCode = HWA_disableParamSetInterrupt(hwaHandle, index + rangeProcCmpObj->hwaCfg.paramSetStartIdx, 
                HWA_PARAMDONE_INTERRUPT_TYPE_CPU |HWA_PARAMDONE_INTERRUPT_TYPE_DMA);
        if (errCode != 0)
        {
			goto exit;
        }
    }

    /***********************/
    /* PING DUMMY PARAMSET */
    /***********************/
    hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_DMA;
    hwaParamCfg[paramsetIdx].dmaTriggerSrc = hwParamsetIdx;
    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_NONE;
    errCode = HWA_configParamSet(hwaHandle,
                                  hwParamsetIdx,
                                  &hwaParamCfg[paramsetIdx],NULL);
    if (errCode != 0)
    {
		goto exit;
    }

    /*****************************/
    /* PING FFT PROCESS PARAMSET */
    /*****************************/
    paramsetIdx++;
    hwParamsetIdx++;
    pingFFTParamSetIdx = paramsetIdx;

    /* adcbuf mapped */
    if(rangeProcCmpObj->hwaCfg.dataInputMode == DPU_RangeProcCmpHWA_InputMode_MAPPED)
    {
        hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_DFE;
    }
    else
    {
        /*adcbuf not mapped, should trigger after edma copy is done */
        hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_DMA;
        hwaParamCfg[paramsetIdx].dmaTriggerSrc = hwParamsetIdx;
    }

    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_FFT;
    hwaParamCfg[paramsetIdx].source.srcAddr = hwaMemSrcPingOffset; 

    hwaParamCfg[paramsetIdx].source.srcShift = 0;
    hwaParamCfg[paramsetIdx].source.srcCircShiftWrap = 0;
    hwaParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
    hwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg[paramsetIdx].source.srcConjugate = 0;
    hwaParamCfg[paramsetIdx].source.srcScale = 8;
    hwaParamCfg[paramsetIdx].source.bpmEnable = 0;
    hwaParamCfg[paramsetIdx].source.bpmPhase = 0;
    hwaParamCfg[paramsetIdx].dest.dstAddr = hwaMemFFTDestPingOffset;

    hwaParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
    hwaParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_SIGNED; 
    hwaParamCfg[paramsetIdx].dest.dstConjugate = 0; 
    hwaParamCfg[paramsetIdx].dest.dstScale = 0;
    hwaParamCfg[paramsetIdx].dest.dstSkipInit = 0; 

    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftEn = 1;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(pDPParams->numRangeBins);
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.butterflyScaling = 0xF;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.interfZeroOutEn = 0;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowEn = 1;
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.windowStart = rangeProcCmpObj->hwaCfg.hwaWinRamOffset; 
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.winSymm = rangeProcCmpObj->hwaCfg.hwaWinSym; 
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.winInterpolateMode = 0; 
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED; 
    hwaParamCfg[paramsetIdx].accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;
    hwaParamCfg[paramsetIdx].complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;

    /* HWA range FFT src/dst configuration*/
    if(rangeProcCmpObj->interleave == DPIF_RXCHAN_INTERLEAVE_MODE)
    {
        if(rangeProcCmpObj->radarCubeLayout == rangeProcCmp_dataLayout_RANGE_DOPPLER_TxAnt_RxAnt)
        {
            hwaParamCfg[paramsetIdx].source.srcAcnt = pDPParams->numAdcSamples - 1; /* this is samples - 1 */
            hwaParamCfg[paramsetIdx].source.srcAIdx = pDPParams->numRxAntennas* sizeof(uint32_t);
            hwaParamCfg[paramsetIdx].source.srcBcnt = pDPParams->numRxAntennas-1;
            hwaParamCfg[paramsetIdx].source.srcBIdx = sizeof(uint32_t);

            hwaParamCfg[paramsetIdx].dest.dstAcnt = pDPParams->numRangeBins-1;
            hwaParamCfg[paramsetIdx].dest.dstAIdx = pDPParams->numRxAntennas * sizeof(uint32_t);
            hwaParamCfg[paramsetIdx].dest.dstBIdx = sizeof(uint32_t);
        }
        else
        {
			hwaParamCfg[paramsetIdx].source.srcAcnt = pDPParams->numAdcSamples - 1; /* this is samples - 1 */
			hwaParamCfg[paramsetIdx].source.srcAIdx = pDPParams->numRxAntennas* sizeof(uint32_t);
			hwaParamCfg[paramsetIdx].source.srcBcnt = pDPParams->numRxAntennas-1;
			hwaParamCfg[paramsetIdx].source.srcBIdx = sizeof(uint32_t);

			hwaParamCfg[paramsetIdx].dest.dstAcnt = pDPParams->numRangeBins-1;
			hwaParamCfg[paramsetIdx].dest.dstAIdx = pDPParams->numRxAntennas * sizeof(uint32_t);
			hwaParamCfg[paramsetIdx].dest.dstBIdx = sizeof(uint32_t);
        }
    }
    else
    {
        if(rangeProcCmpObj->radarCubeLayout == rangeProcCmp_dataLayout_RANGE_DOPPLER_TxAnt_RxAnt)
        {
            hwaParamCfg[paramsetIdx].source.srcAcnt = pDPParams->numAdcSamples - 1;
            hwaParamCfg[paramsetIdx].source.srcAIdx = sizeof(uint32_t);
            hwaParamCfg[paramsetIdx].source.srcBcnt = pDPParams->numRxAntennas-1;
            hwaParamCfg[paramsetIdx].source.srcBIdx = rangeProcCmpObj->rxChanOffset;

            hwaParamCfg[paramsetIdx].dest.dstAcnt = pDPParams->numRangeBins-1;
            hwaParamCfg[paramsetIdx].dest.dstAIdx = sizeof(uint32_t) * pDPParams->numRxAntennas; 
            hwaParamCfg[paramsetIdx].dest.dstBIdx = sizeof(uint32_t);
        }
        else
        {
            hwaParamCfg[paramsetIdx].source.srcAcnt = pDPParams->numAdcSamples - 1;
            hwaParamCfg[paramsetIdx].source.srcAIdx = sizeof(uint32_t); 
            hwaParamCfg[paramsetIdx].source.srcBcnt = pDPParams->numRxAntennas-1; 
            hwaParamCfg[paramsetIdx].source.srcBIdx = rangeProcCmpObj->rxChanOffset; 
            hwaParamCfg[paramsetIdx].dest.dstAcnt = pDPParams->numRangeBins-1;
            hwaParamCfg[paramsetIdx].dest.dstAIdx = sizeof(uint32_t); 
            hwaParamCfg[paramsetIdx].dest.dstBIdx = pDPParams->numRangeBins * sizeof(uint32_t); 
        }
    }
	

    errCode = HWA_configParamSet(hwaHandle,
                                  hwParamsetIdx,
                                  &hwaParamCfg[paramsetIdx],NULL);
    if (errCode != 0)
    {
		goto exit;
    }

    /*****************************/
    /* PING CMP PROCESS PARAMSET */
    /*****************************/
    paramsetIdx++;
    hwParamsetIdx++;
    pingCMPParamSetIdx = paramsetIdx;

    hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_IMMEDIATE;
    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_COMPRESS;

    hwaParamCfg[paramsetIdx].source.srcAddr = hwaMemFFTDestPingOffset;
    hwaParamCfg[paramsetIdx].source.srcShift = 0;
    hwaParamCfg[paramsetIdx].source.srcCircShiftWrap = 0;
    hwaParamCfg[paramsetIdx].source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg[paramsetIdx].source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
    hwaParamCfg[paramsetIdx].source.srcSign = HWA_SAMPLES_SIGNED;
    hwaParamCfg[paramsetIdx].source.srcConjugate = 0;
    hwaParamCfg[paramsetIdx].source.srcScale = 0;
    hwaParamCfg[paramsetIdx].source.bpmEnable = 0;
    hwaParamCfg[paramsetIdx].source.bpmPhase = 0;

    hwaParamCfg[paramsetIdx].dest.dstAddr = hwaMemCmpDestPingOffset;
    hwaParamCfg[paramsetIdx].dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
    hwaParamCfg[paramsetIdx].dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
    hwaParamCfg[paramsetIdx].dest.dstSign = HWA_SAMPLES_UNSIGNED; 
    hwaParamCfg[paramsetIdx].dest.dstConjugate = 0; 
    hwaParamCfg[paramsetIdx].dest.dstScale = 0;
    hwaParamCfg[paramsetIdx].dest.dstSkipInit = 0; 

    hwaParamCfg[paramsetIdx].accelModeArgs.compressMode.compressDecompress  = HWA_CMP_DCMP_COMPRESS;
    hwaParamCfg[paramsetIdx].accelModeArgs.compressMode.method   = HWA_COMPRESS_METHOD_EGE;
    hwaParamCfg[paramsetIdx].accelModeArgs.compressMode.ditherEnable = HWA_FEATURE_BIT_ENABLE;
    hwaParamCfg[paramsetIdx].accelModeArgs.compressMode.passSelect  = HWA_COMPRESS_PATHSELECT_BOTHPASSES;
    hwaParamCfg[paramsetIdx].accelModeArgs.compressMode.headerEnable = HWA_FEATURE_BIT_ENABLE;
    hwaParamCfg[paramsetIdx].accelModeArgs.compressMode.scaleFactorBW = 4;
    hwaParamCfg[paramsetIdx].accelModeArgs.compressMode.EGEKarrayLength = 3;

    /* HWA range FFT src/dst configuration*/
    hwaParamCfg[paramsetIdx].source.srcAcnt = numSamplesPerBlockIn - 1; 
    hwaParamCfg[paramsetIdx].source.srcAIdx = sizeof(uint32_t);
    hwaParamCfg[paramsetIdx].source.srcBcnt = numBlocks - 1;
    hwaParamCfg[paramsetIdx].source.srcBIdx = numSamplesPerBlockIn*hwaParamCfg[paramsetIdx].source.srcAIdx;

    hwaParamCfg[paramsetIdx].dest.dstAcnt = numSamplesPerBlockOut - 1; 
    hwaParamCfg[paramsetIdx].dest.dstAIdx = sizeof(uint32_t);
    hwaParamCfg[paramsetIdx].dest.dstBIdx = (hwaParamCfg[paramsetIdx].dest.dstAcnt+1)*hwaParamCfg[paramsetIdx].dest.dstAIdx; 
	
    errCode = HWA_configParamSet(hwaHandle,
                                  hwParamsetIdx,
                                  &hwaParamCfg[paramsetIdx],NULL);
    if (errCode != 0)
    {
		goto exit;
    }

    /* enable the DMA hookup to this paramset so that data gets copied out */
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
    paramISRConfig.dma.dstChannel = destChanPing;

    errCode = HWA_enableParamSetInterrupt(hwaHandle,hwParamsetIdx,&paramISRConfig);
    if (errCode != 0)
    {
        goto exit;
    }

    /***********************/
    /* PONG DUMMY PARAMSET */
    /***********************/
    paramsetIdx++;
    hwParamsetIdx++;

    hwaParamCfg[paramsetIdx].triggerMode = HWA_TRIG_MODE_DMA;
    hwaParamCfg[paramsetIdx].dmaTriggerSrc = hwParamsetIdx;
    hwaParamCfg[paramsetIdx].accelMode = HWA_ACCELMODE_NONE;
    errCode = HWA_configParamSet(hwaHandle, 
                                  hwParamsetIdx,
                                  &hwaParamCfg[paramsetIdx],NULL);
    if (errCode != 0)
    {
        goto exit;
    }

    /*****************************/
    /* PONG FFT PROCESS PARAMSET */
    /*****************************/
    paramsetIdx++;
    hwParamsetIdx++;
    hwaParamCfg[paramsetIdx] = hwaParamCfg[pingFFTParamSetIdx];
    hwaParamCfg[paramsetIdx].source.srcAddr = hwaMemSrcPongOffset; 
    hwaParamCfg[paramsetIdx].dest.dstAddr 	= hwaMemFFTDestPongOffset;

    hwaParamCfg[paramsetIdx].dmaTriggerSrc = hwParamsetIdx;

    errCode = HWA_configParamSet(hwaHandle,
                                  hwParamsetIdx,
                                  &hwaParamCfg[paramsetIdx],NULL);
    if (errCode != 0)
    {
        goto exit;
    }

    paramsetIdx++;
    hwParamsetIdx++;
    hwaParamCfg[paramsetIdx] = hwaParamCfg[pingCMPParamSetIdx];
    hwaParamCfg[paramsetIdx].source.srcAddr = hwaMemFFTDestPongOffset; 
    hwaParamCfg[paramsetIdx].dest.dstAddr = hwaMemCmpDestPongOffset;
    
    errCode = HWA_configParamSet(hwaHandle,
                                  hwParamsetIdx,
                                  &hwaParamCfg[paramsetIdx],NULL);
    if (errCode != 0)
    {
        goto exit;
    }

    /* Enable the DMA hookup to this paramset so that data gets copied out */
    paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
    paramISRConfig.dma.dstChannel = destChanPong;
    errCode = HWA_enableParamSetInterrupt(hwaHandle, 
                                           hwParamsetIdx,
                                           &paramISRConfig);
    if (errCode != 0)
    {
        goto exit;
    }
exit:
    return(errCode);
}

/**
 *  @b Description
 *  @n
 *      Trigger HWA for range processing.
 *
 *  @param[in]  rangeProcCmpObj              Pointer to rangeProcCmp object
 *
 *  \ingroup    DPU_RANGEPROCCMP_INTERNAL_FUNCTION
 *
 *  @retVal
 *      Success     - 0
 *  @retVal
 *      Error       - <0
 */
static int32_t rangeProcCmpHWA_TriggerHWA
(
    rangeProcCmpHWAObj     *rangeProcCmpObj
)
{
    int32_t             retVal = 0;
    HWA_Handle          hwaHandle;

    /* Get HWA driver handle */
    hwaHandle = rangeProcCmpObj->initParms.hwaHandle;

    /* Configure HWA common parameters */
    retVal = rangeProcCmpHWA_ConfigHWACommon(rangeProcCmpObj);
    if(retVal < 0)
    {
        goto exit;
    }

    /* Enable the HWA */
    retVal = HWA_enable(hwaHandle, 1);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Trigger the HWA paramset for Ping */
    retVal = HWA_setDMA2ACCManualTrig(hwaHandle, rangeProcCmpObj->dataOutTrigger[0]);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Trigger the HWA paramset for Pong */
    retVal = HWA_setDMA2ACCManualTrig(hwaHandle, rangeProcCmpObj->dataOutTrigger[1]);
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      EDMA configuration for rangeProcCmp data output in interleave mode
 *
 *  @param[in]  rangeProcCmpObj              Pointer to rangeProcCmp object
 *  @param[in]  DPParams                  Pointer to datapath parameter
 *  @param[in]  pHwConfig                 Pointer to rangeProcCmp hardware resources
 *  @param[in]  hwaOutPingOffset          Ping HWA memory address offset
 *  @param[in]  hwaOutPongOffset          Pong HWA memory address offset
 *
 *  \ingroup    DPU_RANGEPROCCMP_INTERNAL_FUNCTION
 *
 *  @retVal
 *      Success     - 0
 *  @retVal
 *      Error       - <0
 */

static int32_t rangeProcCmpHWA_ConfigEDMA_DataOut_interleave
(
    rangeProcCmpHWAObj     *rangeProcCmpObj,
    rangeProcCmp_dpParams  *DPParams,
    DPU_RangeProcCmpHWA_HW_Resources *pHwConfig,
    uint32_t            hwaOutPingOffset,
    uint32_t            hwaOutPongOffset
)
{
    int32_t             errorCode = EDMA_NO_ERROR;
    EDMA_Handle         handle ;
    DPEDMA_ChainingCfg       chainingCfg;
	uint32_t compressionRatio = DPParams->compressCfg.ratio;
	uint32_t pongOffsetIndx = (((uint32_t)DPParams->numRxAntennas * DPParams->compressCfg.numRangeBinsPerBlock) *  compressionRatio) >> HWA_CMP_RATIO_BW;
    
    /* Get rangeProcCmp hardware resources pointer */
    handle = rangeProcCmpObj->edmaHandle;

    /* Setup Chaining configuration */
    chainingCfg.chainingChan = pHwConfig->edmaOutCfg.dataOutSignature.channel;
    chainingCfg.isIntermediateChainingEnabled = true;
    chainingCfg.isFinalChainingEnabled = true;
	
	
    errorCode = rangeProcCmpHWA_ConfigEDMATranspose(DPParams,
                                        handle,
                                        &pHwConfig->edmaOutCfg.u.fmt1.dataOutPing,
                                        &chainingCfg,
                                        hwaOutPingOffset,
                                        (uint32_t)rangeProcCmpObj->radarCubebuf,
                                        false,  /* isTransferCompletionEnabled */
                                        NULL,   /* transferCompletionCallbackFxn */
                                        NULL);
    if (errorCode != EDMA_NO_ERROR)
    {
        goto exit;
    }
	
	errorCode = rangeProcCmpHWA_ConfigEDMATranspose(DPParams,
                                        handle,
                                        &pHwConfig->edmaOutCfg.u.fmt1.dataOutPong,
                                        &chainingCfg,
                                        hwaOutPongOffset,
                                        (uint32_t)&rangeProcCmpObj->radarCubebuf[pongOffsetIndx],
                                        true,
                                        rangeProcCmpHWA_EDMA_transferCompletionCallbackFxn,  
                                        (uintptr_t)rangeProcCmpObj);
    if (errorCode != EDMA_NO_ERROR)
    {
        goto exit;
    }

     /**************************************************************************
      *  HWA hot signature EDMA, chained to the transpose EDMA channels
      *************************************************************************/
    errorCode = DPEDMAHWA_configTwoHotSignature(handle, 
                                                  &pHwConfig->edmaOutCfg.dataOutSignature,
                                                  rangeProcCmpObj->initParms.hwaHandle,
                                                  rangeProcCmpObj->dataOutTrigger[0],
                                                  rangeProcCmpObj->dataOutTrigger[1],
                                                  false);
    if (errorCode != EDMA_NO_ERROR)
    {
        goto exit;
    }

exit:
    return(errorCode);
}


/**
 *   Description
 *  @n
 *      EDMA configuration for rangeProcCmp data in when EDMA is used to copy data from 
 *  ADCBuf to HWA memory
 *
 *  @param[in]  rangeProcCmpObj              Pointer to rangeProcCmp object handle
 *  @param[in]  DPParams                  Pointer to datapath parameter
 *  @param[in]  pHwConfig                 Pointer to rangeProcCmp hardware resources
 *
 *  \ingroup    DPU_RANGEPROCCMP_INTERNAL_FUNCTION
 *
 *  @retVal
 *      Success     - 0
 *  @retVal
 *      Error       - <0
 */
DPEDMA_syncACfg    syncACfg;

static int32_t rangeProcCmpHWA_ConfigEDMA_DataIn
(
    rangeProcCmpHWAObj         *rangeProcCmpObj,
    rangeProcCmp_dpParams      *DPParams,
    DPU_RangeProcCmpHWA_HW_Resources *pHwConfig
)
{
    int32_t             errorCode = EDMA_NO_ERROR;
    EDMA_Handle         handle ;
    DPEDMA_ChainingCfg  chainingCfg;
	uint32_t bytePerRxChan;
    /* Get rangeProcCmp Configuration */
    handle = rangeProcCmpObj->edmaHandle;

    bytePerRxChan = DPParams->numAdcSamples * sizeof(cmplx16ImRe_t);

    /**********************************************/
    /* ADCBuf -> Ping/Pong Buffer(M0 and M1)           */
    /**********************************************/
    chainingCfg.chainingChan = pHwConfig->edmaInCfg.dataInSignature.channel;
    chainingCfg.isFinalChainingEnabled = true;
    chainingCfg.isIntermediateChainingEnabled = true;

    if (rangeProcCmpObj->interleave == DPIF_RXCHAN_NON_INTERLEAVE_MODE)
    {
		errorCode = -234234;
		goto exit;
    }
    else
    {
        
        syncACfg.srcAddress = (uint32_t)rangeProcCmpObj->ADCdataBuf;
        syncACfg.destAddress = rangeProcCmpObj->hwaMemBankAddr[0];
        syncACfg.aCount = bytePerRxChan * DPParams->numRxAntennas;
        syncACfg.bCount =2U; /* ping and pong */
        syncACfg.srcBIdx=0U;
        syncACfg.dstBIdx=((uint32_t)rangeProcCmpObj->hwaMemBankAddr[1] - (uint32_t)rangeProcCmpObj->hwaMemBankAddr[0]);

        errorCode = DPEDMA_configSyncA_singleFrame(handle,
                                        &pHwConfig->edmaInCfg.dataIn,
                                        &chainingCfg,
                                        &syncACfg,
                                        true,    /* isEventTriggered */
                                        false,   /* isIntermediateTransferInterruptEnabled */
                                        false,   /* isFinalTransferInterruptEnabled */
                                        NULL,
                                        (uintptr_t)NULL);
    }

    if (errorCode != EDMA_NO_ERROR)
    {
        goto exit;
    }

    /*************************************************/
    /* Generate Hot Signature to trigger Ping/Pong paramset   */
    /*************************************************/

    errorCode = DPEDMAHWA_configTwoHotSignature(handle, 
                                                  &pHwConfig->edmaInCfg.dataInSignature,
                                                  rangeProcCmpObj->initParms.hwaHandle,
                                                  rangeProcCmpObj->dataInTrigger[0],
                                                  rangeProcCmpObj->dataInTrigger[1],
                                                  false);

    if (errorCode != EDMA_NO_ERROR)
    {
        goto exit;
    }
exit:
    return(errorCode);
}

/**
 *  @b Description
 *  @n
 *      rangeProcCmp configuraiton in interleaved mode
 *
 *  @param[in]  rangeProcCmpObj                 Pointer to rangeProcCmp object
 *  @param[in]  DPParams                     Pointer to data path common params
 *  @param[in]  pHwConfig                    Pointer to rangeProcCmp hardware resources
 *
 *  \ingroup    DPU_RANGEPROCCMP_INTERNAL_FUNCTION
 *
 *  @retVal
 *      Success     - 0
 *  @retVal
 *      Error       - <0
 *      Error       - <0
 */
uint8_t             destChanPing;
uint8_t             destChanPong;
static int32_t rangeProcCmpHWA_ConfigInterleaveMode
(
    rangeProcCmpHWAObj         *rangeProcCmpObj,
    rangeProcCmp_dpParams      *DPParams,
    DPU_RangeProcCmpHWA_HW_Resources *pHwConfig
)
{
    int32_t             retVal = 0;
    HWA_Handle          hwaHandle;

    hwaHandle = rangeProcCmpObj->initParms.hwaHandle;

    /* In interleave mode, only edmaOutCfgFmt is supported */
    retVal = HWA_getDMAChanIndex(hwaHandle, pHwConfig->edmaOutCfg.u.fmt1.dataOutPing.channel, &destChanPing);
    if (retVal != 0)
    {
        goto exit;
    }

    /* In interleave mode, only edmaOutCfgFmt is supported */
    retVal = HWA_getDMAChanIndex(hwaHandle, pHwConfig->edmaOutCfg.u.fmt1.dataOutPong.channel, &destChanPong);
    if (retVal != 0)
    {
        goto exit;
    }

    if(pHwConfig->hwaCfg.dataInputMode == DPU_RangeProcCmpHWA_InputMode_ISOLATED)
    {
        /* Copy data from ADC buffer to HWA buffer */
        rangeProcCmpHWA_ConfigEDMA_DataIn(rangeProcCmpObj,    DPParams, pHwConfig);

        /* Range FFT configuration in HWA */
		retVal = rangeFFTandCmpProcHWA_ConfigHWA(rangeProcCmpObj,
            destChanPing,
            destChanPong,
            ADDR_TRANSLATE_CPU_TO_HWA(rangeProcCmpObj->hwaMemBankAddr[0]),
            ADDR_TRANSLATE_CPU_TO_HWA(rangeProcCmpObj->hwaMemBankAddr[1]),
            ADDR_TRANSLATE_CPU_TO_HWA(rangeProcCmpObj->hwaMemBankAddr[2]),
            ADDR_TRANSLATE_CPU_TO_HWA(rangeProcCmpObj->hwaMemBankAddr[3]),
            ADDR_TRANSLATE_CPU_TO_HWA(rangeProcCmpObj->hwaMemBankAddr[0]),
            ADDR_TRANSLATE_CPU_TO_HWA(rangeProcCmpObj->hwaMemBankAddr[1])
        );
        if(retVal < 0)
        {
            goto exit;
        }
		
		/* EDMA configuration */
		retVal = rangeProcCmpHWA_ConfigEDMA_DataOut_interleave(rangeProcCmpObj,
													  DPParams,
													  pHwConfig,
													  (uint32_t)rangeProcCmpObj->hwaMemBankAddr[0],
													  (uint32_t)rangeProcCmpObj->hwaMemBankAddr[1]);
		
    }
    else
    {
		retVal = -1231;
    }

    

exit:
    return (retVal);
}



/**
 *  @b Description
 *  @n
 *      Compensation of DC range antenna signature Init function
 *
 *  @param[in]  rangeProcCmpObj                 Pointer to rangeProcCmp object
 *  @param[in]  calibDcRangeSigCfg           Pointer DC range compensation configuration
 *  @param[in]  resetMeanBuffer              Flag to indicate if buffer need to be reset
 *
 *  \ingroup    DPU_RANGEPROCCMP_INTERNAL_FUNCTION
 *
 *  @retVal
 *      Success     - 0
 *  @retVal
 *      Error       - <0
 */
static int32_t rangeProcCmpHWA_dcRangeSignatureCompensation_init
(
    rangeProcCmpHWAObj     *rangeProcCmpObj,
    DPU_RangeProcCmp_CalibDcRangeSigCfg *calibDcRangeSigCfg,
    uint8_t             resetMeanBuffer
)
{
    int32_t                 retVal = 0;
    uint32_t                meanbufSize;

    meanbufSize = DPU_RANGEPROCCMP_SIGNATURE_COMP_MAX_BIN_SIZE * rangeProcCmpObj->params.numVirtualAntennas
                 * sizeof(cmplx32ImRe_t);

    /* Validate DC revomal configuraiton */
    if(calibDcRangeSigCfg->enabled)
    {
        if(rangeProcCmpObj->dcRangeSigMean == (cmplx32ImRe_t*)NULL)
        {
            /* Check DC range average buffer pointer */
            retVal = DPU_RANGEPROCCMPHWA_EDCREMOVAL;
            goto exit;
        }
        else if(meanbufSize > rangeProcCmpObj->dcRangeSigMeanSize)
        {
            /* Check DC range average buffer pointer */
            retVal = DPU_RANGEPROCCMPHWA_EDCREMOVAL;
            goto exit;
        }
        else
        {
            /* Initialize memory */
            if (resetMeanBuffer == 1U)
            {
                memset((void *)rangeProcCmpObj->dcRangeSigMean, 0, meanbufSize);
                rangeProcCmpObj->dcRangeSigCalibCntr = 0;
            }
            rangeProcCmpObj->calibDcNumLog2AvgChirps = mathUtils_floorLog2(calibDcRangeSigCfg->numAvgChirps);
        }
    }
    else
    {
        /* Feature is disabled , nothing needs to done here */
    }

exit:
    return (retVal);
}


/**
 *  @b Description
 *  @n
 *      Internal function to parse rangeProcCmp configuration and save in internal rangeProcCmp object
 *
 *  @param[in]  rangeProcCmpObj              Pointer to rangeProcCmp object
 *  @param[in]  pConfigIn                 Pointer to rangeProcCmpHWA configuration structure
 *
 *  \ingroup    DPU_RANGEPROCCMP_INTERNAL_FUNCTION
 *
 *  @retVal
 *      Success     - 0
 *  @retVal
 *      Error       - <0
 */
static int32_t rangeProcCmpHWA_ParseConfig
(
    rangeProcCmpHWAObj         *rangeProcCmpObj,
    DPU_RangeProcCmpHWA_Config  *pConfigIn
)
{
    int32_t                 retVal = 0;
    rangeProcCmp_dpParams      *params;
    DPU_RangeProcCmpHWA_StaticConfig   *pStaticCfg;

    /* Get configuration pointers */
    pStaticCfg = &pConfigIn->staticCfg;
    params    = &rangeProcCmpObj->params;

    /* Save datapath parameters */
    params->numTxAntennas = pStaticCfg->numTxAntennas;
    params->numRxAntennas = pStaticCfg->ADCBufData.dataProperty.numRxAntennas;
    params->numVirtualAntennas = pStaticCfg->numVirtualAntennas;
    params->numChirpsPerChirpEvent = pStaticCfg->ADCBufData.dataProperty.numChirpsPerChirpEvent;
    params->numAdcSamples = pStaticCfg->ADCBufData.dataProperty.numAdcSamples;
    params->numRangeBins = pStaticCfg->numRangeBins;
    params->numChirpsPerFrame = pStaticCfg->numChirpsPerFrame;
    params->numDopplerChirps = pStaticCfg->numChirpsPerFrame/pStaticCfg->numTxAntennas;
	params->compressCfg  = pStaticCfg->compressCfg;
	
    /* Save buffers */
    rangeProcCmpObj->ADCdataBuf        = (cmplx16ImRe_t *)pStaticCfg->ADCBufData.data;
    rangeProcCmpObj->radarCubebuf      = (cmplx16ImRe_t *)pConfigIn->hwRes.radarCube.data;

    /* Save interleave mode from ADCBuf configuraiton */
    rangeProcCmpObj->interleave = pStaticCfg->ADCBufData.dataProperty.interleave;

    if((rangeProcCmpObj->interleave ==DPIF_RXCHAN_NON_INTERLEAVE_MODE) &&
        (rangeProcCmpObj->params.numRxAntennas > 1) )
    {
        /* For rangeProcDPU needs rx channel has same offset from one channel to the next channel
           Use first two channel offset to calculate the BIdx for EDMA
         */
        rangeProcCmpObj->rxChanOffset = pStaticCfg->ADCBufData.dataProperty.rxChanOffset[1] - 
                                    pStaticCfg->ADCBufData.dataProperty.rxChanOffset[0];

        /* rxChanOffset should be 16 bytes aligned and should be big enough to hold numAdcSamples */
        if ((rangeProcCmpObj->rxChanOffset < (rangeProcCmpObj->params.numAdcSamples * sizeof(cmplx16ImRe_t))) ||
          ((rangeProcCmpObj->rxChanOffset & 0xF) != 0))
        {
            retVal = DPU_RANGEPROCCMPHWA_EADCBUF_INTF;
            goto exit;
        }
    }

    /* Save RadarCube format */
    if (pConfigIn->hwRes.radarCube.datafmt == DPIF_RADARCUBE_FORMAT_2)
    {
        rangeProcCmpObj->radarCubeLayout = rangeProcCmp_dataLayout_RANGE_DOPPLER_TxAnt_RxAnt;
    }
    else if(pConfigIn->hwRes.radarCube.datafmt == DPIF_RADARCUBE_FORMAT_1)
    {
        rangeProcCmpObj->radarCubeLayout = rangeProcCmp_dataLayout_TxAnt_DOPPLER_RxAnt_RANGE;
    }
    else
    {
        retVal = DPU_RANGEPROCCMPHWA_EINTERNAL;
        goto exit;
    }

    /* The following case can not be handled with the current 1TX EDMA scheme, reason is the Bindex exceeds what EDMA can handle. */
    if( (params->numRangeBins == 1024U) &&
       (params->numTxAntennas == 1U) &&
       (params->numRxAntennas == 4U) &&
       (rangeProcCmpObj->radarCubeLayout == rangeProcCmp_dataLayout_TxAnt_DOPPLER_RxAnt_RANGE) )
    {
        retVal = DPU_RANGEPROCCMPHWA_ENOTIMPL;
        goto exit;
    }

    /* The following case can not be handled with the current 3TX EDMA scheme, reason is the Bindex exceeds what EDMA(jump index<32768) can handle. */
    if( (params->numRangeBins == 1024U) &&
       (params->numTxAntennas == 3U) &&
       (params->numRxAntennas == 4U) &&
       (rangeProcCmpObj->radarCubeLayout == rangeProcCmp_dataLayout_TxAnt_DOPPLER_RxAnt_RANGE) )
    {
        retVal = DPU_RANGEPROCCMPHWA_ENOTIMPL;
        goto exit;
    }

    /* Prepare internal hardware resouces = trigger source matchs its  paramset index */
    rangeProcCmpObj->dataInTrigger[0]      = 1U + pConfigIn->hwRes.hwaCfg.paramSetStartIdx;
    rangeProcCmpObj->dataInTrigger[1]      = 4U + pConfigIn->hwRes.hwaCfg.paramSetStartIdx;
    rangeProcCmpObj->dataOutTrigger[0]     = 0U + pConfigIn->hwRes.hwaCfg.paramSetStartIdx;
    rangeProcCmpObj->dataOutTrigger[1]     = 3U + pConfigIn->hwRes.hwaCfg.paramSetStartIdx;

    /* Save hardware resources that will be used at runtime */
    rangeProcCmpObj->edmaHandle= pConfigIn->hwRes.edmaHandle;
    rangeProcCmpObj->dataOutSignatureChan = pConfigIn->hwRes.edmaOutCfg.dataOutSignature.channel;
    rangeProcCmpObj->dcRangeSigMean = pConfigIn->hwRes.dcRangeSigMean;
    rangeProcCmpObj->dcRangeSigMeanSize = pConfigIn->hwRes.dcRangeSigMeanSize;
    memcpy((void *)&rangeProcCmpObj->hwaCfg, (void *)&pConfigIn->hwRes.hwaCfg, sizeof(DPU_RangeProcCmpHWA_HwaConfig));

exit:
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      Internal function to config HWA/EDMA to perform range FFT
 *
 *  @param[in]  rangeProcCmpObj              Pointer to rangeProcCmp object
 *  @param[in]  pHwConfig                 Pointer to rangeProcCmp hardware resources
 *
 *  \ingroup    DPU_RANGEPROCCMP_INTERNAL_FUNCTION
 *
 *  @retVal
 *      Success     - 0 
 *  @retVal
 *      Error       - <0
 */
static int32_t rangeProcCmpHWA_HardwareConfig
(
    rangeProcCmpHWAObj         *rangeProcCmpObj,
    DPU_RangeProcCmpHWA_HW_Resources *pHwConfig
)
{
    int32_t                 retVal = 0;
    rangeProcCmp_dpParams      *DPParams;
    DPParams    = &rangeProcCmpObj->params;

    if (rangeProcCmpObj->interleave == DPIF_RXCHAN_INTERLEAVE_MODE)
    {
        retVal = rangeProcCmpHWA_ConfigInterleaveMode(rangeProcCmpObj, DPParams, pHwConfig);
        if (retVal != 0)
        {
            goto exit;
        }
    }
    else
    {
		retVal = -5017;

		goto exit;
	}
exit:
    return(retVal);
}

/**************************************************************************
 ************************RangeProcCmpHWA_ External APIs **************************
 **************************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is rangeProcCmp DPU init function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @param[in]  initParams              Pointer to DPU init parameters
 *  @param[in]  errCode                 Pointer to errCode generates from the API
 *
 *  \ingroup    DPU_RANGEPROCCMP_EXTERNAL_FUNCTION
 *
 *  @retVal
 *      Success     - valid rangeProcCmp handle
 *  @retVal
 *      Error       - NULL
 */
DPU_RangeProcCmpHWA_Handle DPU_RangeProcCmpHWA_init
(
    DPU_RangeProcCmpHWA_InitParams     *initParams,
    int32_t*                        errCode
)
{
    rangeProcCmpHWAObj     *rangeProcCmpObj = NULL;
    SemaphoreP_Params   semParams;
    HWA_MemInfo         hwaMemInfo;
    uint8_t             index;

    *errCode = 0;

    if( (initParams == NULL) ||
       (initParams->hwaHandle == NULL) )
    {
        *errCode = DPU_RANGEPROCCMPHWA_EINVAL;
        goto exit;
    }

    /* Allocate Memory for rangeProcCmp */
    rangeProcCmpObj = MemoryP_ctrlAlloc(sizeof(rangeProcCmpHWAObj), 0);
    if(rangeProcCmpObj == NULL)
    {
        *errCode = DPU_RANGEPROCCMPHWA_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)rangeProcCmpObj, 0, sizeof(rangeProcCmpHWAObj));

    memcpy((void *)&rangeProcCmpObj->initParms, initParams, sizeof(DPU_RangeProcCmpHWA_InitParams));

    /* Set HWA bank memory address */
    *errCode =  HWA_getHWAMemInfo(initParams->hwaHandle, &hwaMemInfo);
    if (*errCode < 0)
    {
        goto exit;
    }

    for (index = 0; index < hwaMemInfo.numBanks; index++)
    {
        rangeProcCmpObj->hwaMemBankAddr[index] = hwaMemInfo.baseAddress + index * hwaMemInfo.bankSize;
    }

    /* Create semaphore for EDMA done */
    SemaphoreP_Params_init(&semParams);
    semParams.mode = SemaphoreP_Mode_BINARY;
    rangeProcCmpObj->edmaDoneSemaHandle = SemaphoreP_create(0, &semParams);
    if(rangeProcCmpObj->edmaDoneSemaHandle == NULL)
    {
        *errCode = DPU_RANGEPROCCMPHWA_ESEMA;
        goto exit;
    }

    /* Create semaphore for HWA done */
    SemaphoreP_Params_init(&semParams);
    semParams.mode = SemaphoreP_Mode_BINARY;
    rangeProcCmpObj->hwaDoneSemaHandle = SemaphoreP_create(0, &semParams);
    if(rangeProcCmpObj->hwaDoneSemaHandle == NULL)
    {
        *errCode = DPU_RANGEPROCCMPHWA_ESEMA;
        goto exit;
    }

exit:
    if(*errCode < 0)
    {
        if(rangeProcCmpObj != NULL)
        {
            MemoryP_ctrlFree(rangeProcCmpObj, sizeof(rangeProcCmpHWAObj));
        }

        rangeProcCmpObj = (DPU_RangeProcCmpHWA_Handle)NULL;
    }
    else
    {
        /* Fall through */
    }
    return ((DPU_RangeProcCmpHWA_Handle)rangeProcCmpObj);

}


/**
 *  @b Description
 *  @n
 *      The function is rangeProcCmp DPU config function. It saves buffer pointer and configurations 
 *  including system resources and configures HWA and EDMA for runtime range processing.
 *  
 *  @pre    DPU_RangeProcCmpHWA_init() has been called
 *
 *  @param[in]  handle                  rangeProcCmp DPU handle
 *  @param[in]  pConfigIn               Pointer to rangeProcCmp configuration data structure
 *
 *  \ingroup    DPU_RANGEPROCCMP_EXTERNAL_FUNCTION
 *
 *  @retVal
 *      Success     - 0
 *  @retVal
 *      Error       - <0
 */
int32_t DPU_RangeProcCmpHWA_config
(
    DPU_RangeProcCmpHWA_Handle  handle,
    DPU_RangeProcCmpHWA_Config  *pConfigIn
)
{
    rangeProcCmpHWAObj                 *rangeProcCmpObj;
    DPU_RangeProcCmpHWA_StaticConfig   *pStaticCfg;
    HWA_Handle                      hwaHandle;
    int32_t                         retVal = 0;
    uint32_t estRadarCubeSize;
	uint16_t compressionRatio;
    rangeProcCmpObj = (rangeProcCmpHWAObj *)handle;
    if(rangeProcCmpObj == NULL)
    {
        retVal = DPU_RANGEPROCCMPHWA_EINVAL;
		goto exit;
    }

    /* Get configuration pointers */
    pStaticCfg = &pConfigIn->staticCfg;
    hwaHandle = rangeProcCmpObj->initParms.hwaHandle;
	compressionRatio = pStaticCfg->compressCfg.ratio;

#if DEBUG_CHECK_PARAMS
    /* Validate params */
    if(!pConfigIn ||
      !(pConfigIn->hwRes.edmaHandle) ||
       (pConfigIn->hwRes.hwaCfg.numParamSet != DPU_RANGEPROCCMPHWA_NUM_HWA_PARAM_SETS)
      )
    {
        retVal = DPU_RANGEPROCCMPHWA_EINVAL;
		goto exit;
    }

    /* Parameter check: validate Adc data interface configuration
        Support:
            - 1 chirp per chirpEvent
            - Complex 16bit ADC data in IMRE format
     */
    if( (pStaticCfg->ADCBufData.dataProperty.dataFmt != DPIF_DATAFORMAT_COMPLEX16_IMRE) ||
       (pStaticCfg->ADCBufData.dataProperty.numChirpsPerChirpEvent != 1U) )
    {
        retVal = DPU_RANGEPROCCMPHWA_EADCBUF_INTF;
        goto exit;
    }

    /* Parameter check: windowing Size */
    {
        uint16_t expectedWinSize;

        if( pConfigIn->hwRes.hwaCfg.hwaWinSym == HWA_FFT_WINDOW_SYMMETRIC)
        {
            /* Only half of the windowing factor is needed for symmetric window */
            expectedWinSize = ((pStaticCfg->ADCBufData.dataProperty.numAdcSamples + 1U) / 2U ) * sizeof(uint32_t);
        }
        else
        {
            expectedWinSize = pStaticCfg->ADCBufData.dataProperty.numAdcSamples * sizeof(uint32_t);
        }

        if(pStaticCfg->windowSize != expectedWinSize)
        {
            retVal = DPU_RANGEPROCCMPHWA_EWINDOW;
			retVal = -3234;
        
			goto exit;
        }
    }

    /* Refer to radar cube definition for FORMAT_x , the following are the only supported formats
        Following assumption is made upon radar cube FORMAT_x definition 
           1. data type is complex in cmplx16ImRe_t format only
           2. It is always 1D range output.
     */
    if( (pConfigIn->hwRes.radarCube.datafmt != DPIF_RADARCUBE_FORMAT_1) &&
       (pConfigIn->hwRes.radarCube.datafmt != DPIF_RADARCUBE_FORMAT_2) )
    {
        retVal = DPU_RANGEPROCCMPHWA_ERADARCUBE_INTF;
		goto exit;
    }

    /* Not supported input & output format combination */
    if ((pStaticCfg->ADCBufData.dataProperty.interleave != DPIF_RXCHAN_INTERLEAVE_MODE) &&
         (pConfigIn->hwRes.radarCube.datafmt == DPIF_RADARCUBE_FORMAT_1) )
    {
        retVal = DPU_RANGEPROCCMPHWA_ENOTIMPL;
        goto exit;
    }
    if (pStaticCfg->ADCBufData.dataProperty.numRxAntennas == 3U)
    {
        retVal = DPU_RANGEPROCCMPHWA_ENOTIMPL;
		goto exit;
    }

	estRadarCubeSize = ((pStaticCfg->numRangeBins* sizeof(cmplx16ImRe_t) *
                                      pStaticCfg->numChirpsPerFrame * compressionRatio *
                                      pStaticCfg->ADCBufData.dataProperty.numRxAntennas) >> HWA_CMP_RATIO_BW);
    /* Parameter check: radarcube buffer Size */
    if(pConfigIn->hwRes.radarCube.dataSize !=  estRadarCubeSize)
    {
        retVal = DPU_RANGEPROCCMPHWA_ERADARCUBE_INTF;
		goto exit;
    }
#endif

    /* Save hardware resources */
    memcpy((void *)&rangeProcCmpObj->calibDcRangeSigCfg, (void *)pConfigIn->dynCfg.calibDcRangeSigCfg, sizeof(DPU_RangeProcCmp_CalibDcRangeSigCfg));

    retVal = rangeProcCmpHWA_ParseConfig(rangeProcCmpObj, pConfigIn);
    if (retVal < 0)
    {
		goto exit;
    }

    /* DC calibration and compensation init */
    retVal = rangeProcCmpHWA_dcRangeSignatureCompensation_init(rangeProcCmpObj, pConfigIn->dynCfg.calibDcRangeSigCfg, pStaticCfg->resetDcRangeSigMeanBuffer);
    if (retVal < 0)
    {
		goto exit;
    }

    /* Disable the HWA */
    retVal = HWA_enable(hwaHandle, 0);
    if (retVal != 0)
    {
		goto exit;
    }

    /* Reset the internal state of the HWA */
    retVal = HWA_reset(hwaHandle);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Windowing configuraiton in HWA */
    retVal = HWA_configRam(hwaHandle,
                            HWA_RAM_TYPE_WINDOW_RAM,
                            (uint8_t *)pStaticCfg->window,
                            pStaticCfg->windowSize,   /* size in bytes */
                            pConfigIn->hwRes.hwaCfg.hwaWinRamOffset * sizeof(uint32_t));
    if (retVal != 0)
    {
        goto exit;
    }

    /* Clear stats */
    rangeProcCmpObj->numProcess = 0U;

    /* Initial configuration of rangeProcCmp */
    retVal = rangeProcCmpHWA_HardwareConfig(rangeProcCmpObj, &pConfigIn->hwRes);

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is rangeProcCmp DPU process function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @pre    DPU_RangeProcCmpHWA_init() has been called
 *
 *  @param[in]  handle                  rangeProcCmp DPU handle
 *  @param[in]  outParams               DPU output parameters
 *
 *  \ingroup    DPU_RANGEPROCCMP_EXTERNAL_FUNCTION
 *
 *  @retVal
 *      Success     - 0
 *  @retVal
 *      Error       - <0
 */
int32_t DPU_RangeProcCmpHWA_process
(
    DPU_RangeProcCmpHWA_Handle     handle,
    DPU_RangeProcCmpHWA_OutParams  *outParams
)
{
    rangeProcCmpHWAObj     *rangeProcCmpObj;
    int32_t             retVal = 0;

    rangeProcCmpObj = (rangeProcCmpHWAObj *)handle;
    if ((rangeProcCmpObj == NULL) ||
        (outParams == NULL))
    {
        retVal = DPU_RANGEPROCCMPHWA_EINVAL;
        goto exit;
    }

    /* Set inProgress state */
    rangeProcCmpObj->inProgress = true;
    outParams->endOfChirp = false;

    /**********************************************/
    /* WAIT FOR HWA NUMLOOPS INTERRUPT            */
    /**********************************************/
    /* wait for the all paramSets done interrupt */
    SemaphoreP_pend(rangeProcCmpObj->hwaDoneSemaHandle, SemaphoreP_WAIT_FOREVER);

    /**********************************************/
    /* WAIT FOR EDMA INTERRUPT                    */
    /**********************************************/
    SemaphoreP_pend(rangeProcCmpObj->edmaDoneSemaHandle, SemaphoreP_WAIT_FOREVER);

    /* Range FFT is done, disable Done interrupt */
    HWA_disableDoneInterrupt(rangeProcCmpObj->initParms.hwaHandle);

    /* Disable the HWA */
    retVal = HWA_enable(rangeProcCmpObj->initParms.hwaHandle, 0);
    if (retVal != 0)
    {
        goto exit;
    }

    /* calib DC processing is not allowed. */
    if(rangeProcCmpObj->calibDcRangeSigCfg.enabled)
    {
         rangeProcCmpObj->calibDcRangeSigCfg.enabled = 0;
    }
    /* Update stats and output parameters */
    rangeProcCmpObj->numProcess++;

    /* Following stats is not available for rangeProcCmpHWA */
    outParams->stats.processingTime = 0;
    outParams->stats.waitTime= 0;

    outParams->endOfChirp = true;

    /* Clear inProgress state */
    rangeProcCmpObj->inProgress = false;

exit:

    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function is rangeProcCmp DPU control function. 
 *
 *  @pre    DPU_RangeProcCmpHWA_init() has been called
 *
 *  @param[in]  handle           rangeProcCmp DPU handle
 *  @param[in]  cmd              rangeProcCmp DPU control command
 *  @param[in]  arg              rangeProcCmp DPU control argument pointer
 *  @param[in]  argSize          rangeProcCmp DPU control argument size
 *
 *  \ingroup    DPU_RANGEPROCCMP_EXTERNAL_FUNCTION
 *
 *  @retVal
 *      Success     - 0
 *  @retVal
 *      Error       - <0
 */
int32_t DPU_RangeProcCmpHWA_control
(
    DPU_RangeProcCmpHWA_Handle handle,
    DPU_RangeProcCmpHWA_Cmd    cmd,
    void*                   arg,
    uint32_t                argSize
)
{
    int32_t             retVal = 0;
    rangeProcCmpHWAObj     *rangeProcCmpObj;

    /* Get rangeProcCmp data object */
    rangeProcCmpObj = (rangeProcCmpHWAObj *)handle;

    /* Sanity check */
    if (rangeProcCmpObj == NULL)
    {
        retVal = DPU_RANGEPROCCMPHWA_EINVAL;
        goto exit;
    }

    /* Check if control() is called during processing time */
    if(rangeProcCmpObj->inProgress == true)
    {
        retVal = DPU_RANGEPROCCMPHWA_EINPROGRESS;
        goto exit;
    }

    /* Control command handling */
    switch(cmd)
    {
        case DPU_RangeProcCmpHWA_Cmd_dcRangeCfg:
        
           retVal = -345; 
        break;

        case DPU_RangeProcCmpHWA_Cmd_triggerProc:
            /* Trigger rangeProcCmp in HWA */
            retVal = rangeProcCmpHWA_TriggerHWA( rangeProcCmpObj);
            if(retVal != 0)
            {
                goto exit;
            }
        break;

        default:
            retVal = DPU_RANGEPROCCMPHWA_ECMD;
            break;
    }
exit:
    return (retVal);
}


/**
 *  @b Description
 *  @n
 *      The function is rangeProcCmp DPU deinit function. It frees the resources used for the DPU.
 *
 *  @pre    DPU_RangeProcCmpHWA_init() has been called
 *
 *  @param[in]  handle           rangeProcCmp DPU handle
 *
 *  \ingroup    DPU_RANGEPROCCMP_EXTERNAL_FUNCTION
 *
 *  @retVal
 *      Success     - 0
 *  @retVal
 *      Error       - <0
 */
int32_t DPU_RangeProcCmpHWA_deinit
(
    DPU_RangeProcCmpHWA_Handle     handle
)
{
    rangeProcCmpHWAObj     *rangeProcCmpObj;
    int32_t             retVal = 0;

    /* Sanity Check */
    rangeProcCmpObj = (rangeProcCmpHWAObj *)handle;
    if(rangeProcCmpObj == NULL)
    {
        retVal = DPU_RANGEPROCCMPHWA_EINVAL;
        goto exit;
    }

    /* Delete Semaphores */
    SemaphoreP_delete(rangeProcCmpObj->edmaDoneSemaHandle);
    SemaphoreP_delete(rangeProcCmpObj->hwaDoneSemaHandle);

    /* Free memory */
    MemoryP_ctrlFree(handle, sizeof(rangeProcCmpHWAObj));
exit:

    return (retVal);
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
 *  @retVal
 *      Success     - 0
 *  @retVal
 *      Error       - <0
 */
void  cfgEGEParamListRangeProc (uint8_t * cmpEGEArr, uint16_t compressionRatio, uint16_t srcWidth)
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
