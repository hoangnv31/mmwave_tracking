/**
 *   @file  aoaprocdcmphwa.c
 *
 *   @brief
 *      Implements Data path processing Unit using HWA.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2018-2019 Texas Instruments, Inc.
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

/* mmWave SDK drivers/common Include Files */
#include <ti/common/sys_common.h>
#include <ti/drivers/osal/SemaphoreP.h>
#include <ti/drivers/osal/MemoryP.h>

/* Compression constants. */
#include <ti/demo/xwr64xx_compression/mmw/compressionConsts.h>
/* Data Path Include Files */
#include <ti/datapath/dpedma/dpedma.h>
#include <ti/datapath/dpedma/dpedmahwa.h>
#include <ti/datapath/dpc/dpu/aoaprocdcmp/include/aoaprocdcmphwa_internal.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/utils/mathutils/mathutils.h>



//#define DBG_AOA_HWA_OBJ_DPU

#ifdef DBG_AOA_HWA_OBJ_DPU
volatile AOAHwaObj *gAoaHwaObj[RL_MAX_SUBFRAMES] = {NULL};
volatile uint32_t gAoaHwaObjInd = 0;
#endif

//#define PROFILE_AOA_HWA_OBJ_DPU

#ifdef PROFILE_AOA_HWA_OBJ_DPU
DPU_AoAProcDcmpHWA_OutParams       gAoAProcDcmpStats[16];
volatile uint32_t gAoAProcDcmpStatsIdx = 0;
#endif

 void intrleavdInp_interleavdOutput_FFTCfg(
	HWA_ParamConfig *pHWA_ParamConfig, 
	uint16_t trigMode, 
	uint16_t trigSrc, 
	uint16_t srcAddr,  
	uint16_t dstAddr,
	uint16_t numDopplerChirps, 
	uint16_t numRxAnt, 
	uint16_t numDopplerBins, 
	uint16_t numVirtualAnt, 
	uint32_t windowOffset, 
	uint8_t  winSym);
void linear_50p_DcmpCfg(
	HWA_ParamConfig* pHWA_ParamConfig, 
	uint16_t trigMode, 
	uint16_t trigSrc, 
	uint32_t srcAddr, 
	uint32_t dstAddr, 
	uint16_t numSamplesPerBlockIn,  
	uint16_t numSamplesPerBlockOut, 
	uint16_t numBlocks);
void  cfgEGEParamListAoaProc (uint8_t * cmpEGEArr, uint16_t compressionRatio, uint16_t srcWidth);

/**
 *  @b Description
 *  @n
 *      HWA processing completion call back function as per HWA API.
 *      Depending on the programmed transfer completion codes,
 *      posts HWA done semaphore.
 *
 *  @param[in]  arg                 Argument to the callback function
 *
 *  \ingroup    DPU_AOAPROCDCMP_INTERNAL_FUNCTION
 *
 *  @retval     None
 */
static void AOAProcHWADoneIsrCallback(void * arg)
{
    if (arg != NULL) 
    {
        SemaphoreP_post((SemaphoreP_Handle)arg);
    }
}

static uint32_t AoAProcDcmpHWA_angleEstimationAzimElev
(
    AOAHwaObj       *aoaHwaObj,
    uint32_t        objInIdx,
    uint32_t        pingPongIdx,
    uint32_t        objOutIdx,
    uint16_t *azimFFTAbsPtr,
	cmplx16ImRe_t * azimFFTPtrBase
);


static int32_t HWAutil_configHWA_extendedVelocityScheme
(
    HWA_Handle      hwaHandle,
    DPU_AoAProcDcmpHWA_HW_Resources *res,
    DPU_AoAProcDcmpHWA_StaticConfig *DPParams,
    uint32_t        *hwaMemBankAddr,
    uint8_t         numHypotheses,
    uint32_t        numAzimuthBins
);

/**
 *   @b Description
 *   @n
 *      AoAProcDcmpHWA Rx channel phase/gain compensation. The function performs phase/gain
 *      compensation on the Rx virtual antenna symbols of the detected object. The
 *      function is also used to perform the phase/gain compensation on the two dimensional
 *      array of only azimuth antennas which is used for the azimuth heat map display
 *      on the host. In this case the two dimensional array is laid out as
 *      X[number of range bins][number of azimuth antennas].
 *
 * @param[in]      rxChComp Rx channel phase/gain compensation coefficients
 *
 * @param[in]      numObj Number of detected points
 *
 * @param[in]      numAnt number of antennas
 *
 * @param[in]      symbolsIn Pointer to array of input symbols
 *
 * @param[out]     symbolsOut Pointer to array of output symbols
 *
 * @return         void
 *
 *  \ingroup    DPU_AOAPROCDCMP_INTERNAL_FUNCTION
 *
 */
static void AoAProcDcmpHWA_rxChanPhaseBiasCompensation(cmplx16ImRe_t *rxChComp,
                                                   uint32_t numObj,
                                                   uint32_t numAnt,
                                                   cmplx16ImRe_t *symbolsIn,
                                                   cmplx16ImRe_t *symbolsOut)
{
    int32_t Re, Im;
    uint32_t i, j;
    uint32_t antIndx;
    uint32_t objIdx;

    for (objIdx = 0; objIdx < numObj; objIdx++)
    {
        j = 0;
        /* Compensation of azimuth antennas */
        for (antIndx = 0; antIndx < numAnt; antIndx++)
        {
            i = objIdx*numAnt + antIndx;
            Re = (int32_t) symbolsIn[i].real * (int32_t) rxChComp[j].real -
                 (int32_t) symbolsIn[i].imag * (int32_t) rxChComp[j].imag;

            Im = (int32_t) symbolsIn[i].real * (int32_t) rxChComp[j].imag +
                 (int32_t) symbolsIn[i].imag * (int32_t) rxChComp[j].real;
            MATHUTILS_ROUND_AND_SATURATE_Q15(Re);
            MATHUTILS_ROUND_AND_SATURATE_Q15(Im);

            symbolsOut[i].real = (int16_t) Re;
            symbolsOut[i].imag = (int16_t) Im;
            j++;
        }
    }
}

/**
 *  @b Description
 *  @n
 *      The function configures EDMA for data transfers to/from HWA in a two stage
 *      data processing where the first stage is 2D-FFT (Doppler FFT) computation,
 *      and the second stage is 3D-FFT (azimuth FFT) computation.
 *      The 2D + 3D processing is organized in a ping/pong manner to run in parallel
 *      with input/output data transfers. Each ping/pong path processes one
 *      detected object at a time. Total 4 physical EDMA channels are used, two  per ping/pong path.
 *      In each path one channel is used for input data transfers to both first and second stage,
 *      and the other channel is used for output data transfers from both first and second stage.
 *
 *  @param[in]  hwaHandle           Handle to HWA
 *  @param[in]  res                 Pointer to hardware resources structure
 *  @param[in]  DPParams            Pointer to profile parameters
 *  @param[in]  srcIn2DFFTBuffAddr  Source address of the 1D-FFT radar cube
 *                                  matrix, the address is set in every iteration
 *                                  by local core per detected object according
 *                                  to its index.
 *  @param[in]  dstIn2DFFTBuffAddr  Destination addresses in HWA memory of
 *                                  ping/pong paths, 0-ping, 1-pong, for 2D-FFT calculation.
 *  @param[in]  srcOut2DFFTBuffAddr Source 2DFFT output addresses in HWA memory
 *                                  of ping/pong paths, 0-ping, 1-pong, the address
 *                                  is set in every iteration by local core per
 *                                  detected object according to its doppler index
 *  @param[in]  dstOut2DFFTBuffAddr Destination 2DFFT output address in local
 *                                  core memory of ping/pong paths. Output data
 *                                  consists of array of 2D antenna symbols corresponding
 *                                  to detected object range/doppler index. The
 *                                  2D array format is X[numTxAntenna][numRxAntenna]
 *  @param[in]  srcIn3DFFTBuffAddr  Address of 2DFFT antenna symbols, (Doppler and
 *                                  phase/gain compensated), of ping/pong paths,
 *                                  0-ping, 1-pong. In each path, it is 3D array
 *                                  in the form X[numberHypotheses][numTxAntenna][numRxAntenna],
 *                                  where the number of hypotheses is equal to number
 *                                  of Tx antennas in TDM MIMO scheme when extended
 *                                  maximum velocity scheme is enabled, otherwise
 *                                  number of hypotheses is equal to one.
 *  @param[in]  dstIn3DFFTBuffAddr  Destination addresses in HWA memory of ping/pong
 *                                  paths, 0-ping, 1-pong, for 3D FFT calculation.
 *  @param[in]  srcOut3DFFTBuffAddr Source 3DFFT output addresses in HWA memory of
 *                                  ping/pong paths, 0-ping, 1-pong.
 *  @param[in]  dstOut3DFFTBuffAddr Destination 3DFFT output address in local core
 *                                  memory, 0-ping, 1-pong.
 *  @param[in]  extMaxVelEnabled    0 - extended maximum velocity feature enabled,
 *                                  1 - disabled
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 *
 *  \ingroup    DPU_AOAPROCDCMP_INTERNAL_FUNCTION
 *
 */
static int32_t AoAProcDcmpHWA_config_EDMA
(
    HWA_Handle  hwaHandle,
    DPU_AoAProcDcmpHWA_HW_Resources        *res,
    DPU_AoAProcDcmpHWA_StaticConfig  *DPParams,
    uint32_t           srcIn2DFFTBuffAddr,
    uint32_t           *dstIn2DFFTBuffAddr,
    uint32_t           *srcOut2DFFTBuffAddr,
    uint32_t           *dstOut2DFFTBuffAddr,

    uint32_t           *srcIn3DFFTBuffAddr,
    uint32_t           *dstIn3DFFTBuffAddr,
    uint32_t           *srcOut3DFFTBuffAddr,
    uint32_t           *dstOut3DFFTBuffAddr,
    uint8_t            extMaxVelEnabled

)
{
    volatile bool       edmaTransComplete = false ;
    uint16_t numBlocks;
    uint32_t pingPongIdx;
    uint16_t numIterations = 1; //EDMA will be reloaded per target
    uint16_t numHypotheses;
	
    EDMA_channelConfig_t    config;
    EDMA_paramConfig_t      paramCfg;
    int32_t errorCode = EDMA_NO_ERROR;
    bool isEventTriggered;
    HWA_SrcDMAConfig        dmaConfig;
    EDMA_Handle             handle = res->edmaHandle;


    if(extMaxVelEnabled)
    {
        numHypotheses = DPParams->numTxAntennas;
    }
    else
    {
        numHypotheses = 1;
    }

    /* Set common fields for Params configuration */
    paramCfg.transferCompletionCallbackFxn = NULL;
    paramCfg.transferCompletionCallbackFxnArg = 0;
    paramCfg.paramSetConfig.linkAddress = EDMA_NULL_LINK_ADDRESS;
    paramCfg.paramSetConfig.sourceAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;
    paramCfg.paramSetConfig.destinationAddressingMode = (uint8_t) EDMA3_ADDRESSING_MODE_LINEAR;
    paramCfg.paramSetConfig.fifoWidth = (uint8_t) EDMA3_FIFO_WIDTH_8BIT;
    paramCfg.paramSetConfig.isStaticSet = false;
    paramCfg.paramSetConfig.isEarlyCompletion = false;
    paramCfg.transferCompletionCallbackFxn = NULL;
    paramCfg.transferCompletionCallbackFxnArg = 0;

    /* Set a common configuration for physical input/output channels - initially set to dummy channel */
    config.channelType = (uint8_t)EDMA3_CHANNEL_TYPE_DMA;
    config.transferCompletionCallbackFxn = NULL;
    config.transferCompletionCallbackFxnArg = 0;
    memset(&config.paramSetConfig, 0, sizeof(EDMA_paramSetConfig_t));
    config.paramSetConfig.aCount = 1;
    config.paramSetConfig.bCount = 0;
    config.paramSetConfig.cCount = 0;


    numBlocks = (DPParams->numVirtualAntAzim + DPParams->numVirtualAntElev) *DPParams->compressCfg.numRangeBinsPerBlock *DPParams->numDopplerChirps;
    for (pingPongIdx = 0; pingPongIdx < 2; pingPongIdx++)
    {

        /************************************************************/
        /* Physical Input channel is initially set to dummy channel */
        /************************************************************/
        config.channelId = res->edmaHwaExt[pingPongIdx].chIn.channel;
        config.paramId = res->edmaHwaExt[pingPongIdx].chIn.channel;
        config.eventQueueId = res->edmaHwaExt[pingPongIdx].eventQueue;
        isEventTriggered = false;
        if ((errorCode = EDMA_configChannel(handle, &config, isEventTriggered)) != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /************************************************************/
        /* Physical Output channel is initially set to dummy channel */
        /************************************************************/
        config.channelId = res->edmaHwaExt[pingPongIdx].chOut.channel;
        config.paramId = res->edmaHwaExt[pingPongIdx].chOut.channel;
        config.eventQueueId = res->edmaHwaExt[pingPongIdx].eventQueue;
        isEventTriggered = true;
        if ((errorCode = EDMA_configChannel(handle, &config, isEventTriggered)) != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /* Set EDMA Transfer type to AB type */
        paramCfg.paramSetConfig.transferType = (uint8_t)EDMA3_SYNC_AB;
        /************************/
        /* Param set: 2D-FFT In */
        /************************/
        paramCfg.paramSetConfig.sourceAddress = SOC_translateAddress(srcIn2DFFTBuffAddr,
                                              SOC_TranslateAddr_Dir_TO_EDMA,
                                              NULL);
        paramCfg.paramSetConfig.destinationAddress = SOC_translateAddress(dstIn2DFFTBuffAddr[pingPongIdx],
                                                   SOC_TranslateAddr_Dir_TO_EDMA,
                                                   NULL);
        paramCfg.paramSetConfig.aCount = (sizeof(cmplx16ImRe_t)*numBlocks * DPParams->compressCfg.ratio) >> HWA_CMP_RATIO_BW;
        paramCfg.paramSetConfig.bCount = 1;
        paramCfg.paramSetConfig.cCount = numIterations;
        paramCfg.paramSetConfig.bCountReload = 0;

        paramCfg.paramSetConfig.sourceBindex = 0;
        paramCfg.paramSetConfig.destinationBindex = 0;

        paramCfg.paramSetConfig.sourceCindex = 0;
        paramCfg.paramSetConfig.destinationCindex = 0;

        paramCfg.paramSetConfig.isFinalTransferInterruptEnabled = false;
        paramCfg.paramSetConfig.isIntermediateTransferInterruptEnabled = false;
        paramCfg.paramSetConfig.transferCompletionCode = res->edmaHwaExt[pingPongIdx].chIn.channel;
        paramCfg.paramSetConfig.isFinalChainingEnabled = true;
        paramCfg.paramSetConfig.isIntermediateChainingEnabled = true;

        if ((errorCode = EDMA_configParamSet(handle, res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramIn,
                                             &paramCfg)) != EDMA_NO_ERROR)
        {
            goto exit;
        }
        /* Set EDMA Transfer type to A type */
        paramCfg.paramSetConfig.transferType = (uint8_t)EDMA3_SYNC_A;

        /**********************************/
        /* Param set: 2D-FFT Signature In */
        /**********************************/
        HWA_getDMAconfig(hwaHandle,
                         res->hwaCfg.paramSetStartIdx + (pingPongIdx * 2),
                         &dmaConfig);

        paramCfg.paramSetConfig.sourceAddress = SOC_translateAddress(dmaConfig.srcAddr,
                                              SOC_TranslateAddr_Dir_TO_EDMA,
                                              NULL);
        paramCfg.paramSetConfig.destinationAddress = SOC_translateAddress(dmaConfig.destAddr,
                                                   SOC_TranslateAddr_Dir_TO_EDMA,
                                                   NULL);
        paramCfg.paramSetConfig.aCount = dmaConfig.aCnt;
        paramCfg.paramSetConfig.bCount = dmaConfig.bCnt;
        paramCfg.paramSetConfig.cCount = dmaConfig.cCnt;
        paramCfg.paramSetConfig.bCountReload = dmaConfig.bCnt;

        paramCfg.paramSetConfig.sourceBindex = 0;
        paramCfg.paramSetConfig.destinationBindex = 0;

        paramCfg.paramSetConfig.sourceCindex = 0;
        paramCfg.paramSetConfig.destinationCindex = 0;

        paramCfg.paramSetConfig.isFinalTransferInterruptEnabled = false;
        paramCfg.paramSetConfig.isIntermediateTransferInterruptEnabled = false;

        paramCfg.paramSetConfig.transferCompletionCode = 0;
        paramCfg.paramSetConfig.isFinalChainingEnabled = false;
        paramCfg.paramSetConfig.isIntermediateChainingEnabled = false;

        if ((errorCode = EDMA_configParamSet(handle, res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramInSignature,
                                             &paramCfg)) != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /************************/
        /* Param set: 3D-FFT In */
        /************************/
        paramCfg.paramSetConfig.sourceAddress = SOC_translateAddress(srcIn3DFFTBuffAddr[pingPongIdx],
                                              SOC_TranslateAddr_Dir_TO_EDMA,
                                              NULL);

        paramCfg.paramSetConfig.destinationAddress = SOC_translateAddress(dstIn3DFFTBuffAddr[pingPongIdx],
                                                   SOC_TranslateAddr_Dir_TO_EDMA,
                                                   NULL);

        paramCfg.paramSetConfig.aCount = numHypotheses * (DPParams->numVirtualAntAzim + DPParams->numVirtualAntElev) * sizeof(cmplx16ImRe_t);
        paramCfg.paramSetConfig.bCount = 1;
        paramCfg.paramSetConfig.cCount = 1;
        paramCfg.paramSetConfig.bCountReload = 0;

        paramCfg.paramSetConfig.sourceBindex = 0;
        paramCfg.paramSetConfig.destinationBindex = 0;

        paramCfg.paramSetConfig.sourceCindex = 0;
        paramCfg.paramSetConfig.destinationCindex = 0;

        paramCfg.paramSetConfig.isFinalTransferInterruptEnabled = false;
        paramCfg.paramSetConfig.isIntermediateTransferInterruptEnabled = false;

        paramCfg.paramSetConfig.transferCompletionCode = res->edmaHwaExt[pingPongIdx].chIn.channel;
        paramCfg.paramSetConfig.isFinalChainingEnabled = true;
        paramCfg.paramSetConfig.isIntermediateChainingEnabled = true;

        if ((errorCode = EDMA_configParamSet(handle, res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramIn,
                                             &paramCfg)) != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /**********************************/
        /* Param set: 3D-FFT Signature In */
        /**********************************/
        uint32_t num3DFFTParamSets = ((DPParams->numVirtualAntElev>0)*1 + 2);
        HWA_getDMAconfig(hwaHandle,
                         res->hwaCfg.paramSetStartIdx + (2 * 2) + (pingPongIdx * num3DFFTParamSets),
                         &dmaConfig);


        paramCfg.paramSetConfig.sourceAddress = SOC_translateAddress(dmaConfig.srcAddr,
                                              SOC_TranslateAddr_Dir_TO_EDMA,
                                              NULL);
        paramCfg.paramSetConfig.destinationAddress = SOC_translateAddress(dmaConfig.destAddr,
                                                   SOC_TranslateAddr_Dir_TO_EDMA,
                                                   NULL);
        paramCfg.paramSetConfig.aCount = dmaConfig.aCnt;
        paramCfg.paramSetConfig.bCount = dmaConfig.bCnt;
        paramCfg.paramSetConfig.cCount = dmaConfig.cCnt;
        paramCfg.paramSetConfig.bCountReload = dmaConfig.bCnt;

        paramCfg.paramSetConfig.sourceBindex = 0;
        paramCfg.paramSetConfig.destinationBindex = 0;

        paramCfg.paramSetConfig.sourceCindex = 0;
        paramCfg.paramSetConfig.destinationCindex = 0;

        paramCfg.paramSetConfig.isFinalTransferInterruptEnabled = false;
        paramCfg.paramSetConfig.isIntermediateTransferInterruptEnabled = false;

        paramCfg.paramSetConfig.transferCompletionCode = 0;
        paramCfg.paramSetConfig.isFinalChainingEnabled = false;
        paramCfg.paramSetConfig.isIntermediateChainingEnabled = false;

        if ((errorCode = EDMA_configParamSet(handle,
                             res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramInSignature,
                             &paramCfg)) != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /**********************************************/
        /* Link Input physical channel and param sets */
        /**********************************************/
        if ((errorCode = EDMA_linkParamSets(handle,
                            (uint16_t) res->edmaHwaExt[pingPongIdx].chIn.channel,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramIn)) != EDMA_NO_ERROR)
        {
            goto exit;
        }
        if ((errorCode = EDMA_linkParamSets(handle,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramIn,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramInSignature)) != EDMA_NO_ERROR)
        {
            goto exit;
        }
        if ((errorCode = EDMA_linkParamSets(handle,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramInSignature,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramIn)) != EDMA_NO_ERROR)
        {
            goto exit;
        }
        if ((errorCode = EDMA_linkParamSets(handle,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramIn,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramInSignature)) != EDMA_NO_ERROR)
        {
            goto exit;
        }
        if ((errorCode = EDMA_linkParamSets(handle,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramInSignature,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramIn)) != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /********************************/
        /* Bring in the first param set */
        /********************************/
        errorCode = EDMA_startDmaTransfer(handle,
                                       res->edmaHwaExt[pingPongIdx].chIn.channel);
        if (errorCode != 0)
        {
            goto exit;
        }

        /*********************/
        /* 2D-FFT EDMA Out:  */
        /*********************/
        paramCfg.paramSetConfig.sourceAddress = SOC_translateAddress(srcOut2DFFTBuffAddr[pingPongIdx],
                                              SOC_TranslateAddr_Dir_TO_EDMA,
                                              NULL);
        paramCfg.paramSetConfig.destinationAddress = SOC_translateAddress(dstOut2DFFTBuffAddr[pingPongIdx],
                                                   SOC_TranslateAddr_Dir_TO_EDMA,
                                                   NULL);
        paramCfg.paramSetConfig.aCount = (DPParams->numVirtualAntAzim + DPParams->numVirtualAntElev) * sizeof(cmplx16ImRe_t);
        paramCfg.paramSetConfig.bCount = 1;
        paramCfg.paramSetConfig.cCount = 1;
        paramCfg.paramSetConfig.bCountReload = 0;

        paramCfg.paramSetConfig.sourceBindex = 0;
        paramCfg.paramSetConfig.destinationBindex = 0;

        paramCfg.paramSetConfig.sourceCindex = 0;
        paramCfg.paramSetConfig.destinationCindex = 0;

        paramCfg.paramSetConfig.isFinalTransferInterruptEnabled = true;
        paramCfg.paramSetConfig.isIntermediateTransferInterruptEnabled = true;
        paramCfg.paramSetConfig.transferCompletionCode = res->edmaHwaExt[pingPongIdx].chOut.channel;
        paramCfg.paramSetConfig.isFinalChainingEnabled = false;
        paramCfg.paramSetConfig.isIntermediateChainingEnabled = false;

        if ((errorCode = EDMA_configParamSet(handle, res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramOut,
                                             &paramCfg)) != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /*********************/
        /* 3D-FFT EDMA Out:  */
        /*********************/
        paramCfg.paramSetConfig.sourceAddress = SOC_translateAddress(srcOut3DFFTBuffAddr[pingPongIdx],
                                              SOC_TranslateAddr_Dir_TO_EDMA,
                                              NULL);
        paramCfg.paramSetConfig.destinationAddress = SOC_translateAddress(dstOut3DFFTBuffAddr[pingPongIdx],
                                                   SOC_TranslateAddr_Dir_TO_EDMA,
                                                   NULL);
        paramCfg.paramSetConfig.aCount = numHypotheses * DPU_AOAPROCDCMPHWA_NUM_ANGLE_BINS * sizeof(uint16_t);
        paramCfg.paramSetConfig.bCount = 1;
        paramCfg.paramSetConfig.cCount = 1;
        paramCfg.paramSetConfig.bCountReload = 0;

        paramCfg.paramSetConfig.sourceBindex = 0;
        paramCfg.paramSetConfig.destinationBindex = 0;

        paramCfg.paramSetConfig.sourceCindex = 0;
        paramCfg.paramSetConfig.destinationCindex = 0;

        paramCfg.paramSetConfig.isFinalTransferInterruptEnabled = true;
        paramCfg.paramSetConfig.isIntermediateTransferInterruptEnabled = true;
        paramCfg.paramSetConfig.transferCompletionCode = res->edmaHwaExt[pingPongIdx].chOut.channel;
        paramCfg.paramSetConfig.isFinalChainingEnabled = false;
        paramCfg.paramSetConfig.isIntermediateChainingEnabled = false;

        if ((errorCode = EDMA_configParamSet(handle, res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramOut,
                                             &paramCfg)) != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /***********************************************/
        /* Link Output physical channel and param sets */
        /***********************************************/
        if ((errorCode = EDMA_linkParamSets(handle,
                            (uint16_t) res->edmaHwaExt[pingPongIdx].chOut.channel,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramOut)) != EDMA_NO_ERROR)
        {
            goto exit;
        }
        if ((errorCode = EDMA_linkParamSets(handle,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramOut,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramOut)) != EDMA_NO_ERROR)
        {
            goto exit;
        }
        if ((errorCode = EDMA_linkParamSets(handle,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramOut,
                            res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramOut)) != EDMA_NO_ERROR)
        {
            goto exit;
        }

        /********************************/
        /* Bring in the first param set */
        /********************************/
        errorCode = EDMA_startDmaTransfer(handle,
                                       res->edmaHwaExt[pingPongIdx].chOut.channel);
        if (errorCode != 0)
        {
            goto exit;
        }
    }

exit:
    return(errorCode);
}



/**
 *  @b Description
 *  @n
 *      The function configures source address of the input EDMA (based on object's
 *      range index), and source address of the output EDMA (based on object's Doppler index).
 *      The function then triggers input EDMA to transfer data to HWA. This trigger initiates the
 *      first stage of cloud point processing, 2D-FFT.
 *
 *  @param[in] aoaHwaObj    Pointer to AoA DPU internal object
 *  @param[in] detObjIdx    Object index in detected object list
 *  @param[in] pingPongIdx  Data path index, 0-ping, 1-pong
 *
 *  @retval EDMA error code, see EDMA API.
 *
 *  \ingroup    DPU_AOAPROCDCMP_INTERNAL_FUNCTION
 */
#if NUM_RANGE_BINS_PER_COMPRESSED_BLOCK > 1
int32_t AoAProcDcmpHWA_cfgAndTrigger_EDMA_2DFFT(AOAHwaObj *aoaHwaObj,
                                 uint32_t detObjIdx,
                                 uint8_t pingPongIdx)
{
    int32_t retVal;
    DPU_AoAProcDcmpHWA_HW_Resources *res = &aoaHwaObj->res;
    DPU_AoAProcDcmpHWA_StaticConfig *DPParams = &aoaHwaObj->params;
    cmplx16ImRe_t *radarCubeBase = (cmplx16ImRe_t *)res->radarCube.data;
    uint32_t numVirtualAnt = (DPParams->numVirtualAntAzim + DPParams->numVirtualAntElev);
	uint32_t virtAntennaBinSizeBytes = sizeof(cmplx16ImRe_t) * numVirtualAnt;
    uint32_t srcBuffAddr;
	uint32_t compressionRatio  = DPParams->compressCfg.ratio;
    uint32_t numRangeGateSizeSamples = ((DPParams->numDopplerChirps * numVirtualAnt)*compressionRatio) >> HWA_CMP_RATIO_BW;
	uint32_t radarCubeOffset = numRangeGateSizeSamples*res->cfarRngDopSnrList[detObjIdx].rangeIdx;
	/* EDMA In Source address in radar cube */
    srcBuffAddr = (uint32_t) (&radarCubeBase[radarCubeOffset]);
    retVal = EDMA_setSourceAddress(res->edmaHandle,
          res->edmaHwaExt[pingPongIdx].chIn.channel, //NOTE
          SOC_translateAddress(srcBuffAddr, SOC_TranslateAddr_Dir_TO_EDMA, NULL));
    if (retVal != 0)
    {
        goto exit;
    }

    /* EDMA Out Source address in M0 or M1 */
    srcBuffAddr =  aoaHwaObj->edmaSrcOut2DFFTBuffAddr[pingPongIdx] +
                   res->cfarRngDopSnrList[detObjIdx].dopplerIdx *  virtAntennaBinSizeBytes;
    retVal = EDMA_setSourceAddress(res->edmaHandle,
          res->edmaHwaExt[pingPongIdx].chOut.channel,
          SOC_translateAddress(srcBuffAddr, SOC_TranslateAddr_Dir_TO_EDMA, NULL));
    if (retVal != 0)
    {
        goto exit;
    }

    /* Trigger single bin 2D FFT */
    retVal = EDMA_startDmaTransfer(res->edmaHandle,
                                    res->edmaHwaExt[pingPongIdx].chIn.channel);
    if (retVal != 0)
    {
        goto exit;
    }
exit:
    return retVal;
}
#else
int32_t AoAProcDcmpHWA_cfgAndTrigger_EDMA_2DFFT(AOAHwaObj *aoaHwaObj,
                                 uint32_t detObjIdx,
                                 uint8_t pingPongIdx)
{
    int32_t retVal;
	uint32_t srcBuffAddr;
    DPU_AoAProcDcmpHWA_HW_Resources *res = &aoaHwaObj->res;
    DPU_AoAProcDcmpHWA_StaticConfig *DPParams = &aoaHwaObj->params;
    cmplx16ImRe_t *radarCubeBase = (cmplx16ImRe_t *)res->radarCube.data;
    uint32_t numVirtualAnt = (DPParams->numVirtualAntAzim + DPParams->numVirtualAntElev);
	uint32_t virtAntennaBinSizeBytes = sizeof(cmplx16ImRe_t) * numVirtualAnt;
	uint32_t numRangeGateSizeBytes = (DPParams->numDopplerChirps * numVirtualAnt) * sizeof(cmplx16ImRe_t); /* (Decompressed) */
	uint32_t compressionRatio  = DPParams->compressCfg.ratio;
    uint32_t compressedblockSizeSamples = ((DPParams->numDopplerChirps * numVirtualAnt * DPParams->compressCfg.numRangeBinsPerBlock)*compressionRatio) >> HWA_CMP_RATIO_BW;
	uint32_t radarCubeOffset = compressedblockSizeSamples*res->cfarRngDopSnrList[detObjIdx].rangeIdx/DPParams->compressCfg.numRangeBinsPerBlock;
	uint32_t rangeIdx = (res->cfarRngDopSnrList[detObjIdx].rangeIdx % DPParams->compressCfg.numRangeBinsPerBlock);
	uint32_t dopplerIdx = res->cfarRngDopSnrList[detObjIdx].dopplerIdx;
	
	
    /* EDMA In Source address in radar cube */
    srcBuffAddr = (uint32_t) (&radarCubeBase[radarCubeOffset]);
    retVal = EDMA_setSourceAddress(res->edmaHandle,
          res->edmaHwaExt[pingPongIdx].chIn.channel, //NOTE
          SOC_translateAddress(srcBuffAddr, SOC_TranslateAddr_Dir_TO_EDMA, NULL));
    if (retVal != 0)
    {
        goto exit;
    }
	
	
    /* EDMA Out Source address in M0 or M1 */
    srcBuffAddr =  aoaHwaObj->edmaSrcOut2DFFTBuffAddr[pingPongIdx] +
                   (rangeIdx*numRangeGateSizeBytes) +  (dopplerIdx * virtAntennaBinSizeBytes);

    retVal = EDMA_setSourceAddress(res->edmaHandle,
          res->edmaHwaExt[pingPongIdx].chOut.channel,
          SOC_translateAddress(srcBuffAddr, SOC_TranslateAddr_Dir_TO_EDMA, NULL));
    if (retVal != 0)
    {
        goto exit;
    }

    /* Trigger single bin 2D FFT */
    retVal = EDMA_startDmaTransfer(res->edmaHandle,
                                    res->edmaHwaExt[pingPongIdx].chIn.channel);
    if (retVal != 0)
    {
        goto exit;
    }
exit:
    return retVal;
}


/**
 *  @b Description
 *  @n
 *      The function configures source address of the output EDMA (based on object's Doppler index).
 *      The function then triggers input EDMA to transfer data from the HWA. 
 *
 *  @param[in] aoaHwaObj    Pointer to AoA DPU internal object
 *  @param[in] detObjIdx    Object index in detected object list
 *  @param[in] pingPongIdx  Data path index, 0-ping, 1-pong
 *
 *  @retval EDMA error code, see EDMA API.
 *
 *  \ingroup    DPU_AOAPROCDCMP_INTERNAL_FUNCTION
 */
int32_t AoAProcDcmpHWA_Extract2DFFT(AOAHwaObj *aoaHwaObj,
                                 uint32_t detObjIdx,
								 cmplx16ImRe_t * dstBuffAddr,
                                 uint8_t pingPongIdx)
								 
{
	int32_t retVal;
	uint32_t srcBuffAddr;
    
    DPU_AoAProcDcmpHWA_HW_Resources *res = &aoaHwaObj->res;
    DPU_AoAProcDcmpHWA_StaticConfig *DPParams = &aoaHwaObj->params;
    uint32_t numVirtualAnt = (DPParams->numVirtualAntAzim + DPParams->numVirtualAntElev);
	uint32_t virtAntennaBinSizeBytes = numVirtualAnt * sizeof(cmplx16ImRe_t);
    uint32_t numRangeGateSizeBytes = (DPParams->numDopplerChirps * numVirtualAnt) * sizeof(cmplx16ImRe_t);
	uint32_t rangeIdx = (res->cfarRngDopSnrList[detObjIdx].rangeIdx % DPParams->compressCfg.numRangeBinsPerBlock);
	uint32_t dopplerIdx = res->cfarRngDopSnrList[detObjIdx].rangeIdx;
	
	/* EDMA Out Destination address  */
    retVal = EDMA_setDestinationAddress(res->edmaHandle,
          res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramOut, //NOTE
          SOC_translateAddress((uint32_t) dstBuffAddr, SOC_TranslateAddr_Dir_TO_EDMA, NULL));
    if (retVal != 0)
    {
        goto exit;
    }
    /* EDMA Out Source address in M0 or M1 */
    srcBuffAddr =  aoaHwaObj->edmaSrcOut2DFFTBuffAddr[pingPongIdx] +
                   (rangeIdx*numRangeGateSizeBytes) +  (dopplerIdx * virtAntennaBinSizeBytes);
    
	retVal = EDMA_setSourceAddress(res->edmaHandle,
          res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramOut,
          SOC_translateAddress(srcBuffAddr, SOC_TranslateAddr_Dir_TO_EDMA, NULL));
    if (retVal != 0)
    {
        goto exit;
    }

    /* Trigger single bin 2D FFT */
    retVal = EDMA_startDmaTransfer(res->edmaHandle,
                                    res->edmaHwaExt[pingPongIdx].stage[AOAHWA_2DFFT_STAGE].paramOut);
    if (retVal != 0)
    {
        goto exit;
    }
exit:
    return retVal;
}
/**
 *  @b Description
 *  @n
 *      The function triggers input EDMA to transfer data to HWA. This trigger initiates the
 *      second stage of cloud point processing, 3D-FFTs.
 *
 *  @param[in] aoaHwaObj    Pointer to AoA DPU internal object
 *  @param[in] pingPongIdx  Data path index, 0-ping, 1-pong
 *
 *  @retval EDMA error code, see EDMA API.
 *
 *  \ingroup    DPU_AOAPROCDCMP_INTERNAL_FUNCTION
 */
int32_t AoAProcDcmpHWA_trigger_multi_EDMA_3DFFT(AOAHwaObj *aoaHwaObj,
                                 uint8_t pingPongIdx, uint16_t numObj, uint16_t numHypotheses)
{
	int32_t retVal;
	DPU_AoAProcDcmpHWA_HW_Resources *res = &aoaHwaObj->res;
    DPU_AoAProcDcmpHWA_StaticConfig *DPParams = &aoaHwaObj->params;
    uint16_t Acnt, Bcnt; 
	Acnt = numHypotheses * (DPParams->numVirtualAntAzim + DPParams->numVirtualAntElev) * sizeof(cmplx16ImRe_t);
	Bcnt = numObj;
	
	retVal = EDMA_setABcnt(res->edmaHandle,
          res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramIn,
	      Acnt, Bcnt);
		  
	Acnt = numHypotheses * DPU_AOAPROCDCMPHWA_NUM_ANGLE_BINS * sizeof(uint16_t);	  
	retVal = EDMA_setABcnt(res->edmaHandle,
    	  res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramOut,
		  Acnt, Bcnt);
    
    /* Trigger 3D-FFT */
    retVal = EDMA_startDmaTransfer(res->edmaHandle,
                                   res->edmaHwaExt[pingPongIdx].stage[AOAHWA_3DFFT_STAGE].paramIn);

    return retVal;
}
#endif
/**
 *  @b Description
 *  @n
 *      The function triggers input EDMA to transfer data to HWA. This trigger initiates the
 *      second stage of cloud point processing, 3D-FFTs.
 *
 *  @param[in] aoaHwaObj    Pointer to AoA DPU internal object
 *  @param[in] pingPongIdx  Data path index, 0-ping, 1-pong
 *
 *  @retval EDMA error code, see EDMA API.
 *
 *  \ingroup    DPU_AOAPROCDCMP_INTERNAL_FUNCTION
 */
int32_t AoAProcDcmpHWA_trigger_EDMA_3DFFT(AOAHwaObj *aoaHwaObj,
                                 uint8_t pingPongIdx)
{
    int32_t retVal;
    DPU_AoAProcDcmpHWA_HW_Resources *res = &aoaHwaObj->res;

    /* Trigger 3D-FFT */
    retVal = EDMA_startDmaTransfer(res->edmaHandle,
                                   res->edmaHwaExt[pingPongIdx].chIn.channel);

    return retVal;
}


/**
 *   @b Description
 *   @n
 *      The function configures HWA Param sets for 2D-FFT and 3D-FFT calculation.
 *      Processing is organized in a ping pong manner, to run in parallel with
 *      EDMA in/out transfers. THe PARAM sets are split in two groups, one for ping
 *      path and the other for pong path. Processing in each path is divided in
 *      two stages. In the first stage 2D-FFT is calculated, and in the second stage
 *      3D-FFT is calculated. One HWA loop iteration processes 2 cloud points.
 *      (If the number of cloud-points is odd, a dummy one is appended at the end
 *      of the list of detected points.) One loop runs in the following order: ping
 *      path 2D-FFT, pong path 2D-FFT, ping path 3D-FFTs. pong path 3D-FFTs. The
 *      number of HWA Param sets for 2D-FFT calculation is equal to number of Tx
 *      antennas in TDM MIMO scheme. The number of Param sets for 3D-FFT calculation
 *      depends on whether the elevation Tx antenna is enabled or disabled. If the
 *      Tx elevation antenna is disabled, the number of param sets is equal to 2, the
 *      first one calculates 3D-FFT complex output samples, the second one calculates
 *      magnitude square of the complex samples. If the Tx elevation antenna is enabled,
 *      the number of param sets is equal to 3, the additional param set calculates 3D-FFT
 *      complex samples of the elevation antennas.
 *
 *   @param[in] hwaHandle           HWA driver handle
 *   @param[in] res                 Hardware resources
 *   @param[in] DPParams            Static parameters
 *   @param[in] hwaMemBankAddr      Array of 4 HWA bank addresses
 *   @param[in] extMaxVelEnabled    0 - Extended maximum velocity disabled, 1 - enabled
 *   @param[in] numAzimuthBins      Size of 3D-FFT (angle FFT)
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error code  - <0
 *
 *   \ingroup    DPU_AOAPROCDCMP_INTERNAL_FUNCTION
 *
 */
static int32_t HWAutil_configHWA_extendedVelocityScheme
(
    HWA_Handle      hwaHandle,
    DPU_AoAProcDcmpHWA_HW_Resources *res,
    DPU_AoAProcDcmpHWA_StaticConfig *DPParams,
    uint32_t        *hwaMemBankAddr,
    uint8_t         extMaxVelEnabled,
    uint32_t        numAzimuthBins
)
{
    HWA_InterruptConfig     paramISRConfig;
    int32_t retVal = 0;
    uint8_t         dmaTriggerDest;
    uint8_t         numHypotheses;
	uint16_t compressionRatio = DPParams->compressCfg.ratio;
	int32_t aoaParamSetIdx;
		
	uint32_t pingPongidx;
	uint8_t         paramSetStartIdx;    
	HWA_ParamConfig hwaFFTParamCfg, hwaDcmpParamCfg, hwa3DFFTParamCfg;

    if(extMaxVelEnabled)
    {
        numHypotheses = DPParams->numTxAntennas;
    }
    else
    {
        numHypotheses = 1;
    }

    paramSetStartIdx = res->hwaCfg.paramSetStartIdx;
    uint32_t        windowOffset = res->hwaCfg.winRamOffset;
    uint8_t         winSym = res->hwaCfg.winSym;
    uint32_t        numDopplerChirps = DPParams->numDopplerChirps;
    uint32_t        numDopplerBins = DPParams->numDopplerBins;
    uint8_t         numRxAnt = DPParams->numRxAntennas;
    uint8_t         numVirtualAntAzim = DPParams->numVirtualAntAzim;
    uint8_t         numVirtualAntElev = DPParams->numVirtualAntElev;
	uint16_t         hwaMemAzimSource[2];
	uint16_t         hwaMemAzimDest[2];
	
    uint16_t        hwaMem[4];

	uint16_t numSamplesPerBlockOut = DPParams->numRxAntennas * DPParams->compressCfg.numRangeBinsPerBlock;
	uint16_t numSamplesPerBlockIn = (uint16_t) ((numSamplesPerBlockOut * (uint32_t)compressionRatio) >> HWA_CMP_RATIO_BW);
	uint16_t numBlocks = DPParams->numDopplerChirps*DPParams->numTxAntennas;
	  

    hwaMem[0] = ADDR_TRANSLATE_CPU_TO_HWA(hwaMemBankAddr[0]);
    hwaMem[1] = ADDR_TRANSLATE_CPU_TO_HWA(hwaMemBankAddr[1]);
    hwaMem[2] = ADDR_TRANSLATE_CPU_TO_HWA(hwaMemBankAddr[2]);
    hwaMem[3] = ADDR_TRANSLATE_CPU_TO_HWA(hwaMemBankAddr[3]);
    hwaMemAzimSource[0] = ADDR_TRANSLATE_CPU_TO_HWA(hwaMemBankAddr[0]);
    hwaMemAzimSource[1] = ADDR_TRANSLATE_CPU_TO_HWA(hwaMemBankAddr[1]);
    hwaMemAzimDest[0] = ADDR_TRANSLATE_CPU_TO_HWA(hwaMemBankAddr[2]);
    hwaMemAzimDest[1] = ADDR_TRANSLATE_CPU_TO_HWA(hwaMemBankAddr[3]);



    /* Disable the HWA */
    retVal = HWA_enable(hwaHandle, 0);
    if (retVal != 0)
    {
        goto exit;
    }

    /**********************************************************/
    /******************* Configure 2D-FFT *********************/
    /**********************************************************/
    aoaParamSetIdx = paramSetStartIdx;
	pingPongidx = 0;

	for (pingPongidx = 0; pingPongidx < 2; pingPongidx++)
    {
		memset( (void*) &hwaDcmpParamCfg, 0, sizeof(HWA_ParamConfig));
		memset( (void*) &hwaFFTParamCfg, 0, sizeof(HWA_ParamConfig));
		if (pingPongidx == 0)
		{
			linear_50p_DcmpCfg(&hwaDcmpParamCfg, 
										HWA_TRIG_MODE_DMA,  
										aoaParamSetIdx, 
										hwaMem[0], 
										hwaMem[2], 
										numSamplesPerBlockIn, 
										numSamplesPerBlockOut, 
										numBlocks);
										
			intrleavdInp_interleavdOutput_FFTCfg(&hwaFFTParamCfg, 
								HWA_TRIG_MODE_IMMEDIATE, 
								0, 
								hwaMem[2], 
								hwaMem[0],
								numDopplerChirps, 
								numRxAnt*DPParams->compressCfg.numRangeBinsPerBlock, 
								numDopplerBins, 
								numVirtualAntAzim + numVirtualAntElev,
								windowOffset, 
								winSym);
		}
		else
		{
			linear_50p_DcmpCfg(&hwaDcmpParamCfg, 
								HWA_TRIG_MODE_DMA,  
								aoaParamSetIdx, 
								hwaMem[1], 
								hwaMem[3], 
								numSamplesPerBlockIn, 
								numSamplesPerBlockOut, 
								numBlocks);
								
			intrleavdInp_interleavdOutput_FFTCfg(&hwaFFTParamCfg, 
									HWA_TRIG_MODE_IMMEDIATE, 
									0, 
									hwaMem[3], 
									hwaMem[1],
									numDopplerChirps, 
									numRxAnt * DPParams->compressCfg.numRangeBinsPerBlock, 
									numDopplerBins, 
									numVirtualAntAzim + numVirtualAntElev,
									windowOffset, 
									winSym);
		}
		/* Ping configuration. */
		retVal = HWA_configParamSet(hwaHandle, (uint8_t) aoaParamSetIdx, &hwaDcmpParamCfg, NULL);
		if (retVal != 0)
		{
			goto exit;
		}
		retVal = HWA_disableParamSetInterrupt(hwaHandle,
									  aoaParamSetIdx,
									  HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
		if (retVal != 0)
		{
			goto exit;
		}

		aoaParamSetIdx++;
		retVal = HWA_configParamSet(hwaHandle, (uint8_t) aoaParamSetIdx, &hwaFFTParamCfg, NULL);
		if (retVal != 0)
		{
			goto exit;
		}
		
		/* Disable DMA/interrupt hookup to all except the last one */
		retVal = HWA_disableParamSetInterrupt(hwaHandle,
											  aoaParamSetIdx,
											  HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
		if (retVal != 0)
		{
			goto exit;
		}
		/* Last param set: enable the DMA hookup to this paramset so that data gets copied out */
		retVal = HWA_getDMAChanIndex(hwaHandle,
									 res->edmaHwaExt[pingPongidx].chOut.channel,
									 &dmaTriggerDest);
		if (retVal != 0)
		{
			goto exit;
		}
		paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
		paramISRConfig.dma.dstChannel = dmaTriggerDest;  /* EDMA channel to trigger to copy the data out */
		paramISRConfig.cpu.callbackArg = NULL;
		retVal = HWA_enableParamSetInterrupt(hwaHandle,
											 aoaParamSetIdx, //Note that aoaParamSetIdx + 1 is the last paramSet.
											 &paramISRConfig);
		if (retVal != 0)
		{
			goto exit;
		}
		
		aoaParamSetIdx++;
	}
	
		
	
    /**********************************************************/
    /******************* Configure 3D-FFT *********************/
    /**********************************************************/
    for (pingPongidx =0; pingPongidx < 2; pingPongidx++)
    {
        /************** Param set computes the complex values of the azimuth-FFT **********/
        memset( (void*) &hwa3DFFTParamCfg, 0, sizeof(hwa3DFFTParamCfg));
        hwa3DFFTParamCfg.dmaTriggerSrc = aoaParamSetIdx;
        hwa3DFFTParamCfg.triggerMode = HWA_TRIG_MODE_DMA;

        hwa3DFFTParamCfg.source.srcAddr = (uint16_t) hwaMemAzimSource[pingPongidx];
        hwa3DFFTParamCfg.source.srcAcnt = numVirtualAntAzim-1;
        hwa3DFFTParamCfg.source.srcAIdx = sizeof(cmplx16ImRe_t);
        hwa3DFFTParamCfg.source.srcBIdx = (numVirtualAntAzim + numVirtualAntElev) * sizeof(cmplx16ImRe_t);
        hwa3DFFTParamCfg.source.srcBcnt = numHypotheses-1;
        hwa3DFFTParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
        hwa3DFFTParamCfg.source.srcScale = 8;

        hwa3DFFTParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
        hwa3DFFTParamCfg.accelModeArgs.fftMode.fftEn = 1;
        hwa3DFFTParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(numAzimuthBins);//assumes power of 2;
        hwa3DFFTParamCfg.accelModeArgs.fftMode.windowEn = 0;
        hwa3DFFTParamCfg.accelModeArgs.fftMode.butterflyScaling = 0; //no scaling

        hwa3DFFTParamCfg.dest.dstAddr =  (uint16_t) hwaMemAzimDest[pingPongidx];
        hwa3DFFTParamCfg.dest.dstAcnt = numAzimuthBins-1;
        hwa3DFFTParamCfg.dest.dstAIdx = sizeof(cmplx16ImRe_t);
        hwa3DFFTParamCfg.dest.dstBIdx = numAzimuthBins * sizeof(cmplx16ImRe_t);
        hwa3DFFTParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
        hwa3DFFTParamCfg.dest.dstScale = 3;

        retVal = HWA_configParamSet(hwaHandle, aoaParamSetIdx, &hwa3DFFTParamCfg, NULL);
        if (retVal != 0)
        {
          goto exit;
        }
        retVal = HWA_disableParamSetInterrupt(hwaHandle,
                                           aoaParamSetIdx,
                                           HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
        if (retVal != 0)
        {
          goto exit;
        }

        aoaParamSetIdx++;

        if (numVirtualAntElev > 0)
        {
            /************** Param set computes the complex values of the elevation-FFT **********/
            memset( (void*) &hwa3DFFTParamCfg, 0, sizeof(hwa3DFFTParamCfg));
            hwa3DFFTParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;

            hwa3DFFTParamCfg.source.srcAddr = (uint16_t) (hwaMemAzimSource[pingPongidx] +
                                                     (numVirtualAntAzim * sizeof(cmplx16ImRe_t)));
            hwa3DFFTParamCfg.source.srcAcnt = numVirtualAntElev-1;
            hwa3DFFTParamCfg.source.srcAIdx = sizeof(cmplx16ImRe_t);
            hwa3DFFTParamCfg.source.srcBIdx = (numVirtualAntAzim + numVirtualAntElev) * sizeof(cmplx16ImRe_t);
            hwa3DFFTParamCfg.source.srcBcnt = numHypotheses-1;
            hwa3DFFTParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
            hwa3DFFTParamCfg.source.srcScale = 8;


            hwa3DFFTParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
            hwa3DFFTParamCfg.accelModeArgs.fftMode.fftEn = 1;
            hwa3DFFTParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(numAzimuthBins);//assumes power of 2;
            hwa3DFFTParamCfg.accelModeArgs.fftMode.windowEn = 0;
            hwa3DFFTParamCfg.accelModeArgs.fftMode.butterflyScaling = 0; //no scaling

            hwa3DFFTParamCfg.dest.dstAddr =  (uint16_t) (hwaMemAzimDest[pingPongidx] +
                                                    (numHypotheses * numAzimuthBins * sizeof(cmplx16ImRe_t)));
            hwa3DFFTParamCfg.dest.dstAcnt = numAzimuthBins-1;
            hwa3DFFTParamCfg.dest.dstAIdx = sizeof(cmplx16ImRe_t);
            hwa3DFFTParamCfg.dest.dstBIdx = numAzimuthBins * sizeof(cmplx16ImRe_t);
            hwa3DFFTParamCfg.dest.dstSign = HWA_SAMPLES_SIGNED;
            hwa3DFFTParamCfg.dest.dstScale = 3;

            retVal = HWA_configParamSet(hwaHandle, aoaParamSetIdx, &hwa3DFFTParamCfg, NULL);
            if (retVal != 0)
            {
                goto exit;
            }
            retVal = HWA_disableParamSetInterrupt(hwaHandle,
                                               aoaParamSetIdx,
                                               HWA_PARAMDONE_INTERRUPT_TYPE_DMA | HWA_PARAMDONE_INTERRUPT_TYPE_CPU);
            if (retVal != 0)
            {
              goto exit;
            }

            aoaParamSetIdx++;
        }
        //This calculates magnitude
        /************** Param set computes magnitude of input complex values of the azimuth-FFT **********/
        memset( (void*) &hwa3DFFTParamCfg, 0, sizeof(hwa3DFFTParamCfg));
        hwa3DFFTParamCfg.triggerMode = HWA_TRIG_MODE_IMMEDIATE;

        hwa3DFFTParamCfg.source.srcAddr = (uint16_t) hwaMemAzimDest[pingPongidx];
        hwa3DFFTParamCfg.source.srcAcnt = numAzimuthBins - 1;
        hwa3DFFTParamCfg.source.srcAIdx = sizeof(cmplx16ImRe_t);
        hwa3DFFTParamCfg.source.srcBIdx = numAzimuthBins * sizeof(cmplx16ImRe_t);
        hwa3DFFTParamCfg.source.srcBcnt = numHypotheses-1;
        hwa3DFFTParamCfg.source.srcSign = HWA_SAMPLES_SIGNED;
        hwa3DFFTParamCfg.source.srcScale = 8;

        hwa3DFFTParamCfg.complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
        hwa3DFFTParamCfg.accelModeArgs.fftMode.fftEn = 0;
        hwa3DFFTParamCfg.accelModeArgs.fftMode.fftSize = mathUtils_ceilLog2(numAzimuthBins);//Ignored
        hwa3DFFTParamCfg.accelModeArgs.fftMode.windowEn = 0;
        hwa3DFFTParamCfg.accelModeArgs.fftMode.butterflyScaling = 0; //no scaling
        hwa3DFFTParamCfg.accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_ONLY_ENABLED;

        hwa3DFFTParamCfg.dest.dstAddr =  (uint16_t) hwaMemAzimSource[pingPongidx] + DPU_AOAPROCDCMPHWA_3DFFT_MAG_SQUARE_ADDRSS_OFFSET;
        hwa3DFFTParamCfg.dest.dstAcnt = numAzimuthBins-1;
        hwa3DFFTParamCfg.dest.dstAIdx = sizeof(uint16_t);
        hwa3DFFTParamCfg.dest.dstBIdx = numAzimuthBins * sizeof(uint16_t);
        hwa3DFFTParamCfg.dest.dstSign = HWA_SAMPLES_UNSIGNED;
        hwa3DFFTParamCfg.dest.dstScale = 0;
        hwa3DFFTParamCfg.dest.dstRealComplex = HWA_SAMPLES_FORMAT_REAL;

        retVal = HWA_configParamSet(hwaHandle, aoaParamSetIdx, &hwa3DFFTParamCfg, NULL);
        if (retVal != 0)
        {
          goto exit;
        }

        /* Enable the DMA hookup to EDMA */
        retVal = HWA_getDMAChanIndex(hwaHandle,
                                     res->edmaHwaExt[pingPongidx].chOut.channel,
                                     &dmaTriggerDest);
        if (retVal != 0)
        {
             goto exit;
        }
        paramISRConfig.interruptTypeFlag = HWA_PARAMDONE_INTERRUPT_TYPE_DMA;
        paramISRConfig.dma.dstChannel = dmaTriggerDest;  /* EDMA channel to trigger to copy the data out */
        paramISRConfig.cpu.callbackArg = NULL;
        retVal = HWA_enableParamSetInterrupt(hwaHandle,
                                             aoaParamSetIdx,
                                             &paramISRConfig);
        if (retVal != 0)
        {
             goto exit;
        }
        aoaParamSetIdx++;

    }
exit:
    return (retVal);
}


/**
 *  @b Description
 *  @n
 *      Configures HWA common register. It sets the Params start index and
 *      number of iteration loops.
 *
 *   @param[in] hwaHandle               HWA driver handle
 *   @param[in] numLoops                Number of HWA loops
 *   @param[in] paramStartIdx           Param start index
 *   @param[in] numParams               Number of params
 *   @param[in] compressionRatio        compressionRatio ( with 
 * 						                HWA_CMP_RATIO_BW fraction bits)
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error code  - <0
 *
 *  \ingroup    DPU_AOAPROCDCMP_INTERNAL_FUNCTION
 *
 */
static int32_t AoAProcDcmp_HWAutil_configCommon (HWA_Handle hwaHandle,
                                                uint16_t numLoops,
                                                uint16_t paramStartIdx,
                                                uint16_t numParams,
												uint16_t compressionRatio)
{
    int32_t errCode = 0;
    HWA_CommonConfig    hwaCommonConfig;
	/* Disable the HWA */
     errCode = HWA_enable(hwaHandle, 0);
     if (errCode != 0)
     {
         goto exit;
     }
    /***********************/
    /* HWA COMMON CONFIG   */
    /***********************/
    /* Config Common Registers */
    hwaCommonConfig.configMask =
        HWA_COMMONCONFIG_MASK_NUMLOOPS |
        HWA_COMMONCONFIG_MASK_PARAMSTARTIDX |
        HWA_COMMONCONFIG_MASK_PARAMSTOPIDX |
        HWA_COMMONCONFIG_MASK_FFT1DENABLE |
        HWA_COMMONCONFIG_MASK_INTERFERENCETHRESHOLD |
		HWA_COMMONCONFIG_MASK_EGECOMRESS_KPARAM;;

    hwaCommonConfig.numLoops = numLoops;
    hwaCommonConfig.paramStartIdx = paramStartIdx;
    hwaCommonConfig.paramStopIdx = paramStartIdx + (numParams-1);

    hwaCommonConfig.fftConfig.fft1DEnable = HWA_FEATURE_BIT_DISABLE;
    hwaCommonConfig.fftConfig.interferenceThreshold = 0xFFFFFF;

	cfgEGEParamListAoaProc(&hwaCommonConfig.compressMode.EGEKparam[0], compressionRatio,HWA_SAMPLES_WIDTH_16BIT);
    
    errCode = HWA_configCommon(hwaHandle, &hwaCommonConfig);
exit:
    return (errCode);
}

/**
 *  @b Description
 *  @n
 *      Wait for angle estimation to complete. This is a blocking function.
 *
 *  @param[in] semaHandle  Semaphore handle
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error code  - <0
 *
 *  \ingroup    DPU_AOAPROCDCMP_INTERNAL_FUNCTION
 *
 *
 */
int32_t AoAProcDcmpHWA_dataPathWait2DFFTDetObj(SemaphoreP_Handle semaHandle)
{
    return((int32_t) SemaphoreP_pend(semaHandle, SemaphoreP_WAIT_FOREVER));
}

/**
 *  @b Description
 *  @n
 *      This function is called per object and it calculates its x/y/z coordinates
 *      based on Azimuth FFT output. It stores the coordinates
 *      to the output list of type @ref DPIF_PointCloudCartesian_t and it also
 *      calculates object's SNR and the noise level and writes into the side
 *      information list of type @ref DPIF_PointCloudSideInfo_t
 *
 *  @param[in] aoaHwaObj Pointer to AoA DPU internal object
 *
 *  @param[in] pingPongIdx ping/pong index
 *
 *  @param[in] objInIdx Index of the current object in CFAR output list @ref DPIF_CFARDetList_t
 *
 *  @param[in] objOutIdx Index of the current object in the output lists @ref DPIF_PointCloudCartesian_t and @ref DPIF_PointCloudSideInfo_t
 *
 *  @param[in] maxIdx Peak position in the FFT output of azimuth antennas
 *
 *  @param[in] dopplerIdx Doppler index of detected object
 *
 *  @param[in] hypothesisIdx Hypothesis index related to extended maximum velocity feature.
 *              (If the feature is disabled, hypothesisIdx is passed as zero)
 *
 *  @retval objOutIdx Index for the next object in the output list
 *
 *
 *  \ingroup    DPU_AOAPROCDCMP_INTERNAL_FUNCTION
 *
 */
uint32_t AoAProcDcmpHWA_XYZestimation
(
    AOAHwaObj   *aoaHwaObj,
    uint32_t    pingPongIdx,
    uint32_t    objInIdx,
    uint32_t    objOutIdx,
    uint32_t    maxIdx,
    int32_t     dopplerIdx,
    uint32_t    hypothesisIdx,
	cmplx16ImRe_t *azimFFTPtrBase
)
{
    int32_t         sMaxIdx;
    float           temp;
    float           Wx, Wz;
    float           range;
    float           limitScale;
    float           x, y, z;
    float           peakAzimRe, peakAzimIm, peakElevRe, peakElevIm;
    uint32_t numAngleBins = DPU_AOAPROCDCMPHWA_NUM_ANGLE_BINS;
    DPU_AoAProcDcmpHWA_HW_Resources            *res = &aoaHwaObj->res;
    DPIF_CFARDetList        *objIn = res->cfarRngDopSnrList;
    DPIF_PointCloudCartesian *objOut = res->detObjOut;
    DPIF_PointCloudSideInfo *objOutSideInfo = res->detObjOutSideInfo;

    DPU_AoAProcDcmpHWA_StaticConfig *params = &aoaHwaObj->params;
    //uint16_t numDopplerBins = params->numDopplerBins;

    cmplx16ImRe_t *azimFFTPtr = &azimFFTPtrBase[hypothesisIdx*numAngleBins];
 
    uint32_t numHypotheses;

    if (aoaHwaObj->dynLocalCfg.extMaxVelCfg.enabled)
    {
        numHypotheses = aoaHwaObj->params.numTxAntennas;
    }
    else
    {
        numHypotheses = 1;
    }


    range = ((float)objIn[objInIdx].rangeIdx) * params->rangeStep;

    /* Compensate for range bias */
    range -= aoaHwaObj->dynLocalCfg.compRxChanCfg.rangeBias;
    if (range < 0)
    {
        range = 0;
    }

    if(maxIdx > (numAngleBins/2 -1))
    {
        sMaxIdx = maxIdx - numAngleBins;
    }
    else
    {
        sMaxIdx = maxIdx;
    }

    Wx = 2 * (float) sMaxIdx / numAngleBins;
    /* Check if it is within configured field of view */

    x = range * Wx;

    if (params->numVirtualAntElev > 0)
    {
        peakAzimIm = (float) azimFFTPtr[maxIdx].imag;
        peakAzimRe = (float) azimFFTPtr[maxIdx].real;
        peakElevIm = (float) azimFFTPtr[maxIdx + numHypotheses*numAngleBins].imag;
        peakElevRe = (float) azimFFTPtr[maxIdx + numHypotheses*numAngleBins].real;

        Wz = atan2f(peakAzimIm * peakElevRe - peakAzimRe * peakElevIm,
                   peakAzimRe * peakElevRe + peakAzimIm * peakElevIm)/PI_ + (2 * Wx);

        if (Wz > 1)
        {
            Wz = Wz - 2;
        }
        else if (Wz < -1)
        {
            Wz = Wz + 2;
        }
        /* Check if it is within configured field of view */
        if((Wz < aoaHwaObj->dynLocalCfg.fovAoaLocalCfg.minElevationSineVal) || (Wz > aoaHwaObj->dynLocalCfg.fovAoaLocalCfg.maxElevationSineVal))
        {
            goto exit;
        }

        z = range * Wz;
        /*record wz for debugging/testing*/
        res->detObjElevationAngle[objOutIdx] = Wz;
        limitScale = sqrt(1 - Wz*Wz);
    }
    else
    {
        z = 0;
        limitScale = 1;
    }

    if((Wx < (limitScale * aoaHwaObj->dynLocalCfg.fovAoaLocalCfg.minAzimuthSineVal)) ||
       (Wx > (limitScale * aoaHwaObj->dynLocalCfg.fovAoaLocalCfg.maxAzimuthSineVal)))
    {
        goto exit;
    }

    temp = range*range -x*x -z*z;
    if (temp > 0)
    {
        y = sqrtf(temp);
    }
    else
    {
        goto exit;
    }

    objOut[objOutIdx].x = x;
    objOut[objOutIdx].y = y;
    objOut[objOutIdx].z = z;

    objOut[objOutIdx].velocity = params->dopplerStep * dopplerIdx;
    objOutSideInfo[objOutIdx].noise = objIn[objInIdx].noise;
    objOutSideInfo[objOutIdx].snr = objIn[objInIdx].snr;
    res->detObj2dAzimIdx[objOutIdx] = maxIdx;		
	objOutIdx++;
exit:
    return (objOutIdx);
}

/*!
 *  @b Description
 *  @n
 *      This function performs Doppler compensation on a single antenna symbol.
 *
 *  @param[in]  in           Pointer to the Input Symbol 
 *
 *  @param[in]  out          Pointer to the Output Symbol
 *
 *  @param[in]  Cos          Cosine value depending on doppler index
 *
 *  @param[in]  Sin          Sine value depending on doppler index
 *
 *  @retval None
 *
 *  \ingroup    DPU_AOAPROCDCMP_INTERNAL_FUNCTION
 *
 */
static void aoaHwa_dopplerComp
(
    cmplx16ImRe_t *in,
    cmplx16ImRe_t *out,
    float  Cos,
    float  Sin
)
{
    float           yRe, yIm;

    /* Rotate symbol (correct the phase) */
    yRe = in->real * Cos + in->imag * Sin;
    yIm = in->imag * Cos - in->real * Sin;

    out->real = (int16_t) yRe;
    out->imag = (int16_t) yIm;
}

/**
 *   @b Description
 *   @n
 *      Function performs Doppler compensation on antenna symbols.
 *
 *  @param[in]  srcPtr              Input pointer to antenna symbols
 *
 *  @param[in]  cfarOutList         CFAR detection list
 *
 *  @param[in]  dstPtr              Output pointer to antenna symbols
 *
 *  @param[in]  numTxAnt            Number of Tx antennas
 *
 *  @param[in]  numRxAnt            Number of physical Rx antennas
 *
 *  @param[in]  numVirtualAntAzim   Number of virtual azimuth Rx antennas
 *
 *  @param[in]  numVirtualAntElev   Number of virtual elevation Rx antennas
 *
 *  @param[in]  numDopplerBins      Number of Doppler bins
 *
 *  @param[in]  extendedMaxVelEnabled      0: extended maximum velocity feature disabled, 1: enabled
 *
 *  @return None
 *
 *  \ingroup    DPU_AOAPROCDCMP_INTERNAL_FUNCTION
 *
 */
static void aoaHwa_dopplerCompensation
(
    uint32_t *srcPtr,
    DPIF_CFARDetList *cfarOutList,
    uint32_t *dstPtr,
    uint32_t numTxAnt,
    uint32_t numRxAnt,
    uint32_t numVirtualAntAzim,
    uint32_t numVirtualAntElev,
    uint32_t numDopplerBins,
    uint32_t extendedMaxVelEnabled
)
{
    uint32_t    index;
    uint32_t    j;
    uint32_t    virtAntIdx;
    uint32_t    txAntIdx;
    uint16_t    dopplerIdx;
    int32_t     dopplerSignIdx;
    float       dopplerCompensationIdx;
    int32_t     wrapInd;
    uint32_t    numHypotheses;
    float       Cos,Sin,temp;
    uint32_t    numVirtualAnt;

    dopplerIdx = cfarOutList->dopplerIdx;
    dopplerSignIdx = (int32_t) dopplerIdx;
    if(dopplerIdx >= (numDopplerBins/2))
    {
        dopplerSignIdx = dopplerSignIdx - (int32_t) numDopplerBins;
    }

    if(extendedMaxVelEnabled)
    {
        numHypotheses = numTxAnt;
        wrapInd = - ((int32_t) (numHypotheses >> 1));
        if (!(numTxAnt & 0x1) && (dopplerSignIdx < 0))
        {
            wrapInd++;
        }
    }
    else
    {
        numHypotheses = 1;
        wrapInd = 0;
    }

    numVirtualAnt = numVirtualAntAzim + numVirtualAntElev;
    
    for(index = 0; index < numHypotheses; index++)
    {
        virtAntIdx = 0;
        /* transfer data corresponding to azimuth virtual antennas (corresponding to chirp of antenna Tx0) */
        for(j = 0; j < numRxAnt; j++)
        {
            dstPtr[virtAntIdx + index*numVirtualAnt] = srcPtr[virtAntIdx];
            virtAntIdx++;
        }

        if(numTxAnt > 1)
        {
            dopplerCompensationIdx = (dopplerSignIdx + (wrapInd * (int32_t)numDopplerBins)) / (float)numTxAnt;
            Cos = cosf(2*PI_*dopplerCompensationIdx/numDopplerBins);
            Sin = sinf(2*PI_*dopplerCompensationIdx/numDopplerBins);

            for(txAntIdx=1; txAntIdx < numTxAnt; txAntIdx++)
            {
                for(j = 0; j < numRxAnt; j++)
                {
                    aoaHwa_dopplerComp((cmplx16ImRe_t *)&srcPtr[virtAntIdx],
                                       (cmplx16ImRe_t *)&dstPtr[virtAntIdx + index*numVirtualAnt],
                                       Cos,
                                       Sin);
                    virtAntIdx++;
                }

                if (txAntIdx < (numTxAnt-1))
                {
                    /* Increment Doppler phase shift */
                    temp = Cos * Cos - Sin * Sin;
                    Sin = 2 * Cos * Sin;
                    Cos = temp;
                }

            }
        }
        wrapInd++;
    }
}


/**
 *  @b  Description
 *  @n
 *      Function calculates x/y/z coordinates of the objects detected by CFAR DPU.
 *      The number of output objects may be larger than input if multiobject beam
 *      forming is enabled, and more than one objects are detected at the
 *      same range/doppler bin.
 *
 *  @param[in]  aoaHwaObj          Pointer to AoAProcDcmp DPU internal data Object
 *  @param[in]  objInIdx           Index to the next object in the output list
 *  @param[in]  pingPongIdx        Ping/pong index
 *  @param[in]  objOutIdx          Index to the next object in the output list
 *
 *  @retval    objOutIdx           Number of detected objects in the output list
 *
 *  \ingroup    DPU_AOAPROCDCMP_INTERNAL_FUNCTION
 *
 */

static uint32_t AoAProcDcmpHWA_angleEstimationAzimElev
(
    AOAHwaObj       *aoaHwaObj,
    uint32_t        objInIdx,
    uint32_t        pingPongIdx,
    uint32_t        objOutIdx,
    uint16_t *azimFFTAbsPtr,
	cmplx16ImRe_t * azimFFTPtrBase
)
{
    uint32_t j, maxVal = 0,maxIdx = 0, tempVal;
    //uint32_t objInHwaIdx;
    DPU_AoAProcDcmpHWA_HW_Resources *res = &aoaHwaObj->res;
    DPU_AoAProcDcmpHWA_StaticConfig *DPParams = &aoaHwaObj->params;
    DPIF_CFARDetList        *objIn = res->cfarRngDopSnrList;


    uint32_t numAngleBins = DPU_AOAPROCDCMPHWA_NUM_ANGLE_BINS;
    uint16_t maxNumObj = res->detObjOutMaxSize;
    uint32_t numHypotheses;
    int32_t  hypothesisMaxIdx;
    
    int16_t  dopplerSignIdx;
    int32_t  wrapStartInd;
    uint8_t numTxAnt = DPParams->numTxAntennas;
    uint16_t numDopplerBins = DPParams->numDopplerBins;

    if (objOutIdx >= maxNumObj)
    {
        return objOutIdx;
    }

    dopplerSignIdx = AOA_DOPPLER_IDX_TO_SIGNED(objIn[objInIdx].dopplerIdx, numDopplerBins);
	
    if(aoaHwaObj->dynLocalCfg.extMaxVelCfg.enabled)
    {
        numHypotheses = aoaHwaObj->params.numTxAntennas;
        wrapStartInd = - ((int32_t) (numHypotheses >> 1));
        if (!(numTxAnt & 0x1) && (dopplerSignIdx < 0))
        {
            wrapStartInd++;
        }
    }
    else
    {
        numHypotheses = 1;
        wrapStartInd = 0;
    }



    maxVal = 0;
    for(j=0; j < numHypotheses * numAngleBins; j++)
    {
        tempVal = azimFFTAbsPtr[j];
        if(tempVal > maxVal)
        {
            maxVal = tempVal;
            maxIdx = j;
        }
    }

    hypothesisMaxIdx = maxIdx >> DPU_AOAPROCDCMPHWA_LOG2_NUM_ANGLE_BINS;
    maxIdx = maxIdx & (numAngleBins - 1);

    dopplerSignIdx += (wrapStartInd + hypothesisMaxIdx) * (int16_t) numDopplerBins;

    /* Estimate x,y,z */
    objOutIdx = AoAProcDcmpHWA_XYZestimation(aoaHwaObj,
                                         pingPongIdx,
                                         objInIdx,
                                         objOutIdx,
                                         maxIdx,
                                         dopplerSignIdx,
                                         hypothesisMaxIdx,
										 azimFFTPtrBase);		 

	if (objOutIdx >= maxNumObj)
    {
        return objOutIdx;
    }

    /* Multi peak azimuzth search?*/
    if (aoaHwaObj->dynLocalCfg.multiObjBeamFormingCfg.enabled)
    {
        uint32_t leftSearchIdx;
        uint32_t rightSearchIdx;
        uint32_t secondSearchLen;
        uint32_t iModAzimLen;
        uint32_t maxVal2;
        int32_t k;
        uint32_t t;
        uint16_t azimIdx = maxIdx;
        uint16_t* azimuthMag = &azimFFTAbsPtr[hypothesisMaxIdx * numAngleBins];

        /* Find right edge of the first peak */
        t = azimIdx;
        leftSearchIdx = (t + 1) & (numAngleBins-1);
        k = numAngleBins;
        while ((azimuthMag[t] >= azimuthMag[leftSearchIdx]) && (k > 0))
        {
            t = (t + 1) & (numAngleBins-1);
            leftSearchIdx = (leftSearchIdx + 1) & (numAngleBins-1);
            k--;
        }

        /* Find left edge of the first peak */
        t = azimIdx;
        rightSearchIdx = (t - 1) & (numAngleBins-1);
        k = numAngleBins;
        while ((azimuthMag[t] >= azimuthMag[rightSearchIdx]) && (k > 0))
        {
            t = (t - 1) & (numAngleBins-1);
            rightSearchIdx = (rightSearchIdx - 1) & (numAngleBins-1);
            k--;
        }

        secondSearchLen = ((rightSearchIdx - leftSearchIdx) & (numAngleBins-1)) + 1;
        /* Find second peak */
        maxVal2 = azimuthMag[leftSearchIdx];
        azimIdx = leftSearchIdx;
        for (t = leftSearchIdx; t < (leftSearchIdx + secondSearchLen); t++)
        {
            iModAzimLen = t & (numAngleBins-1);
            if (azimuthMag[iModAzimLen] > maxVal2)
            {
                azimIdx = iModAzimLen;
                maxVal2 = azimuthMag[iModAzimLen];
            }
        }
        /* Is second peak greater than threshold? */
        if ( (maxVal2 >( ((uint32_t)(maxVal * aoaHwaObj->dynLocalCfg.multiObjBeamFormingCfg.multiPeakThrsScal)))) && (objOutIdx < maxNumObj) )
        {

            /* Estimate x,y,z for second peak */
            objOutIdx = AoAProcDcmpHWA_XYZestimation(aoaHwaObj,
                                                 pingPongIdx,
                                                 objInIdx,
                                                 objOutIdx,
                                                 azimIdx,
                                                 dopplerSignIdx,
                                                 hypothesisMaxIdx, 
												 azimFFTPtrBase);
        }
    }
    return(objOutIdx);
}

/**
 *  @b Description
 *  @n
 *      The function waits for 2D FFT single bin data to be transfered to the output buffer.
 *      This is a blocking function. The function polls for EDMA transfer completion.
 *
 *  @param[in]  aoaHwaObj    Pointer to internal AoAProcDcmpHWA data object
 *  @param[in]  channel      EDMA channel number
 *
 *  \ingroup    DPU_AOAPROCDCMP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t  AOAHWA_waitEdma(AOAHwaObj * aoaHwaObj,
                         uint8_t channel)
{
    DPU_AoAProcDcmpHWA_HW_Resources *res = &aoaHwaObj->res;
    int32_t errCode = EDMA_NO_ERROR;

    /* wait until transfer done */
    volatile bool isTransferDone;
    do {
        if ((errCode = EDMA_isTransferComplete(res->edmaHandle,
                                    channel,
                                    (bool *)&isTransferDone)) != EDMA_NO_ERROR)
        {
            break;
        }
    } while (isTransferDone == false);

    return errCode;
}


/**
 *  @b Description
 *  @n
 *      The function converts angle of arrival field of view values specified
 *      in degrees to the values appropriate for internal DPU comparison.
 *
 *  @param[in]  aoaHwaObj    Pointer to internal AoAProcDcmpHWA data object
 *
 *  @param[in]  fovAoaCfg    Pointer to field of view configuration
 *
 *  \ingroup    DPU_AOAPROCDCMP_INTERNAL_FUNCTION
 *
 *  @retval  None
 *
 */
void AoAProcDcmpHWA_ConvertFov(AOAHwaObj *aoaHwaObj,
                           DPU_AoAProcDcmp_FovAoaCfg *fovAoaCfg)
{
    aoaHwaObj->dynLocalCfg.fovAoaLocalCfg.minAzimuthSineVal = sin(fovAoaCfg->minAzimuthDeg / 180. * PI_);
    aoaHwaObj->dynLocalCfg.fovAoaLocalCfg.maxAzimuthSineVal = sin(fovAoaCfg->maxAzimuthDeg / 180. * PI_);
    aoaHwaObj->dynLocalCfg.fovAoaLocalCfg.minElevationSineVal = sin(fovAoaCfg->minElevationDeg / 180. * PI_);
    aoaHwaObj->dynLocalCfg.fovAoaLocalCfg.maxElevationSineVal = sin(fovAoaCfg->maxElevationDeg  / 180. * PI_);
}

DPU_AoAProcDcmpHWA_Handle DPU_AoAProcDcmpHWA_init
(
    DPU_AoAProcDcmpHWA_InitParams *initParams,
    int32_t*            errCode
)
{
    AOAHwaObj     *aoaHwaObj = NULL;
    HWA_MemInfo   hwaMemInfo;
    uint8_t       index;
    SemaphoreP_Params  semParams;

    if ((initParams == NULL) || (initParams->hwaHandle == NULL))
    {
        *errCode = DPU_AOAPROCDCMPHWA_EINVAL;
        goto exit;
    }

    aoaHwaObj = MemoryP_ctrlAlloc(sizeof(AOAHwaObj), 0);
    if(aoaHwaObj == NULL)
    {
        *errCode = DPU_AOAPROCDCMPHWA_ENOMEM;
        goto exit;
    }

    /* Save for debugging */
#ifdef DBG_AOA_HWA_OBJ_DPU
    if (gAoaHwaObjInd < RL_MAX_SUBFRAMES)
    {
        gAoaHwaObj[gAoaHwaObjInd++] = aoaHwaObj;
    }
#endif

    /* Set HWA bank memory address */
    *errCode =  HWA_getHWAMemInfo(initParams->hwaHandle, &hwaMemInfo);
    if (*errCode < 0)
    {
        goto exit;
    }

    /* Initialize memory */
    memset((void *)aoaHwaObj, 0, sizeof(AOAHwaObj));

    /* Save init config params */
    aoaHwaObj->hwaHandle   = initParams->hwaHandle;

    for (index = 0; index < hwaMemInfo.numBanks; index++)
    {
        aoaHwaObj->hwaMemBankAddr[index] = hwaMemInfo.baseAddress + index * hwaMemInfo.bankSize;
    }

    /* Create semaphore for HWA done */
    SemaphoreP_Params_init(&semParams);
    semParams.mode = SemaphoreP_Mode_BINARY;
    aoaHwaObj->hwaDone_semaHandle = SemaphoreP_create(0, &semParams);
    if(aoaHwaObj->hwaDone_semaHandle == NULL)
    {
        *errCode = DPU_AOAPROCDCMPHWA_ESEMA;
        goto exit;
    }


    aoaHwaObj->edmaDstIn2DFFTBuffAddr[0] = aoaHwaObj->hwaMemBankAddr[0];  //HWA M0
    aoaHwaObj->edmaDstIn2DFFTBuffAddr[1] = aoaHwaObj->hwaMemBankAddr[1];  //HWA M1

    aoaHwaObj->edmaSrcOut2DFFTBuffAddr[0] = aoaHwaObj->hwaMemBankAddr[0];  //HWA M2
    aoaHwaObj->edmaSrcOut2DFFTBuffAddr[1] = aoaHwaObj->hwaMemBankAddr[1];  //HWA M3

    aoaHwaObj->edmaDstIn3DFFTBuffAddr[0] = aoaHwaObj->hwaMemBankAddr[0];  //HWA M0
    aoaHwaObj->edmaDstIn3DFFTBuffAddr[1] = aoaHwaObj->hwaMemBankAddr[1];  //HWA M1

    aoaHwaObj->edmaSrcOut3DFFTBuffAddr[0] = aoaHwaObj->hwaMemBankAddr[0] + DPU_AOAPROCDCMPHWA_3DFFT_MAG_SQUARE_ADDRSS_OFFSET;  //HWA M0
    aoaHwaObj->edmaSrcOut3DFFTBuffAddr[1] = aoaHwaObj->hwaMemBankAddr[1] + DPU_AOAPROCDCMPHWA_3DFFT_MAG_SQUARE_ADDRSS_OFFSET;  //HWA M1

    aoaHwaObj->hwaAzimuthFftCmplxOutBuffAddr[0] = aoaHwaObj->hwaMemBankAddr[2];  //HWA M2
    aoaHwaObj->hwaAzimuthFftCmplxOutBuffAddr[1] = aoaHwaObj->hwaMemBankAddr[3];  //HWA M3

exit:
    return ((DPU_AoAProcDcmpHWA_Handle)aoaHwaObj);
}

int32_t DPU_AoAProcDcmpHWA_config
(
    DPU_AoAProcDcmpHWA_Handle    handle,
    DPU_AoAProcDcmpHWA_Config    *aoaHwaCfg
)
{
    int32_t   retVal = 0;
    AOAHwaObj *aoaHwaObj = (AOAHwaObj *)handle;
    int32_t i;

    if(aoaHwaObj == NULL)
    {
        retVal = DPU_AOAPROCDCMPHWA_EINVAL;
        goto exit;
    }

    /* Check if radar cube formats are supported. */
    if (!(aoaHwaCfg->res.radarCube.datafmt == DPIF_RADARCUBE_FORMAT_1))
    {
        retVal = DPU_AOAPROCDCMPHWA_EINVAL__RADARCUBE_DATAFORMAT;
        goto exit;
    }


    if MEM_IS_NOT_ALIGN(aoaHwaCfg->res.cfarRngDopSnrList,
                        DPU_AOAPROCDCMPHWA_CFAR_DET_LIST_BYTE_ALIGNMENT)
    {
        retVal = DPU_AOAPROCDCMPHWA_ENOMEMALIGN_CFAR_DET_LIST;
        goto exit;
    }

    if MEM_IS_NOT_ALIGN(aoaHwaCfg->res.detObjOut,
                        DPU_AOAPROCDCMPHWA_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT)
    {
        retVal = DPU_AOAPROCDCMPHWA_ENOMEMALIGN_POINT_CLOUD_CARTESIAN;
        goto exit;
    }

    if MEM_IS_NOT_ALIGN(aoaHwaCfg->res.detObjOutSideInfo,
                        DPU_AOAPROCDCMPHWA_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT)
    {
        retVal = DPU_AOAPROCDCMPHWA_ENOMEMALIGN_POINT_CLOUD_SIDE_INFO;
        goto exit;
    }

    if MEM_IS_NOT_ALIGN(aoaHwaCfg->res.azimuthStaticHeatMap,
                        DPU_AOAPROCDCMPHWA_AZIMUTH_STATIC_HEAT_MAP_BYTE_ALIGNMENT)
    {
        retVal = DPU_AOAPROCDCMPHWA_ENOMEMALIGN_AZIMUTH_STATIC_HEAT_MAP;
        goto exit;
    }

    /* Check if radar cube range column fits into one HWA memory bank */
    if((aoaHwaCfg->staticCfg.numTxAntennas * aoaHwaCfg->staticCfg.numRxAntennas *
            aoaHwaCfg->staticCfg.numDopplerChirps * sizeof(cmplx16ImRe_t)) > (SOC_HWA_MEM_SIZE/SOC_HWA_NUM_MEM_BANKS))
    {
        retVal = DPU_AOAPROCDCMPHWA_EEXCEEDHWAMEM;
        goto exit;
    }

    /* Check if complex values of 2D FFT fit in one HWA memory bank (16B) */
    /* HWA is running in ping/pong manner, so only one bank is available */
    if((aoaHwaCfg->staticCfg.numTxAntennas * aoaHwaCfg->staticCfg.numRxAntennas *
            aoaHwaCfg->staticCfg.numDopplerBins * sizeof(cmplx16ImRe_t)) > (SOC_HWA_MEM_SIZE/SOC_HWA_NUM_MEM_BANKS))
    {
        retVal = DPU_AOAPROCDCMPHWA_EEXCEEDHWAMEM;
        goto exit;
    }

    /* Check Heapmap configuration */
    if (aoaHwaCfg->dynCfg.prepareRangeAzimuthHeatMap)
    {
        if (aoaHwaCfg->res.azimuthStaticHeatMapSize !=
          (aoaHwaCfg->staticCfg.numRangeBins * aoaHwaCfg->staticCfg.numVirtualAntAzim))
        {
            retVal = DPU_AOAPROCDCMPHWA_ENOMEM__AZIMUTH_STATIC_HEAT_MAP;
            goto exit;
        }

        if (aoaHwaCfg->res.azimuthStaticHeatMap == NULL)
        {
            retVal = DPU_AOAPROCDCMPHWA_EINVAL;
            goto exit;
        }
    }

    if MEM_IS_NOT_ALIGN(aoaHwaCfg->res.detObjElevationAngle,
                        DPU_AOAPROCDCMPHWA_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT)
    {
        retVal = DPU_AOAPROCDCMPHWA_ENOMEMALIGN_DET_OBJ_ELEVATION_ANGLE;
        goto exit;
    }

    if (aoaHwaCfg->dynCfg.prepareRangeAzimuthHeatMap && (aoaHwaCfg->staticCfg.numVirtualAntAzim == 1))
    {
        retVal = DPU_AOAPROCDCMPHWA_EINVALID_NUM_VIRT_ANT_AND_AZIMUTH_STATIC_HEAT_MAP;
        goto exit;
    }

    if (aoaHwaCfg->res.detObjOutMaxSize & 0x1)
    {
        retVal = DPU_AOAPROCDCMPHWA_EDETECTED_OBJECT_LIST_SIZE_ODD_NUMBER;
        goto exit;
    }

    for(i = 0; i < DPU_AOAPROCDCMPHWA_NUM_LOCAL_SCRATCH_BUFFERS; i++)
    {
        if MEM_IS_NOT_ALIGN(aoaHwaCfg->res.localScratchBuffer[i],
                            DPU_AOAPROCDCMPHWA_LOCAL_SCRATCH_BYTE_ALIGNMENT)
        {
            retVal = DPU_AOAPROCDCMPHWA_ENOMEMALIGN_LOCAL_SCRATCH_BUF;
            goto exit;
        }
        if (aoaHwaCfg->res.localScratchBuffer[i] == NULL)
        {
            retVal = DPU_AOAPROCDCMPHWA_EINVAL;
            goto exit;
        }
    }
    if (aoaHwaCfg->res.localScratchBufferSizeBytes !=
      (DPU_AOAPROCDCMPHWA_NUM_LOCAL_SCRATCH_BUFFER_SIZE_BYTES(aoaHwaCfg->staticCfg.numTxAntennas)))
    {
        retVal = DPU_AOAPROCDCMPHWA_ENOMEM_LOCAL_SCRATCH_BUF;
        goto exit;
    }

    if MEM_IS_NOT_ALIGN(aoaHwaCfg->res.radarCube.data,
                        DPU_AOAPROCDCMPHWA_RADAR_CUBE_BYTE_ALIGNMENT)
    {
        retVal = DPU_AOAPROCDCMPHWA_ENOMEMALIGN_RADAR_CUBE;
        goto exit;
    }


#if DEBUG_CHECK_PARAMS
    /* Validate params */
    if(!aoaHwaCfg->edmaHandle ||
      !aoaHwaCfg->hwaHandle ||
      !aoaHwaCfg->params ||
      !aoaHwaCfg->radarCube ||
      !aoaHwaCfg->detObjOut ||
      (aoaHwaCfg->numParamSet != 1)
      )
    {
        retVal= DPU_AOAPROCDCMPHWA_EINVAL;
        goto exit;
    }
#endif
    aoaHwaObj->res = aoaHwaCfg->res;
    aoaHwaObj->params = aoaHwaCfg->staticCfg;

    aoaHwaObj->dynLocalCfg.compRxChanCfg  = *aoaHwaCfg->dynCfg.compRxChanCfg;
    aoaHwaObj->dynLocalCfg.multiObjBeamFormingCfg = *aoaHwaCfg->dynCfg.multiObjBeamFormingCfg;
    aoaHwaObj->dynLocalCfg.prepareRangeAzimuthHeatMap  = aoaHwaCfg->dynCfg.prepareRangeAzimuthHeatMap;
    AoAProcDcmpHWA_ConvertFov(aoaHwaObj, aoaHwaCfg->dynCfg.fovAoaCfg);

    aoaHwaObj->dynLocalCfg.extMaxVelCfg = *aoaHwaCfg->dynCfg.extMaxVelCfg;

    /* Allocate buffers for ping and pong paths: */
    for (i = 0; i < DPU_AOAPROCDCMPHWA_NUM_LOCAL_SCRATCH_BUFFERS; i++)
    {
        uint32_t byteAlignment;

        aoaHwaObj->azimuthFftOutMagBuf[i] = (uint16_t *) aoaHwaCfg->res.localScratchBuffer[i];
        aoaHwaObj->azimElevLocalBuf[i] = (uint32_t *) aoaHwaCfg->res.localScratchBuffer[i];

#ifndef SOC_XWR68XX
        /* Speculative workaround for an issue where EDMA is not completing transfer. */
        if (((uint32_t)aoaHwaCfg->staticCfg.numTxAntennas * (uint32_t)aoaHwaCfg->staticCfg.numTxAntennas *
             (uint32_t)aoaHwaCfg->staticCfg.numRxAntennas * sizeof(cmplx16ImRe_t)) == 64U)
        {
            /* Note this will only happen when numTxAntennas = 2 and numRxAntennas = 4 */
            byteAlignment = 64U;
        }
        else
#endif
        {
            byteAlignment = 1U;
        }
        aoaHwaObj->azimElevLocalHypothesesBuf[i] = 
            (uint32_t *) MEM_ALIGN((uint32_t)&aoaHwaObj->azimElevLocalBuf[i][aoaHwaCfg->staticCfg.numTxAntennas *
                                                                             aoaHwaCfg->staticCfg.numRxAntennas * aoaHwaCfg->staticCfg.compressCfg.numRangeBinsPerBlock],
                                    byteAlignment);
    }

    aoaHwaObj->edmaDstOut2DFFTBuffAddr[0] = (uint32_t) &aoaHwaObj->azimElevLocalBuf[0][0];
    aoaHwaObj->edmaDstOut2DFFTBuffAddr[1] = (uint32_t) &aoaHwaObj->azimElevLocalBuf[1][0];
    aoaHwaObj->edmaSrcIn3DFFTBuffAddr[0] = (uint32_t) &aoaHwaObj->azimElevLocalHypothesesBuf[0][0];
    aoaHwaObj->edmaSrcIn3DFFTBuffAddr[1] = (uint32_t) &aoaHwaObj->azimElevLocalHypothesesBuf[1][0];
    aoaHwaObj->edmaDstOut3DFFTBuffAddr[0] = (uint32_t) &aoaHwaObj->azimuthFftOutMagBuf[0][0];
    aoaHwaObj->edmaDstOut3DFFTBuffAddr[1] = (uint32_t) &aoaHwaObj->azimuthFftOutMagBuf[1][0];


    /* Windowing configuraiton in HWA */
    retVal = HWA_configRam(aoaHwaObj->hwaHandle,
                           HWA_RAM_TYPE_WINDOW_RAM,
                           (uint8_t *)aoaHwaCfg->res.hwaCfg.window,
                           aoaHwaCfg->res.hwaCfg.windowSize, /* size in bytes */
                           aoaHwaCfg->res.hwaCfg.winRamOffset * sizeof(uint32_t));
    if (retVal != 0)
    {
        goto exit;
    }

exit:
    return retVal;
}


#if  NUM_RANGE_BINS_PER_COMPRESSED_BLOCK > 1
int16_t findDetObjsPerRangeGateRange(uint16_t* pCurrRangeBin, uint16_t numRangeBins, AOAHwaObj *aoaHwaObj, int16_t * objList, uint16_t numbObj)
{
	DPU_AoAProcDcmpHWA_HW_Resources *res = &aoaHwaObj->res;
    DPIF_CFARDetList        *objIn = res->cfarRngDopSnrList;
	/* Note : There is an implicit assumption that objects are ordered in increasing order (in rangeIdx). 
	 * The first detected object is used to start the range-gate range. */
	uint16_t objIdxOut = 0;
	uint16_t currObjNum;
	
	while(objIdxOut == 0)
	{
		uint16_t startRangeBin = *pCurrRangeBin;
		uint16_t endRangeBin = *pCurrRangeBin + DPParams->compressCfg.numRangeBinsPerBlock;
		
		
		if (startRangeBin == numRangeBins)
		{
			return 0;
		}
		else if (endRangeBin > numRangeBins)
		{
			// To be an error. 
			return -1;
		}
			
		
		objIn = res->cfarRngDopSnrList;
		for (currObjNum = 0; currObjNum < numbObj; currObjNum++)
		{
			if ( (objIn[currObjNum].rangeIdx >= startRangeBin) &&
				 (objIn[currObjNum].rangeIdx < endRangeBin) )
			{
				objList[objIdxOut] = currObjNum;
				objIdxOut++;
				
				if (objIdxOut == MAX_NUM_OBJS_PER_RANGE_GATE_RANGE)
				{
					return objIdxOut;
				}
			}
		}
		
		*pCurrRangeBin = endRangeBin;
	}

	return objIdxOut;
}

#endif

int32_t DPU_AoAProcDcmpHWA_process
(
    DPU_AoAProcDcmpHWA_Handle    handle,
    uint32_t        numObjsIn,
    DPU_AoAProcDcmpHWA_OutParams  *outParams
)
{
    volatile uint32_t   startTime;
    volatile uint32_t   startTime1;
    volatile uint32_t   waitTimeLocal = 0;
    int32_t             retVal = 0;
    uint16_t            idx;
	#if NUM_RANGE_BINS_PER_COMPRESSED_BLOCK == 1
    uint16_t            detObjIdxIn;
	#endif
    uint16_t            detObjIdxProc;
    uint8_t             pingPongIdx;
    uint32_t            trueNumObjsIn;

    float               range;
    uint32_t            numObjsOut = 0;
    uint16_t numParams;
	
    HWA_Handle hwaHandle;

    AOAHwaObj *aoaHwaObj;
    DPU_AoAProcDcmpHWA_HW_Resources *res;
    DPU_AoAProcDcmpHWA_StaticConfig *DPParams;

    if (handle == NULL)
    {
        retVal = DPU_AOAPROCDCMPHWA_EINVAL;
        goto exit;
    }
    aoaHwaObj = (AOAHwaObj *)handle;
    res = &aoaHwaObj->res;
    DPParams = &aoaHwaObj->params;
    hwaHandle = aoaHwaObj->hwaHandle;

    startTime = Cycleprofiler_getTimeStamp();

    if ( (numObjsIn == 0U) || (DPParams->compressCfg.numRangeBinsPerBlock > 1))
    {
        outParams->numAoADetectedPoints = numObjsIn;
        outParams->stats.waitTime = 0;
        outParams->stats.processingTime = 0;
        retVal = 0;
        goto exit;
    }

    /**********************************************/
    /* ENABLE NUMLOOPS DONE INTERRUPT FROM HWA */
    /**********************************************/
    retVal = HWA_enableDoneInterrupt(hwaHandle,
                                     AOAProcHWADoneIsrCallback,
                                     aoaHwaObj->hwaDone_semaHandle);
    if (retVal != 0)
    {
	   goto exit;
    }

    /* Azimuth heap doppler compensation */
    if (aoaHwaObj->dynLocalCfg.prepareRangeAzimuthHeatMap)
    {
        /* Perform 2D FFT for all range bins and save zero Doppler bin */
		retVal = -123;
        if (retVal != 0)
        {
            goto exit;
        }
    }

    /* Angle estimation */
    if(DPParams->numVirtualAntAzim == 1)
    {
       /*If there is only one virtual antenna, there is no
         need of azimuth FFT as azimuth can not be estimated.*/

        /* Limit number of input samples */
        if (numObjsIn > res->detObjOutMaxSize)
        {
          numObjsIn = res->detObjOutMaxSize;
        }

        /* Fill the output list */
        for(idx=0; idx < numObjsIn; idx++)
        {
            range = res->cfarRngDopSnrList[idx].rangeIdx * DPParams->rangeStep;
            res->detObjOut[idx].y = range;
            res->detObjOut[idx].x = 0.;
            res->detObjOut[idx].z = 0.;

            res->detObjOut[idx].velocity = DPParams->dopplerStep *
                 AOA_DOPPLER_IDX_TO_SIGNED(res->cfarRngDopSnrList[idx].dopplerIdx,
                                       DPParams->numDopplerBins);
            res->detObjOutSideInfo[idx].snr = res->cfarRngDopSnrList[idx].snr;
            res->detObjOutSideInfo[idx].noise = res->cfarRngDopSnrList[idx].noise;
        }
        numObjsOut = numObjsIn;
    }
    else
    {
        /* Limit number of input samples */
        if (numObjsIn > res->detObjOutMaxSize)
        {
          numObjsIn = res->detObjOutMaxSize;
        }
        trueNumObjsIn = numObjsIn;
        /* Make number of detected objects EVEN */
        if (numObjsIn & 0x1)
        {
            /* If odd number of input objects, add dummy as duplicated last object */
            res->cfarRngDopSnrList[numObjsIn] = res->cfarRngDopSnrList[numObjsIn-1];
            numObjsIn++;
        }


        /* Configure HWA Params*/
        retVal = HWAutil_configHWA_extendedVelocityScheme(hwaHandle,
												res,
												DPParams,
												aoaHwaObj->hwaMemBankAddr,
												aoaHwaObj->dynLocalCfg.extMaxVelCfg.enabled,
												DPU_AOAPROCDCMPHWA_NUM_ANGLE_BINS);
        if (retVal != 0)
        {

            goto exit;
        }

		
        /* Configure Common HWA registers */
        numParams = 2*(2 + 2 + ((DPParams->numVirtualAntElev > 0) & 0x1));
        retVal =  AoAProcDcmp_HWAutil_configCommon (hwaHandle,
                                                numObjsIn/2, //numLoops,
                                                res->hwaCfg.paramSetStartIdx, //paramStartIdx,
                                                numParams, 
												DPParams->compressCfg.ratio);
        if (retVal != 0)
        {
            goto exit;
        }
		
        
        /* Configure EDMA */
        retVal = AoAProcDcmpHWA_config_EDMA(hwaHandle,
                                        res,
                                        DPParams,
                                        0,//srcIn2DFFTBuffAddr
                                        aoaHwaObj->edmaDstIn2DFFTBuffAddr,
                                        aoaHwaObj->edmaSrcOut2DFFTBuffAddr,
                                        aoaHwaObj->edmaDstOut2DFFTBuffAddr,
                                        aoaHwaObj->edmaSrcIn3DFFTBuffAddr,
                                        aoaHwaObj->edmaDstIn3DFFTBuffAddr,
                                        aoaHwaObj->edmaSrcOut3DFFTBuffAddr,
                                        aoaHwaObj->edmaDstOut3DFFTBuffAddr,
                                        aoaHwaObj->dynLocalCfg.extMaxVelCfg.enabled);
        if (retVal != 0)
        {
            goto exit;
        }

        /* Enable the HWA */
        retVal = HWA_enable(hwaHandle, 1);
        if (retVal != 0)
        {
            goto exit;
        }

#if NUM_RANGE_BINS_PER_COMPRESSED_BLOCK == 1
		/* If each compressed block contains only one range bin 
		 * use the pre-existing code. */
		/* Trigger first two objects */
        detObjIdxIn = 0;
        pingPongIdx = 0;
        AoAProcDcmpHWA_cfgAndTrigger_EDMA_2DFFT(aoaHwaObj,
                                      detObjIdxIn,
                                      pingPongIdx);
        detObjIdxIn = 1;
        pingPongIdx = 1;
        AoAProcDcmpHWA_cfgAndTrigger_EDMA_2DFFT(aoaHwaObj,
                                      detObjIdxIn,
                                      pingPongIdx);
        /* Loop through the list two objects per loop */
        detObjIdxIn = 2;
        detObjIdxProc = 0;
		while(detObjIdxProc < numObjsIn)
        {
			for (pingPongIdx = 0; pingPongIdx < 2; pingPongIdx++)
			{
				/* Wait until EDMA output is done */
				startTime1 = Cycleprofiler_getTimeStamp();
				retVal = AOAHWA_waitEdma(aoaHwaObj, res->edmaHwaExt[pingPongIdx].chOut.channel);
				waitTimeLocal += Cycleprofiler_getTimeStamp() - startTime1;
				if (retVal != 0)
				{
					  goto exit;
				}
				/* Rx channel gain/phase offset compensation */
				AoAProcDcmpHWA_rxChanPhaseBiasCompensation(&aoaHwaObj->dynLocalCfg.compRxChanCfg.rxChPhaseComp[0],
													1,
													DPParams->numVirtualAntAzim + DPParams->numVirtualAntElev,
													(cmplx16ImRe_t *) aoaHwaObj->azimElevLocalBuf[pingPongIdx],
													(cmplx16ImRe_t *) aoaHwaObj->azimElevLocalBuf[pingPongIdx]);
				/* Doppler Compensation - */
				aoaHwa_dopplerCompensation((uint32_t *) aoaHwaObj->azimElevLocalBuf[pingPongIdx],
										   &res->cfarRngDopSnrList[detObjIdxProc + pingPongIdx],
										   (uint32_t *) aoaHwaObj->azimElevLocalHypothesesBuf[pingPongIdx],
										   DPParams->numTxAntennas,
										   DPParams->numRxAntennas,
										   DPParams->numVirtualAntAzim,
										   DPParams->numVirtualAntElev,
										   DPParams->numDopplerBins,
										   aoaHwaObj->dynLocalCfg.extMaxVelCfg.enabled);
				/* Trigger 3D-FFT */
				AoAProcDcmpHWA_trigger_EDMA_3DFFT(aoaHwaObj, pingPongIdx);
			}
	
			for (pingPongIdx = 0; pingPongIdx < 2; pingPongIdx++)
			{
				/* Wait until EDMA outputs 3D-FFTs and Maximum peaks */
				startTime1 = Cycleprofiler_getTimeStamp();
				retVal = AOAHWA_waitEdma(aoaHwaObj, res->edmaHwaExt[pingPongIdx].chOut.channel);
				waitTimeLocal += Cycleprofiler_getTimeStamp() - startTime1;
				if (retVal != 0)
				{
					
					  goto exit;
				}
				if (detObjIdxProc < trueNumObjsIn)
				{
					numObjsOut = AoAProcDcmpHWA_angleEstimationAzimElev(aoaHwaObj,
																	detObjIdxProc,
																	pingPongIdx,
																	numObjsOut,
																	(uint16_t *)aoaHwaObj->azimuthFftOutMagBuf[pingPongIdx],
																	(cmplx16ImRe_t *)aoaHwaObj->hwaAzimuthFftCmplxOutBuffAddr[pingPongIdx]);
				}
				if (detObjIdxIn < numObjsIn)
				{
					/* Trigger 2D-FFT - next target */
					AoAProcDcmpHWA_cfgAndTrigger_EDMA_2DFFT(aoaHwaObj,
														 detObjIdxIn,
														 pingPongIdx);
				}
				detObjIdxProc++;
				detObjIdxIn++;
			}
		} /* Loop per two detected points */
#else
		{
			uint16_t *localBufAddrAbsSrc, *currAddrAbsSrc;
			cmplx16ImRe_t *localBufAddrSrc, *currAddrSrc, *localBufAddrDst, *currAddrDst, *localBufAddrFFTSrc, *currAddrFFTSrc, *NextAddr;
			uint16_t currObj;
			uint16_t currRangeBin, numHypotheses, currObjIdx;
			int16_t objListLen[2];
			int16_t objList[2][MAX_NUM_OBJS_PER_RANGE_GATE_RANGE];
			uint16_t numVirtualAnt = DPParams->numVirtualAntAzim + DPParams->numVirtualAntElev;
			
			currRangeBin = 0; 
			if(aoaHwaObj->dynLocalCfg.extMaxVelCfg.enabled)
			{
				numHypotheses = aoaHwaObj->params.numTxAntennas;
			}
			else
			{
				numHypotheses = 1;
			}
			
			/* Since multiple range bins are being processed together for 2D FFT, 
			   it is more efficient to find all the objects that exist in the range 
			   bins that are being processed. This list of objects are then used
			   to do a set of angle FFTs using the 2D FFT output. 
			   THis approach reduces the number of 2D FFTs that need to be performed. 
			 */



		
			while(objListLen[0] + objListLen[1] > 0)
			{
				
				
				for (pingPongIdx = 0; pingPongIdx < 2; pingPongIdx++)
				{
					/* Wait until EDMA output is done */
					startTime1 = Cycleprofiler_getTimeStamp();
					retVal = AOAHWA_waitEdma(aoaHwaObj, res->edmaHwaExt[pingPongIdx].chOut.channel);
					waitTimeLocal += Cycleprofiler_getTimeStamp() - startTime1;
				}
				
				for (pingPongIdx = 0; pingPongIdx < 2; pingPongIdx++)
				{
					if (objListLen[pingPongIdx] == 0)
					{
						continue;
					}
					/* The 3D FFT input comes to azimElevLocalBuf. 
					   After processing, it is moved to azimElevLocalHypothesesBuf.
					   */
					localBufAddrSrc = (cmplx16ImRe_t *) aoaHwaObj->azimElevLocalBuf[pingPongIdx];
					localBufAddrDst = (cmplx16ImRe_t *) aoaHwaObj->azimElevLocalHypothesesBuf[pingPongIdx];
						
					
					for (currObjIdx = 0; currObjIdx < objListLen[0] + objListLen[1]; currObjIdx++)
					{
						currObj = objList[pingPongIdx][currObjIdx];
						currAddrSrc = localBufAddrSrc + (currObjIdx*numVirtualAnt);
						currAddrDst  = localBufAddrDst;
						
						if (retVal != 0)
						{
							  goto exit;
						}
					
						if (currObjIdx < objListLen[pingPongIdx] -1)
						{
							NextAddr  = localBufAddrSrc + ((currObjIdx+1)*numVirtualAnt);
							/* Configure the next EDMA. To bring in the next object to the 
							 * next offset address of azimElevLocalBuf[pingPongIdx]. */
							AoAProcDcmpHWA_Extract2DFFT(aoaHwaObj,
											  objList[pingPongIdx][currObjIdx+1],
											  NextAddr,	
											  pingPongIdx);
						}
						/* Rx channel gain/phase offset compensation */
						AoAProcDcmpHWA_rxChanPhaseBiasCompensation
												(&aoaHwaObj->dynLocalCfg.compRxChanCfg.rxChPhaseComp[0],
											1,
											numVirtualAnt,
											currAddrSrc,currAddrSrc);
					}
					/* Trigger 3D-FFT */
					AoAProcDcmpHWA_trigger_multi_EDMA_3DFFT(aoaHwaObj, pingPongIdx,objListLen[pingPongIdx], numHypotheses);
				
				}

				
				/* In this loop we process the 3D FFT output. */
				for (pingPongIdx = 0; pingPongIdx < 2; pingPongIdx++)
				{
					
					if (objListLen[pingPongIdx] == 0)
					{
						continue;
					}
					
					

					
					/* Wait until EDMA outputs 3D-FFTs and Maximum peaks */
					startTime1 = Cycleprofiler_getTimeStamp();
					retVal = AOAHWA_waitEdma(aoaHwaObj, res->edmaHwaExt[pingPongIdx].chOut.channel);
					waitTimeLocal += Cycleprofiler_getTimeStamp() - startTime1;
					
					localBufAddrAbsSrc = (uint16_t *)aoaHwaObj->azimuthFftOutMagBuf[pingPongIdx];
																		
					localBufAddrFFTSrc = (cmplx16ImRe_t *)aoaHwaObj->hwaAzimuthFftCmplxOutBuffAddr[pingPongIdx];
					
					for (currObjIdx = 0; currObjIdx < objListLen[pingPongIdx]; currObjIdx++)
					{
						currObj = objList[pingPongIdx][currObjIdx];
						currAddrFFTSrc = localBufAddrFFTSrc  + (currObjIdx*numVirtualAnt*numHypotheses);
						currAddrAbsSrc = localBufAddrAbsSrc  + (currObjIdx*numVirtualAnt*numHypotheses);
						if (retVal != 0)
						{
							  goto exit;
						}
						if (detObjIdxProc < trueNumObjsIn)
						{
							numObjsOut = AoAProcDcmpHWA_angleEstimationAzimElev(aoaHwaObj,
																			currObj,
																			pingPongIdx,
																			numObjsOut,
																			currAddrAbsSrc,
																			currAddrFFTSrc);
						}
						
						detObjIdxProc++;
					}
					
					/* Trigger next two sets of objects */
					{
						objListLen[pingPongIdx] = findDetObjsPerRangeGateRange(&currRangeBin, aoaHwaObj->params.numRangeBins,aoaHwaObj,objList[pingPongIdx],numObjsIn);
					
						if (objListLen[pingPongIdx] == 0)
						{
							continue;
						}
						else if (objListLen[pingPongIdx] < 0)
						{
							retVal = -12312;
							goto exit;
						}

						AoAProcDcmpHWA_cfgAndTrigger_EDMA_2DFFT(aoaHwaObj,
													  objList[pingPongIdx][0],
													  pingPongIdx);
					}
				}
			} 
		}
#endif

        /* Disable the HWA */
        retVal = HWA_enable(hwaHandle, 0);
        if (retVal != 0)
        {
            goto exit;
        }

    }

    outParams->numAoADetectedPoints = numObjsOut;
    outParams->stats.waitTime = waitTimeLocal;
    outParams->stats.processingTime = Cycleprofiler_getTimeStamp() - startTime - waitTimeLocal;
#ifdef PROFILE_AOA_HWA_OBJ_DPU
    gAoAProcDcmpStats[gAoAProcDcmpStatsIdx] = *outParams;
    gAoAProcDcmpStatsIdx = (gAoAProcDcmpStatsIdx +1) & 0xf;
#endif

    /* Process is done, disable Done interrupt */
    HWA_disableDoneInterrupt(hwaHandle);
exit:
    return retVal;
}

int32_t DPU_AoAProcDcmpHWA_control
(
   DPU_AoAProcDcmpHWA_Handle handle,
   DPU_AoAProcDcmpHWA_Cmd cmd,
   void *arg,
   uint32_t argSize
)
{
   int32_t retVal = 0;
   AOAHwaObj *aoaHwaObj = (AOAHwaObj *)handle;

   /* Get rangeProc data object */
   if (aoaHwaObj == NULL)
   {
       retVal = DPU_AOAPROCDCMPHWA_EINVAL;
       goto exit;
   }

   switch(cmd)
   {
       case DPU_AoAProcDcmpHWA_Cmd_FovAoACfg:
       {
           if((argSize != sizeof(DPU_AoAProcDcmp_FovAoaCfg)) ||
              (arg == NULL))
           {
               retVal = DPU_AOAPROCDCMPHWA_EINVAL;
               goto exit;
           }
           else
           {
               /* Save configuration */
               AoAProcDcmpHWA_ConvertFov(aoaHwaObj, (DPU_AoAProcDcmp_FovAoaCfg *) arg);

           }
       }
       break;
       case DPU_AoAProcDcmpHWA_Cmd_MultiObjBeamFormingCfg:
       {
           if((argSize != sizeof(DPU_AoAProcDcmp_MultiObjBeamFormingCfg)) ||
              (arg == NULL))
           {
               retVal = DPU_AOAPROCDCMPHWA_EINVAL;
               goto exit;
           }
           else
           {
               /* Save configuration */
               memcpy((void *)&aoaHwaObj->dynLocalCfg.multiObjBeamFormingCfg, arg, argSize);
           }
       }
       break;
       case DPU_AoAProcDcmpHWA_Cmd_ExtMaxVelocityCfg:
       {
           if((argSize != sizeof(DPU_AoAProcDcmp_ExtendedMaxVelocityCfg)) ||
              (arg == NULL))
           {
               retVal = DPU_AOAPROCDCMPHWA_EINVAL;
               goto exit;
           }
           else
           {
               /* Save configuration */
               memcpy((void *)&aoaHwaObj->dynLocalCfg.extMaxVelCfg, arg, argSize);
           }
       }
       break;
       case DPU_AoAProcDcmpHWA_Cmd_CompRxChannelBiasCfg:
       {
           if((argSize != sizeof(DPU_AoAProcDcmp_compRxChannelBiasCfg)) ||
              (arg == NULL))
           {
               retVal = DPU_AOAPROCDCMPHWA_EINVAL;
               goto exit;
           }
           else
           {
               /* Save configuration */
               memcpy((void *)&aoaHwaObj->dynLocalCfg.compRxChanCfg, arg, argSize);
           }
       }
       break;
       case DPU_AoAProcDcmpHWA_Cmd_PrepareRangeAzimuthHeatMap:
       {
           if((argSize != sizeof(bool)) ||
              (arg == NULL))
           {
               retVal = DPU_AOAPROCDCMPHWA_EINVAL;
               goto exit;
           }
           else
           {
                /* Sanity check on saved heapmap ptr and size */
                if ((aoaHwaObj->res.azimuthStaticHeatMapSize !=
                    (aoaHwaObj->params.numRangeBins * aoaHwaObj->params.numVirtualAntAzim)) ||
                    (aoaHwaObj->res.azimuthStaticHeatMap == NULL))
                {
                   retVal = DPU_AOAPROCDCMPHWA_EINVAL;
                   goto exit;
                }

                /* Save configuration */
                memcpy((void *)&aoaHwaObj->dynLocalCfg.prepareRangeAzimuthHeatMap, arg, argSize);
           }
       }
       break;
       default:
           retVal = DPU_AOAPROCDCMPHWA_EINVAL;
           break;
   }
exit:
   return (retVal);
}

int32_t DPU_AoAProcDcmpHWA_deinit(DPU_AoAProcDcmpHWA_Handle handle)
{
    int32_t retVal = 0;

    if (handle == NULL)
    {
        retVal = DPU_AOAPROCDCMPHWA_EINVAL;
        goto exit;
    }
    MemoryP_ctrlFree(handle, sizeof(AOAHwaObj));
exit:
    return (retVal);
}

/**
 *  @b Description
 *  @n
 *      The function populates the HWA param set to perform decompression 
 * 		at 50 % compression ratio.  
 *
 *  @param[out]  HWA_ParamConfig     pointer to paramset to be populated. 
 *  @param[in]  trigMode 	trigger mode.  
 *  @param[in]  srcAddr 		source address (in the HWA). 
 *  @param[in]  dstAddr 		destination address (in the HWA). 
 *  
 *
 *  \ingroup    DPU_RANGEPROCCMP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
void linear_50p_DcmpCfg(
	HWA_ParamConfig* pHWA_ParamConfig, 
	uint16_t trigMode, 
	uint16_t trigSrc, 
	uint32_t srcAddr, 
	uint32_t dstAddr, 
	uint16_t numSamplesPerBlockIn,  
	uint16_t numSamplesPerBlockOut, 
	uint16_t numBlocks)
{
	/************* Decompression for first TX antenna *********************************/
/************* Decompression for first TX antenna *********************************/
	pHWA_ParamConfig->triggerMode = trigMode; 
	pHWA_ParamConfig->dmaTriggerSrc = trigSrc; 
	pHWA_ParamConfig->accelMode = HWA_ACCELMODE_COMPRESS; 

	pHWA_ParamConfig->source.srcAddr = srcAddr;
	pHWA_ParamConfig->source.srcShift = 0;
	pHWA_ParamConfig->source.srcCircShiftWrap = 0;
	pHWA_ParamConfig->source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
	pHWA_ParamConfig->source.srcWidth = HWA_SAMPLES_WIDTH_16BIT;
	pHWA_ParamConfig->source.srcSign = HWA_SAMPLES_UNSIGNED;
	pHWA_ParamConfig->source.srcConjugate = 0;
	pHWA_ParamConfig->source.srcScale = 0;
	pHWA_ParamConfig->source.bpmEnable = 0;
	pHWA_ParamConfig->source.bpmPhase = 0;

	pHWA_ParamConfig->dest.dstAddr = dstAddr;
	pHWA_ParamConfig->dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX;
	pHWA_ParamConfig->dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT;
	pHWA_ParamConfig->dest.dstSign = HWA_SAMPLES_SIGNED; 
	pHWA_ParamConfig->dest.dstConjugate = 0; 
	pHWA_ParamConfig->dest.dstScale = 0;
	pHWA_ParamConfig->dest.dstSkipInit = 0; 

	pHWA_ParamConfig->accelModeArgs.compressMode.compressDecompress  = HWA_CMP_DCMP_COMPRESS;
	pHWA_ParamConfig->accelModeArgs.compressMode.method   = HWA_COMPRESS_METHOD_EGE;
	pHWA_ParamConfig->accelModeArgs.compressMode.ditherEnable = HWA_FEATURE_BIT_ENABLE;
	pHWA_ParamConfig->accelModeArgs.compressMode.passSelect  = HWA_COMPRESS_PATHSELECT_BOTHPASSES;
	pHWA_ParamConfig->accelModeArgs.compressMode.headerEnable = HWA_FEATURE_BIT_ENABLE;
	pHWA_ParamConfig->accelModeArgs.compressMode.scaleFactorBW = 4;
	pHWA_ParamConfig->accelModeArgs.compressMode.EGEKarrayLength = 3;

	/* HWA range decompression src/dst configuration*/
	pHWA_ParamConfig->source.srcAcnt = numSamplesPerBlockIn - 1; 
	pHWA_ParamConfig->source.srcAIdx = sizeof(uint32_t);
	pHWA_ParamConfig->source.srcBcnt = numBlocks-1;
	pHWA_ParamConfig->source.srcBIdx = numSamplesPerBlockIn*pHWA_ParamConfig->source.srcAIdx;

	pHWA_ParamConfig->dest.dstAcnt = numSamplesPerBlockOut-1; 
	pHWA_ParamConfig->dest.dstAIdx = sizeof(cmplx16ImRe_t);
	pHWA_ParamConfig->dest.dstBIdx = numSamplesPerBlockOut*pHWA_ParamConfig->dest.dstAIdx; 
}

/**
 *  @b Description
 *  @n
 *      The function populates the HWA param set to perform an FFT.
 * 	    The input format is interleaved. 
 * 		The output format is deinterleaved.  
 *
 *  @param[out]  HWA_ParamConfig     pointer to paramset to be populated. 
 *  @param[in]  trigMode 	trigger mode.  
 *  @param[in]  srcAddr 		source address (in the HWA). 
 *  @param[in]  dstAddr 		destination address (in the HWA). 
 *  
 *
 *  \ingroup    DPU_RANGEPROCCMP_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
void intrleavdInp_interleavdOutput_FFTCfg(
	HWA_ParamConfig *pHWA_ParamConfig, 
	uint16_t trigMode, 
	uint16_t trigSrc, 
	uint16_t srcAddr,  
	uint16_t dstAddr,
	uint16_t numDopplerChirps, 
	uint16_t numRxAnt, 
	uint16_t numDopplerBins, 
	uint16_t numVirtualAnt,
	uint32_t windowOffset,
    uint8_t  winSym
    )
{
		pHWA_ParamConfig->triggerMode   = trigMode;
	pHWA_ParamConfig->dmaTriggerSrc = trigSrc;
	pHWA_ParamConfig->accelMode = HWA_ACCELMODE_FFT; //do FFT

	pHWA_ParamConfig->source.srcAddr = srcAddr;
	pHWA_ParamConfig->source.srcAcnt = numDopplerChirps - 1; //size in samples - 1
	pHWA_ParamConfig->source.srcAIdx = numVirtualAnt * sizeof(cmplx16ImRe_t); //
	pHWA_ParamConfig->source.srcBcnt = (numVirtualAnt) - 1;
	pHWA_ParamConfig->source.srcBIdx = sizeof(cmplx16ImRe_t); //should be dont care
	pHWA_ParamConfig->source.srcShift = 0; //no shift
	pHWA_ParamConfig->source.srcCircShiftWrap = 0; //no shift
	pHWA_ParamConfig->source.srcRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //complex data
	pHWA_ParamConfig->source.srcWidth = HWA_SAMPLES_WIDTH_16BIT; //16-bit
	pHWA_ParamConfig->source.srcSign = HWA_SAMPLES_SIGNED; //signed
	pHWA_ParamConfig->source.srcConjugate = 0; //no conjugate
	pHWA_ParamConfig->source.srcScale = 0;
	pHWA_ParamConfig->source.bpmEnable = 0; //bpm removal not enabled
	pHWA_ParamConfig->source.bpmPhase = 0; //dont care

	pHWA_ParamConfig->dest.dstAddr = dstAddr;
	pHWA_ParamConfig->dest.dstAcnt = numDopplerBins -1;
	pHWA_ParamConfig->dest.dstAIdx = (numVirtualAnt) * sizeof(cmplx16ImRe_t);
	pHWA_ParamConfig->dest.dstBIdx = sizeof(cmplx16ImRe_t) ; //should be dont care
	pHWA_ParamConfig->dest.dstRealComplex = HWA_SAMPLES_FORMAT_COMPLEX; //same as input - complex
	pHWA_ParamConfig->dest.dstWidth = HWA_SAMPLES_WIDTH_16BIT; //same as input - 16 bit
	pHWA_ParamConfig->dest.dstSign = HWA_SAMPLES_SIGNED; //same as input - signed
	pHWA_ParamConfig->dest.dstConjugate = 0; //no conjugate
	pHWA_ParamConfig->dest.dstScale = 8 ;
	pHWA_ParamConfig->dest.dstSkipInit = 0; // no skipping

	pHWA_ParamConfig->accelModeArgs.fftMode.fftEn = 1;
	pHWA_ParamConfig->accelModeArgs.fftMode.fftSize = mathUtils_floorLog2(numDopplerBins);
	pHWA_ParamConfig->accelModeArgs.fftMode.butterflyScaling = 0x3FF; //LSB fftSize bits are relevant
	pHWA_ParamConfig->accelModeArgs.fftMode.interfZeroOutEn = 0; //disabled
	pHWA_ParamConfig->accelModeArgs.fftMode.windowEn = 1; //enabled
	pHWA_ParamConfig->accelModeArgs.fftMode.windowStart = windowOffset; //start of window RAM
	pHWA_ParamConfig->accelModeArgs.fftMode.winSymm = winSym;
	pHWA_ParamConfig->accelModeArgs.fftMode.winInterpolateMode = 0;
	pHWA_ParamConfig->accelModeArgs.fftMode.magLogEn = HWA_FFT_MODE_MAGNITUDE_LOG2_DISABLED;
	pHWA_ParamConfig->accelModeArgs.fftMode.fftOutMode = HWA_FFT_MODE_OUTPUT_DEFAULT;

	pHWA_ParamConfig->complexMultiply.mode = HWA_COMPLEX_MULTIPLY_MODE_DISABLE;
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
void  cfgEGEParamListAoaProc (uint8_t * cmpEGEArr, uint16_t compressionRatio, uint16_t srcWidth)
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

