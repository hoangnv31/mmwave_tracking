/*
 *   @file  objectdetectioncmp.c
 *
 *   @brief
 *      Object Detection with Compression DPC implementation.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2017 Texas Instruments, Inc.
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
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define DBG_DPC_OBJDET


/* mmWave SDK Include Files: */
#include <xdc/runtime/System.h>
#include <ti/drivers/soc/soc.h>
#include <ti/common/sys_common.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/osal/MemoryP.h>
#include <ti/drivers/edma/edma.h>
#include <ti/utils/mathutils/mathutils.h>
#include <ti/drivers/uart/UART.h>
#include <ti/utils/cli/cli.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/control/dpm/dpm.h>

/* Compression constants. */
#include <ti/demo/xwr64xx_compression/mmw/compressionConsts.h>


#ifdef SUBSYS_DSS

/* C674x mathlib */
/* Suppress the mathlib.h warnings
 *  #48-D: incompatible redefinition of macro "TRUE"
 *  #48-D: incompatible redefinition of macro "FALSE"
 */
#pragma diag_push
#pragma diag_suppress 48
#include <ti/mathlib/mathlib.h>
#pragma diag_pop
#endif


 /** @addtogroup DPC_OBJDET_IOCTL__INTERNAL_DEFINITIONS
  @{ */

/*! This is supplied at command line when application builds this file. This file
 * is owned by the application and contains all resource partitioning, an
 * application may include more than one DPC and also use resources outside of DPCs.
 * The resource definitions used by this Object Detection with Compression DPC are prefixed by DPC_OBJDET_ */
#include APP_RESOURCE_FILE

/* Obj Det instance etc */
#include <ti/datapath/dpc/objectdetection/objdetcmphwa/include/objectdetectioncmpinternal.h>
#include <ti/datapath/dpc/objectdetection/objdetcmphwa/objectdetectioncmp.h>

#ifdef DBG_DPC_OBJDET
ObjDetCmpObj     *gObjDetCmpObj;
#endif

/*! Radar cube data buffer alignment in bytes. No DPU module specifying alignment
 *  need (through a \#define) implies that no alignment is needed i.e 1 byte alignment.
 *  But we do the natural data type alignment which is 2 bytes (as radar cube is complex16-bit type)
 *  because radar cube is exported out of DPC in processing result so assume CPU may access
 *  it for post-DPC processing.
 */
#define DPC_OBJDET_RADAR_CUBE_DATABUF_BYTE_ALIGNMENT       (sizeof(int16_t))

/*! Detection matrix alignment is declared by CFAR dpu, we size to
 *  the max of this and CPU alignment for accessing detection matrix
 *  it is exported out of DPC in processing result so assume CPU may access
 *  it for post-DPC processing. Note currently the CFAR alignment is the same as
 *  CPU alignment so this max is redundant but it is more to illustrate the
 *  generality of alignments should be done.
 */
#define DPC_OBJDET_DET_MATRIX_DATABUF_BYTE_ALIGNMENT       (MAX(sizeof(uint16_t), \
                                                                DPU_CFARCAPROCHWA_DET_MATRIX_BYTE_ALIGNMENT))

/*! cfar list alignment is declared by cfar and AoA dpu, for debug purposes we size to
 *  the max of these and CPU alignment for accessing cfar list for debug purposes.
 *  Note currently the dpu alignments are the same as CPU alignment so these max are
 *  redundant but it is more to illustrate the generality of alignments should be done.
 */
#define DPC_OBJDET_CFAR_DET_LIST_BYTE_ALIGNMENT            (MAX(MAX(DPU_CFARCAPROCHWA_CFAR_DET_LIST_BYTE_ALIGNMENT,  \
                                                                    DPU_AOAPROCHWA_CFAR_DET_LIST_BYTE_ALIGNMENT),\
                                                                DPIF_CFAR_DET_LIST_CPU_BYTE_ALIGNMENT))


/*! Point cloud cartesian alignment is declared by AoA dpu, we size to
 *  the max of this and CPU alignment for accessing this as it is exported out as result of
 *  processing and so may be accessed by the CPU during post-DPC processing.
 *  Note currently the AoA alignment is the same as CPU alignment so this max is
 *  redundant but it is more to illustrate the generality of alignments should be done.
 */
#ifdef SOC_XWR68XX
#define DPC_OBJDET_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT       (MAX(DPU_AOAPROCHWA_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT, \
                                                                   DPIF_POINT_CLOUD_CARTESIAN_CPU_BYTE_ALIGNMENT))
#else
/* Speculative workaround for an issue where EDMA is not completing transfer. */
#define DPC_OBJDET_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT       (MAX(MAX(DPU_AOAPROCHWA_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT, \
                                                                       DPIF_POINT_CLOUD_CARTESIAN_CPU_BYTE_ALIGNMENT), \
                                                                   64U))
#endif

/*! Point cloud side info alignment is declared by AoA dpu, we size to
 *  the max of this and CPU alignment for accessing this as it is exported out as result of
 *  processing and so may be accessed by the CPU during post-DPC processing.
 *  Note currently the AoA alignment is the same as CPU alignment so this max is
 *  redundant but it is more to illustrate the generality of alignments should be done.
 */
#ifdef SOC_XWR68XX
#define DPC_OBJDET_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT       (MAX(DPU_AOAPROCHWA_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT, \
                                                                   DPIF_POINT_CLOUD_SIDE_INFO_CPU_BYTE_ALIGNMENT))
#else
/* Speculative workaround for an issue where EDMA is not completing transfer. */
#define DPC_OBJDET_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT       (MAX(MAX(DPU_AOAPROCHWA_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT, \
                                                                       DPIF_POINT_CLOUD_SIDE_INFO_CPU_BYTE_ALIGNMENT), \
                                                                   64U))
#endif

/*! Azimuth static heat map alignment is declared by AoA dpu, we size to
 *  the max of this and CPU alignment for accessing this as it is exported out as result of
 *  processing and so may be accessed by the CPU during post-DPC processing.
 *  Note currently the AoA alignment is the same as CPU alignment so this max is
 *  redundant but it is more to illustrate the generality of alignments should be done.
 */
#define DPC_OBJDET_AZIMUTH_STATIC_HEAT_MAP_BYTE_ALIGNMENT     (MAX(DPU_AOAPROCHWA_AZIMUTH_STATIC_HEAT_MAP_BYTE_ALIGNMENT, \
                                                                   sizeof(int16_t)))

/*! Elevation angle alignment is declared by AoA dpu, we size to
 *  the max of this and CPU alignment for accessing this as it is exported out as result of
 *  processing and so may be accessed by the CPU during post-DPC processing.
 *  Note currently the AoA alignment is the same as CPU alignment so this max is
 *  redundant but it is more to illustrate the generality of alignments should be done.
 */
#define DPC_OBJDET_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT     (MAX(DPU_AOAPROCHWA_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT, \
                                                                   sizeof(float)))

/**
@}
*/

#define DPC_OBJDET_HWA_MAX_WINDOW_RAM_SIZE_IN_SAMPLES    SOC_HWA_WINDOW_RAM_SIZE_IN_SAMPLES
#define DPC_OBJDET_HWA_NUM_PARAM_SETS                    SOC_HWA_NUM_PARAM_SETS

/*! Maximum Number of objects that can be detected in a frame */
#define DPC_OBJDET_MAX_NUM_OBJECTS                       500U

/******************************************************************************/
/* Local definitions */

#define DPC_USE_SYMMETRIC_WINDOW_RANGE_DPU
#define DPC_USE_SYMMETRIC_WINDOW_DOPPLER_DPU
#define DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE            MATHUTILS_WIN_BLACKMAN
#define DPC_DPU_DOPPLERPROC_FFT_WINDOW_TYPE          MATHUTILS_WIN_HANNING

/**************************************************************************
 ************************** Local Functions *******************************
 **************************************************************************/
/**
 *  @b Description
 *  @n
 *      Utility function for reseting memory pool.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      none.
 */
static void DPC_ObjDetCmp_MemPoolReset(MemPoolObj *pool)
{
    pool->currAddr = (uintptr_t)pool->cfg.addr;
    pool->maxCurrAddr = pool->currAddr;
}

/**
 *  @b Description
 *  @n
 *      Utility function for setting memory pool to desired address in the pool.
 *      Helps to rewind for example.
 *
 *  @param[in]  pool Handle to pool object.
 *  @param[in]  addr Address to assign to the pool's current address.
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      None
 */
static void DPC_ObjDetCmp_MemPoolSet(MemPoolObj *pool, void *addr)
{
    pool->currAddr = (uintptr_t)addr;
    pool->maxCurrAddr = MAX(pool->currAddr, pool->maxCurrAddr);
}

/**
 *  @b Description
 *  @n
 *      Utility function for getting memory pool current address.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      pointer to current address of the pool (from which next allocation will
 *      allocate to the desired alignment).
 */
static void *DPC_ObjDetCmp_MemPoolGet(MemPoolObj *pool)
{
    return((void *)pool->currAddr);
}

#if 0 /* may be useful in future */
/**
 *  @b Description
 *  @n
 *      Utility function for getting current memory pool usage.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  @retval
 *      Amount of pool used in bytes.
 */
static uint32_t DPC_ObjDetCmp_MemPoolGetCurrentUsage(MemPoolObj *pool)
{
    return((uint32_t)(pool->currAddr - (uintptr_t)pool->cfg.addr));
}
#endif

/**
 *  @b Description
 *  @n
 *      Utility function for getting maximum memory pool usage.
 *
 *  @param[in]  pool Handle to pool object.
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Amount of pool used in bytes.
 */
static uint32_t DPC_ObjDetCmp_MemPoolGetMaxUsage(MemPoolObj *pool)
{
    return((uint32_t)(pool->maxCurrAddr - (uintptr_t)pool->cfg.addr));
}

/**
 *  @b Description
 *  @n
 *      Utility function for allocating from a static memory pool.
 *
 *  @param[in]  pool Handle to pool object.
 *  @param[in]  size Size in bytes to be allocated.
 *  @param[in]  align Alignment in bytes
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      pointer to beginning of allocated block. NULL indicates could not
 *      allocate.
 */
static void *DPC_ObjDetCmp_MemPoolAlloc(MemPoolObj *pool,
                              uint32_t size,
                              uint8_t align)
{
    void *retAddr = NULL;
    uintptr_t addr;

    addr = MEM_ALIGN(pool->currAddr, align);
    if ((addr + size) <= ((uintptr_t)pool->cfg.addr + pool->cfg.size))
    {
        retAddr = (void *)addr;
        pool->currAddr = addr + size;
        pool->maxCurrAddr = MAX(pool->currAddr, pool->maxCurrAddr);
    }

    return(retAddr);
}

static DPM_DPCHandle DPC_ObjectDetectionCmp_init
(
    DPM_Handle          dpmHandle,
    DPM_InitCfg*        ptrInitCfg,
    int32_t*            errCode
);

static int32_t DPC_ObjectDetectionCmp_execute
(
    DPM_DPCHandle handle,
    DPM_Buffer*       ptrResult
);

static int32_t DPC_ObjectDetectionCmp_ioctl
(
    DPM_DPCHandle   handle,
    uint32_t            cmd,
    void*               arg,
    uint32_t            argLen
);

static int32_t DPC_ObjectDetectionCmp_start  (DPM_DPCHandle handle);
static int32_t DPC_ObjectDetectionCmp_stop   (DPM_DPCHandle handle);
static int32_t DPC_ObjectDetectionCmp_deinit (DPM_DPCHandle handle);
static void    DPC_ObjectDetectionCmp_frameStart (DPM_DPCHandle handle);
static void    DPC_ObjectDetectionCmp_chirpCnt (DPM_DPCHandle handle);

/**************************************************************************
 ************************* Global Declarations ****************************
 **************************************************************************/

/** @addtogroup DPC_OBJDET__GLOBAL
 @{ */

/**
 * @brief   Global used to register Object Detection with Compression DPC in DPM
 */
DPM_ProcChainCfg gDPC_ObjectDetectionCmpCfg =
{
    DPC_ObjectDetectionCmp_init,            /* Initialization Function:         */
    DPC_ObjectDetectionCmp_start,           /* Start Function:                  */
    DPC_ObjectDetectionCmp_execute,         /* Execute Function:                */
    DPC_ObjectDetectionCmp_ioctl,           /* Configuration Function:          */
    DPC_ObjectDetectionCmp_stop,            /* Stop Function:                   */
    DPC_ObjectDetectionCmp_deinit,          /* Deinitialization Function:       */
    NULL,                                	/* Inject Data Function:            */
    DPC_ObjectDetectionCmp_chirpCnt,        /* Chirp Available Function:        */
    DPC_ObjectDetectionCmp_frameStart       /* Frame Start Function:            */
};

/* @} */


/**
 *  @b Description
 *  @n
 *      Sends Assert
 *
 *  @retval
 *      Not Applicable.
 */
void _DPC_Objdet_Assert(DPM_Handle handle, int32_t expression,
                        const char *file, int32_t line)
{
    DPM_DPCAssert       fault;

    if (!expression)
    {
        fault.lineNum = (uint32_t)line;
        fault.arg0    = 0U;
        fault.arg1    = 0U;
        strncpy (fault.fileName, file, (DPM_MAX_FILE_NAME_LEN-1));

        /* Report the fault to the DPM entities */
        DPM_ioctl (handle,
                   DPM_CMD_DPC_ASSERT,
                   (void*)&fault,
                   sizeof(DPM_DPCAssert));
    }
}

/**
 *  @b Description
 *  @n
 *      DPC frame start function registered with DPM. This is invoked on reception
 *      of the frame start ISR from the RF front-end. This API is also invoked
 *      when application issues @ref DPC_OBJDET_IOCTL__TRIGGER_FRAME to simulate
 *      a frame trigger (e.g for unit testing purpose).
 *
 *  @param[in]  handle DPM's DPC handle
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
uint32_t chirpCnt = 0;
static void DPC_ObjectDetectionCmp_frameStart (DPM_DPCHandle handle)
{
    ObjDetCmpObj     *objDetCmpObj = (ObjDetCmpObj *) handle;

    objDetCmpObj->stats.frameStartTimeStamp = Cycleprofiler_getTimeStamp();

    DebugP_log2("ObjDetCmp DPC: Frame Start, frameIndx = %d, subFrameIndx = %d\n",
                objDetCmpObj->stats.frameStartIntCounter, objDetCmpObj->subFrameIndx);
	
    /* Check if previous frame (sub-frame) processing has completed */
    DPC_Objdet_Assert(objDetCmpObj->dpmHandle, (objDetCmpObj->interSubFrameProcToken == 0));
    objDetCmpObj->interSubFrameProcToken++;
	
	chirpCnt = 0;
    /* Increment interrupt counter for debugging and reporting purpose */
    if (objDetCmpObj->subFrameIndx == 0)
    {
        objDetCmpObj->stats.frameStartIntCounter++;
    }

    /* Notify the DPM Module that the DPC is ready for execution */
    DebugP_assert (DPM_notifyExecute (objDetCmpObj->dpmHandle, handle, true) == 0);
    return;
}

/**
 *  @b Description
 *  @n
 * 		chirp counter (for debug)
 *
 *  @param[in]  handle DPM's DPC handle
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Not applicable
 */
 
static void DPC_ObjectDetectionCmp_chirpCnt (DPM_DPCHandle handle)
{
    chirpCnt ++;
	
	
    return;
}



/**
 *  @b Description
 *  @n
 *      Utility function to do a parabolic/quadratic fit on 3 input points
 *      and return the coordinates of the peak. This is used to accurately estimate
 *      range bias.
 *
 *  @param[in]  x Pointer to array of 3 elements representing the x-coordinate
 *              of the points to fit
 *  @param[in]  y Pointer to array of 3 elements representing the y-coordinate
 *              of the points to fit
 *  @param[out] xv Pointer to output x-coordinate of the peak value
 *  @param[out] yv Pointer to output y-coordinate of the peak value
 *
 *  @retval   None
 *
 * \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static void DPC_ObjDetCmp_quadFit(float *x, float*y, float *xv, float *yv)
{
    float a, b, c, denom;
    float x0 = x[0];
    float x1 = x[1];
    float x2 = x[2];
    float y0 = y[0];
    float y1 = y[1];
    float y2 = y[2];

    denom = (x0 - x1)*(x0 - x2)*(x1 - x2);
    a = (x2 * (y1 - y0) + x1 * (y0 - y2) + x0 * (y2 - y1)) / denom;
    b = (x2*x2 * (y0 - y1) + x1*x1 * (y2 - y0) + x0*x0 * (y1 - y2)) / denom;
    c = (x1 * x2 * (x1 - x2) * y0 + x2 * x0 * (x2 - x0) * y1 + x0 * x1 * (x0 - x1) * y2) / denom;

    *xv = -b/(2*a);
    *yv = c - b*b/(4*a);
}

/**
 *  @b Description
 *  @n
 *      Computes the range bias and rx phase compensation from the detection matrix
 *      during calibration measurement procedure of these parameters.
 *
 *  @param[in]  staticCfg Pointer to static configuration
 *  @param[in]  targetDistance Target distance in meters
 *  @param[in]  searchWinSize Search window size in meters
 *  @param[in] detMatrix Pointer to detection matrix
 *  @param[in] symbolMatrix Pointer to symbol matrix
 *  @param[out] compRxChanCfg computed output range bias and rx phase comp vector
 *
 *  @retval   None
 *
 * \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static void DPC_ObjDetCmp_rangeBiasRxChPhaseMeasure
(
    DPC_ObjectDetectionCmp_StaticCfg       *staticCfg,
    float                   targetDistance,
    float                   searchWinSize,
    uint16_t                *detMatrix,
    uint32_t                *symbolMatrix,
    DPU_AoAProcDcmp_compRxChannelBiasCfg *compRxChanCfg
)
{
    cmplx16ImRe_t rxSym[SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL];
    cmplx16ImRe_t *tempPtr;
    float sumSqr;
    uint32_t * rxSymPtr = (uint32_t * ) rxSym;
    float xMagSq[SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL];
    int32_t iMax;
    float xMagSqMin;
    float scal;
    float truePosition;
    int32_t truePositionIndex;
    float y[3];
    float x[3];
    int32_t halfWinSize ;
    float estPeakPos;
    float estPeakVal;
    int32_t i, ind;
    int32_t txIdx, rxIdx;

    uint32_t numRxAntennas = staticCfg->ADCBufData.dataProperty.numRxAntennas;
    uint32_t numTxAntennas = staticCfg->numTxAntennas;
    uint32_t numRangeBins = staticCfg->numRangeBins;
    uint32_t numDopplerChirps = staticCfg->numDopplerChirps;
    uint32_t numSymPerTxAnt = numDopplerChirps * numRxAntennas * numRangeBins;
    uint32_t symbolMatrixIndx;

    uint16_t maxVal = 0;

    truePosition = targetDistance / staticCfg->rangeStep;
    truePositionIndex = (int32_t) (truePosition + 0.5);

    halfWinSize = (int32_t) (0.5 * searchWinSize / staticCfg->rangeStep + 0.5);

    /**** Range calibration ****/
    iMax = truePositionIndex;
    for (i = truePositionIndex - halfWinSize; i <= truePositionIndex + halfWinSize; i++)
    {
        if (detMatrix[i * staticCfg->numDopplerBins] > maxVal)
        {
            maxVal = detMatrix[i * staticCfg->numDopplerBins];
            iMax = i;
        }
    }

    /* Fine estimate of the peak position using quadratic fit */
    ind = 0;
    for (i = iMax-1; i <= iMax+1; i++)
    {
        sumSqr = 0.0;
        for (txIdx=0; txIdx < numTxAntennas; txIdx++)
        {
            for (rxIdx=0; rxIdx < numRxAntennas; rxIdx++)
            {
                symbolMatrixIndx = txIdx * numSymPerTxAnt + rxIdx * numRangeBins + i;
                tempPtr = (cmplx16ImRe_t *) &symbolMatrix[symbolMatrixIndx];
                sumSqr += (float) tempPtr->real * (float) tempPtr->real +
                          (float) tempPtr->imag * (float) tempPtr->imag;
            }
        }
#ifdef SUBSYS_DSS
        y[ind] = sqrtsp(sumSqr);
#else
        y[ind] = sqrt(sumSqr);
#endif
        x[ind] = (float)i;
        ind++;
    }
    DPC_ObjDetCmp_quadFit(x, y, &estPeakPos, &estPeakVal);
    compRxChanCfg->rangeBias = (estPeakPos - truePosition) * staticCfg->rangeStep;

    /*** Calculate Rx channel phase/gain compensation coefficients ***/
    for (txIdx = 0; txIdx < numTxAntennas; txIdx++)
    {
        for (rxIdx = 0; rxIdx < numRxAntennas; rxIdx++)
        {
            i = txIdx * numRxAntennas + rxIdx;
            symbolMatrixIndx = txIdx * numSymPerTxAnt + rxIdx * numRangeBins + iMax;
            rxSymPtr[i] = symbolMatrix[symbolMatrixIndx];
            xMagSq[i] = (float) rxSym[i].real * (float) rxSym[i].real +
                        (float) rxSym[i].imag * (float) rxSym[i].imag;
        }
    }
    xMagSqMin = xMagSq[0];
    for (i = 1; i < staticCfg->numVirtualAntennas; i++)
    {
        if (xMagSq[i] < xMagSqMin)
        {
            xMagSqMin = xMagSq[i];
        }
    }

    for (txIdx=0; txIdx < staticCfg->numTxAntennas; txIdx++)
    {
        for (rxIdx=0; rxIdx < numRxAntennas; rxIdx++)
        {
            int32_t temp;
            i = txIdx * numRxAntennas + rxIdx;
            scal = 32768./ xMagSq[i] * sqrt(xMagSqMin);

            temp = (int32_t) MATHUTILS_ROUND_FLOAT(scal * rxSym[i].real);
            MATHUTILS_SATURATE16(temp);
            compRxChanCfg->rxChPhaseComp[staticCfg->txAntOrder[txIdx] * numRxAntennas +
                                         rxIdx].real = (int16_t) (temp);

            temp = (int32_t) MATHUTILS_ROUND_FLOAT(-scal * rxSym[i].imag);
            MATHUTILS_SATURATE16(temp);
            compRxChanCfg->rxChPhaseComp[staticCfg->txAntOrder[txIdx] * numRxAntennas
                                         + rxIdx].imag = (int16_t) (temp);
        }
    }
}

/**
 *  @b Description
 *  @n
 *      Computes the length of window to generate for range DPU.
 *
 *  @param[in]  cfg Range DPU configuration
 *
 *  @retval   Length of window to generate
 *
 * \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static uint32_t DPC_ObjDetCmp_GetRangeWinGenLen(DPU_RangeProcCmpHWA_Config *cfg)
{
    uint16_t numAdcSamples;
    uint32_t winGenLen;

    numAdcSamples = cfg->staticCfg.ADCBufData.dataProperty.numAdcSamples;

#ifdef DPC_USE_SYMMETRIC_WINDOW_RANGE_DPU
    winGenLen = (numAdcSamples + 1)/2;
#else
    winGenLen = numAdcSamples;
#endif
    return(winGenLen);
}

#ifdef SOC_XWR68XX
/* Current SOC has hotter receive level (even at lowest Rx Gain setting) that causes
 * overflow in FFT processing. So scale window for range FFT down compared to 18xx.
 * TODO: Future SOC release that does not have this problem should set Q format to 17
 */
#define DPC_OBJDET_QFORMAT_RANGE_FFT 15
#else
#define DPC_OBJDET_QFORMAT_RANGE_FFT 17
#endif
#define DPC_OBJDET_QFORMAT_DOPPLER_FFT 17

/**
 *  @b Description
 *  @n
 *      Generate the range DPU window using mathutils API.
 *
 *  @param[in]  cfg Range DPU configuration, output window is generated in window
 *                  pointer in the staticCfg of this.
 *
 *  @retval   None
 *
 * \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static void DPC_ObjDetCmp_GenRangeWindow(DPU_RangeProcCmpHWA_Config *cfg)
{
    mathUtils_genWindow((uint32_t *)cfg->staticCfg.window,
                        cfg->staticCfg.ADCBufData.dataProperty.numAdcSamples,
                        DPC_ObjDetCmp_GetRangeWinGenLen(cfg),
                        DPC_DPU_RANGEPROC_FFT_WINDOW_TYPE,
                        DPC_OBJDET_QFORMAT_RANGE_FFT);
}

/**
 *  @b Description
 *  @n
 *      Computes the length of window to generate for doppler DPU.
 *
 *  @param[in]  cfg Doppler DPU configuration
 *
 *  @retval   Length of window to generate
 *
 * \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static uint32_t DPC_ObjDetCmp_GetDopplerWinGenLen(DPU_DopplerProcDcmpHWA_Config *cfg)
{
    uint16_t numDopplerChirps;
    uint32_t winGenLen;

    numDopplerChirps = cfg->staticCfg.numDopplerChirps;

#ifdef DPC_USE_SYMMETRIC_WINDOW_DOPPLER_DPU
    winGenLen = (numDopplerChirps + 1)/2;
#else
    winGenLen = numDopplerChirps;
#endif
    return(winGenLen);
}

/**
 *  @b Description
 *  @n
 *      Generate the doppler DPU window using mathutils API.
 *
 *  @param[in]  cfg Doppler DPU configuration, output window is generated in window
 *                  pointer embedded in this configuration.
 *
 *  @retval   winType window type, see mathutils.h
 *
 * \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static uint32_t DPC_ObjDetCmp_GenDopplerWindow(DPU_DopplerProcDcmpHWA_Config *cfg)
{
    uint32_t winType;

    /* For too small window, force rectangular window to avoid loss of information
     * due to small window values (e.g. hanning has first and last coefficients 0) */
    if (cfg->staticCfg.numDopplerChirps <= 4)
    {
        winType = MATHUTILS_WIN_RECT;
    }
    else
    {
        winType = DPC_DPU_DOPPLERPROC_FFT_WINDOW_TYPE;
    }

    mathUtils_genWindow((uint32_t *)cfg->hwRes.hwaCfg.window,
                        cfg->staticCfg.numDopplerChirps,
                        DPC_ObjDetCmp_GetDopplerWinGenLen(cfg),
                        winType,
                        DPC_OBJDET_QFORMAT_DOPPLER_FFT);
                        
    return(winType);
}

/**
 *  @b Description
 *  @n
 *      Extracts the sub-frame specific vector from the common (full vector for all antennnas)
 *      input vector of the range bias and rx phase compensation. Uses the antenna order of
 *      the sub-frame.
 *  @param[in]  staticCfg Static configuration of the sub-frame
 *  @param[in]  inpCfg The full vector.
 *  @param[out] outCfg Sub-frame specific compensation vector that will be used during processing
 *
 *  @retval   None
 *
 * \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static void DPC_ObjDetCmp_GetRxChPhaseComp(DPC_ObjectDetectionCmp_StaticCfg *staticCfg,
                                 DPU_AoAProcDcmp_compRxChannelBiasCfg *inpCfg,
                                 DPU_AoAProcDcmp_compRxChannelBiasCfg *outCfg)
{
    uint32_t tx, rx, numTxAnt, numRxAnt;
    uint8_t *txAntOrder, *rxAntOrder;
    cmplx16ImRe_t one;

    one.imag = 0;
    one.real = 0x7fff;

    numTxAnt = staticCfg->numTxAntennas;
    numRxAnt = staticCfg->ADCBufData.dataProperty.numRxAntennas;
    txAntOrder = staticCfg->txAntOrder;
    rxAntOrder = staticCfg->rxAntOrder;
    outCfg->rangeBias = inpCfg->rangeBias;

    for(tx = 0; tx < numTxAnt; tx++)
    {
        for(rx = 0; rx < numRxAnt; rx++)
        {
            if (staticCfg->isValidProfileHasOneTxPerChirp == 1)
            {
                outCfg->rxChPhaseComp[tx * numRxAnt + rx] =
                    inpCfg->rxChPhaseComp[txAntOrder[tx] * SYS_COMMON_NUM_RX_CHANNEL +
                                          rxAntOrder[rx]];
            }
            else
            {
                outCfg->rxChPhaseComp[tx * numRxAnt + rx] = one;
            }
        }
    }
}

uint32_t NCallsObjDetDPU = 0;;
/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) execute function which is invoked by the application
 *      in the DPM's execute context when the DPC issues DPM_notifyExecute API from
 *      its registered @ref DPC_ObjectDetectionCmp_frameStart API that is invoked every
 *      frame interrupt.
 *
 *  @param[in]  handle       DPM's DPC handle
 *  @param[out]  ptrResult   Pointer to the result
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
volatile int32_t retVal2=-234;
int32_t DPC_ObjectDetectionCmp_execute
(
    DPM_DPCHandle   handle,
    DPM_Buffer*     ptrResult
)
{
    ObjDetCmpObj   *objDetCmpObj;
    SubFrameObj *subFrmObj;
    DPU_RangeProcCmpHWA_OutParams outRangeProc;
    DPU_DopplerProcDcmpHWA_OutParams outDopplerProc;
    DPU_AoAProcDcmpHWA_OutParams outAoaProc;
	DPU_CFARCAProcHWA_OutParams outCfarcaProc;

    int32_t retVal;
    DPC_ObjectDetectionCmp_ExecuteResult *result;
    DPC_ObjectDetectionCmp_ProcessCallBackCfg *processCallBack;
    int32_t i;

    objDetCmpObj = (ObjDetCmpObj *) handle;
    DebugP_assert (objDetCmpObj != NULL);
    DebugP_assert (ptrResult != NULL);
    
    DebugP_log1("ObjDetCmp DPC: Processing sub-frame %d\n", objDetCmpObj->subFrameIndx);

    processCallBack = &objDetCmpObj->processCallBackCfg;

    if (processCallBack->processFrameBeginCallBackFxn != NULL)
    {
        (*processCallBack->processFrameBeginCallBackFxn)(objDetCmpObj->subFrameIndx);
    }

    result = &objDetCmpObj->executeResult;

    subFrmObj = &objDetCmpObj->subFrameObj[objDetCmpObj->subFrameIndx];

    retVal = DPU_RangeProcCmpHWA_process(subFrmObj->dpuRangeObj, &outRangeProc);
    if (retVal != 0)
    {
		goto exit;
    }
    DebugP_assert(outRangeProc.endOfChirp == true);

		
    if (processCallBack->processInterFrameBeginCallBackFxn != NULL)
    {
        (*processCallBack->processInterFrameBeginCallBackFxn)(objDetCmpObj->subFrameIndx);
    }

    objDetCmpObj->stats.interFrameStartTimeStamp = Cycleprofiler_getTimeStamp();

	
    DebugP_log0("ObjDetCmp DPC: Range Proc Done\n");

  
    DPC_ObjDetCmp_GenDopplerWindow(&subFrmObj->dpuCfg.dopplerCfg);
    retVal = DPU_DopplerProcDcmpHWA_config(subFrmObj->dpuDopplerObj, &subFrmObj->dpuCfg.dopplerCfg);
    if (retVal != 0)
    {
        goto exit;
    }
    retVal = DPU_DopplerProcDcmpHWA_process(subFrmObj->dpuDopplerObj, &outDopplerProc);
    if (retVal != 0)
    {
        goto exit;
    }

	
    /* Procedure for range bias measurement and Rx channels gain/phase offset measurement */
    if(objDetCmpObj->commonCfg.measureRxChannelBiasCfg.enabled)
    {
        DPC_ObjDetCmp_rangeBiasRxChPhaseMeasure(&subFrmObj->staticCfg,
            objDetCmpObj->commonCfg.measureRxChannelBiasCfg.targetDistance,
            objDetCmpObj->commonCfg.measureRxChannelBiasCfg.searchWinSize,
            subFrmObj->dpuCfg.dopplerCfg.hwRes.detMatrix.data,
            (uint32_t *) subFrmObj->dpuCfg.rangeCfg.hwRes.radarCube.data,
            &objDetCmpObj->compRxChanCfgMeasureOut);
    }

	retVal = DPU_CFARCAProcHWA_config(subFrmObj->dpuCFARCAObj, &subFrmObj->dpuCfg.cfarCfg);
	if (retVal != 0)
	{
		goto exit;
	}

    retVal = DPU_CFARCAProcHWA_process(subFrmObj->dpuCFARCAObj, &outCfarcaProc);
    if (retVal != 0)
    {
        goto exit;
    }

    DebugP_log1("ObjDetCmp DPC: number of detected objects after CFAR = %d\n",
                outCfarcaProc.numCfarDetectedPoints);
	    
	DPU_AoAProcDcmp_compRxChannelBiasCfg outCompRxCfg;

	/* Generate FFT window, note doppler window is used for AoA */
	DPC_ObjDetCmp_GenDopplerWindow(&subFrmObj->dpuCfg.dopplerCfg);
	DPC_ObjDetCmp_GetRxChPhaseComp(&subFrmObj->staticCfg,
								&objDetCmpObj->commonCfg.compRxChanCfg, &outCompRxCfg);
	subFrmObj->dpuCfg.aoaCfg.dynCfg.compRxChanCfg = &outCompRxCfg;
	
	retVal = DPU_AoAProcDcmpHWA_config(subFrmObj->dpuAoAObj, &subFrmObj->dpuCfg.aoaCfg);
	if (retVal != 0)
	{
		goto exit;
	}

	retVal = DPU_AoAProcDcmpHWA_process(subFrmObj->dpuAoAObj,
                 outCfarcaProc.numCfarDetectedPoints, &outAoaProc);
    if (retVal != 0)
    {
        goto exit;
    }

    /* Set DPM result with measure (bias, phase) and detection info */
    result->numObjOut = outAoaProc.numAoADetectedPoints;
    result->subFrameIdx = objDetCmpObj->subFrameIndx;
    result->objOut               = subFrmObj->dpuCfg.aoaCfg.res.detObjOut;
    result->objOutSideInfo       = subFrmObj->dpuCfg.aoaCfg.res.detObjOutSideInfo;
    result->azimuthStaticHeatMap = subFrmObj->dpuCfg.aoaCfg.res.azimuthStaticHeatMap;
    result->azimuthStaticHeatMapSize = subFrmObj->dpuCfg.aoaCfg.res.azimuthStaticHeatMapSize;
    result->radarCube            = subFrmObj->dpuCfg.aoaCfg.res.radarCube;
    result->detMatrix            = subFrmObj->dpuCfg.dopplerCfg.hwRes.detMatrix;
	
    if (objDetCmpObj->commonCfg.measureRxChannelBiasCfg.enabled == 1)
    {
        result->compRxChanBiasMeasurement = &objDetCmpObj->compRxChanCfgMeasureOut;
    }
    else
    {
        result->compRxChanBiasMeasurement = NULL;
    }
	
	
    /* For rangeProcHwa, interChirpProcessingMargin is not available */
    objDetCmpObj->stats.interChirpProcessingMargin = 0;

    objDetCmpObj->stats.interFrameEndTimeStamp = Cycleprofiler_getTimeStamp();
    result->stats = &objDetCmpObj->stats;

    /* populate DPM_resultBuf - first pointer and size are for results of the
     * processing */
    ptrResult->ptrBuffer[0] = (uint8_t *)result;
    ptrResult->size[0] = sizeof(DPC_ObjectDetectionCmp_ExecuteResult);
	


    /* clear rest of the result */
    for (i = 1; i < DPM_MAX_BUFFER; i++)
    {
        ptrResult->ptrBuffer[i] = NULL;
        ptrResult->size[i] = 0;
    }

exit:

    return retVal;
}

/**
 *  @b Description
 *  @n
 *      Sub-frame reconfiguration, used when switching sub-frames. Invokes the
 *      DPU configuration using the configuration that was stored during the
 *      pre-start configuration so reconstruction time is saved  because this will
 *      happen in real-time.
 *  @param[in]  objDetCmpObj Pointer to DPC object
 *  @param[in]  subFrameIndx Sub-frame index.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 * \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static int32_t DPC_ObjDetCmp_reconfigSubFrame(ObjDetCmpObj *objDetCmpObj, uint8_t subFrameIndx)
{
    int32_t retVal = 0;
    DPU_AoAProcDcmp_compRxChannelBiasCfg outCompRxCfg;
    SubFrameObj *subFrmObj;

    subFrmObj = &objDetCmpObj->subFrameObj[subFrameIndx];

    DPC_ObjDetCmp_GenRangeWindow(&subFrmObj->dpuCfg.rangeCfg);
    retVal = DPU_RangeProcCmpHWA_config(subFrmObj->dpuRangeObj, &subFrmObj->dpuCfg.rangeCfg);
    if (retVal != 0)
    {
        goto exit;
    }

	if (0)
	{
		retVal = DPU_CFARCAProcHWA_config(subFrmObj->dpuCFARCAObj, &subFrmObj->dpuCfg.cfarCfg);
		if (retVal != 0)
		{
			goto exit;
		}

		DPC_ObjDetCmp_GenDopplerWindow(&subFrmObj->dpuCfg.dopplerCfg);
		retVal = DPU_DopplerProcDcmpHWA_config(subFrmObj->dpuDopplerObj, &subFrmObj->dpuCfg.dopplerCfg);
		if (retVal != 0)
		{
			goto exit;
		}

		/* Note doppler window will be used for AoA, so maintain the sequence as in
		 * pre-start config. We need to regenerate the rxChPhaseComp because it was
		 * temporary (note DPUs get pointers to dynamic configs) */
		DPC_ObjDetCmp_GetRxChPhaseComp(&subFrmObj->staticCfg,
									&objDetCmpObj->commonCfg.compRxChanCfg, &outCompRxCfg);
		subFrmObj->dpuCfg.aoaCfg.dynCfg.compRxChanCfg = &outCompRxCfg;
		retVal = DPU_AoAProcDcmpHWA_config(subFrmObj->dpuAoAObj, &subFrmObj->dpuCfg.aoaCfg);
		if (retVal != 0)
		{
			goto exit;
		}
	}
exit:
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) start function which is invoked by the
 *      application using DPM_start API.
 *
 *  @param[in]  handle  DPM's DPC handle
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t DPC_ObjectDetectionCmp_start (DPM_DPCHandle handle)
{
    ObjDetCmpObj   *objDetCmpObj;
    SubFrameObj *subFrmObj;
    int32_t retVal = 0;

    objDetCmpObj = (ObjDetCmpObj *) handle;
    DebugP_assert (objDetCmpObj != NULL);

    objDetCmpObj->stats.frameStartIntCounter = 0;

    /* Start marks consumption of all pre-start configs, reset the flag to check
     * if pre-starts were issued only after common config was issued for the next
     * time full configuration happens between stop and start */
    objDetCmpObj->isCommonCfgReceived = false;

    /* App must issue export of last frame after stop which will switch to sub-frame 0,
     * so start should always see sub-frame indx of 0, check */
    DebugP_assert(objDetCmpObj->subFrameIndx == 0);

    /* Pre-start cfgs for sub-frames may have come in any order, so need
     * to ensure we reconfig for the current (0) sub-frame before starting */
    DPC_ObjDetCmp_reconfigSubFrame(objDetCmpObj, objDetCmpObj->subFrameIndx);

    /* Trigger Range DPU, related to reconfig above */
    subFrmObj = &objDetCmpObj->subFrameObj[objDetCmpObj->subFrameIndx];
    retVal = DPU_RangeProcCmpHWA_control(subFrmObj->dpuRangeObj,
                 DPU_RangeProcCmpHWA_Cmd_triggerProc, NULL, 0);
    if(retVal < 0)
    {
        goto exit;
    }

    DebugP_log0("ObjDetCmp DPC: Start done\n");
exit:
    return(retVal);
}

/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) stop function which is invoked by the
 *      application using DPM_stop API.
 *
 *  @param[in]  handle  DPM's DPC handle
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t DPC_ObjectDetectionCmp_stop (DPM_DPCHandle handle)
{
    ObjDetCmpObj   *objDetCmpObj;

    objDetCmpObj = (ObjDetCmpObj *) handle;
    DebugP_assert (objDetCmpObj != NULL);

    /* We can be here only after complete frame processing is done, which means
     * processing token must be 0 and subFrameIndx also 0  */
    DebugP_assert((objDetCmpObj->interSubFrameProcToken == 0) && (objDetCmpObj->subFrameIndx == 0));

    DebugP_log0("ObjDetCmp DPC: Stop done\n");
    return(0);
}



/**
 *  @b Description
 *  @n
 *      Configures DPC for Range Bias and Phase Comp measurement.
 *
 *  @param[in]  obj
 *      Pointer to DPC object
 *  @param[in] cfg
 *      Pointer to Range Bias and Phase Comp measurement configuration
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t DPC_ObjDetCmp_Config_MeasureRxChannelBiasCfg(ObjDetCmpObj *obj,
                   DPC_ObjectDetectionCmp_MeasureRxChannelBiasCfg *cfg)
{
    int32_t retVal = 0;

    if (cfg->enabled == 1)
    {
        if ((-cfg->searchWinSize/2.0f + cfg->targetDistance) <= 0.0f)
        {
            retVal = DPC_OBJECTDETECTIONCMP_EINVAL__MEASURE_RX_CHANNEL_BIAS_CFG;
            goto exit;
        }
    }
    obj->commonCfg.measureRxChannelBiasCfg = *cfg;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *     Configure range DPU.
 *
 *  @param[in]  dpuHandle Handle to DPU
 *  @param[in]  staticCfg Pointer to static configuration of the sub-frame
 *  @param[in]  dynCfg    Pointer to dynamic configuration of the sub-frame
 *  @param[in]  edmaHandle Handle to edma driver to be used for the DPU
 *  @param[in]  radarCube Pointer to DPIF radar cube, which is output of range
 *                        processing.
 *  @param[in]  CoreLocalRamObj Pointer to core local RAM object to allocate local memory
 *              for the DPU, only for scratch purposes
 *  @param[in,out]  windowOffset Window coefficients that are generated by this function
 *                               (in heap memory) are passed to DPU configuration API to
 *                               configure the HWA window RAM starting from this offset.
 *                               The end offset after this configuration will be returned
 *                               in this variable which could be the begin offset for the
 *                               next DPU window RAM.
 *  @param[out]  CoreLocalRamScratchUsage Core Local RAM's scratch usage in bytes
 *  @param[out] cfgSave Configuration that is built in local
 *                      (stack) variable is saved here. This is for facilitating
 *                      quick reconfiguration later without having to go through
 *                      the construction of the configuration.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static int32_t DPC_ObjDetCmp_rangeConfig(DPU_RangeProcCmpHWA_Handle dpuHandle,
                   DPC_ObjectDetectionCmp_StaticCfg *staticCfg,
                   DPC_ObjectDetectionCmp_DynCfg    *dynCfg,
                   EDMA_Handle                   edmaHandle,
                   DPIF_RadarCube                *radarCube,
                   MemPoolObj                    *CoreLocalRamObj,
                   uint32_t                      *windowOffset,
                   uint32_t                      *CoreLocalRamScratchUsage,
                   DPU_RangeProcCmpHWA_Config       *cfgSave)
{
    int32_t retVal = 0;
    DPU_RangeProcCmpHWA_Config rangeCfg;
    DPU_RangeProcCmpHWA_HW_Resources *hwRes = &rangeCfg.hwRes;
    DPU_RangeProcCmpHWA_EDMAInputConfig *edmaIn = &hwRes->edmaInCfg;
    DPU_RangeProcCmpHWA_EDMAOutputConfig *edmaOut = &hwRes->edmaOutCfg;
    DPU_RangeProcCmpHWA_HwaConfig *hwaCfg = &hwRes->hwaCfg;
    int32_t *windowBuffer;
    uint32_t numRxAntennas, winGenLen;

    memset(&rangeCfg, 0, sizeof(rangeCfg));

    numRxAntennas = staticCfg->ADCBufData.dataProperty.numRxAntennas;

    /* Object Detection with Compression DPC only supports  interleaved at present */
    DebugP_assert(staticCfg->ADCBufData.dataProperty.interleave == DPIF_RXCHAN_INTERLEAVE_MODE);


    /* static configuration */
    rangeCfg.staticCfg.ADCBufData         = staticCfg->ADCBufData;
    rangeCfg.staticCfg.numChirpsPerFrame  = staticCfg->numChirpsPerFrame;
    rangeCfg.staticCfg.numRangeBins       = staticCfg->numRangeBins;
    rangeCfg.staticCfg.numTxAntennas      = staticCfg->numTxAntennas;
    rangeCfg.staticCfg.numVirtualAntennas = staticCfg->numVirtualAntennas;
    rangeCfg.staticCfg.resetDcRangeSigMeanBuffer = 1;
	rangeCfg.staticCfg.compressCfg        = staticCfg->compressCfg;
    /* radarCube */
    rangeCfg.hwRes.radarCube = *radarCube;

    /* static configuration - window */
    /* Generating 1D window, allocate first */
    winGenLen = DPC_ObjDetCmp_GetRangeWinGenLen(&rangeCfg);
    rangeCfg.staticCfg.windowSize = winGenLen * sizeof(uint32_t);
    windowBuffer = (int32_t *)DPC_ObjDetCmp_MemPoolAlloc(CoreLocalRamObj, rangeCfg.staticCfg.windowSize, sizeof(uint32_t));
    if (windowBuffer == NULL)
    {
        retVal = DPC_OBJECTDETECTIONCMP_ENOMEM__CORE_LOCAL_RAM_RANGE_HWA_WINDOW;
        goto exit;
    }
    rangeCfg.staticCfg.window = windowBuffer;
    DPC_ObjDetCmp_GenRangeWindow(&rangeCfg);

    /* hwres */
    /* hwres - dcRangeSig, allocate from heap, this needs to persist within sub-frame/frame
     * processing and across sub-frames */
    hwRes->dcRangeSigMeanSize = DPU_RANGEPROC_SIGNATURE_COMP_MAX_BIN_SIZE *
               staticCfg->numTxAntennas * numRxAntennas * sizeof(cmplx32ImRe_t);
    hwRes->dcRangeSigMean = (cmplx32ImRe_t *) MemoryP_ctrlAlloc (hwRes->dcRangeSigMeanSize,
                                       (uint8_t) 0 /*sizeof(cmplx32ImRe_t)*/);
    DebugP_assert(rangeCfg.hwRes.dcRangeSigMeanSize == hwRes->dcRangeSigMeanSize);

    /* hwres - edma */
    hwRes->edmaHandle = edmaHandle;
    /* We have choosen ISOLATE mode, so we have to fill in dataIn */
    edmaIn->dataIn.channel                  = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_CH;
    edmaIn->dataIn.channelShadow            = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SHADOW;
    edmaIn->dataIn.eventQueue               = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_EVENT_QUE;
    edmaIn->dataInSignature.channel         = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_CH;
    edmaIn->dataInSignature.channelShadow   = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_SHADOW;
    edmaIn->dataInSignature.eventQueue      = DPC_OBJDET_DPU_RANGEPROC_EDMAIN_SIG_EVENT_QUE;

    /* We are radar Cube FORMAT1 and interleaved ADC, so for 3 tx antenna case, we have to
     * fill format2, otherwise format1
     */
    {
        /* Ping */
        edmaOut->u.fmt1.dataOutPing.channel              = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT1_PING_CH;
        edmaOut->u.fmt1.dataOutPing.channelShadow        = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT1_PING_SHADOW;
        edmaOut->u.fmt1.dataOutPing.eventQueue           = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT1_PING_EVENT_QUE;

        /* Pong */
        edmaOut->u.fmt1.dataOutPong.channel              = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT1_PONG_CH;
        edmaOut->u.fmt1.dataOutPong.channelShadow        = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT1_PONG_SHADOW;
        edmaOut->u.fmt1.dataOutPong.eventQueue           = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_FMT1_PONG_EVENT_QUE;

    }

    edmaOut->dataOutSignature.channel                    = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_SIG_CH;
    edmaOut->dataOutSignature.channelShadow              = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_SIG_SHADOW;
    edmaOut->dataOutSignature.eventQueue                 = DPC_OBJDET_DPU_RANGEPROC_EDMAOUT_SIG_EVENT_QUE;

    /* hwres - hwa */
    /* Use ISOLATED mode to support CBUFF in future */
    hwaCfg->dataInputMode = DPU_RangeProcCmpHWA_InputMode_ISOLATED;

#ifdef DPC_USE_SYMMETRIC_WINDOW_RANGE_DPU
    hwaCfg->hwaWinSym = HWA_FFT_WINDOW_SYMMETRIC;
#else
    hwaCfg->hwaWinSym = HWA_FFT_WINDOW_NONSYMMETRIC;
#endif
    hwaCfg->hwaWinRamOffset = (uint16_t) *windowOffset;
    if ((hwaCfg->hwaWinRamOffset + winGenLen) > DPC_OBJDET_HWA_MAX_WINDOW_RAM_SIZE_IN_SAMPLES)
    {
        retVal = DPC_OBJECTDETECTIONCMP_ENOMEM_HWA_WINDOW_RAM;
        goto exit;
    }
    *windowOffset += winGenLen;

    hwaCfg->numParamSet = DPU_RANGEPROCCMPHWA_NUM_HWA_PARAM_SETS;
    hwaCfg->paramSetStartIdx = DPC_OBJDET_DPU_RANGEPROC_PARAMSET_START_IDX;

    retVal = DPU_RangeProcCmpHWA_config(dpuHandle, &rangeCfg);
    if (retVal != 0)
    {
		goto exit;
    }

    /* store configuration for use in intra-sub-frame processing and
     * inter-sub-frame switching, although window will need to be regenerated and
     * dc range sig should not be reset. */
    rangeCfg.staticCfg.resetDcRangeSigMeanBuffer = 0;
    *cfgSave = rangeCfg;

    /* report scratch usage */
    *CoreLocalRamScratchUsage = rangeCfg.staticCfg.windowSize;
exit:

    return retVal;
}

/**
 *  @b Description
 *  @n
 *     Configure Doppler DPU.
 *
 *  @param[in]  dpuHandle Handle to DPU
 *  @param[in]  staticCfg Pointer to static configuration of the sub-frame
 *  @param[in]  log2NumDopplerBins log2 of numDopplerBins of the static config.
 *  @param[in]  dynCfg Pointer to dynamic configuration of the sub-frame
 *  @param[in]  edmaHandle Handle to edma driver to be used for the DPU
 *  @param[in]  radarCube Pointer to DPIF radar cube, which will be the input
 *              to doppler processing
 *  @param[in]  detMatrix Pointer to DPIF detection matrix, which will be the output
 *              of doppler processing
 *  @param[in]  CoreLocalRamObj Pointer to core local RAM object to allocate local memory
 *              for the DPU, only for scratch purposes
 *  @param[in,out]  windowOffset Window coefficients that are generated by this function
 *                               (in heap memory) are passed to DPU configuration API to
 *                               configure the HWA window RAM starting from this offset.
 *                               The end offset after this configuration will be returned
 *                               in this variable which could be the begin offset for the
 *                               next DPU window RAM.
 *  @param[out]  CoreLocalRamScratchUsage Core Local RAM's scratch usage in bytes
 *  @param[out] cfgSave Configuration that is built in local
 *                      (stack) variable is saved here. This is for facilitating
 *                      quick reconfiguration later without having to go through
 *                      the construction of the configuration.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */

static int32_t DPC_ObjDetCmp_dopplerConfig(DPU_DopplerProcDcmpHWA_Handle dpuHandle,
                   DPC_ObjectDetectionCmp_StaticCfg *staticCfg,
                   uint8_t                       log2NumDopplerBins,
                   DPC_ObjectDetectionCmp_DynCfg    *dynCfg,
                   EDMA_Handle                   edmaHandle,
                   DPIF_RadarCube                *radarCube,
                   DPIF_DetMatrix                *detMatrix,
                   MemPoolObj                    *CoreLocalRamObj,
                   uint32_t                      *windowOffset,
                   uint32_t                      *CoreLocalRamScratchUsage,
                   DPU_DopplerProcDcmpHWA_Config     *cfgSave)
{
    int32_t retVal = 0;
    DPU_DopplerProcDcmpHWA_Config dopCfg;
    DPU_DopplerProcDcmpHWA_HW_Resources  *hwRes;
    DPU_DopplerProcDcmpHWA_StaticConfig  *dopStaticCfg;
    DPU_DopplerProcDcmpHWA_EdmaCfg *edmaCfg;
    DPU_DopplerProcDcmpHWA_HwaCfg *hwaCfg;
    uint32_t *windowBuffer, winGenLen, winType;

    hwRes = &dopCfg.hwRes;
    dopStaticCfg = &dopCfg.staticCfg;
    edmaCfg = &hwRes->edmaCfg;
    hwaCfg = &hwRes->hwaCfg;

    memset(&dopCfg, 0, sizeof(dopCfg));

    dopStaticCfg->numDopplerChirps   = staticCfg->numDopplerChirps;
    dopStaticCfg->numDopplerBins     = staticCfg->numDopplerBins;
    dopStaticCfg->numRangeBins       = staticCfg->numRangeBins;
    dopStaticCfg->numRxAntennas      = staticCfg->ADCBufData.dataProperty.numRxAntennas;
    dopStaticCfg->numVirtualAntennas = staticCfg->numVirtualAntennas;
    dopStaticCfg->log2NumDopplerBins = log2NumDopplerBins;
    dopStaticCfg->numTxAntennas      = staticCfg->numTxAntennas;
	dopStaticCfg->compressCfg        = staticCfg->compressCfg;
    /* hwRes */
    hwRes->radarCube = *radarCube;
    hwRes->detMatrix = *detMatrix;

    /* hwRes - edmaCfg */
    edmaCfg->edmaHandle = edmaHandle;

    /* edmaIn - ping */
    edmaCfg->edmaIn.ping.channel =            DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_PING_CH;
    edmaCfg->edmaIn.ping.channelShadow =      DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_PING_SHADOW;
    edmaCfg->edmaIn.ping.eventQueue =         DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_PING_EVENT_QUE;

    /* edmaIn - pong */
    edmaCfg->edmaIn.pong.channel =            DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_PONG_CH;
    edmaCfg->edmaIn.pong.channelShadow =      DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_PONG_SHADOW;
    edmaCfg->edmaIn.pong.eventQueue =         DPC_OBJDET_DPU_DOPPLERPROC_EDMAIN_PONG_EVENT_QUE;

    /* edmaOut - ping */
    edmaCfg->edmaOut.ping.channel =           DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_PING_CH;
    edmaCfg->edmaOut.ping.channelShadow =     DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_PING_SHADOW;
    edmaCfg->edmaOut.ping.eventQueue =        DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_PING_EVENT_QUE;

    /* edmaOut - pong */
    edmaCfg->edmaOut.pong.channel =           DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_PONG_CH;
    edmaCfg->edmaOut.pong.channelShadow =     DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_PONG_SHADOW;
    edmaCfg->edmaOut.pong.eventQueue =        DPC_OBJDET_DPU_DOPPLERPROC_EDMAOUT_PONG_EVENT_QUE;

    /* edmaHotSig - ping */
    edmaCfg->edmaHotSig.ping.channel =        DPC_OBJDET_DPU_DOPPLERPROC_EDMA_PING_SIG_CH;
    edmaCfg->edmaHotSig.ping.channelShadow =  DPC_OBJDET_DPU_DOPPLERPROC_EDMA_PING_SIG_SHADOW;
    edmaCfg->edmaHotSig.ping.eventQueue =     DPC_OBJDET_DPU_DOPPLERPROC_EDMA_PING_SIG_EVENT_QUE;

    /* edmaHotSig - pong */
    edmaCfg->edmaHotSig.pong.channel =        DPC_OBJDET_DPU_DOPPLERPROC_EDMA_PONG_SIG_CH;
    edmaCfg->edmaHotSig.pong.channelShadow =  DPC_OBJDET_DPU_DOPPLERPROC_EDMA_PONG_SIG_SHADOW;
    edmaCfg->edmaHotSig.pong.eventQueue =     DPC_OBJDET_DPU_DOPPLERPROC_EDMA_PONG_SIG_EVENT_QUE;

    /* hwaCfg */
    hwaCfg->numParamSets = 6;
    hwaCfg->paramSetStartIdx = 0; //DPC_OBJDET_DPU_DOPPLERPROC_PARAMSET_START_IDX;

    /* hwaCfg - window */
    winGenLen = DPC_ObjDetCmp_GetDopplerWinGenLen(&dopCfg);
    hwaCfg->windowSize = winGenLen * sizeof(int32_t);
    windowBuffer = DPC_ObjDetCmp_MemPoolAlloc(CoreLocalRamObj, hwaCfg->windowSize, sizeof(uint32_t));
    if (windowBuffer == NULL)
    {
        retVal = DPC_OBJECTDETECTIONCMP_ENOMEM__CORE_LOCAL_RAM_DOPPLER_HWA_WINDOW;
        goto exit;
    }
    hwaCfg->window = (int32_t *)windowBuffer;
    hwaCfg->winRamOffset = (uint16_t) *windowOffset;
    winType = DPC_ObjDetCmp_GenDopplerWindow(&dopCfg);

#ifdef DPC_USE_SYMMETRIC_WINDOW_DOPPLER_DPU
    hwaCfg->winSym = HWA_FFT_WINDOW_SYMMETRIC;
#else
    hwaCfg->winSym = HWA_FFT_WINDOW_NONSYMMETRIC;
#endif
    if ((hwaCfg->winRamOffset + winGenLen) > DPC_OBJDET_HWA_MAX_WINDOW_RAM_SIZE_IN_SAMPLES)
    {
        retVal = DPC_OBJECTDETECTIONCMP_ENOMEM_HWA_WINDOW_RAM;
        goto exit;
    }
    *windowOffset += winGenLen;

    /* Disable first stage scaling if window type is Hanning because Hanning scales
       by half */
    if (winType == MATHUTILS_WIN_HANNING)
    {
        hwaCfg->firstStageScaling = DPU_DOPPLERPROCHWA_FIRST_SCALING_DISABLED;
    }
    else
    {
        hwaCfg->firstStageScaling = DPU_DOPPLERPROCHWA_FIRST_SCALING_ENABLED;
    }

    retVal = DPU_DopplerProcDcmpHWA_config(dpuHandle, &dopCfg);
    if (retVal != 0)
    {
		goto exit;
    }

    /* store configuration for use in intra-sub-frame processing and
     * inter-sub-frame switching, although window will need to be regenerated */
    *cfgSave = dopCfg;

    /* report scratch usage */
    *CoreLocalRamScratchUsage = hwaCfg->windowSize;
exit:

    return retVal;
}

#if 0
/**
 *  @b Description
 *  @n
 *     Configure Static Clutter DPU.
 *
 *  @param[in]  dpuHandle Handle to DPU
 *  @param[in]  staticCfg Pointer to static configuration of the sub-frame
 *  @param[in]  dynCfg Pointer to dynamic configuration of the sub-frame
 *  @param[in]  edmaHandle Handle to edma driver to be used for the DPU
 *  @param[in]  radarCube Pointer to DPIF radar cube, which will be the input and output of
 *              static clutter processing
 *  @param[in]  CoreLocalRamObj Pointer to core local RAM object to allocate local memory
 *              for the DPU, only for scratch purposes
 *  @param[out] CoreLocalRamScratchUsage Core Local RAM's scratch usage in bytes
 *  @param[out] cfgSave Configuration that is built in local
 *                      (stack) variable is saved here. This is for facilitating
 *                      quick reconfiguration later without having to go through
 *                      the construction of the configuration.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static int32_t DPC_ObjDetCmp_staticClutterConfig(DPU_StaticClutterProc_Handle dpuHandle,
                   DPC_ObjectDetectionCmp_StaticCfg *staticCfg,
                   DPC_ObjectDetectionCmp_DynCfg    *dynCfg,
                   EDMA_Handle                   edmaHandle,
                   DPIF_RadarCube                *radarCube,
                   MemPoolObj                    *CoreLocalRamObj,
                   uint32_t                      *CoreLocalRamScratchUsage,
                   DPU_StaticClutterProc_Config  *cfgSave)
{
    int32_t retVal = 0;
    DPU_StaticClutterProc_Config clutCfg;
    DPU_StaticClutterProc_HW_Resources  *hwRes;
    DPU_StaticClutterProc_ScratchBuf scratchBuf;

    hwRes = &clutCfg.hwRes;
    memset(&clutCfg, 0, sizeof(clutCfg));

#ifdef SUBSYS_DSS
    /* staticClutter requires the number of samples to be mulitple of 4 on DSS. */
    scratchBuf.bufSize = NEXT_MULTIPLE_OF(staticCfg->numDopplerChirps , 4) * 2 * sizeof(cmplx16ImRe_t);

    scratchBuf.buf = DPC_ObjDetCmp_MemPoolAlloc(CoreLocalRamObj, scratchBuf.bufSize,
                        DPU_STATICCLUTTERPROC_SCRATCHBUFFER_BYTE_ALIGNMENT_DSP);
#else
    scratchBuf.bufSize = staticCfg->numDopplerChirps * 2 * sizeof(cmplx16ImRe_t);

    scratchBuf.buf = DPC_ObjDetCmp_MemPoolAlloc(CoreLocalRamObj, scratchBuf.bufSize,
                        DPU_STATICCLUTTERPROC_SCRATCHBUFFER_BYTE_ALIGNMENT_R4F);
#endif
    if (scratchBuf.buf == NULL)
    {
        retVal = DPC_OBJECTDETECTIONCMP_ENOMEM__CORE_LOCAL_RAM_STATIC_CLUTTER_SCRATCH;
        goto exit;
    }

    /* static Config */
    clutCfg.staticCfg.numDopplerChirps = staticCfg->numDopplerChirps;
    clutCfg.staticCfg.numRangeBins = staticCfg->numRangeBins;
    clutCfg.staticCfg.numRxAntennas = staticCfg->ADCBufData.dataProperty.numRxAntennas;
    clutCfg.staticCfg.numTxAntennas = staticCfg->numTxAntennas;
    clutCfg.staticCfg.numVirtualAntennas = staticCfg->numVirtualAntennas;

    /* hwRes */
    hwRes->edmaHandle = edmaHandle;
    /* edmaIn - ping */
    hwRes->edmaIn.ping.channel =        DPC_OBJDET_DPU_STATIC_CLUTTER_PROC_EDMAIN_PING_CH;
    hwRes->edmaIn.ping.channelShadow =  DPC_OBJDET_DPU_STATIC_CLUTTER_PROC_EDMAIN_PING_SHADOW;
    hwRes->edmaIn.ping.eventQueue =     DPC_OBJDET_DPU_STATIC_CLUTTER_PROC_EDMAIN_PING_EVENT_QUE;

    /* edmaIn - pong */
    hwRes->edmaIn.pong.channel =        DPC_OBJDET_DPU_STATIC_CLUTTER_PROC_EDMAIN_PONG_CH;
    hwRes->edmaIn.pong.channelShadow =  DPC_OBJDET_DPU_STATIC_CLUTTER_PROC_EDMAIN_PONG_SHADOW;
    hwRes->edmaIn.pong.eventQueue =     DPC_OBJDET_DPU_STATIC_CLUTTER_PROC_EDMAIN_PONG_EVENT_QUE;

    /* edmaOut - ping */
    hwRes->edmaOut.ping.channel =       DPC_OBJDET_DPU_STATIC_CLUTTER_PROC_EDMAOUT_PING_CH;
    hwRes->edmaOut.ping.channelShadow = DPC_OBJDET_DPU_STATIC_CLUTTER_PROC_EDMAOUT_PING_SHADOW;
    hwRes->edmaOut.ping.eventQueue =    DPC_OBJDET_DPU_STATIC_CLUTTER_PROC_EDMAOUT_PING_EVENT_QUE;

    /* edmaOut - pong */
    hwRes->edmaOut.pong.channel =       DPC_OBJDET_DPU_STATIC_CLUTTER_PROC_EDMAOUT_PONG_CH;
    hwRes->edmaOut.pong.channelShadow = DPC_OBJDET_DPU_STATIC_CLUTTER_PROC_EDMAOUT_PONG_SHADOW;
    hwRes->edmaOut.pong.eventQueue =    DPC_OBJDET_DPU_STATIC_CLUTTER_PROC_EDMAOUT_PONG_EVENT_QUE;

    hwRes->radarCube = *radarCube;
    hwRes->scratchBuf = scratchBuf;

    retVal = DPU_StaticClutterProc_config(dpuHandle, &clutCfg);
    if (retVal != 0)
    {
        goto exit;
    }

    /* store configuration for use in intra-sub-frame processing and
     * inter-sub-frame switching, although window will need to be regenerated */
    *cfgSave = clutCfg;

    /* report scratch usage */
    *CoreLocalRamScratchUsage = scratchBuf.bufSize;
exit:

    return retVal;
}
#endif
/**
 *  @b Description
 *  @n
 *     Configure CFARCA DPU.
 *
 *  @param[in]  dpuHandle Handle to DPU
 *  @param[in]  staticCfg Pointer to static configuration of the sub-frame
 *  @param[in]  log2NumDopplerBins log2 of numDopplerBins of the static config.
 *  @param[in]  dynCfg Pointer to dynamic configuration of the sub-frame
 *  @param[in]  edmaHandle Handle to edma driver to be used for the DPU
 *  @param[in]  detMatrix Pointer to DPIF detection matrix, which will be the input
 *              to the CFARCA
 *  @param[in]  cfarRngDopSnrList Pointer to range-doppler SNR list, which will be
 *              the output of CFARCA
 *  @param[in]  cfarRngDopSnrListSize Range-doppler SNR List Size to which the list will be
 *              capped.
 *  @param[in]  CoreLocalRamObj Pointer to core local RAM object to allocate local memory
 *              for the DPU, only for scratch purposes
 *  @param[in]  hwaMemBankAddr pointer to HWA Memory Bank addresses that will be used
 *              to allocate various scratch areas for the DPU processing
 *  @param[in]  hwaMemBankSize Size in bytes of each of HWA memory banks
 *  @param[in]  rangeBias  Range Bias which will be used to adjust fov min value.
 *  @param[out]  CoreLocalRamScratchUsage Core Local RAM's scratch usage in bytes
 *  @param[out] cfgSave Configuration that is built in local
 *                      (stack) variable is saved here. This is for facilitating
 *                      quick reconfiguration later without having to go through
 *                      the construction of the configuration.
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static int32_t DPC_ObjDetCfg_CFARCAconfig(DPU_CFARCAProcHWA_Handle dpuHandle,
                   DPC_ObjectDetectionCmp_StaticCfg *staticCfg,
                   uint8_t                       log2NumDopplerBins,
                   DPC_ObjectDetectionCmp_DynCfg    *dynCfg,
                   EDMA_Handle                   edmaHandle,
                   DPIF_DetMatrix                *detMatrix,
                   DPIF_CFARDetList              *cfarRngDopSnrList,
                   uint32_t                      cfarRngDopSnrListSize,
                   MemPoolObj                    *CoreLocalRamObj,
                   uint32_t                      hwaMemBankAddr[4],
                   uint16_t                      hwaMemBankSize,
                   float                         rangeBias,
                   uint32_t                      *CoreLocalRamScratchUsage,
                   DPU_CFARCAProcHWA_Config      *cfgSave)
{
    int32_t retVal = 0;
    DPU_CFARCAProcHWA_Config cfarCfg;
    DPU_CFARCAProcHWA_HW_Resources *hwRes;
    uint32_t bitMaskCoreLocalRamSize;

    hwRes = &cfarCfg.res;
    memset(&cfarCfg, 0, sizeof(cfarCfg));

    /* static config */
    cfarCfg.staticCfg.log2NumDopplerBins = log2NumDopplerBins;
    cfarCfg.staticCfg.numDopplerBins     = staticCfg->numDopplerBins;
    cfarCfg.staticCfg.numRangeBins       = staticCfg->numRangeBins;
    cfarCfg.staticCfg.rangeStep          = staticCfg->rangeStep;
    cfarCfg.staticCfg.dopplerStep        = staticCfg->dopplerStep;

    /* dynamic config */
    cfarCfg.dynCfg.cfarCfgDoppler = &dynCfg->cfarCfgDoppler;
    cfarCfg.dynCfg.cfarCfgRange   = &dynCfg->cfarCfgRange;
    cfarCfg.dynCfg.fovDoppler     = &dynCfg->fovDoppler;
    cfarCfg.dynCfg.fovRange       = &dynCfg->fovRange;

    /* need to adjust min by range bias */
    cfarCfg.dynCfg.fovRange->min  += rangeBias;

    /* hwres config */
    hwRes->detMatrix = *detMatrix;

    hwRes->edmaHandle = edmaHandle;

    hwRes->edmaHwaIn.channel =                DPC_OBJDET_DPU_CFARCA_PROC_EDMAIN_CH;
    hwRes->edmaHwaIn.channelShadow =          DPC_OBJDET_DPU_CFARCA_PROC_EDMAIN_SHADOW;
    hwRes->edmaHwaIn.eventQueue =             DPC_OBJDET_DPU_CFARCA_PROC_EDMAIN_EVENT_QUE;

    hwRes->edmaHwaInSignature.channel =       DPC_OBJDET_DPU_CFARCA_PROC_EDMAIN_SIG_CH;
    hwRes->edmaHwaInSignature.channelShadow = DPC_OBJDET_DPU_CFARCA_PROC_EDMAIN_SIG_SHADOW;
    hwRes->edmaHwaInSignature.eventQueue =    DPC_OBJDET_DPU_CFARCA_PROC_EDMAIN_SIG_EVENT_QUE;

    hwRes->hwaCfg.numParamSet = DPU_CFARCAPROCHWA_NUM_HWA_PARAM_SETS;
    hwRes->hwaCfg.paramSetStartIdx = DPC_OBJDET_DPU_CFARCA_PROC_PARAMSET_START_IDX(staticCfg->numTxAntennas);

    /* Give M0 and M1 memory banks for detection matrix scratch. */
    hwRes->hwaMemInp = (uint16_t *) hwaMemBankAddr[0];
    hwRes->hwaMemInpSize = (hwaMemBankSize * 2) / sizeof(uint16_t);

    /* Entire M2 bank for doppler output */
    hwRes->hwaMemOutDoppler = (DPU_CFARCAProcHWA_CfarDetOutput *) hwaMemBankAddr[2];
    hwRes->hwaMemOutDopplerSize = hwaMemBankSize /
                                  sizeof(DPU_CFARCAProcHWA_CfarDetOutput);

    /* Entire M3 bank for range output */
    hwRes->hwaMemOutRange = (DPU_CFARCAProcHWA_CfarDetOutput *) hwaMemBankAddr[3];
    hwRes->hwaMemOutRangeSize = hwaMemBankSize /
                                sizeof(DPU_CFARCAProcHWA_CfarDetOutput);

    hwRes->cfarDopplerDetOutBitMaskSize = (staticCfg->numRangeBins *
        staticCfg->numDopplerBins) / 32;
    bitMaskCoreLocalRamSize = hwRes->cfarDopplerDetOutBitMaskSize * sizeof(uint32_t);
    hwRes->cfarDopplerDetOutBitMask = (uint32_t *) DPC_ObjDetCmp_MemPoolAlloc(CoreLocalRamObj,
                                                       bitMaskCoreLocalRamSize,
                                                       DPU_CFARCAPROCHWA_DOPPLER_DET_OUT_BIT_MASK_BYTE_ALIGNMENT);
    if (hwRes->cfarDopplerDetOutBitMask == NULL)
    {
        retVal = DPC_OBJECTDETECTIONCMP_ENOMEM__CORE_LOCAL_RAM_CFARCA_DOPPLER_DET_OUT_BIT_MASK;
        goto exit;
    }

    hwRes->cfarRngDopSnrList = cfarRngDopSnrList;
    hwRes->cfarRngDopSnrListSize = cfarRngDopSnrListSize;

    retVal = DPU_CFARCAProcHWA_config(dpuHandle, &cfarCfg);
    if (retVal != 0)
    {
        goto exit;
    }

    /* store configuration for use in intra-sub-frame processing and
     * inter-sub-frame switching, although window will need to be regenerated */
    *cfgSave = cfarCfg;

    /* report scratch usage */
    *CoreLocalRamScratchUsage = bitMaskCoreLocalRamSize;

exit:
        return retVal;
}

/**
 *  @b Description
 *  @n
 *     Configure AoA DPU. Note window information is passed to this function that
 *     is expected to be the same that was used in doppler processing because
 *     the AoA recomputes the doppler (2D) FFT. The reason for this recompute
 *     is because doppler does not update the radar cube (radar cube is range (1D) output).
 *     The window is used in AoA only for doppler FFT recompute, no window
 *     (in other words rectangular) is used for angle (3D) FFT computation.
 *     Note no DPIF info is passed here for AoA output because these DPIF buffers
 *     are not required to be shared with any other DPUs (but only at the exit
 *     f DPC's processing chain which will be consumed by the app) hence they are
 *     allocated within this function, this way more allocation remains localized
 *     in this function instead of being managed by the caller.
 *
 *  @param[in]  dpuHandle Handle to DPU
 *  @param[in]  inpCommonCompRxCfg Range bias and rx phase compensation common (across sub-frames)
 *              configuration, this will be used to extract the rx phase for sub-frame specific
 *              configuration based on antenna order.
 *  @param[in]  staticCfg Pointer to static configuration of the sub-frame
 *  @param[in]  dynCfg Pointer to dynamic configuration of the sub-frame
 *  @param[in]  edmaHandle Handle to edma driver to be used for the DPU
 *  @param[in]  radarCube Pointer to DPIF radar cube, which will be the
 *              input for AoA processing
 *  @param[in]  cfarRngDopSnrList Pointer to range-doppler SNR list, which will be
 *              input for AoA processing
 *  @param[in]  cfarRngDopSnrListSize Range-doppler SNR List Size to which the list
 *              was capped by cfar processing
 *  @param[in]  CoreLocalRamObj Pointer to core local RAM object to allocate local memory
 *              for the DPU, all allocated memory will be permanent (within frame/sub-frame)
 *  @param[in]  L3RamObj Pointer to L3 RAM object to allocate L3RAM memory
 *              for the DPU, all allocated memory will be permanent (within frame/sub-frame)
 *  @param[in]  dopplerWindowSym Flag to indicate if HWA windowing is symmetric
 *                               see HWA_WINDOW_SYMM definitions in HWA driver's doxygen documentation
 *  @param[in]  dopplerWinSize Doppler FFT window size in bytes. See doppler DPU
 *                             configuration for more information.
 *  @param[in]  dopplerWindow Pointer to doppler FFT window coefficients
 *  @param[in]  dopplerWinRamOffset HWA window RAM offset of doppler FFT
 *  @param[in]  cfarParamSetStartIdx  Start index of the cfar param set, will be used
 *                                    as the start of AoA's param set if overlap with
 *                                    CFAR is needed based on configuration.
 *  @param[out]  isAoAHWAparamSetOverlappedWithCFAR true if AoA's param set overlaps
 *               with CFAR (depends on the configuration input)
 *  @param[out] cfgSave Configuration that is built in local
 *                      (stack) variable is saved here. This is for facilitating
 *                      quick reconfiguration later without having to go through
 *                      the construction of the configuration.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static int32_t DPC_ObjDetCmp_AoAconfig(DPU_AoAProcDcmpHWA_Handle dpuHandle,
                   DPU_AoAProcDcmp_compRxChannelBiasCfg *inpCommonCompRxCfg,
                   DPC_ObjectDetectionCmp_StaticCfg    *staticCfg,
                   DPC_ObjectDetectionCmp_DynCfg       *dynCfg,
                   EDMA_Handle                      edmaHandle,
                   DPIF_RadarCube                   *radarCube,
                   DPIF_CFARDetList                 *cfarRngDopSnrList,
                   uint32_t                         cfarRngDopSnrListSize,
                   MemPoolObj                       *CoreLocalRamObj,
                   MemPoolObj                       *L3RamObj,

                   /* doppler window parameters */
                   uint8_t                          dopplerWindowSym,
                   uint32_t                         dopplerWinSize,
                   int32_t                          *dopplerWindow,
                   uint32_t                         dopplerWinRamOffset,

                   uint8_t                          cfarParamSetStartIdx,
                   bool                             *isAoAHWAparamSetOverlappedWithCFAR,
                   DPU_AoAProcDcmpHWA_Config            *cfgSave)
{
    int32_t retVal = 0;
    DPU_AoAProcDcmpHWA_Config aoaCfg;
    DPU_AoAProcDcmpHWA_HW_Resources *res;
    DPU_AoAProcDcmp_compRxChannelBiasCfg outCompRxCfg;
    int32_t i;

    res = &aoaCfg.res;
    memset(&aoaCfg, 0, sizeof(aoaCfg));

    /* Static config */
    aoaCfg.staticCfg.numDopplerChirps   = staticCfg->numDopplerChirps;
    aoaCfg.staticCfg.numDopplerBins     = staticCfg->numDopplerBins;
    aoaCfg.staticCfg.numRangeBins       = staticCfg->numRangeBins;
    aoaCfg.staticCfg.numRxAntennas      = staticCfg->ADCBufData.dataProperty.numRxAntennas;
    aoaCfg.staticCfg.dopplerStep        = staticCfg->dopplerStep;
    aoaCfg.staticCfg.rangeStep          = staticCfg->rangeStep;
    aoaCfg.staticCfg.numTxAntennas      = staticCfg->numTxAntennas;
    aoaCfg.staticCfg.numVirtualAntAzim  = staticCfg->numVirtualAntAzim;
    aoaCfg.staticCfg.numVirtualAntElev  = staticCfg->numVirtualAntElev;
	aoaCfg.staticCfg.compressCfg 		= staticCfg->compressCfg;
	/* dynamic config */
    DPC_ObjDetCmp_GetRxChPhaseComp(staticCfg, inpCommonCompRxCfg, &outCompRxCfg);
    aoaCfg.dynCfg.compRxChanCfg              = &outCompRxCfg;
    aoaCfg.dynCfg.fovAoaCfg                  = &dynCfg->fovAoaCfg;
    aoaCfg.dynCfg.multiObjBeamFormingCfg     = &dynCfg->multiObjBeamFormingCfg;
    aoaCfg.dynCfg.prepareRangeAzimuthHeatMap = dynCfg->prepareRangeAzimuthHeatMap;
    aoaCfg.dynCfg.extMaxVelCfg               = &dynCfg->extMaxVelCfg;

    /* res */
    res->radarCube = *radarCube;
    res->cfarRngDopSnrList = cfarRngDopSnrList;
    res->cfarRngDopSnrListSize = cfarRngDopSnrListSize;

    res->detObjOutMaxSize = DPC_OBJDET_MAX_NUM_OBJECTS;

    res->detObjOut = DPC_ObjDetCmp_MemPoolAlloc(CoreLocalRamObj,
                         res->detObjOutMaxSize *sizeof(DPIF_PointCloudCartesian),
                         DPC_OBJDET_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT);
    if (res->detObjOut == NULL)
    {
        retVal = DPC_OBJECTDETECTIONCMP_ENOMEM__CORE_LOCAL_RAM_AOA_DET_OBJ_OUT;
        goto exit;
    }

    res->detObjOutSideInfo = DPC_ObjDetCmp_MemPoolAlloc(CoreLocalRamObj,
                                 res->detObjOutMaxSize *sizeof(DPIF_PointCloudSideInfo),
                                 DPC_OBJDET_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT);
    if (res->detObjOutSideInfo == NULL)
    {
        retVal = DPC_OBJECTDETECTIONCMP_ENOMEM__CORE_LOCAL_RAM_AOA_DET_OBJ_OUT_SIDE_INFO;
        goto exit;
    }

    res->detObj2dAzimIdx = DPC_ObjDetCmp_MemPoolAlloc(CoreLocalRamObj,
                               res->detObjOutMaxSize *sizeof(uint8_t),
                               1);
    if (res->detObj2dAzimIdx == NULL)
    {
        retVal = DPC_OBJECTDETECTIONCMP_ENOMEM__CORE_LOCAL_RAM_AOA_DET_OBJ_2_AZIM_IDX;
        goto exit;
    }

    res->detObjElevationAngle = DPC_ObjDetCmp_MemPoolAlloc(CoreLocalRamObj,
                                    res->detObjOutMaxSize *sizeof(float),
                                    DPC_OBJDET_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT);
    if (res->detObjElevationAngle == NULL)
    {
        retVal = DPC_OBJECTDETECTIONCMP_ENOMEM__CORE_LOCAL_RAM_AOA_DET_OBJ_ELEVATION_ANGLE;
        goto exit;
    }

	/* Allocate buffers for ping and pong paths: */
    res->localScratchBufferSizeBytes = DPU_AOAPROCDCMPHWA_NUM_LOCAL_SCRATCH_BUFFER_SIZE_BYTES(aoaCfg.staticCfg.numTxAntennas);
    for (i = 0; i < DPU_AOAPROCHWA_NUM_LOCAL_SCRATCH_BUFFERS; i++)
    {
        res->localScratchBuffer[i] = DPC_ObjDetCmp_MemPoolAlloc(CoreLocalRamObj,
                                         res->localScratchBufferSizeBytes,
                                         DPU_AOAPROCHWA_LOCAL_SCRATCH_BYTE_ALIGNMENT);
       if (res->localScratchBuffer[i] == NULL)
       {
           retVal = DPC_OBJECTDETECTIONCMP_ENOMEM__CORE_LOCAL_RAM_AOA_SCRATCH_BUFFER;
           goto exit;
       }
    }

    if(aoaCfg.dynCfg.prepareRangeAzimuthHeatMap)
    {
        res->azimuthStaticHeatMapSize = staticCfg->numRangeBins * staticCfg->numVirtualAntAzim;
#if defined(SUBSYS_MSS)
        res->azimuthStaticHeatMap = DPC_ObjDetCmp_MemPoolAlloc(CoreLocalRamObj,
                                         res->azimuthStaticHeatMapSize *sizeof(cmplx16ImRe_t),
                                         DPC_OBJDET_AZIMUTH_STATIC_HEAT_MAP_BYTE_ALIGNMENT);
#elif  defined(SUBSYS_DSS)
        res->azimuthStaticHeatMap = DPC_ObjDetCmp_MemPoolAlloc(L3RamObj,
                                         res->azimuthStaticHeatMapSize *sizeof(cmplx16ImRe_t),
                                         DPC_OBJDET_AZIMUTH_STATIC_HEAT_MAP_BYTE_ALIGNMENT);
#else
#error "Error: Unknown subsystem"
#endif
        if (res->azimuthStaticHeatMap == NULL)
        {
            retVal = DPC_OBJECTDETECTIONCMP_ENOMEM__CORE_LOCAL_RAM_AOA_AZIMUTH_STATIC_HEAT_MAP;
            goto exit;
        }
    }

    res->edmaHandle = edmaHandle;
    /* For Azimuth Heatmap ping/pong paths */
    res->edmaHwa[0].in.channel =                DPC_OBJDET_DPU_AOA_PROC_EDMA_CH_0;
    res->edmaHwa[0].in.channelShadow =          DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_0;
    res->edmaHwa[0].in.eventQueue =             DPC_OBJDET_DPU_AOA_PROC_EDMAIN_PING_EVENT_QUE;
    res->edmaHwa[0].inSignature.channel =       DPC_OBJDET_DPU_AOA_PROC_EDMA_CH_1;
    res->edmaHwa[0].inSignature.channelShadow = DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_1;
    res->edmaHwa[0].inSignature.eventQueue =    DPC_OBJDET_DPU_AOA_PROC_EDMAIN_PING_EVENT_QUE;
    res->edmaHwa[0].out.channel =               DPC_OBJDET_DPU_AOA_PROC_EDMA_HWA_OUTPUT_CH_0;
    res->edmaHwa[0].out.channelShadow =         DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_2;
    res->edmaHwa[0].out.eventQueue =            DPC_OBJDET_DPU_AOA_PROC_EDMAOUT_PING_EVENT_QUE;

    res->edmaHwa[1].in.channel =                DPC_OBJDET_DPU_AOA_PROC_EDMA_CH_2;
    res->edmaHwa[1].in.channelShadow =          DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_3;
    res->edmaHwa[1].in.eventQueue =             DPC_OBJDET_DPU_AOA_PROC_EDMAIN_PONG_EVENT_QUE;
    res->edmaHwa[1].inSignature.channel =       DPC_OBJDET_DPU_AOA_PROC_EDMA_CH_3;
    res->edmaHwa[1].inSignature.channelShadow = DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_4;
    res->edmaHwa[1].inSignature.eventQueue =    DPC_OBJDET_DPU_AOA_PROC_EDMAIN_PONG_EVENT_QUE;
    res->edmaHwa[1].out.channel =               DPC_OBJDET_DPU_AOA_PROC_EDMA_HWA_OUTPUT_CH_1;
    res->edmaHwa[1].out.channelShadow =         DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_5;
    res->edmaHwa[1].out.eventQueue =            DPC_OBJDET_DPU_AOA_PROC_EDMAOUT_PONG_EVENT_QUE;

    /* For main data processing ping/pong paths */
    res->edmaHwaExt[0].chIn.channel =               DPC_OBJDET_DPU_AOA_PROC_EDMA_CH_0;
    res->edmaHwaExt[0].chIn.eventQueue =            DPC_OBJDET_DPU_AOA_PROC_EDMAIN_PING_EVENT_QUE;
    res->edmaHwaExt[0].chOut.channel =              DPC_OBJDET_DPU_AOA_PROC_EDMA_HWA_OUTPUT_CH_0;
    res->edmaHwaExt[0].chOut.eventQueue =           DPC_OBJDET_DPU_AOA_PROC_EDMAOUT_PING_EVENT_QUE;
    res->edmaHwaExt[0].stage[0].paramIn =           DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_0;
    res->edmaHwaExt[0].stage[0].paramInSignature =  DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_1;
    res->edmaHwaExt[0].stage[0].paramOut =          DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_2;
    res->edmaHwaExt[0].stage[1].paramIn =           DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_3;
    res->edmaHwaExt[0].stage[1].paramInSignature =  DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_4;
    res->edmaHwaExt[0].stage[1].paramOut =          DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_5;
    res->edmaHwaExt[0].eventQueue = 0;

    res->edmaHwaExt[1].chIn.channel =               DPC_OBJDET_DPU_AOA_PROC_EDMA_CH_1;
    res->edmaHwaExt[1].chIn.eventQueue =            DPC_OBJDET_DPU_AOA_PROC_EDMAIN_PONG_EVENT_QUE;
    res->edmaHwaExt[1].chOut.channel =              DPC_OBJDET_DPU_AOA_PROC_EDMA_HWA_OUTPUT_CH_1;
    res->edmaHwaExt[1].chOut.eventQueue =           DPC_OBJDET_DPU_AOA_PROC_EDMAOUT_PONG_EVENT_QUE;
    res->edmaHwaExt[1].stage[0].paramIn =           DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_6;
    res->edmaHwaExt[1].stage[0].paramInSignature =  DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_7;
    res->edmaHwaExt[1].stage[0].paramOut =          DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_8;
    res->edmaHwaExt[1].stage[1].paramIn =           DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_9;
    res->edmaHwaExt[1].stage[1].paramInSignature =  DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_10;
    res->edmaHwaExt[1].stage[1].paramOut =          DPC_OBJDET_DPU_AOA_PROC_EDMA_VIRT_CH_11;
    res->edmaHwaExt[1].eventQueue = 0;

    res->hwaCfg.numParamSet = 2*(2 + 2 + ((staticCfg->numVirtualAntElev > 0)*1)); /**/

    res->hwaCfg.paramSetStartIdx = 0;
    *isAoAHWAparamSetOverlappedWithCFAR = true;

    res->hwaCfg.window = dopplerWindow;
    res->hwaCfg.winSym = dopplerWindowSym;
    res->hwaCfg.winRamOffset = dopplerWinRamOffset;
    res->hwaCfg.windowSize = dopplerWinSize;

    retVal = DPU_AoAProcDcmpHWA_config(dpuHandle, &aoaCfg);
    if (retVal != 0)
    {
        goto exit;
    }

    /* store configuration for use in intra-sub-frame processing and
     * inter-sub-frame switching, although window and compRx will need to be regenerated */
    *cfgSave = aoaCfg;

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *     Performs processing related to pre-start configuration, which is per sub-frame,
 *     by configuring each of the DPUs involved in the processing chain.
 *  Memory management notes:
 *  1. Core Local Memory that needs to be preserved across sub-frames (such as range DPU's calib DC buffer)
 *     will be allocated using MemoryP_alloc.
 *  2. Core Local Memory that needs to be preserved within a sub-frame across DPU calls
 *     (the DPIF * type memory) or for intermediate private scratch memory for
 *     DPU (i.e no preservation is required from process call to process call of the DPUs
 *     within the sub-frame) will be allocated from the Core Local RAM configuration supplied in
 *     @ref DPC_ObjectDetectionCmp_InitParams given to @ref DPC_ObjectDetectionCmp_init API
 *  3. L3 memory will only be allocated from the L3 RAM configuration supplied in
 *     @ref DPC_ObjectDetectionCmp_InitParams given to @ref DPC_ObjectDetectionCmp_init API
 *     No L3 buffers are presently required that need to be preserved across sub-frames
 *     (type described in #1 above), neither are L3 scratch buffers required for
 *     intermediate processing within DPU process call.
 *
 *  @param[in]  obj Pointer to sub-frame object
 *  @param[in]  commonCfg Pointer to pre-start common configuration
 *  @param[in]  staticCfg Pointer to static configuration of the sub-frame
 *  @param[in]  dynCfg Pointer to dynamic configuration of the sub-frame
 *  @param[in]  edmaHandle Pointer to array of EDMA handles for the device, this
 *              can be distributed among the DPUs, although presently we only
 *              use the first handle for all DPUs.
 *  @param[in]  L3ramObj Pointer to L3 RAM memory pool object
 *  @param[in]  CoreLocalRamObj Pointer to Core Local RAM memory pool object
 *  @param[in]  hwaMemBankAddr pointer to HWA Memory Bank addresses that will be used
 *              to allocate various scratch areas for the DPU processing
 *  @param[in]  hwaMemBankSize Size in bytes of each of HWA memory banks
 *  @param[out] L3RamUsage Net L3 RAM memory usage in bytes as a result of allocation
 *              by the DPUs.
 *  @param[out] CoreLocalRamUsage Net Core Local RAM memory usage in bytes as a
 *              result of allocation by the DPUs.
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 */
static int32_t DPC_ObjDetCmp_preStartConfig(SubFrameObj *obj,
                  DPC_ObjectDetectionCmp_PreStartCommonCfg *commonCfg,
                   DPC_ObjectDetectionCmp_StaticCfg *staticCfg,
                   DPC_ObjectDetectionCmp_DynCfg    *dynCfg,
                   EDMA_Handle                   edmaHandle[EDMA_NUM_CC],
                   MemPoolObj                    *L3ramObj,
                   MemPoolObj                    *CoreLocalRamObj,
                   uint32_t                      hwaMemBankAddr[4],
                   uint16_t                      hwaMemBankSize,
                   uint32_t                      *L3RamUsage,
                   uint32_t                      *CoreLocalRamUsage)
{
    int32_t retVal = 0;
    DPIF_RadarCube radarCube;
    DPIF_DetMatrix detMatrix;
    uint32_t hwaWindowOffset;
    uint32_t rangeCoreLocalRamScratchUsage, dopplerCoreLocalRamScratchUsage, 
	cfarCoreLocalRamScratchUsage;
    DPIF_CFARDetList *cfarRngDopSnrList;
    uint32_t cfarRngDopSnrListSize;
    void *CoreLocalScratchStartPoolAddr;

    /* save configs to object. We need to pass this stored config (instead of
       the input arguments to this function which will be in stack) to
       the DPU config functions inside of this function because the DPUs
       have pointers to dynamic configurations which are later going to be
       reused during re-configuration (intra sub-frame or inter sub-frame)
     */
    obj->staticCfg = *staticCfg;
    obj->dynCfg = *dynCfg;

    hwaWindowOffset = DPC_OBJDET_HWA_WINDOW_RAM_OFFSET;

    /* derived config */
    obj->log2NumDopplerBins = mathUtils_floorLog2(staticCfg->numDopplerBins);

    DPC_ObjDetCmp_MemPoolReset(L3ramObj);
    DPC_ObjDetCmp_MemPoolReset(CoreLocalRamObj);

    /* Allocate DPIF stuff (intra sub-frame buffers) first, except the last AoA output
     * DPIF stuff (see comments before call to AoA's config in this function */

    /* L3 allocations */
    /* L3 - radar cube */
    radarCube.dataSize = (staticCfg->numRangeBins * staticCfg->numDopplerChirps *
                         staticCfg->numVirtualAntennas * sizeof(cmplx16ReIm_t) * staticCfg->compressCfg.ratio) >> HWA_CMP_RATIO_BW ;
    radarCube.data = DPC_ObjDetCmp_MemPoolAlloc(L3ramObj, radarCube.dataSize,
                                             DPC_OBJDET_RADAR_CUBE_DATABUF_BYTE_ALIGNMENT);
    if (radarCube.data == NULL)
    {
        retVal = DPC_OBJECTDETECTIONCMP_ENOMEM__L3_RAM_RADAR_CUBE;
		goto exit;
    }
    radarCube.datafmt = DPIF_RADARCUBE_FORMAT_1;

    /* L3 - detection matrix */
    detMatrix.dataSize = staticCfg->numRangeBins * staticCfg->numDopplerBins * sizeof(uint16_t);
    detMatrix.data = DPC_ObjDetCmp_MemPoolAlloc(L3ramObj, detMatrix.dataSize,
                                             DPC_OBJDET_DET_MATRIX_DATABUF_BYTE_ALIGNMENT);
    if (detMatrix.data == NULL)
    {
        retVal = DPC_OBJECTDETECTIONCMP_ENOMEM__L3_RAM_DET_MATRIX;
        goto exit;
    }
    detMatrix.datafmt = DPIF_DETMATRIX_FORMAT_1;

    /* Core Local - CFAR output list */
    cfarRngDopSnrListSize = DPC_OBJDET_MAX_NUM_OBJECTS;

    cfarRngDopSnrList = DPC_ObjDetCmp_MemPoolAlloc(CoreLocalRamObj,
                            cfarRngDopSnrListSize * sizeof(DPIF_CFARDetList),
                            DPC_OBJDET_CFAR_DET_LIST_BYTE_ALIGNMENT);
    if (cfarRngDopSnrList == NULL)
    {
		retVal = DPC_OBJECTDETECTIONCMP_ENOMEM__CORE_LOCAL_RAM_CFARCA_OUT_DET_LIST;
		
        goto exit;
    }

    /* Remember pool position */
    CoreLocalScratchStartPoolAddr = DPC_ObjDetCmp_MemPoolGet(CoreLocalRamObj);

    retVal = DPC_ObjDetCmp_rangeConfig(obj->dpuRangeObj, &obj->staticCfg, &obj->dynCfg,
                 edmaHandle[DPC_OBJDET_DPU_RANGEPROC_EDMA_INST_ID],
                 &radarCube, CoreLocalRamObj, &hwaWindowOffset,
                 &rangeCoreLocalRamScratchUsage, &obj->dpuCfg.rangeCfg);
    if (retVal != 0)
    {
		
        goto exit;
    }

    /* Rewind to the scratch beginning */
    DPC_ObjDetCmp_MemPoolSet(CoreLocalRamObj, CoreLocalScratchStartPoolAddr);


    retVal = DPC_ObjDetCfg_CFARCAconfig(obj->dpuCFARCAObj, &obj->staticCfg,
                 obj->log2NumDopplerBins, &obj->dynCfg,
                 edmaHandle[DPC_OBJDET_DPU_CFARCA_PROC_EDMA_INST_ID],
                 &detMatrix,
                 cfarRngDopSnrList,
                 cfarRngDopSnrListSize,
                 CoreLocalRamObj,
                 &hwaMemBankAddr[0],
                 hwaMemBankSize,
                 commonCfg->compRxChanCfg.rangeBias,
                 &cfarCoreLocalRamScratchUsage,
                 &obj->dpuCfg.cfarCfg);
    if (retVal != 0)
    {
		
        goto exit;
    }

    /* Rewind to the scratch beginning */
    DPC_ObjDetCmp_MemPoolSet(CoreLocalRamObj, CoreLocalScratchStartPoolAddr);

    /* Note doppler will generate window (that will be used by AoA next)
     * in core local scratch memory, so scratch should not be reset after this point
     * (AoA itself does not need scratch) */
    retVal = DPC_ObjDetCmp_dopplerConfig(obj->dpuDopplerObj, &obj->staticCfg,
                 obj->log2NumDopplerBins, &obj->dynCfg,
                 edmaHandle[DPC_OBJDET_DPU_DOPPLERPROC_EDMA_INST_ID],
                 &radarCube, &detMatrix, CoreLocalRamObj, &hwaWindowOffset,
                 &dopplerCoreLocalRamScratchUsage, &obj->dpuCfg.dopplerCfg);
    if (retVal != 0)
    {
		goto exit;
    }

    /* Presently AoA does not use Core Local scratch because window is fed from doppler above
     * and all its allocation is persistent within sub-frame processing. Given also that AoA
     * is the last module to be called, its DPIF type buffers can be overlaid with
     * scratch buffers used in previous modules in the processing chain. So unlike radarCube
     * and detMatrix, the DPIF buffers of AoA don't need to be allocated up-front like
     * radarCube and detMatrix. There are also some debug buffers in AoA that are tied
     * to the DPIF which are conveniently localized in this function.
     * Given we are feeding doppler window generated in the dopplerConfig call above,
     * we cannot reset the Core Local RAM to the scratchStartPoolAddr.
     */
    retVal = DPC_ObjDetCmp_AoAconfig(obj->dpuAoAObj, &commonCfg->compRxChanCfg,
                 &obj->staticCfg, &obj->dynCfg,
                 edmaHandle[DPC_OBJDET_DPU_AOA_PROC_EDMA_INST_ID],
                 &radarCube,
                 cfarRngDopSnrList, cfarRngDopSnrListSize,
                 CoreLocalRamObj,
                 L3ramObj,
                 obj->dpuCfg.dopplerCfg.hwRes.hwaCfg.winSym,
                 obj->dpuCfg.dopplerCfg.hwRes.hwaCfg.windowSize,
                 obj->dpuCfg.dopplerCfg.hwRes.hwaCfg.window,
                 obj->dpuCfg.dopplerCfg.hwRes.hwaCfg.winRamOffset,
                 DPC_OBJDET_DPU_CFARCA_PROC_PARAMSET_START_IDX(staticCfg->numTxAntennas),
                 &obj->isAoAHWAparamSetOverlappedWithCFAR,
                 &obj->dpuCfg.aoaCfg);

    if (retVal != 0)
    {
		
        goto exit;
    }

    /* Report RAM usage */
    *CoreLocalRamUsage = DPC_ObjDetCmp_MemPoolGetMaxUsage(CoreLocalRamObj);
    *L3RamUsage = DPC_ObjDetCmp_MemPoolGetMaxUsage(L3ramObj);

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      DPC IOCTL commands configuration API which will be invoked by the
 *      application using DPM_ioctl API
 *
 *  @param[in]  handle   DPM's DPC handle
 *  @param[in]  cmd      Capture DPC specific commands
 *  @param[in]  arg      Command specific arguments
 *  @param[in]  argLen   Length of the arguments which is also command specific
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t DPC_ObjectDetectionCmp_ioctl
(
    DPM_DPCHandle   handle,
    uint32_t            cmd,
    void*               arg,
    uint32_t            argLen
)
{
    ObjDetCmpObj   *objDetCmpObj;
    SubFrameObj *subFrmObj;
    int32_t      retVal = 0;

    /* Get the DSS MCB: */
    objDetCmpObj = (ObjDetCmpObj *) handle;
    DebugP_assert(objDetCmpObj != NULL);

	/* Process the commands. Process non sub-frame specific ones first
     * so the sub-frame specific ones can share some code. */
    if (cmd == DPC_OBJDET_IOCTL__TRIGGER_FRAME)
    {
        DPC_ObjectDetectionCmp_frameStart(handle);
    }
    else if (cmd == DPC_OBJDET_IOCTL__STATIC_PRE_START_COMMON_CFG)
    {
		
        DPC_ObjectDetectionCmp_PreStartCommonCfg *cfg;
        int32_t indx;

        DebugP_assert(argLen == sizeof(DPC_ObjectDetectionCmp_PreStartCommonCfg));

        cfg = (DPC_ObjectDetectionCmp_PreStartCommonCfg*)arg;

        /* Free all buffers that were allocated from system (MemoryP) heap.
         * Note we cannot free buffers during allocation time
         * for new config during the pre-start config processing because the heap is not capable
         * of defragmentation. This means pre-start common config must precede
         * all pre-start configs. */
        for(indx = 0; indx < objDetCmpObj->commonCfg.numSubFrames; indx++)
        {
            subFrmObj = &objDetCmpObj->subFrameObj[indx];

            if (subFrmObj->dpuCfg.rangeCfg.hwRes.dcRangeSigMean)
            {
                MemoryP_ctrlFree(subFrmObj->dpuCfg.rangeCfg.hwRes.dcRangeSigMean,
                                 subFrmObj->dpuCfg.rangeCfg.hwRes.dcRangeSigMeanSize);
            }
        }

        objDetCmpObj->commonCfg = *cfg;
        objDetCmpObj->isCommonCfgReceived = true;

        DebugP_log0("ObjDetCmp DPC: Pre-start Common Config IOCTL processed\n");
    }
    else if (cmd == DPC_OBJDET_IOCTL__DYNAMIC_MEASURE_RANGE_BIAS_AND_RX_CHAN_PHASE)
    {
		
        DPC_ObjectDetectionCmp_MeasureRxChannelBiasCfg *cfg;

        DebugP_assert(argLen == sizeof(DPC_ObjectDetectionCmp_MeasureRxChannelBiasCfg));

        cfg = (DPC_ObjectDetectionCmp_MeasureRxChannelBiasCfg*)arg;

        retVal = DPC_ObjDetCmp_Config_MeasureRxChannelBiasCfg(objDetCmpObj, cfg);
        if (retVal != 0)
        {
			
            goto exit;
        }
    }
    else if (cmd == DPC_OBJDET_IOCTL__DYNAMIC_COMP_RANGE_BIAS_AND_RX_CHAN_PHASE)
    {
		
        DPU_AoAProcDcmp_compRxChannelBiasCfg *inpCfg;
        DPU_AoAProcDcmp_compRxChannelBiasCfg outCfg;
        int32_t i;

        DebugP_assert(argLen == sizeof(DPU_AoAProcDcmp_compRxChannelBiasCfg));

        inpCfg = (DPU_AoAProcDcmp_compRxChannelBiasCfg*)arg;

        for(i = 0; i < objDetCmpObj->commonCfg.numSubFrames; i++)
        {
            subFrmObj = &objDetCmpObj->subFrameObj[i];

            DPC_ObjDetCmp_GetRxChPhaseComp(&subFrmObj->staticCfg, inpCfg, &outCfg);

            retVal = DPU_AoAProcDcmpHWA_control(subFrmObj->dpuAoAObj,
                     DPU_AoAProcDcmpHWA_Cmd_CompRxChannelBiasCfg,
                     &outCfg,
                     sizeof(DPU_AoAProcDcmp_compRxChannelBiasCfg));
            if (retVal != 0)
            {
				
                goto exit;
            }
        }

        /* save into object */
        objDetCmpObj->commonCfg.compRxChanCfg = *inpCfg;
    }
    else if (cmd == DPC_OBJDET_IOCTL__DYNAMIC_EXECUTE_RESULT_EXPORTED)
    {
		
        DPC_ObjectDetectionCmp_ExecuteResultExportedInfo *inp;
        volatile uint32_t startTime;

        startTime = Cycleprofiler_getTimeStamp();

        DebugP_assert(argLen == sizeof(DPC_ObjectDetectionCmp_ExecuteResultExportedInfo));

        inp = (DPC_ObjectDetectionCmp_ExecuteResultExportedInfo *)arg;

        /* input sub-frame index must match current sub-frame index */
        DebugP_assert(inp->subFrameIdx == objDetCmpObj->subFrameIndx);

        /* Reconfigure all Range DPUs resources for next frame as all HWA and EDMA
         * resources overlap across DPUs */
        {
            /* Next sub-frame */
            objDetCmpObj->subFrameIndx++;
            if (objDetCmpObj->subFrameIndx == objDetCmpObj->commonCfg.numSubFrames)
            {
                objDetCmpObj->subFrameIndx = 0;
            }

            DPC_ObjDetCmp_reconfigSubFrame(objDetCmpObj, objDetCmpObj->subFrameIndx);
        }

        subFrmObj = &objDetCmpObj->subFrameObj[objDetCmpObj->subFrameIndx];

        /* Trigger Range DPU */
        retVal = DPU_RangeProcCmpHWA_control(subFrmObj->dpuRangeObj,
                     DPU_RangeProcCmpHWA_Cmd_triggerProc, NULL, 0);
        if(retVal < 0)
        {
            goto exit;
        }

        DebugP_log0("ObjDetCmp DPC: Range Proc Triggered in export IOCTL\n");

        objDetCmpObj->stats.subFramePreparationCycles =
            Cycleprofiler_getTimeStamp() - startTime;

        /* mark end of processing of the frame/sub-frame by the DPC and the app */
        objDetCmpObj->interSubFrameProcToken--;
    }
    else
    {
        uint8_t subFrameNum;

        /* First argument is sub-frame number */
        DebugP_assert(arg != NULL);
        subFrameNum = *(uint8_t *)arg;
        subFrmObj = &objDetCmpObj->subFrameObj[subFrameNum];

        switch (cmd)
        {
            /* Range DPU related */
			case DPC_OBJDET_IOCTL__DYNAMIC_CALIB_DC_RANGE_SIG_CFG:
            {
				retVal = -1230; // Not supported
				break;
			}
#if 0
            
                DPC_ObjectDetectionCmp_CalibDcRangeSigCfg *cfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetectionCmp_CalibDcRangeSigCfg));

                cfg = (DPC_ObjectDetectionCmp_CalibDcRangeSigCfg*)arg;

                retVal = DPU_RangeProcCmpHWA_control(subFrmObj->dpuRangeObj,
                             DPU_RangeProcCmpHWA_Cmd_dcRangeCfg,
                             &cfg->cfg,
                             sizeof(DPU_RangeProc_CalibDcRangeSigCfg));
                if (retVal != 0)
                {
                    goto exit;
                }

                /* save into object */
                subFrmObj->dynCfg.calibDcRangeSigCfg = cfg->cfg;

                break;
            }
#endif
            /* CFAR DPU related */
            case DPC_OBJDET_IOCTL__DYNAMIC_CFAR_RANGE_CFG:
            {
                DPC_ObjectDetectionCmp_CfarCfg *cfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetectionCmp_CfarCfg));

                cfg = (DPC_ObjectDetectionCmp_CfarCfg*)arg;

                retVal = DPU_CFARCAProcHWA_control(subFrmObj->dpuCFARCAObj,
                             DPU_CFARCAProcHWA_Cmd_CfarRangeCfg,
                             &cfg->cfg,
                             sizeof(DPU_CFARCAProc_CfarCfg));
                if (retVal != 0)
                {
                    goto exit;
                }

                /* save into object */
                subFrmObj->dynCfg.cfarCfgRange = cfg->cfg;

                break;
            }
            case DPC_OBJDET_IOCTL__DYNAMIC_CFAR_DOPPLER_CFG:
            {
                DPC_ObjectDetectionCmp_CfarCfg *cfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetectionCmp_CfarCfg));

                cfg = (DPC_ObjectDetectionCmp_CfarCfg*)arg;

                retVal = DPU_CFARCAProcHWA_control(subFrmObj->dpuCFARCAObj,
                             DPU_CFARCAProcHWA_Cmd_CfarDopplerCfg,
                             &cfg->cfg,
                             sizeof(DPU_CFARCAProc_CfarCfg));
                if (retVal != 0)
                {
                    goto exit;
                }

                /* save into object */
                subFrmObj->dynCfg.cfarCfgDoppler = cfg->cfg;

                break;
            }
            case DPC_OBJDET_IOCTL__DYNAMIC_FOV_RANGE:
            {
			    DPC_ObjectDetectionCmp_fovRangeCfg *cfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetectionCmp_fovRangeCfg));

                cfg = (DPC_ObjectDetectionCmp_fovRangeCfg*)arg;

                /* Add range bias to the minimum/maximum */
                cfg->cfg.min += objDetCmpObj->commonCfg.compRxChanCfg.rangeBias;
                cfg->cfg.max += objDetCmpObj->commonCfg.compRxChanCfg.rangeBias;

                retVal = DPU_CFARCAProcHWA_control(subFrmObj->dpuCFARCAObj,
                             DPU_CFARCAProcHWA_Cmd_FovRangeCfg,
                             &cfg->cfg,
                             sizeof(DPU_CFARCAProc_FovCfg));
                if (retVal != 0)
                {
                    goto exit;
                }

                /* save into object */
                subFrmObj->dynCfg.fovRange = cfg->cfg;

                break;
            }
            case DPC_OBJDET_IOCTL__DYNAMIC_FOV_DOPPLER:
            {
		
                DPC_ObjectDetectionCmp_fovDopplerCfg *cfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetectionCmp_fovDopplerCfg));

                cfg = (DPC_ObjectDetectionCmp_fovDopplerCfg*)arg;

                retVal = DPU_CFARCAProcHWA_control(subFrmObj->dpuCFARCAObj,
                             DPU_CFARCAProcHWA_Cmd_FovDopplerCfg,
                             &cfg->cfg,
                             sizeof(DPU_CFARCAProc_FovCfg));
                if (retVal != 0)
                {
                    goto exit;
                }

                /* save into object */
                subFrmObj->dynCfg.fovDoppler = cfg->cfg;

                break;
            }

            /* AoA DPU related */
            case DPC_OBJDET_IOCTL__DYNAMIC_MULTI_OBJ_BEAM_FORM_CFG:
            {
		
                DPC_ObjectDetectionCmp_MultiObjBeamFormingCfg *cfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetectionCmp_MultiObjBeamFormingCfg));

                cfg = (DPC_ObjectDetectionCmp_MultiObjBeamFormingCfg*)arg;

                retVal = DPU_AoAProcDcmpHWA_control(subFrmObj->dpuAoAObj,
                             DPU_AoAProcDcmpHWA_Cmd_MultiObjBeamFormingCfg,
                             &cfg->cfg,
                             sizeof(DPU_AoAProcDcmp_MultiObjBeamFormingCfg));
                if (retVal != 0)
                {
                    goto exit;
                }

                /* save into object */
                subFrmObj->dynCfg.multiObjBeamFormingCfg = cfg->cfg;

                break;
            }
            case DPC_OBJDET_IOCTL__DYNAMIC_EXT_MAX_VELOCITY:
            {
		
                DPC_ObjectDetectionCmp_extMaxVelCfg *cfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetectionCmp_extMaxVelCfg));

                cfg = (DPC_ObjectDetectionCmp_extMaxVelCfg*)arg;

                retVal = DPU_AoAProcDcmpHWA_control(subFrmObj->dpuAoAObj,
                             DPU_AoAProcDcmpHWA_Cmd_ExtMaxVelocityCfg,
                             &cfg->cfg,
                             sizeof(DPU_AoAProcDcmp_ExtendedMaxVelocityCfg));
                if (retVal != 0)
                {
                    goto exit;
                }

                /* save into object */
                subFrmObj->dynCfg.extMaxVelCfg = cfg->cfg;

                break;
            }
            case DPC_OBJDET_IOCTL__DYNAMIC_FOV_AOA:
            {
		        DPC_ObjectDetectionCmp_fovAoaCfg *cfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetectionCmp_fovAoaCfg));

                cfg = (DPC_ObjectDetectionCmp_fovAoaCfg*)arg;

                retVal = DPU_AoAProcDcmpHWA_control(subFrmObj->dpuAoAObj,
                             DPU_AoAProcDcmpHWA_Cmd_FovAoACfg,
                             &cfg->cfg,
                             sizeof(DPU_AoAProcDcmp_FovAoaCfg));
                if (retVal != 0)
                {
                    goto exit;
                }

                /* save into object */
                subFrmObj->dynCfg.fovAoaCfg = cfg->cfg;

                break;
            }
            case DPC_OBJDET_IOCTL__DYNAMIC_RANGE_AZIMUTH_HEAT_MAP:
            {
				
                DPC_ObjectDetectionCmp_RangeAzimuthHeatMapCfg *cfg;

                DebugP_assert(argLen == sizeof(DPC_ObjectDetectionCmp_RangeAzimuthHeatMapCfg));

                cfg = (DPC_ObjectDetectionCmp_RangeAzimuthHeatMapCfg*)arg;

                retVal = DPU_AoAProcDcmpHWA_control(subFrmObj->dpuAoAObj,
                             DPU_AoAProcDcmpHWA_Cmd_PrepareRangeAzimuthHeatMap,
                             &cfg->prepareRangeAzimuthHeatMap,
                             sizeof(bool));
                if (retVal != 0)
                {
                    goto exit;
                }

                /* save into object */
                subFrmObj->dynCfg.prepareRangeAzimuthHeatMap = cfg->prepareRangeAzimuthHeatMap;

                break;
            }

            /* Static clutter related */
            case DPC_OBJDET_IOCTL__DYNAMIC_STATICCLUTTER_REMOVAL_CFG:
            {
                /* Error: This is an unsupported command */
                retVal = DPC_OBJECTDETECTIONCMP_EINVAL__COMMAND;
                break;
            }

            /* Related to pre-start configuration */
            case DPC_OBJDET_IOCTL__STATIC_PRE_START_CFG:
            {
				
                DPC_ObjectDetectionCmp_PreStartCfg *cfg;
                DPC_ObjectDetectionCmp_DPC_IOCTL_preStartCfg_memUsage *memUsage;
                MemoryP_Stats statsStart;
                MemoryP_Stats statsEnd;

                /* Pre-start common config must be received before pre-start configs
                 * are received. */
                if (objDetCmpObj->isCommonCfgReceived == false)
                {
                    retVal = DPC_OBJECTDETECTIONCMP_PRE_START_CONFIG_BEFORE_PRE_START_COMMON_CONFIG;
                    goto exit;
                }

                DebugP_assert(argLen == sizeof(DPC_ObjectDetectionCmp_PreStartCfg));

                /* Get system heap size before preStart configuration */
                MemoryP_getStats(&statsStart);

                cfg = (DPC_ObjectDetectionCmp_PreStartCfg*)arg;

                memUsage = &cfg->memUsage;
                memUsage->L3RamTotal = objDetCmpObj->L3RamObj.cfg.size;
                memUsage->CoreLocalRamTotal = objDetCmpObj->CoreLocalRamObj.cfg.size;
                retVal = DPC_ObjDetCmp_preStartConfig(subFrmObj,
                             &objDetCmpObj->commonCfg, &cfg->staticCfg, &cfg->dynCfg,
                             &objDetCmpObj->edmaHandle[0],
                             &objDetCmpObj->L3RamObj,
                             &objDetCmpObj->CoreLocalRamObj,
                             &objDetCmpObj->hwaMemBankAddr[0],
                             objDetCmpObj->hwaMemBankSize,
                             &memUsage->L3RamUsage,
                             &memUsage->CoreLocalRamUsage);
                if (retVal != 0)
                {
					
                    goto exit;
                }

                /* Get system heap size after preStart configuration */
                MemoryP_getStats(&statsEnd);

                /* Populate system heap usage */
                memUsage->SystemHeapTotal = statsEnd.totalSize;
                memUsage->SystemHeapUsed = statsEnd.totalSize -statsEnd.totalFreeSize;
                memUsage->SystemHeapDPCUsed = statsStart.totalFreeSize - statsEnd.totalFreeSize;

                DebugP_log1("ObjDetCmp DPC: Pre-start Config IOCTL processed (subFrameIndx = %d)\n", subFrameNum);
                break;
            }

            default:
            {
    
                /* Error: This is an unsupported command */
                retVal = DPC_OBJECTDETECTIONCMP_EINVAL__COMMAND;
                break;
            }
        }
    }

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) initialization function which is invoked by the
 *      application using DPM_init API. Among other things, this API allocates DPC instance
 *      and DPU instances (by calling DPU's init APIs) from the MemoryP osal
 *      heap. If this API returns an error of any type, the heap is not guaranteed
 *      to be in the same state as before calling the API (i.e any allocations
 *      from the heap while executing the API are not guaranteed to be deallocated
 *      in case of error), so any error from this API should be considered fatal and
 *      if the error is of _ENOMEM type, the application will
 *      have to be built again with a bigger heap size to address the problem.
 *
 *  @param[in]  dpmHandle   DPM's DPC handle
 *  @param[in]  ptrInitCfg  Handle to the framework semaphore
 *  @param[out] errCode     Error code populated on error
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static DPM_DPCHandle DPC_ObjectDetectionCmp_init
(
    DPM_Handle          dpmHandle,
    DPM_InitCfg*        ptrInitCfg,
    int32_t*            errCode
)
{
    int32_t i;
    ObjDetCmpObj     *objDetCmpObj = NULL;
    SubFrameObj   *subFrmObj;
    DPC_ObjectDetectionCmp_InitParams *dpcInitParams;
    DPU_RangeProcCmpHWA_InitParams rangeInitParams;
    DPU_AoAProcDcmpHWA_InitParams aoaInitParams;
    DPU_CFARCAProcHWA_InitParams cfarInitParams;
    DPU_DopplerProcDcmpHWA_InitParams dopplerInitParams;
    HWA_MemInfo         hwaMemInfo;

    *errCode = 0;

    if ((ptrInitCfg == NULL) || (ptrInitCfg->arg == NULL))
    {
        *errCode = DPC_OBJECTDETECTIONCMP_EINVAL;
        goto exit;
    }

    if (ptrInitCfg->argSize != sizeof(DPC_ObjectDetectionCmp_InitParams))
    {
        *errCode = DPC_OBJECTDETECTIONCMP_EINVAL__INIT_CFG_ARGSIZE;
        goto exit;
    }

    dpcInitParams = (DPC_ObjectDetectionCmp_InitParams *) ptrInitCfg->arg;

    objDetCmpObj = MemoryP_ctrlAlloc(sizeof(ObjDetCmpObj), 0);

#ifdef DBG_DPC_OBJDET
    gObjDetCmpObj = objDetCmpObj;
#endif

    DebugP_log1("ObjDetCmp DPC: objDetCmpObj address = %d\n", (uint32_t) objDetCmpObj);

    if(objDetCmpObj == NULL)
    {
        *errCode = DPC_OBJECTDETECTIONCMP_ENOMEM;
        goto exit;
    }

    /* Initialize memory */
    memset((void *)objDetCmpObj, 0, sizeof(ObjDetCmpObj));

    /* Copy over the DPM configuration: */
    memcpy ((void*)&objDetCmpObj->dpmInitCfg, (void*)ptrInitCfg, sizeof(DPM_InitCfg));

    objDetCmpObj->dpmHandle = dpmHandle;
    objDetCmpObj->socHandle = ptrInitCfg->socHandle;
    objDetCmpObj->L3RamObj.cfg = dpcInitParams->L3ramCfg;
    objDetCmpObj->CoreLocalRamObj.cfg = dpcInitParams->CoreLocalRamCfg;

    for(i = 0; i < EDMA_NUM_CC; i++)
    {
        objDetCmpObj->edmaHandle[i] = dpcInitParams->edmaHandle[i];
    }

    objDetCmpObj->processCallBackCfg = dpcInitParams->processCallBackCfg;

    /* Set HWA bank memory address */
    *errCode =  HWA_getHWAMemInfo(dpcInitParams->hwaHandle, &hwaMemInfo);
    if (*errCode != 0)
    {
        goto exit;
    }

    objDetCmpObj->hwaMemBankSize = hwaMemInfo.bankSize;

    for (i = 0; i < hwaMemInfo.numBanks; i++)
    {
        objDetCmpObj->hwaMemBankAddr[i] = hwaMemInfo.baseAddress +
            i * hwaMemInfo.bankSize;
    }

    rangeInitParams.hwaHandle = dpcInitParams->hwaHandle;
    aoaInitParams.hwaHandle = dpcInitParams->hwaHandle;
    cfarInitParams.hwaHandle = dpcInitParams->hwaHandle;
    dopplerInitParams.hwaHandle = dpcInitParams->hwaHandle;

    for(i = 0; i < RL_MAX_SUBFRAMES; i++)
    {
        subFrmObj = &objDetCmpObj->subFrameObj[i];

        subFrmObj->dpuRangeObj = DPU_RangeProcCmpHWA_init(&rangeInitParams, errCode);

        if (*errCode != 0)
        {
            goto exit;
        }


        subFrmObj->dpuCFARCAObj = DPU_CFARCAProcHWA_init(&cfarInitParams, errCode);

        if (*errCode != 0)
        {
            goto exit;
        }
        
        subFrmObj->dpuDopplerObj = DPU_DopplerProcDcmpHWA_init(&dopplerInitParams, errCode);

        if (*errCode != 0)
        {
            goto exit;
        }

        subFrmObj->dpuAoAObj = DPU_AoAProcDcmpHWA_init(&aoaInitParams, errCode);

        if (*errCode != 0)
        {
            goto exit;
        }
    }

exit:

    if(*errCode != 0)
    {
        if(objDetCmpObj != NULL)
        {
            MemoryP_ctrlFree(objDetCmpObj, sizeof(ObjDetCmpObj));
            objDetCmpObj = NULL;
        }
    }

    return ((DPM_DPCHandle)objDetCmpObj);
}

/**
 *  @b Description
 *  @n
 *      DPC's (DPM registered) de-initialization function which is invoked by the
 *      application using DPM_deinit API.
 *
 *  @param[in]  handle  DPM's DPC handle
 *
 *  \ingroup DPC_OBJDET__INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *  @retval
 *      Error   -   <0
 */
static int32_t DPC_ObjectDetectionCmp_deinit (DPM_DPCHandle handle)
{
    ObjDetCmpObj *objDetCmpObj = (ObjDetCmpObj *) handle;
    SubFrameObj   *subFrmObj;
    int32_t retVal = 0;
    int32_t i;

    if (handle == NULL)
    {
        retVal = DPC_OBJECTDETECTIONCMP_EINVAL;
        goto exit;
    }

    for(i = 0; i < RL_MAX_SUBFRAMES; i++)
    {
        subFrmObj = &objDetCmpObj->subFrameObj[i];

        retVal = DPU_RangeProcCmpHWA_deinit(subFrmObj->dpuRangeObj);

        if (retVal != 0)
        {
            goto exit;
        }

        retVal = DPU_DopplerProcDcmpHWA_deinit(subFrmObj->dpuDopplerObj);

        if (retVal != 0)
        {
            goto exit;
        }

     
        retVal = DPU_CFARCAProcHWA_deinit(subFrmObj->dpuCFARCAObj);

        if (retVal != 0)
        {
            goto exit;
        }
        retVal = DPU_AoAProcDcmpHWA_deinit(subFrmObj->dpuAoAObj);

        if (retVal != 0)
        {
            goto exit;
        }
    }

    MemoryP_ctrlFree(handle, sizeof(ObjDetCmpObj));
exit:

    return (retVal);
}
