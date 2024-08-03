/**
 *   @file  aoaprocdcmphwa.h
 *
 *   @brief
 *      Implements Data path processing functionality.
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
#ifndef AOAPROCDCMP_HWA_H
#define AOAPROCDCMP_HWA_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* mmWave SDK Driver/Common Include Files */
#include <ti/drivers/osal/SemaphoreP.h>
#include <ti/drivers/hwa/hwa.h>

/* Datapath files */
#include <ti/datapath/dpif/dpif_radarcube.h>
#include <ti/datapath/dpif/dpif_pointcloud.h>
#include <ti/datapath/dpedma/dpedma.h>
#include <ti/datapath/dpif/dp_error.h>
#include <ti/datapath/dpc/dpu/aoaprocdcmp/aoaprocdcmp_common.h>


/** @addtogroup DPU_AOAPROCDCMP_EXTERNAL_DEFINITIONS
 *
 @{ */

/*! @brief Number of HWA parameter sets */
#define DPU_AOAPROCDCMPHWA_NUM_HWA_PARAM_SETS(numTxAnt, numRxVirtAntElevation) \
                     (2 * ((numTxAnt) + 2 + (1 & ((numRxVirtAntElevation) > 0))))

/*! @brief Alignment for memory allocation purpose. There is CPU access of this buffer
 *         in the implementation.
 */
#define DPU_AOAPROCDCMPHWA_CFAR_DET_LIST_BYTE_ALIGNMENT          DPIF_CFAR_DET_LIST_CPU_BYTE_ALIGNMENT

/*! @brief Alignment for memory allocation purpose. There is CPU access of this buffer
 *         in the implementation.
 */
#define DPU_AOAPROCDCMPHWA_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT  DPIF_POINT_CLOUD_CARTESIAN_CPU_BYTE_ALIGNMENT

/*! @brief Alignment for memory allocation purpose. There is CPU access of this buffer
 *         in the implementation.
 */
#define DPU_AOAPROCDCMPHWA_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT  DPIF_POINT_CLOUD_SIDE_INFO_CPU_BYTE_ALIGNMENT

/*! @brief Alignment for memory allocation purpose. There is CPU access of this buffer
 *         in the implementation.
 */
#define DPU_AOAPROCDCMPHWA_AZIMUTH_STATIC_HEAT_MAP_BYTE_ALIGNMENT (sizeof(int16_t))

/*! @brief   Log2 of the size of azimuth FFT */
#define DPU_AOAPROCDCMPHWA_LOG2_NUM_ANGLE_BINS 6 /* FFT Size = 64 */

/*! @brief   Size of azimuth FFT */
#define DPU_AOAPROCDCMPHWA_NUM_ANGLE_BINS (1 << DPU_AOAPROCDCMPHWA_LOG2_NUM_ANGLE_BINS)

/*! @brief Alignment for memory allocation purpose. There is CPU access of this buffer
 *         in the implementation.
 */
#define DPU_AOAPROCDCMPHWA_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT (sizeof(float))

/*! @brief Radar cube alignment must be the same as the size of the complex sample
           Even though HWA processing does not need this alignment, the 4 KB boundary
           problem of the EDMA requires that we make sure the source address
           is aligned to the EDMA's Acount (= sample size = 4 bytes which divides 4 KB)
 */
#define DPU_AOAPROCDCMPHWA_RADAR_CUBE_BYTE_ALIGNMENT (sizeof(cmplx16ImRe_t))

/*! @brief Alignment for local scratch memory allocation purpose. There is CPU access of this buffer
 *         in the implementation.
 */
#define DPU_AOAPROCDCMPHWA_LOCAL_SCRATCH_BYTE_ALIGNMENT (sizeof(uint32_t))

/*! @brief Number of local scratch buffers */
#define DPU_AOAPROCDCMPHWA_NUM_LOCAL_SCRATCH_BUFFERS (2)

/*! @brief Local scratch buffer size. The size maximum of the two buffers that are overlayed:
 *         Buffer 1 (azimElevLocalBuf and azimElevLocalHypothesesBuf) size:
 *                    (NTxAnt + 1) * NTxAnt * NRxAnt * sizeof(uint32_t)
 *         Buffer 2 (azimuthFftOutMagBuf) size:
 *                    NTxAnt * @ref DPU_AOAPROCDCMPHWA_NUM_ANGLE_BINS * sizeof(uint16_t)
 *         The size of the second buffer is greater than  the first one as long as
 *         (NTxAnt + 1) * NRxAnt < 32, which is true, therefore the size of the
 *         second buffer is defined below. 
          Note: For nTx*nTx*nRx*sizeof(uint32_t) == 64, this is the case where
                only combination that is valid is nTx = 2, nRx = 4,
                we do the EDMA 4K speculative correction for some devices
                which advances the azimElevLocalHypothesesBuf by a maximum of 64.
                In this case, buffer1 size = 32 + (64 + 64) = 160
                buffer2 size = 256, so condition above is still true */
#define DPU_AOAPROCDCMPHWA_NUM_LOCAL_SCRATCH_BUFFER_SIZE_BYTES(numTxAntennas) \
    ((numTxAntennas) * DPU_AOAPROCDCMPHWA_NUM_ANGLE_BINS * sizeof(uint16_t))

/**
@}
*/

#ifdef __cplusplus
extern "C" {
#endif


/** @addtogroup DPU_AOAPROCDCMP_ERROR_CODE
 *  Base error code for the aoaProcDcmp DPU is defined in the
 *  \include ti/datapath/dpif/dp_error.h
 @{ */

/**
 * @brief   Error Code: Invalid argument (general e.g NULL pointers)
 */
#define DPU_AOAPROCDCMPHWA_EINVAL                  (DP_ERRNO_AOA_PROC_BASE-1)

/**
 * @brief   Error Code: Invalid argument - radar cube format
 */
#define DPU_AOAPROCDCMPHWA_EINVAL__RADARCUBE_DATAFORMAT  (DP_ERRNO_AOA_PROC_BASE-2)

/**
 * @brief   Error Code: Out of memory when allocating using MemoryP_osal
 */
#define DPU_AOAPROCDCMPHWA_ENOMEM                               (DP_ERRNO_AOA_PROC_BASE-20)

/**
 * @brief   Error Code: Memory not aligned for @ref DPU_AoAProcDcmpHWA_HW_Resources::cfarRngDopSnrList
 */
#define DPU_AOAPROCDCMPHWA_ENOMEMALIGN_CFAR_DET_LIST            (DP_ERRNO_AOA_PROC_BASE-21)

/**
 * @brief   Error Code: Memory not aligned for @ref DPU_AoAProcDcmpHWA_HW_Resources::detObjOut
 */
#define DPU_AOAPROCDCMPHWA_ENOMEMALIGN_POINT_CLOUD_CARTESIAN    (DP_ERRNO_AOA_PROC_BASE-22)

/**
 * @brief   Error Code: Memory not aligned for @ref DPU_AoAProcDcmpHWA_HW_Resources::detObjOutSideInfo
 */
#define DPU_AOAPROCDCMPHWA_ENOMEMALIGN_POINT_CLOUD_SIDE_INFO    (DP_ERRNO_AOA_PROC_BASE-23)

/**
 * @brief   Error Code: Insufficient memory allocated for azimuth static heat map
 */
#define DPU_AOAPROCDCMPHWA_ENOMEM__AZIMUTH_STATIC_HEAT_MAP      (DP_ERRNO_AOA_PROC_BASE-24)

/**
 * @brief   Error Code: Memory not aligned for @ref DPU_AoAProcDcmpHWA_HW_Resources::azimuthStaticHeatMap
 */
#define DPU_AOAPROCDCMPHWA_ENOMEMALIGN_AZIMUTH_STATIC_HEAT_MAP  (DP_ERRNO_AOA_PROC_BASE-25)

/**
 * @brief   Error Code: Memory not aligned for @ref DPU_AoAProcDcmpHWA_HW_Resources::detObjElevationAngle
 */
#define DPU_AOAPROCDCMPHWA_ENOMEMALIGN_DET_OBJ_ELEVATION_ANGLE  (DP_ERRNO_AOA_PROC_BASE-26)

/**
 * @brief   Error Code: azimuth heat-map flag enabled and single virtual antenna not valid combination
 */
#define DPU_AOAPROCDCMPHWA_EINVALID_NUM_VIRT_ANT_AND_AZIMUTH_STATIC_HEAT_MAP (DP_ERRNO_AOA_PROC_BASE-27)

/**
 * @brief   Error Code: Configure parameters exceed HWA memory bank size
 */
#define DPU_AOAPROCDCMPHWA_EEXCEEDHWAMEM (DP_ERRNO_AOA_PROC_BASE-28)

/**
 * @brief   Error Code: Detected object list size is not even number
 */
#define DPU_AOAPROCDCMPHWA_EDETECTED_OBJECT_LIST_SIZE_ODD_NUMBER (DP_ERRNO_AOA_PROC_BASE-29)

/**
 * @brief   Error Code: Memory not aligned for @ref DPU_AoAProcDcmpHWA_HW_Resources::localScratchBuffer
 */
#define DPU_AOAPROCDCMPHWA_ENOMEMALIGN_LOCAL_SCRATCH_BUF  (DP_ERRNO_AOA_PROC_BASE-30)

/**
 * @brief   Error Code: Insufficient memory allocated for @ref DPU_AoAProcDcmpHWA_HW_Resources::localScratchBuffer
 */
#define DPU_AOAPROCDCMPHWA_ENOMEM_LOCAL_SCRATCH_BUF      (DP_ERRNO_AOA_PROC_BASE-31)

/**
 * @brief   Error Code: Memory not aligned for @ref DPU_AoAProcDcmpHWA_HW_Resources::radarCube
 */
#define DPU_AOAPROCDCMPHWA_ENOMEMALIGN_RADAR_CUBE  (DP_ERRNO_AOA_PROC_BASE-32)

/**
 * @brief   Error Code: Internal error
 */
#define DPU_AOAPROCDCMPHWA_EINTERNAL               (DP_ERRNO_AOA_PROC_BASE-40)

/**
 * @brief   Error Code: Not implemented
 */
#define DPU_AOAPROCDCMPHWA_ENOTIMPL                (DP_ERRNO_AOA_PROC_BASE-50)

/**
 * @brief   Error Code: Semaphore error
 */
#define DPU_AOAPROCDCMPHWA_ESEMA                   (DP_ERRNO_AOA_PROC_BASE-60)

/**
@}
*/


/**
* @brief
*  AoAProcDcmpHWA DPU control command
*
* @details
*  The enum defines the AoAProcDcmpHWA supported run time command
*
*  \ingroup DPU_AOAPROCDCMP_EXTERNAL_DATA_STRUCTURE
*/
typedef enum DPU_AoAProcDcmpHWA_Cmd_e
{
 /*! @brief     Command to update field of view configuration, azimuth and elevation selected range*/
 DPU_AoAProcDcmpHWA_Cmd_FovAoACfg,
 /*! @brief     Command to update multiobject beam forming configuration */
 DPU_AoAProcDcmpHWA_Cmd_MultiObjBeamFormingCfg,
 /*! @brief     Command to update rx channel phase compensation */
 DPU_AoAProcDcmpHWA_Cmd_CompRxChannelBiasCfg,
 /*! @brief     Command to update Azimuth heat-map configuration */
 DPU_AoAProcDcmpHWA_Cmd_PrepareRangeAzimuthHeatMap,
 /*! @brief     Command to update field of extended maximum velocity */
 DPU_AoAProcDcmpHWA_Cmd_ExtMaxVelocityCfg
} DPU_AoAProcDcmpHWA_Cmd;

/**
 * @brief
 *  AoAProcDcmpHWA DPU configuration
 *
 * @details
 *  The structure is used to hold the HWA configuration needed for AOA
 *
 *  \ingroup DPU_AOAPROCDCMP_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_AoAProcDcmpHWA_hwaAoAConfig_t
{
    /** @brief     HWA paramset Start index */
    uint8_t    paramSetStartIdx;

    /** @brief     number of HWA paramset, must be @ref DPU_AOAPROCDCMPHWA_NUM_HWA_PARAM_SETS */
    uint8_t    numParamSet;

    /*! @brief     Flag to indicate if HWA windowing is symmetric
                    see HWA_WINDOW_SYMM definitions in HWA driver's doxygen documentation
     */
    uint8_t   winSym;

    /*! @brief Doppler FFT window size in bytes.
         This is the number of coefficients to be programmed in the HWA for the windowing
         functionality. The size is a function of @ref DPU_AoAProcDcmpHWA_StaticConfig::numDopplerChirps as follows:\n
         If non-symmetric window is selected: windowSize = numDopplerChirps * sizeof(int32_t) \n
         If symmetric window is selected and numDopplerChirps is even:
         windowSize = numDopplerChirps * sizeof(int32_t) / 2 \n
         If symmetric window is selected and numDopplerChirps is odd:
         windowSize = (numDopplerChirps + 1) * sizeof(int32_t) / 2
    */
    uint32_t    windowSize;

    /*! @brief Pointer to doppler FFT window coefficients. */
    int32_t     *window;

    /*! @brief   Offset in HWA window RAM for singleBin doppler FFT
     *            in number of samples */
    uint32_t     winRamOffset;
}DPU_AoAProcDcmpHWA_hwaAoAConfig;

/**
 * @brief
 *  AoAProcDcmpHWA DPU initial configuration parameters
 *
 * @details
 *  The structure is used to hold the DPU initial configurations parameters.
 *
 *  \ingroup DPU_AOAPROCDCMP_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_AoAProcDcmpHWA_InitParams_t
{
    /*! @brief HWA Handle */
    HWA_Handle  hwaHandle;
}DPU_AoAProcDcmpHWA_InitParams;

/**
 * @brief
 *  AoAProcDcmpHWA DPU EDMA configuration
 *
 * @details
 *  EDMA configuration for input/output to HWA for calculation of zero-Doppler 2D-FFT used for azimuth heatmap
 *
 *  \ingroup DPU_AOAPROCDCMP_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_AoAProcDcmpHWA_EdmaHwaInOut_t
{
    /*! @brief     EDMA configuration for AOA data In */
    DPEDMA_ChanCfg       in;

    /*! @brief     EDMA configuration for AOA data In signaure */
    DPEDMA_ChanCfg       inSignature;

    /*! @brief     EDMA configuration for AOA data Out */
    DPEDMA_ChanCfg       out;

} DPU_AoAProcDcmpHWA_EdmaHwaInOut;

/**
 * @brief
 *  AoAProcDcmpHWA DPU EDMA channel configuration
 *
 * @details
 *  DMA physical channel configuration
 *
 *
 *  \ingroup DPU_AOAPROCDCMP_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_AoAProcDcmpHWA_ChanCfg_t
{
    /*! @brief     EDMA channel id */
    uint8_t             channel;

    /*! @brief     EDMA event Queue used for the transfer */
    uint8_t             eventQueue;
} DPU_AoAProcDcmpHWA_ChanCfg;

/**
 * @brief
 *  AoAProcDcmpHWA DPU EDMA configuration for HWA input/output
 *
 * @details
 *  EDMA Param sets used for data transfer to and from HWA
 *
 *
 *  \ingroup DPU_AOAPROCDCMP_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_AoAProcDcmpHWA_EdmaHwaInOutParamId_t
{
    /*! @brief     EDMA configuration for AOA data In */
    uint16_t       paramIn;

    /*! @brief     EDMA configuration for AOA data In signaure */
    uint16_t       paramInSignature;

    /*! @brief     EDMA configuration for AOA data Out */
    uint16_t       paramOut;

} DPU_AoAProcDcmpHWA_EdmaHwaInOutParamId;

/**
 * @brief
 *  AoAProcDcmpHWA DPU EDMA configuration
 *
 * @details
 *  EDMA configuration for input/output to HWA for 2D-FFT and 3D-FFT stages
 *
 *
 *  \ingroup DPU_AOAPROCDCMP_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_AoAProcDcmpHWA_EdmaHwaDataInOut_t
{
    /*! @brief     EDMA physical channel In */
    DPU_AoAProcDcmpHWA_ChanCfg       chIn;

    /*! @brief     EDMA physical channel Out */
    DPU_AoAProcDcmpHWA_ChanCfg       chOut;

    /*! @brief     EDMA configuration in/out per stage: 0: 2D-FFT and 1: 3D-FFT */
    DPU_AoAProcDcmpHWA_EdmaHwaInOutParamId stage[2];

    /*! @brief     EDMA event Queue used for the transfer */
    uint8_t             eventQueue;
} DPU_AoAProcDcmpHWA_EdmaHwaDataInOut;

/**
 * @brief
 *  AoAProcDcmpHWA DPU Hardware resources
 *
 * @details
 *  AoAProcDcmpHWA DPU Hardware resources
 *
 *
 *  \ingroup DPU_AOAPROCDCMP_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_AoAProcDcmpHWA_Resources_t
{
    /*! @brief     EDMA Handle */
    EDMA_Handle         edmaHandle;

    /*! @brief     EDMA configuration for azimuth heatmap computation (zero-Doppler 2D-FFT calculation for all range bins), 0-ping, 1-pong */
    DPU_AoAProcDcmpHWA_EdmaHwaInOut edmaHwa[2];

    /*! @brief     EDMA configuration, for point-cloud computation including 2D-FFT and 3D-FFT,  0-ping, 1-pong*/
    DPU_AoAProcDcmpHWA_EdmaHwaDataInOut edmaHwaExt[2];

    /*! @brief     HWA Configuration */
    DPU_AoAProcDcmpHWA_hwaAoAConfig    hwaCfg;

    /*! @brief     Radar Cube structure */
    DPIF_RadarCube      radarCube;

    /*! @brief      List of CFAR detected objects of @ref cfarRngDopSnrListSize elements,
     *              must be aligned to @ref DPU_AOAPROCDCMPHWA_CFAR_DET_LIST_BYTE_ALIGNMENT */
    DPIF_CFARDetList *cfarRngDopSnrList;

    /*! @brief      CFAR detected objects list size */
    uint16_t        cfarRngDopSnrListSize;

    /*! @brief      Detected objects output list sized to @ref detObjOutMaxSize elements,
     *              must be aligned to @ref DPU_AOAPROCDCMPHWA_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT  */
    DPIF_PointCloudCartesian *detObjOut;

    /*! @brief      Detected objects side information (snr + noise) output list,
     *              sized to @ref detObjOutMaxSize elements,
     *              must be aligned to @ref DPU_AOAPROCDCMPHWA_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT */
    DPIF_PointCloudSideInfo *detObjOutSideInfo;

    /*! @brief      This field dimensions several other fields in this structure as
     *              referred in their descriptions. It is determined by the dpc/application based
     *              on balancing between maximum number of objects expected to be
     *              detected in the scene (this can depend on configuration like cfar thresholds,
     *              static clutter removal etc) and memory and MIPS limitations. */
    uint32_t        detObjOutMaxSize;

    /*! @brief      Pointer to range-azimuth static heat map, this is a 2D FFT
     *              array in range direction (x[numRangeBins][numVirtualAntAzim]),
     *              at doppler index 0, sized to @ref azimuthStaticHeatMapSize elements of
     *              type cplx16ImRe_t.
     *              Alignment should be @ref DPU_AOAPROCDCMPHWA_AZIMUTH_STATIC_HEAT_MAP_BYTE_ALIGNMENT */
    cmplx16ImRe_t   *azimuthStaticHeatMap;

    /*! @brief      Number of elements of azimuthStaticHeatMap, this should be
     *              numVirtualAntAzim * numRangeBins */
    uint32_t        azimuthStaticHeatMapSize;

    /*! @brief      Detected objects azimuth index for debugging,
     *              sized to @ref detObjOutMaxSize elements */
    uint8_t         *detObj2dAzimIdx;

    /*! @brief      Detected object elevation angle for debugging,
     *              sized to @ref detObjOutMaxSize elements, must be aligned to
     *              @ref DPU_AOAPROCDCMPHWA_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT */
    float           *detObjElevationAngle;

    /*! @brief      Local scratch buffer */
    uint8_t         *localScratchBuffer[DPU_AOAPROCDCMPHWA_NUM_LOCAL_SCRATCH_BUFFERS];

    /*! @brief      Local scratch buffer size = numTxAntennas * DPU_AOAPROCDCMPHWA_NUM_ANGLE_BINS * sizeof(uint16_t).
                    A convenient macro @ref DPU_AOAPROCDCMPHWA_NUM_LOCAL_SCRATCH_BUFFER_SIZE_BYTES has
                    been provided for calculating this size */
    uint32_t        localScratchBufferSizeBytes;

} DPU_AoAProcDcmpHWA_HW_Resources;

/**
 * @brief
 *  AoAProcDcmpHWA DPU static configuration parameters
 *
 *  \ingroup DPU_AOAPROCDCMP_EXTERNAL_DATA_STRUCTURE
 *
 * @details
 * The following conditions must be satisfied:
 *    @verbatim
      numTxAntennas * numRxAntennas * numDopplerChirps * sizeof(cmplx16ImRe_t) <= 16 KB (one HWA memory bank)
      numTxAntennas * numRxAntennas * numDopplerBins * sizeof(cmplx16ImRe_t) <= 32 KB (two HWA memory banks)
      @endverbatim
 *
 */
typedef struct DPU_AoAProcDcmpHWA_StaticConfig_t
{
    /*! @brief  Number of transmit antennas */
    uint8_t     numTxAntennas;

    /*! @brief  Number of receive antennas */
    uint8_t     numRxAntennas;

    /*! @brief  Number of virtual azimuth antennas */
    uint8_t     numVirtualAntAzim;

    /*! @brief  Number of virtual elevation antennas */
    uint8_t     numVirtualAntElev;

    /*! @brief  Number of range bins */
    uint16_t    numRangeBins;

    /*! @brief Number of chirps for Doppler computation purposes.
               For example, in TDM/BPM-MIMO scheme, this is the physical chirps
               in a frame per transmit antenna
               i.e numDopplerChirps = numChirpsPerFrame / numTxAntennas */
    uint16_t    numDopplerChirps;

    /*! @brief  Number of doppler bins */
    uint16_t    numDopplerBins;

    /*! @brief  Range conversion factor for range FFT index to meters */
    float       rangeStep;

    /*! @brief  Doppler conversion factor for Doppler FFT index to m/s */
    float       dopplerStep;
	
	/*! @brief      Compression module configuration */
    DPC_ObjectDetectionCmp_CompressCfg  compressCfg;
} DPU_AoAProcDcmpHWA_StaticConfig;

/**
 * @brief
 *  AoAProcDcmpHWA DPU configuration
 *
 * @details
 *  The structure is used to hold the AoAProcDcmpHWA configuration
 *
 *  \ingroup DPU_AOAPROCDCMP_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_AoAProcDcmpHWA_Config_t
{
    /*! @brief     Data path common parameters */
    DPU_AoAProcDcmpHWA_StaticConfig   staticCfg;

    /*! @brief     Hardware resources */
    DPU_AoAProcDcmpHWA_HW_Resources res;

    /*! @brief     Dynamic configuration */
    DPU_AoAProcDcmp_DynamicConfig dynCfg;

}DPU_AoAProcDcmpHWA_Config;

/**
 * @brief
 *  Output parameters populated during Processing time
 *
 * @details
 *  The structure is used to hold the output parameters
 *
 *  \ingroup DPU_AOAPROCDCMP_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_AoAProcDcmpHWA_OutParams_t
{

    /*! @brief     AoAProcDcmpHWA stats */
    DPU_AoAProcDcmp_Stats       stats;

    /*! @brief      Number of AoA DPU detected points*/
    uint32_t numAoADetectedPoints;

}DPU_AoAProcDcmpHWA_OutParams;

/**
 * @brief
 *  AoAProcDcmpHWA DPU Handle
 *
 *
 *  \ingroup DPU_AOAPROCDCMP_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef void* DPU_AoAProcDcmpHWA_Handle ;

/**
 *  @b Description
 *  @n
 *      The function is AoAProcDcmpHWA DPU initialization function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @param[in]  initParams              Pointer to initialization parameters
 *
 *  @param[in]  errCode                 Pointer to errCode generates from the API
 *
 *  \ingroup    DPU_AOAPROCDCMP_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid AoAProcDcmpHWA handle
 *  @retval
 *      Error       - NULL
 */
DPU_AoAProcDcmpHWA_Handle DPU_AoAProcDcmpHWA_init
(
    DPU_AoAProcDcmpHWA_InitParams *initParams,
    int32_t*            errCode
);

/**
 *  @b Description
 *  @n
 *      The function is AoAProcDcmpHWA DPU configuration function. It saves buffer pointer and configurations
 *  including system resources and configures EDMA for runtime range processing.
 *
 *  @pre    DPU_AoAProcDcmpHWA_init() has been called
 *
 *  @param[in]  handle                  AoAProcDcmpHWA DPU handle
 *  @param[in]  aoaHwaCfg              Pointer to AoAProcDcmpHWA configuration data structure
 *
 *  \ingroup    DPU_AOAPROCDCMP_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_AoAProcDcmpHWA_config
(
    DPU_AoAProcDcmpHWA_Handle    handle,
    DPU_AoAProcDcmpHWA_Config    *aoaHwaCfg
);

/**
 *  @b Description
 *  @n
 *      The function is AoAProcDcmpHWA DPU process function. It performs CFAR detection using HWA
 *
 *  @pre    DPU_AoAProcDcmpHWA_init() has been called
 *
 *  @param[in]  handle                  AoAProcDcmpHWA DPU handle
 *
 *  @param[in]  numObjsIn               Number of detected objects by CFAR DPU
 *
 *  @param[in]  outParams               DPU output parameters
 *
 *  \ingroup    DPU_AOAPROCDCMP_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success - 0
 *  @retval
 *      Error   - <0
 */
int32_t DPU_AoAProcDcmpHWA_process
(
    DPU_AoAProcDcmpHWA_Handle    handle,
    uint32_t        numObjsIn,
    DPU_AoAProcDcmpHWA_OutParams  *outParams
);

/**
 *  @b Description
 *  @n
 *      The function is AoAProcDcmpHWA DPU control function.
 *
 *  @pre     DPU_AoAProcDcmpHWA_init() has been called
 *
 *  @param[in]  handle           AoAProcDcmpHWA DPU handle
 *  @param[in]  cmd              AoAProcDcmpHWA DPU control command
 *  @param[in]  arg              AoAProcDcmpHWA DPU control argument pointer
 *  @param[in]  argSize          AoAProcDcmpHWA DPU control argument size
 *
 *  \ingroup    DPU_AOAPROCDCMP_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_AoAProcDcmpHWA_control
(
   DPU_AoAProcDcmpHWA_Handle handle,
   DPU_AoAProcDcmpHWA_Cmd cmd,
   void *arg,
   uint32_t argSize
);

/**
 *  @b Description
 *  @n
 *      The function is AoAProcDcmpHWA DPU deinitialization function. It frees up the
 *   resources allocated during initialization.
 *
 *  @pre    DPU_AoAProcDcmpHWA_init() has been called
 *
 *  @param[in]  handle           AoAProcDcmpHWA DPU handle
 *
 *  \ingroup    DPU_AOAPROCDCMP_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_AoAProcDcmpHWA_deinit
(
    DPU_AoAProcDcmpHWA_Handle handle
);

#ifdef __cplusplus
}
#endif

#endif
