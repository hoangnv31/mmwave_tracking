/**
 *   @file  aoaprocdcmpdsp.h
 *
 *   @brief
 *      AoA DSP header file
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2019 Texas Instruments, Inc.
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
#ifndef AOAPROCDCMP_DSP_H
#define AOAPROCDCMP_DSP_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Datapath files */
#include <ti/datapath/dpif/dpif_radarcube.h>
#include <ti/datapath/dpif/dpif_pointcloud.h>
#include <ti/datapath/dpedma/dpedma.h>
#include <ti/datapath/dpif/dp_error.h>
#include <ti/datapath/dpc/dpu/aoaprocdcmp/aoaprocdcmp_common.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup DPU_AOAPROCDCMP_ERROR_CODE
 *  Base error code for the aoaProcDcmp DPU is defined in the
 *  \include ti/datapath/dpif/dp_error.h
 @{ */

/**
 * @brief   Error Code: Invalid argument 
 */
#define DPU_AOAPROCDCMPDSP_EINVAL                  (DP_ERRNO_AOA_PROC_BASE-1)

/**
 * @brief   Error Code: Invalid radar cube format
 */
#define DPU_AOAPROCDCMPDSP_EINVAL__RADARCUBE_DATAFORMAT  (DP_ERRNO_AOA_PROC_BASE-2)

/**
 * @brief   Error Code: Out of memory 
 */
#define DPU_AOAPROCDCMPDSP_ENOMEM                               (DP_ERRNO_AOA_PROC_BASE-20)

/**
 * @brief   Error Code: Memory not aligned for @ref DPU_AoAProcDcmpDSP_HW_Resources::cfarRngDopSnrList
 */
#define DPU_AOAPROCDCMPDSP_ENOMEMALIGN_CFAR_DET_LIST            (DP_ERRNO_AOA_PROC_BASE-21)

/**
 * @brief   Error Code: Memory not aligned for @ref DPU_AoAProcDcmpDSP_HW_Resources::detObjOut
 */
#define DPU_AOAPROCDCMPDSP_ENOMEMALIGN_POINT_CLOUD_CARTESIAN    (DP_ERRNO_AOA_PROC_BASE-22)

/**
 * @brief   Error Code: Memory not aligned for @ref DPU_AoAProcDcmpDSP_HW_Resources::detObjOutSideInfo
 */
#define DPU_AOAPROCDCMPDSP_ENOMEMALIGN_POINT_CLOUD_SIDE_INFO    (DP_ERRNO_AOA_PROC_BASE-23)

/**
 * @brief   Error Code: Memory not aligned for one of the buffers required by DPU
 */
#define DPU_AOAPROCDCMPDSP_ENOMEMALIGN_BUFF  (DP_ERRNO_AOA_PROC_BASE-24)

/**
 * @brief   Error Code: Insufficient memory allocated for azimuth static heat map
 */
#define DPU_AOAPROCDCMPDSP_ENOMEM__AZIMUTH_STATIC_HEAT_MAP      (DP_ERRNO_AOA_PROC_BASE-25)


/**
 * @brief   Error Code: azimuth heat-map flag enabled and single virtual antenna not valid combination
 */
#define DPU_AOAPROCDCMPDSP_EINVALID_NUM_VIRT_ANT_AND_AZIMUTH_STATIC_HEAT_MAP (DP_ERRNO_AOA_PROC_BASE-27)

/**
 * @brief   Error Code: Number of Doppler chirps is not a multiple of 4
 */
#define DPU_AOAPROCDCMPDSP_ENUMDOPCHIRPS           (DP_ERRNO_AOA_PROC_BASE-28)

/**
 * @brief   Error Code: Number of Doppler bins is less than 16
 */
#define DPU_AOAPROCDCMPDSP_ENUMDOPBINS             (DP_ERRNO_AOA_PROC_BASE-29)

/**
 * @brief   Error Code: One of the provided scratch buffers has insufficient size
 */
#define DPU_AOAPROCDCMPDSP_ESCRATCHSIZE            (DP_ERRNO_AOA_PROC_BASE-30)

/**
 * @brief   Error Code: DPU configuration is invalid. It exceeds the maximum EDMA jump size of (32K - 1)
 */
#define DPU_AOAPROCDCMPDSP_EEXCEEDMAXEDMA           (DP_ERRNO_AOA_PROC_BASE-31)

/**
@}
*/

/*! @brief Alignment for memory allocation purpose.
 */
#define DPU_AOAPROCDCMPDSP_CFAR_DET_LIST_BYTE_ALIGNMENT          DPIF_CFAR_DET_LIST_CPU_BYTE_ALIGNMENT

/*! @brief Alignment for memory allocation purpose. 
 */
#define DPU_AOAPROCDCMPDSP_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT  DPIF_POINT_CLOUD_CARTESIAN_CPU_BYTE_ALIGNMENT

/*! @brief Alignment for memory allocation purpose. 
 */
#define DPU_AOAPROCDCMPDSP_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT  DPIF_POINT_CLOUD_SIDE_INFO_CPU_BYTE_ALIGNMENT

/*! @brief Alignment for memory allocation purpose. 
 */
#define DPU_AOAPROCDCMPDSP_AZIMUTH_STATIC_HEAT_MAP_BYTE_ALIGNMENT (sizeof(int16_t))

/*! @brief Alignment for memory allocation purpose. 
 */
#define DPU_AOAPROCDCMPDSP_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT (sizeof(float))

/*! @brief Alignment for memory allocation purpose of all remaining buffers required by the DPU. 
 */
#define DPU_AOAPROCDCMPDSP_BUFFERS_BYTE_ALIGNMENT 8U

/**
 * @brief   Number of angle bins for the Azimuth/Elevation FFT
 */
#define DPU_AOAPROCDCMPDSP_NUM_ANGLE_BINS          (64U)


/**
* @brief
*  DPU control command
*
* @details
*  The enum defines the DPU supported run time command
*
*  \ingroup DPU_AOAPROCDCMP_EXTERNAL_DATA_STRUCTURE
*/
typedef enum DPU_AoAProcDcmpDSP_Cmd_e
{
 /*! @brief     Command to update field of view configuration, azimuth and elevation selected range*/
 DPU_AoAProcDcmpDSP_Cmd_FovAoACfg,
 /*! @brief     Command to update multi-object beam forming configuration */
 DPU_AoAProcDcmpDSP_Cmd_MultiObjBeamFormingCfg,
 /*! @brief     Command to update rx channel phase compensation */
 DPU_AoAProcDcmpDSP_Cmd_CompRxChannelBiasCfg,
 /*! @brief     Command to update Azimuth heat-map configuration */
 DPU_AoAProcDcmpDSP_Cmd_PrepareRangeAzimuthHeatMap,
 /*! @brief     Command to update static clutter removal configuration.*/
DPU_AoAProcDcmpDSP_Cmd_staticClutterCfg,
 /*! @brief     Command to update field of extended maximum velocity */
 DPU_AoAProcDcmpDSP_Cmd_ExtMaxVelocityCfg
} DPU_AoAProcDcmpDSP_Cmd;


/**
 * @brief
 *  AoAProcDcmpDSP DPU Hardware resources
 *
 * @details
 *  AoAProcDcmpDSP DPU Hardware resources
 *
 *
 *  \ingroup DPU_AOAPROCDCMP_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_AoAProcDcmpDSP_Resources_t
{
    /*! @brief     EDMA Handle */
    EDMA_Handle         edmaHandle;

    /*! @brief     EDMA configuration for AOA data In (Ping)*/
    DPEDMA_ChanCfg       edmaPing;

    /*! @brief     EDMA configuration for AOA data In (Pong)*/
    DPEDMA_ChanCfg       edmaPong;
    
    /*! @brief     Radar Cube structure */
    DPIF_RadarCube      radarCube;

    /*! @brief      List of CFAR detected objects of @ref cfarRngDopSnrListSize elements,
     *              must be aligned to @ref DPU_AOAPROCDCMPDSP_CFAR_DET_LIST_BYTE_ALIGNMENT */
    DPIF_CFARDetList *cfarRngDopSnrList;

    /*! @brief      CFAR detected objects list size */
    uint16_t        cfarRngDopSnrListSize;

    /*! @brief      Detected objects output list sized to @ref detObjOutMaxSize elements,
     *              must be aligned to @ref DPU_AOAPROCDCMPDSP_POINT_CLOUD_CARTESIAN_BYTE_ALIGNMENT  */
    DPIF_PointCloudCartesian *detObjOut;

    /*! @brief      Detected objects side information (snr + noise) output list,
     *              sized to @ref detObjOutMaxSize elements,
     *              must be aligned to @ref DPU_AOAPROCDCMPDSP_POINT_CLOUD_SIDE_INFO_BYTE_ALIGNMENT */
    DPIF_PointCloudSideInfo *detObjOutSideInfo;

    /*! @brief      This field dimensions several other fields in this structure as
     *              referred in their descriptions. It is determined by the dpc/application based
     *              on balancing between maximum number of objects expected to be
     *              detected in the scene (this can depend on configuration like cfar thresholds,
     *              static clutter removal etc) and memory and MIPS limitations. */
    uint32_t        detObjOutMaxSize;
    
    /*! @brief      Detected objects azimuth index for debugging,
     *              sized to @ref detObjOutMaxSize elements */
    uint8_t         *detObj2dAzimIdx;

    /*! @brief      Detected object elevation angle for debugging,
     *              sized to @ref detObjOutMaxSize elements, must be aligned to
     *              @ref DPU_AOAPROCDCMPDSP_DET_OBJ_ELEVATION_ANGLE_BYTE_ALIGNMENT.\n 
     *              This buffer does not need to be allocated if elevation antenna is not used or not supported.*/
    float           *detObjElevationAngle;

    /*! @brief      Pointer to range-azimuth static heat map, this is a 2D FFT
     *              array in range direction (x[numRangeBins][numVirtualAntAzim]),
     *              at doppler index 0, sized to @ref azimuthStaticHeatMapSize elements of
     *              type cplx16ImRe_t.
     *              Alignment should be @ref DPU_AOAPROCDCMPDSP_AZIMUTH_STATIC_HEAT_MAP_BYTE_ALIGNMENT */
    cmplx16ImRe_t   *azimuthStaticHeatMap;

    /*! @brief      Number of elements of azimuthStaticHeatMap, this should be
     *              numVirtualAntAzim * numRangeBins */
    uint32_t        azimuthStaticHeatMapSize;

    /*! @brief      Pointer for buffer with window coefficients. Data in this buffer needs to be preserved
                    (or recomputed) between frames. Window must be symmetric, therefore only numDopplerChirps / 2
                    coefficients are needed to determine the window. Window coefficients must be provided by the application.\n
                    Size: sizeof(int32_t) * numDopplerChirps / 2\n
                    Byte alignment Requirement = @ref DPU_AOAPROCDCMPDSP_BUFFERS_BYTE_ALIGNMENT
    */
    int32_t         *windowCoeff;

    /*!  @brief     2D FFT window size in bytes.*/
    uint32_t        windowSize;
    
    /*! @brief      Scratch buffer pointer for ping pong input from radar cube. \n
                    Size: 2 * sizeof(cmplx16ImRe_t) * numDopplerChirps \n
                    Byte alignment Requirement = @ref DPU_AOAPROCDCMPDSP_BUFFERS_BYTE_ALIGNMENT
     */
    cmplx16ImRe_t   *pingPongBuf; 
    
    /*! @brief      Size of the Ping pong buffer */
    uint32_t        pingPongSize;

    /*! @brief      Pointer for twiddle table for Angle FFT. Data in this buffer needs to be preserved
                    (or recomputed) between frames. This array is populated by the DPU during config time.   \n
                    Size: sizeof(cmplx32ReIm_t) * DPU_AOAPROCDCMPDSP_NUM_ANGLE_BINS\n
                    Byte alignment Requirement = @ref DPU_AOAPROCDCMPDSP_BUFFERS_BYTE_ALIGNMENT
     */
    cmplx32ReIm_t   *angleTwiddle32x32;

    /*! @brief      Size of the buffer for twiddle table */
    uint32_t        angleTwiddleSize;

    /*! @brief      Pointer for twiddle table for 2D FFT. Data in this buffer needs to be preserved
                    (or recomputed) between frames. This array is populated by the DPU during config time.   \n
                    Size: sizeof(cmplx32ReIm_t) * numDopplerBins\n
                    Byte alignment Requirement = @ref DPU_AOAPROCDCMPDSP_BUFFERS_BYTE_ALIGNMENT
     */
    cmplx32ReIm_t   *twiddle32x32;

    /*! @brief      Size of the buffer for twiddle table */
    uint32_t        twiddleSize;
     
    /*! @brief      Pointer for angle (azimuth and elevation) FFT input buffer.    \n
                    Size: sizeof(cmplx32ReIm_t) * DPU_AOAPROCDCMPDSP_NUM_ANGLE_BINS\n
                    Byte alignment Requirement = @ref DPU_AOAPROCDCMPDSP_BUFFERS_BYTE_ALIGNMENT
     */
    cmplx32ReIm_t   *angleFftIn;

    /*! @brief      Size of the buffer for angle FFT input */
    uint32_t        angleFftInSize;

    /*! @brief      Pointer for elevation FFT output buffer.    \n
                    It's also used in extended max velocity feature for temporary buffer of Azimuth FFT output of hypothesis. \n
                    Size: sizeof(cmplx32ReIm_t) * DPU_AOAPROCDCMPDSP_NUM_ANGLE_BINS\n
                    Byte alignment Requirement = @ref DPU_AOAPROCDCMPDSP_BUFFERS_BYTE_ALIGNMENT \n
                    This buffer does not need to be allocated if elevation antenna is not used or not supported,
                    and also extended max velocity feature is disabled.
     */
    cmplx32ReIm_t   *elevationFftOut;

    /*! @brief      Size of the buffer for elevation FFT output */
    uint32_t        elevationFftOutSize;

    /*! @brief      This scratch buffer is used for the following 2 operations:\n
                    1) 2D Windowing output buffer\n
                    2) Azimuth FFT output buffer\n 
                    Size: Max[(sizeof(cmplx32ReIm_t) * numDopplerBins) , (sizeof(cmplx32ReIm_t) * DPU_AOAPROCDCMPDSP_NUM_ANGLE_BINS)] \n
                    Byte alignment Requirement = @ref DPU_AOAPROCDCMPDSP_BUFFERS_BYTE_ALIGNMENT
     */
    void            *scratch1Buff;

    /*! @brief      Size of the scratch1 buffer */
    uint32_t        scratch1Size;

    /*! @brief      This scratch buffer is used for the following 2 operations:\n
                    1) 2D FFT output buffer\n
                    2) Azimuth magnitude squared buffer\n 
                    Size: Max[(sizeof(cmplx32ReIm_t) * numDopplerBins) , (sizeof(float) * DPU_AOAPROCDCMPDSP_NUM_ANGLE_BINS)] \n
                    Byte alignment Requirement = @ref DPU_AOAPROCDCMPDSP_BUFFERS_BYTE_ALIGNMENT
     */
    void            *scratch2Buff;

    /*! @brief      Size of the scratch2 buffer */
    uint32_t        scratch2Size;
} DPU_AoAProcDcmpDSP_HW_Resources;

/**
 * @brief
 *  AoAProcDcmpDSP DPU static configuration parameters
 *
 *  \ingroup DPU_AOAPROCDCMP_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_AoAProcDcmpDSP_StaticConfig_t
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
               i.e numDopplerChirps = numChirpsPerFrame / numTxAntennas.\n
               Must be multiple of 4. */
    uint16_t    numDopplerChirps;

    /*! @brief  Number of doppler bins. Must be at least 16. Must be power of 2. */
    uint16_t    numDopplerBins;

    /*! @brief  Range conversion factor for range FFT index to meters */
    float       rangeStep;

    /*! @brief  Doppler conversion factor for Doppler FFT index to m/s */
    float       dopplerStep;
    
    /*! @brief Flag that indicates if BPM is enabled. 
    BPM can only be enabled/disabled during configuration time.*/
    bool        isBpmEnabled;

} DPU_AoAProcDcmpDSP_StaticConfig;

/**
 * @brief
 *  AoAProcDcmpDSP DPU configuration
 *
 * @details
 *  The structure is used to hold the AoAProcDcmpDSP configuration
 *
 *  \ingroup DPU_AOAPROCDCMP_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_AoAProcDcmpDSP_Config_t
{
    /*! @brief     Data path common parameters */
    DPU_AoAProcDcmpDSP_StaticConfig   staticCfg;

    /*! @brief     Hardware resources */
    DPU_AoAProcDcmpDSP_HW_Resources res;

    /*! @brief     Dynamic configuration */
    DPU_AoAProcDcmp_DynamicConfig dynCfg;

}DPU_AoAProcDcmpDSP_Config;

/**
 * @brief
 *  Output parameters populated during Processing time
 *
 * @details
 *  The structure is used to hold the output parameters
 *
 *  \ingroup DPU_AOAPROCDCMP_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_AoAProcDcmpDSP_OutParams_t
{

    /*! @brief     AoAProcDcmpDSP stats */
    DPU_AoAProcDcmp_Stats       stats;

    /*! @brief      Number of AoA DPU detected points*/
    uint32_t numAoADetectedPoints;

}DPU_AoAProcDcmpDSP_OutParams;

/**
 * @brief
 *  AoAProcDcmpDSP DPU Handle
 *
 *
 *  \ingroup DPU_AOAPROCDCMP_EXTERNAL_DATA_STRUCTURE
 *
 */
typedef void* DPU_AoAProcDcmpDSP_Handle ;

/**
 *  @b Description
 *  @n
 *      The function is AoAProcDcmpDSP DPU initialization function. It allocates memory to store
 *  its internal data object and returns a handle if it executes successfully.
 *
 *  @param[out]  errCode                 Pointer to errCode generates from the API
 *
 *  \ingroup    DPU_AOAPROCDCMP_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - valid AoAProcDcmpDSP handle
 *  @retval
 *      Error       - NULL
 */
DPU_AoAProcDcmpDSP_Handle DPU_AoAProcDcmpDSP_init
(
    int32_t*            errCode
);

/**
 *  @b Description
 *  @n
 *      The function is AoAProcDcmpDSP DPU configuration function. It saves buffer pointer and configurations
 *  including system resources and configures EDMA for runtime.
 *
 *  @pre    DPU_AoAProcDcmpDSP_init() has been called
 *
 *  @param[in]  handle                 AoAProcDcmpDSP DPU handle
 *  @param[in]  aoaDspCfg              Pointer to AoAProcDcmpDSP configuration data structure
 *
 *  \ingroup    DPU_AOAPROCDCMP_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_AoAProcDcmpDSP_config
(
    DPU_AoAProcDcmpDSP_Handle    handle,
    DPU_AoAProcDcmpDSP_Config    *aoaDspCfg
);

/**
 *  @b Description
 *  @n
 *      The function is AoAProcDcmpDSP DPU process function.
 *
 *  @pre    DPU_AoAProcDcmpDSP_init(), DPU_AoAProcDcmpDSP_config() have been called
 *
 *  @param[in]  handle                  AoAProcDcmpDSP DPU handle
 *
 *  @param[in]  numObjsIn               Number of detected objects by CFAR DPU
 *
 *  @param[in]  outParams               DPU output parameters
 *
 *  \ingroup    DPU_AOAPROCDCMP_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success = 0
 *  @retval
 *      Error  != 0
 */
uint32_t DPU_AoAProcDcmpDSP_process
(
    DPU_AoAProcDcmpDSP_Handle    handle,
    uint32_t                 numObjsIn,
    DPU_AoAProcDcmpDSP_OutParams *outParams
);

/**
 *  @b Description
 *  @n
 *      The function is AoAProcDcmpDSP DPU control function.
 *
 *  @pre     DPU_AoAProcDcmpDSP_init() has been called
 *
 *  @param[in]  handle           AoAProcDcmpDSP DPU handle
 *  @param[in]  cmd              AoAProcDcmpDSP DPU control command
 *  @param[in]  arg              AoAProcDcmpDSP DPU control argument pointer
 *  @param[in]  argSize          AoAProcDcmpDSP DPU control argument size
 *
 *  \ingroup    DPU_AOAPROCDCMP_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_AoAProcDcmpDSP_control
(
   DPU_AoAProcDcmpDSP_Handle handle,
   DPU_AoAProcDcmpDSP_Cmd cmd,
   void *arg,
   uint32_t argSize
);

/**
 *  @b Description
 *  @n
 *      The function is AoAProcDcmpDSP DPU deinitialization function. It frees up the
 *   resources allocated during initialization.
 *
 *  @pre    DPU_AoAProcDcmpDSP_init() has been called
 *
 *  @param[in]  handle           AoAProcDcmpDSP DPU handle
 *
 *  \ingroup    DPU_AOAPROCDCMP_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - 0
 *  @retval
 *      Error       - <0
 */
int32_t DPU_AoAProcDcmpDSP_deinit
(
    DPU_AoAProcDcmpDSP_Handle handle
);

#ifdef __cplusplus
}
#endif

#endif
