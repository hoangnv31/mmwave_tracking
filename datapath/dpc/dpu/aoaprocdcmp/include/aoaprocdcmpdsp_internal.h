/**
 *   @file  aoaprocdcmpdsp_internal.h
 *
 *   @brief
 *      Internal header for DSP AoA DPU.
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
#ifndef AOAPROCDCMPDSP_INTERNAL_H
#define AOAPROCDCMPDSP_INTERNAL_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/* mmWave SDK Include Files */
#include <ti/common/sys_common.h>
#include <ti/utils/cycleprofiler/cycle_profiler.h>
#include <ti/datapath/dpc/dpu/aoaprocdcmp/aoaprocdcmpdsp.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! Ping index used for EDMA-CPU processing parallelism */
#define DPU_AOAPROCDCMPDSP_PING_IDX 0

/*! Pong index used for EDMA-CPU processing parallelism */
#define DPU_AOAPROCDCMPDSP_PONG_IDX 1

/**
 * @brief
 *  Field of view - AoA Configuration
 *
 * @details
 *  The structure contains the field of view - Angle of arrival configuration used in data path
 *
 *  \ingroup DPU_AOAPROCDCMP_INTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_AoAProcDcmp_fovAoaLocalCfg_t
{
    /*! @brief    minimum azimuth angle (expressed as sine value) */
    float        minAzimuthSineVal;

    /*! @brief    maximum azimuth angle (expressed as sine value) */
    float        maxAzimuthSineVal;

    /*! @brief    minimum elevation angle (expressed as sine value) */
    float        minElevationSineVal;

    /*! @brief    maximum elevation angle (expressed as sine value) */
    float        maxElevationSineVal;
} DPU_AoAProcDcmp_fovAoaLocalCfg;


/**
 * @brief
 *  AoAProcDcmpDSP DPU dynamic configuration
 *
 * @details
 *  The structure is used to hold the dynamic configuration used for AoAProcDcmpDSP DPU
 *
 *  \ingroup DPU_AOAPROCDCMP_INTERNAL_DATA_STRUCTURE
 *
 */
typedef struct DPU_AoAProcDcmp_DynamicLocalConfig_t
{

    /*! @brief     Multi object beam forming configuration */
    DPU_AoAProcDcmp_MultiObjBeamFormingCfg multiObjBeamFormingCfg;

    /*! @brief     Flag indicates to prepare data for azimuth heat-map */
    bool  prepareRangeAzimuthHeatMap;

    /*! @brief     Pointer to Rx channel compensation configuration */
    DPU_AoAProcDcmp_compRxChannelBiasCfg compRxChanCfg;

    /*! @brief      Field of view configuration for AoA */
    DPU_AoAProcDcmp_fovAoaLocalCfg     fovAoaLocalCfg;

    /*! @brief Static clutter removal configuration. Valid only for DSP version of the AoA DPU.*/
    DPU_AoAProcDcmp_StaticClutterRemovalCfg  staticClutterCfg;

    /*! @brief      Extended maximum velocity configuration */
    DPU_AoAProcDcmp_ExtendedMaxVelocityCfg extMaxVelCfg;
	
	
	
} DPU_AoAProcDcmp_DynamicLocalConfig;


/**
 @}
 */

/**
 * @brief
 *  AoAProcDcmpDSP DPU internal data Object
 *
 * @details
 *  The structure is used to hold AoAProcDcmpDSP DPU internal data object
 *
 *  \ingroup DPU_AOAPROCDCMP_INTERNAL_DATA_STRUCTURE
 *
 */
typedef struct AOADspObj_t
{
    /*! @brief     Data path common parameters */
    DPU_AoAProcDcmpDSP_StaticConfig   params;

    /*! @brief     AoA DPU hardware resources */
    DPU_AoAProcDcmpDSP_HW_Resources res;

    /*! @brief     Dynamic configuration */
    DPU_AoAProcDcmp_DynamicLocalConfig dynLocalCfg;

}AOADspObj;


#ifdef __cplusplus
}
#endif

#endif 
