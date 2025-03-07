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
 *   @file  rangeproccmp_common.h
 *
 *   @brief
 *      Implements Common definition across rangeProcCmp DPU.
 */

/**
 * @mainpage RangeProcCmp DPU
 * [TOC]
 * This DPU implements range processing and compression of range-FFT outputs using hardware accelerator(HWA).
 *
 *  @section dpu_range_intro Introduction
 *
 *  Description
 *  ----------------
 *
 *  Range Processing Unit takes RF data in ADC buffer, computes 1D FFT, compresses it and saves results in
 * radarCube in the requested format.
 *
 *  Range Processing includes only one DPU implementation:
 *
 *   DPU         |  runs on cores
 *  :------------|:----------------
 *  rangeProcCmpHWA|  R4F or DSP
 *
 *
 *  Data Interface
 *  ----------------
 *
 *  Range processing only supports 16bits complex data in ImRe format (@ref DPIF_DATAFORMAT_COMPLEX16_IMRE).\n
 *  Range processing data Input interface is defined by @ref DPIF_ADCBufData_t.\n
 *  Range processing data Output interface is define by @ref DPIF_RadarCube_t.\n
 *  The parameters for radarCube are defined in @ref DPU_RangeProcCmpHWA_StaticConfig_t or @ref DPU_RangeProcCmpDSP_StaticConfig_t.
 *
 *  rangeProcCmpHWA has the same data interface as described above. However rangeProcCmpHWA
 *  supports more input and output formats. Refer to each DPU for more details.
 *
 *
 *  Antenna Coupling Signature Removal
 *  ----------------
 *
 *  Antenna coupling signature dominates the range bins close to the radar.
 *  These are the bins in the range FFT output located around DC.
 *
 *  This feature can be enabled/disabled through configuration @ref DPU_RangeProcCmp_CalibDcRangeSigCfg_t at configure time or at run time.
 *
 *  After the feature is enabled, it does measurements for
 *  @ref DPU_RangeProcCmp_CalibDcRangeSigCfg_t::numAvgChirps doppler chirps.
 *  During measurement, each of the specified range bins
 *  ([@ref DPU_RangeProcCmp_CalibDcRangeSigCfg_t::negativeBinIdx,
 *  @ref DPU_RangeProcCmp_CalibDcRangeSigCfg_t::positiveBinIdx]) for each of the
 *  virtual antennas are accumulated over the specified number of chirps ("numAvgChirps")
 *  and at the end of the period, the average is computed for each range bin
 *  and each virtual antenna combination.
 *
 *  It is assumed that no objects are present in the vicinity of the radar during
 *  this measurement period. After measurement is done, the removal starts for
 *  all subsequent frames for each range bin and virtual  antenna,  average estimate is
 *  subtracted from the corresponding received samples in real-time for subsequent processing.
 *
 *  @note The number of chirps to average(numAvgChirps) must be power of 2.
 *
 *  RangeProcCmp DPUs
 *  ---------------------
 *  - RangeProcCmp using HWA \ref dpu_rangehwa
 *  - RangeProcCmp using DSP \ref dpu_rangedsp
 *
 */
/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#ifndef RANGEPROCCMP_COMMON_H
#define RANGEPROCCMP_COMMON_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

/**
@defgroup DPU_RANGEPROCCMP_EXTERNAL_FUNCTION            rangeProcCmp DPU External Functions
@ingroup RANGE_PROC_DPU
@brief
*   The section has a list of all the exported API which the applications need to
*   invoke in order to use the rangeProcCmp DPU
*/
/**
@defgroup DPU_RANGEPROCCMP_EXTERNAL_DATA_STRUCTURE      rangeProcCmp DPU External Data Structures
@ingroup RANGE_PROC_DPU
@brief
*   The section has a list of all the data structures which are exposed to the application
*/
/**
@defgroup DPU_RANGEPROCCMP_ERROR_CODE                   rangeProcCmp DPU Error Codes
@ingroup RANGE_PROC_DPU
@brief
*   The section has a list of all the error codes which are generated by the sampleProc DPU
*/
/**
@defgroup DPU_RANGEPROCCMP_INTERNAL_FUNCTION            rangeProcCmp DPU Internal Functions
@ingroup RANGE_PROC_DPU
@brief
*   The section has a list of all internal API which are not exposed to the external
*   applications.
*/
/**
@defgroup DPU_RANGEPROCCMP_INTERNAL_DATA_STRUCTURE      rangeProcCmp DPU Internal Data Structures
@ingroup RANGE_PROC_DPU
@brief
*   The section has a list of all internal data structures which are used internally
*   by the rangeProcCmp DPU module.
*/
/**
@defgroup DPU_RANGEPROCCMP_INTERNAL_DEFINITION          rangeProcCmp DPU Internal Definitions
@ingroup RANGE_PROC_DPU
@brief
*   The section has a list of all internal definitions which are used internally
*   by the rangeProcCmp DPU.
*/

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Maximum number of 1D FFT bins in DC range antenna signature compensation */
#define DPU_RANGEPROCCMP_SIGNATURE_COMP_MAX_BIN_SIZE               32

/**
 * @brief
 *  Data Path DC range signature compensation
 *
 * @details
 *  The structure contains the DC range antenna signature removeal configuration used in data path
 *
 *  \ingroup DPU_RANGEPROCCMP_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcCmp_CalibDcRangeSigCfg_t
{
    /*! @brief    enabled flag:  1-enabled 0-disabled */
    uint16_t    enabled;

    /*! @brief    maximum negative range bin (1D FFT index) to be compensated */
    int16_t     negativeBinIdx;

    /*! @brief    maximum positive range bin (1D FFT index) to be compensated */
    int16_t     positiveBinIdx;

    /*! @brief    number of chirps in the averaging phase */
    uint16_t    numAvgChirps;
} DPU_RangeProcCmp_CalibDcRangeSigCfg;

/**
 * @brief
 *  Data processing Unit statistics
 *
 * @details
 *  The structure is used to hold the statistics of the DPU
 *
 *  \ingroup DPU_RANGEPROCCMP_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcCmp_stats_t
{
    /*! @brief total processing time during all chirps in a frame excluding EDMA waiting time*/
    uint32_t            processingTime;

    /*! @brief total wait time for EDMA data transfer during all chirps in a frame*/
    uint32_t            waitTime;
}DPU_RangeProcCmp_stats;

#ifdef __cplusplus
}
#endif

#endif
