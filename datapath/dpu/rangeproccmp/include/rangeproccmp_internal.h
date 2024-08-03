/*
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
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
 *   @file  rangeproccmp_internal.h
 *
 *   @brief
 *      Includes common definitions for rangeProcCmpHWA and rangeProcDSP.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#ifndef RANGEPROCCMP_INTERNAL_H
#define RANGEPROCCMP_INTERNAL_H

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
/* Compression constants. */
#include <ti/demo/xwr64xx_compression/mmw/compressionConsts.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief
 *  Rangeproc supported Radar cube layout format
 *
 * @details
 *  The enumeration describes the radar cube layout format
 *
 *  \ingroup DPU_RANGEPROCCMP_INTERNAL_DATA_STRUCTURE
 */
typedef enum rangeProcRadarCubeLayoutFmt_e
{
    /*! @brief  Data layout: range-Doppler-TxAnt - RxAnt */
    rangeProcCmp_dataLayout_RANGE_DOPPLER_TxAnt_RxAnt,

    /*! @brief  Data layout: TxAnt->doppler->RxAnt->range */
    rangeProcCmp_dataLayout_TxAnt_DOPPLER_RxAnt_RANGE
}rangeProcRadarCubeLayoutFmt;

/**
 * @brief
 *  Data path common parameters needed by RangeProcCmp
 *
 * @details
 *  The structure is used to hold the data path parameters used by both rangeProcCmpHWA and rangeProcDspHWA DPUs.
 *
 *  \ingroup DPU_RANGEPROCCMP_INTERNAL_DATA_STRUCTURE
 *
 */
typedef struct rangeProcCmp_dpParams_t
{
    /*! @brief  Number of transmit antennas */
    uint8_t     numTxAntennas;

    /*! @brief  Number of receive antennas */
    uint8_t     numRxAntennas;

    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas;

    /*! @brief  ADCBUF will generate chirp interrupt event every this many chirps */
    uint8_t     numChirpsPerChirpEvent;

    /*! @brief  Number of ADC samples */
    uint16_t    numAdcSamples;

    /*! @brief  Number of range bins */
    uint16_t    numRangeBins;

    /*! @brief  Number of chirps per frame */
    uint16_t    numChirpsPerFrame;

    /*! @brief  Number of chirps for Doppler computation purposes. */
    uint16_t    numDopplerChirps;
	
	/*! @brief      Compression module configuration */
    DPC_ObjectDetectionCmp_CompressCfg  compressCfg;

	
}rangeProcCmp_dpParams;

#ifdef __cplusplus
}
#endif

#endif
