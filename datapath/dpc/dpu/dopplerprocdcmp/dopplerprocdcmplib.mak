###################################################################################
# dopplerprocdcmp Library Makefile
###################################################################################
.PHONY: dopplerprocdcmpLib dopplerprocdcmpLibClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c src

###################################################################################
# Library Source Files:
###################################################################################
# HWA applicable only to specific platforms
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
DOPPLERPROCDCMP_HWA_LIB_SOURCES = dopplerprocdcmphwa.c
endif
DOPPLERPROCDCMP_DSP_LIB_SOURCES = dopplerprocdcmpdsp.c 			

###################################################################################
# Library objects
#     Build for R4 and DSP
###################################################################################
# HWA applicable only to specific platforms
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
DOPPLERPROCDCMP_HWA_R4F_DRV_LIB_OBJECTS  = $(addprefix $(PLATFORM_OBJDIR)/, $(DOPPLERPROCDCMP_HWA_LIB_SOURCES:.c=.$(R4F_OBJ_EXT)))
DOPPLERPROCDCMP_HWA_C674_DRV_LIB_OBJECTS = $(addprefix $(PLATFORM_OBJDIR)/, $(DOPPLERPROCDCMP_HWA_LIB_SOURCES:.c=.$(C674_OBJ_EXT)))
endif

DOPPLERPROCDCMP_DSP_C674_DRV_LIB_OBJECTS = $(addprefix $(PLATFORM_OBJDIR)/, $(DOPPLERPROCDCMP_DSP_LIB_SOURCES:.c=.$(C674_OBJ_EXT)))

###################################################################################
# Library Dependency:
###################################################################################
# HWA applicable only to specific platforms
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
DOPPLERPROCDCMP_HWA_R4F_DRV_DEPENDS  = $(addprefix $(PLATFORM_OBJDIR)/, $(DOPPLERPROCDCMP_HWA_LIB_SOURCES:.c=.$(R4F_DEP_EXT)))
DOPPLERPROCDCMP_HWA_C674_DRV_DEPENDS = $(addprefix $(PLATFORM_OBJDIR)/, $(DOPPLERPROCDCMP_HWA_LIB_SOURCES:.c=.$(C674_DEP_EXT)))
endif

DOPPLERPROCDCMP_DSP_C674_DRV_DEPENDS = $(addprefix $(PLATFORM_OBJDIR)/, $(DOPPLERPROCDCMP_DSP_LIB_SOURCES:.c=.$(C674_DEP_EXT)))

###################################################################################
# Library Names:
###################################################################################
# HWA applicable only to specific platforms
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
DOPPLERPROCDCMP_HWA_R4F_DRV_LIB  = lib/libdopplerprocdcmp_hwa_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)
DOPPLERPROCDCMP_HWA_C674_DRV_LIB = lib/libdopplerprocdcmp_hwa_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)
endif

DOPPLERPROCDCMP_DSP_C674_DRV_LIB = lib/libdopplerprocdcmp_dsp_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)

###################################################################################
# Library Build:
#     - Build the R4 & DSP Library
###################################################################################
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
dopplerprocdcmpHWALib: buildDirectories $(DOPPLERPROCDCMP_HWA_R4F_DRV_LIB_OBJECTS) $(DOPPLERPROCDCMP_HWA_C674_DRV_LIB_OBJECTS)
	if [ ! -d "lib" ]; then mkdir lib; fi
	echo "Archiving $@"
	$(R4F_AR) $(R4F_AR_OPTS) $(DOPPLERPROCDCMP_HWA_R4F_DRV_LIB) $(DOPPLERPROCDCMP_HWA_R4F_DRV_LIB_OBJECTS)
	$(C674_AR) $(C674_AR_OPTS) $(DOPPLERPROCDCMP_HWA_C674_DRV_LIB) $(DOPPLERPROCDCMP_HWA_C674_DRV_LIB_OBJECTS)
endif

dopplerprocdcmpDSPLib: C674_CFLAGS += -i$(C674x_MATHLIB_INSTALL_PATH)/packages \
								-i$(C64Px_DSPLIB_INSTALL_PATH)/packages/ti/dsplib/src/DSP_fft32x32/c64P
dopplerprocdcmpDSPLib: buildDirectories $(DOPPLERPROCDCMP_DSP_C674_DRV_LIB_OBJECTS)
	if [ ! -d "lib" ]; then mkdir lib; fi
	echo "Archiving $@"
	$(C674_AR) $(C674_AR_OPTS) $(DOPPLERPROCDCMP_DSP_C674_DRV_LIB) $(DOPPLERPROCDCMP_DSP_C674_DRV_LIB_OBJECTS)

ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
dopplerprocdcmpLib: dopplerprocdcmpHWALib dopplerprocdcmpDSPLib
else
dopplerprocdcmpLib: dopplerprocdcmpDSPLib
endif

###################################################################################
# Clean the Libraries
###################################################################################
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
dopplerprocdcmpHWALibClean:
	@echo 'Cleaning the HWA dopplerprocdcmp Library Objects'
	@$(DEL) $(DOPPLERPROCDCMP_HWA_R4F_DRV_LIB_OBJECTS) $(DOPPLERPROCDCMP_HWA_R4F_DRV_LIB)
	@$(DEL) $(DOPPLERPROCDCMP_HWA_C674_DRV_LIB_OBJECTS) $(DOPPLERPROCDCMP_HWA_C674_DRV_LIB)
	@$(DEL) $(DOPPLERPROCDCMP_HWA_R4F_DRV_DEPENDS) $(DOPPLERPROCDCMP_HWA_C674_DRV_DEPENDS)
	@$(DEL) $(PLATFORM_OBJDIR)
endif

dopplerprocdcmpDSPLibClean:
	@echo 'Cleaning the dopplerprocdcmp DSP Library Objects'
	@$(DEL) $(DOPPLER_DSP_C674_DRV_LIB_OBJECTS) $(DOPPLER_DSP_C674_DRV_LIB)
	@$(DEL) $(DOPPLER_DSP_C674_DRV_DEPENDS)
	@$(DEL) $(PLATFORM_OBJDIR)

ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
dopplerprocdcmpLibClean: dopplerprocdcmpHWALibClean dopplerprocdcmpDSPLibClean
else
dopplerprocdcmpLibClean: dopplerprocdcmpDSPLibClean
endif

###################################################################################
# Dependency handling
###################################################################################
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
-include $(DOPPLERPROCDCMP_HWA_R4F_DRV_DEPENDS)
-include $(DOPPLERPROCDCMP_HWA_C674_DRV_DEPENDS)
endif
-include $(DOPPLERPROCDCMP_DSP_C674_DRV_DEPENDS)

