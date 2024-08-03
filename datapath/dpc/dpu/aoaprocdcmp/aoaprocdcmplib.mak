###################################################################################
# aoaprocdcmp Library Makefile
###################################################################################
.PHONY: aoaprocdcmpLib aoaprocdcmpLibClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c src

###################################################################################
# Library Source Files:
###################################################################################
# HWA applicable only to specific platforms
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
AOAPROCDCMP_HWA_LIB_SOURCES = aoaprocdcmphwa.c
endif
AOAPROCDCMP_DSP_LIB_SOURCES = aoaprocdcmpdsp.c 			

###################################################################################
# Library objects
#     Build for R4 and DSP
###################################################################################
# HWA applicable only to specific platforms
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
AOAPROCDCMP_HWA_R4F_DRV_LIB_OBJECTS  = $(addprefix $(PLATFORM_OBJDIR)/, $(AOAPROCDCMP_HWA_LIB_SOURCES:.c=.$(R4F_OBJ_EXT)))
AOAPROCDCMP_HWA_C674_DRV_LIB_OBJECTS = $(addprefix $(PLATFORM_OBJDIR)/, $(AOAPROCDCMP_HWA_LIB_SOURCES:.c=.$(C674_OBJ_EXT)))
endif

AOAPROCDCMP_DSP_C674_DRV_LIB_OBJECTS = $(addprefix $(PLATFORM_OBJDIR)/, $(AOAPROCDCMP_DSP_LIB_SOURCES:.c=.$(C674_OBJ_EXT)))

###################################################################################
# Library Dependency:
###################################################################################
# HWA applicable only to specific platforms
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
AOAPROCDCMP_HWA_R4F_DRV_DEPENDS  = $(addprefix $(PLATFORM_OBJDIR)/, $(AOAPROCDCMP_HWA_LIB_SOURCES:.c=.$(R4F_DEP_EXT)))
AOAPROCDCMP_HWA_C674_DRV_DEPENDS = $(addprefix $(PLATFORM_OBJDIR)/, $(AOAPROCDCMP_HWA_LIB_SOURCES:.c=.$(C674_DEP_EXT)))
endif

AOAPROCDCMP_DSP_C674_DRV_DEPENDS = $(addprefix $(PLATFORM_OBJDIR)/, $(AOAPROCDCMP_DSP_LIB_SOURCES:.c=.$(C674_DEP_EXT)))

###################################################################################
# Library Names:
###################################################################################
# HWA applicable only to specific platforms
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
AOAPROCDCMP_HWA_R4F_DRV_LIB  = lib/libaoaprocdcmp_hwa_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)
AOAPROCDCMP_HWA_C674_DRV_LIB = lib/libaoaprocdcmp_hwa_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)
endif

AOAPROCDCMP_DSP_C674_DRV_LIB = lib/libaoaprocdcmp_dsp_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)

###################################################################################
# Library Build:
#     - Build the R4 & DSP Library
###################################################################################
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
aoaprocdcmpHWALib: buildDirectories $(AOAPROCDCMP_HWA_R4F_DRV_LIB_OBJECTS) $(AOAPROCDCMP_HWA_C674_DRV_LIB_OBJECTS)
	if [ ! -d "lib" ]; then mkdir lib; fi
	echo "Archiving $@"
	$(R4F_AR) $(R4F_AR_OPTS) $(AOAPROCDCMP_HWA_R4F_DRV_LIB) $(AOAPROCDCMP_HWA_R4F_DRV_LIB_OBJECTS)
	$(C674_AR) $(C674_AR_OPTS) $(AOAPROCDCMP_HWA_C674_DRV_LIB) $(AOAPROCDCMP_HWA_C674_DRV_LIB_OBJECTS)
endif

aoaprocdcmpDSPLib: C674_CFLAGS += -i$(C674x_MATHLIB_INSTALL_PATH)/packages \
								-i$(C64Px_DSPLIB_INSTALL_PATH)/packages/ti/dsplib/src/DSP_fft32x32/c64P
aoaprocdcmpDSPLib: buildDirectories $(AOAPROCDCMP_DSP_C674_DRV_LIB_OBJECTS)
	if [ ! -d "lib" ]; then mkdir lib; fi
	echo "Archiving $@"
	$(C674_AR) $(C674_AR_OPTS) $(AOAPROCDCMP_DSP_C674_DRV_LIB) $(AOAPROCDCMP_DSP_C674_DRV_LIB_OBJECTS)

ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
aoaprocdcmpLib: aoaprocdcmpHWALib aoaprocdcmpDSPLib
else
aoaprocdcmpLib: aoaprocdcmpDSPLib
endif

###################################################################################
# Clean the Libraries
###################################################################################
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
aoaprocdcmpHWALibClean:
	@echo 'Cleaning the aoaprocdcmp Library Objects'
	@$(DEL) $(AOAPROCDCMP_HWA_R4F_DRV_LIB_OBJECTS) $(AOAPROCDCMP_HWA_R4F_DRV_LIB)
	@$(DEL) $(AOAPROCDCMP_HWA_C674_DRV_LIB_OBJECTS) $(AOAPROCDCMP_HWA_C674_DRV_LIB)
	@$(DEL) $(AOAPROCDCMP_HWA_R4F_DRV_DEPENDS) $(AOAPROCDCMP_HWA_C674_DRV_DEPENDS)
	@$(DEL) $(PLATFORM_OBJDIR)
endif

aoaprocdcmpDSPLibClean:
	@echo 'Cleaning the aoaprocdcmp DSP Library Objects'
	@$(DEL) $(AOA_DSP_C674_DRV_LIB_OBJECTS) $(AOA_DSP_C674_DRV_LIB)
	@$(DEL) $(AOA_DSP_C674_DRV_DEPENDS)
	@$(DEL) $(PLATFORM_OBJDIR)

ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
aoaprocdcmpLibClean: aoaprocdcmpHWALibClean aoaprocdcmpDSPLibClean
else
aoaprocdcmpLibClean: aoaprocdcmpDSPLibClean
endif

###################################################################################
# Dependency handling
###################################################################################
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
-include $(AOAPROCDCMP_HWA_R4F_DRV_DEPENDS)
-include $(AOAPROCDCMP_HWA_C674_DRV_DEPENDS)
endif
-include $(AOAPROCDCMP_DSP_C674_DRV_DEPENDS)

