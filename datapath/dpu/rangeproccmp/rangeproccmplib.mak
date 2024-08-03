###################################################################################
# rangeproccmpCMP Library Makefile
###################################################################################
.PHONY: rangeproccmpLib rangeproccmpLibClean

###################################################################################
# Setup the VPATH:
###################################################################################
vpath %.c src
vpath %.c platform

###################################################################################
# Library Source Files:
###################################################################################
# HWA applicable only to specific platforms
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
RANGEPROCCMP_HWA_LIB_SOURCES = rangeproccmphwa.c 			
endif
RANGEPROCCMP_DSP_LIB_SOURCES = rangeproccmpdsp.c 			

###################################################################################
# Library objects
#     Build for R4 and DSP
###################################################################################
# HWA applicable only to specific platforms
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
RANGEPROCCMP_HWA_R4F_DRV_LIB_OBJECTS  = $(addprefix $(PLATFORM_OBJDIR)/, $(RANGEPROCCMP_HWA_LIB_SOURCES:.c=.$(R4F_OBJ_EXT)))
RANGEPROCCMP_HWA_C674_DRV_LIB_OBJECTS = $(addprefix $(PLATFORM_OBJDIR)/, $(RANGEPROCCMP_HWA_LIB_SOURCES:.c=.$(C674_OBJ_EXT)))
endif

RANGEPROCCMP_DSP_C674_DRV_LIB_OBJECTS = $(addprefix $(PLATFORM_OBJDIR)/, $(RANGEPROCCMP_DSP_LIB_SOURCES:.c=.$(C674_OBJ_EXT)))

###################################################################################
# Library Dependency:
###################################################################################
# HWA applicable only to specific platforms
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
RANGEPROCCMP_HWA_R4F_DRV_DEPENDS  = $(addprefix $(PLATFORM_OBJDIR)/, $(RANGEPROCCMP_HWA_LIB_SOURCES:.c=.$(R4F_DEP_EXT)))
RANGEPROCCMP_HWA_C674_DRV_DEPENDS = $(addprefix $(PLATFORM_OBJDIR)/, $(RANGEPROCCMP_HWA_LIB_SOURCES:.c=.$(C674_DEP_EXT)))
endif

RANGEPROCCMP_DSP_C674_DRV_DEPENDS = $(addprefix $(PLATFORM_OBJDIR)/, $(RANGEPROCCMP_DSP_LIB_SOURCES:.c=.$(C674_DEP_EXT)))

###################################################################################
# Library Names:
###################################################################################
# HWA applicable only to specific platforms
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
RANGEPROCCMP_HWA_R4F_DRV_LIB  = lib/librangeproccmp_hwa_$(MMWAVE_SDK_DEVICE_TYPE).$(R4F_LIB_EXT)
RANGEPROCCMP_HWA_C674_DRV_LIB = lib/librangeproccmp_hwa_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)
endif

RANGEPROCCMP_DSP_C674_DRV_LIB = lib/librangeproccmp_dsp_$(MMWAVE_SDK_DEVICE_TYPE).$(C674_LIB_EXT)

###################################################################################
# Library Build:
#     - Build the R4 & DSP Library
###################################################################################
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
rangeproccmpHWALib: buildDirectories $(RANGEPROCCMP_HWA_R4F_DRV_LIB_OBJECTS) $(RANGEPROCCMP_HWA_C674_DRV_LIB_OBJECTS)
	if [ ! -d "lib" ]; then mkdir lib; fi
	echo "Archiving $@"
	$(R4F_AR) $(R4F_AR_OPTS) $(RANGEPROCCMP_HWA_R4F_DRV_LIB) $(RANGEPROCCMP_HWA_R4F_DRV_LIB_OBJECTS)
	$(C674_AR) $(C674_AR_OPTS) $(RANGEPROCCMP_HWA_C674_DRV_LIB) $(RANGEPROCCMP_HWA_C674_DRV_LIB_OBJECTS)
endif

rangeproccmpDSPLib: C674_CFLAGS += -i$(C674x_MATHLIB_INSTALL_PATH)/packages \
								-i$(C64Px_DSPLIB_INSTALL_PATH)/packages/ti/dsplib/src/DSP_fft16x16_imre/c64P
rangeproccmpDSPLib: buildDirectories $(RANGEPROCCMP_DSP_C674_DRV_LIB_OBJECTS)
	if [ ! -d "lib" ]; then mkdir lib; fi
	echo "Archiving $@"
	$(C674_AR) $(C674_AR_OPTS) $(RANGEPROCCMP_DSP_C674_DRV_LIB) $(RANGEPROCCMP_DSP_C674_DRV_LIB_OBJECTS)

ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
rangeproccmpLib: rangeproccmpHWALib rangeproccmpDSPLib
else
rangeproccmpLib: rangeproccmpDSPLib
endif

###################################################################################
# Clean the Libraries
###################################################################################
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
rangeproccmpHWALibClean:
	@echo 'Cleaning the rangeproccmp HWA Library Objects'
	@$(DEL) $(RANGEPROCCMP_HWA_R4F_DRV_LIB_OBJECTS) $(RANGEPROCCMP_HWA_R4F_DRV_LIB)
	@$(DEL) $(RANGEPROCCMP_HWA_C674_DRV_LIB_OBJECTS) $(RANGEPROCCMP_HWA_C674_DRV_LIB)
	@$(DEL) $(RANGEPROCCMP_HWA_R4F_DRV_DEPENDS) $(RANGEPROCCMP_HWA_C674_DRV_DEPENDS)
	@$(DEL) $(PLATFORM_OBJDIR)
endif

rangeproccmpDSPLibClean:
	@echo 'Cleaning the rangeproccmp DSP Library Objects'
	@$(DEL) $(RANGEPROCCMP_DSP_C674_DRV_LIB_OBJECTS) $(RANGEPROCCMP_DSP_C674_DRV_LIB)
	@$(DEL) $(RANGEPROCCMP_DSP_C674_DRV_DEPENDS)
	@$(DEL) $(PLATFORM_OBJDIR)

ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
rangeproccmpLibClean: rangeproccmpHWALibClean rangeproccmpDSPLibClean
else
rangeproccmpLibClean: rangeproccmpDSPLibClean
endif

###################################################################################
# Dependency handling
###################################################################################
ifneq ($(filter $(MMWAVE_SDK_DEVICE_TYPE),xwr18xx xwr68xx), )
-include $(RANGEPROCCMP_HWA_R4F_DRV_DEPENDS)
-include $(RANGEPROCCMP_HWA_C674_DRV_DEPENDS)
endif
-include $(RANGEPROCCMP_DSP_C674_DRV_DEPENDS)

