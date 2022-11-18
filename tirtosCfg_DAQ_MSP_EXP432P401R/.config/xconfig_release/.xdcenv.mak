#
_XDCBUILDCOUNT = 0
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/ti/simplelink_msp432p4_sdk_3_40_01_02/source;C:/ti/simplelink_msp432p4_sdk_3_40_01_02/kernel/tirtos/packages;E:/Documents/Courses/MakingEmbeddedSystems/final_project/tirtosCfg_DAQ_MSP_EXP432P401R/.config
override XDCROOT = C:/ti/ccs1210/xdctools_3_62_01_16_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/ti/simplelink_msp432p4_sdk_3_40_01_02/source;C:/ti/simplelink_msp432p4_sdk_3_40_01_02/kernel/tirtos/packages;E:/Documents/Courses/MakingEmbeddedSystems/final_project/tirtosCfg_DAQ_MSP_EXP432P401R/.config;C:/ti/ccs1210/xdctools_3_62_01_16_core/packages;..
HOSTOS = Windows
endif
