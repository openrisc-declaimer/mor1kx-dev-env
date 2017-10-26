# A make rule that creates the test defines verilog file.

# Test defines.v file, called recursively, .PHONY to force its generation
.PHONY: $(TEST_DEFINES_VLG)
$(TEST_DEFINES_VLG):
	$(Q)echo "\`define "$(SIM_TYPE)"_SIM" > $@
	$(Q)echo "\`define SIMULATOR_"`echo $(SIMULATOR) | tr  "[:lower:]" "[:upper:]"` >> $@
	$(Q)echo "\`define TEST_NAME_STRING \""$(TEST)"\"" >> $@
	$(Q)if [ ! -z $$VCD ]; \
		then echo "\`define VCD" >> $@; \
	fi
	$(Q)if [ ! -z $$VCD_DELAY ]; \
		then echo "\`define VCD_DELAY "$$VCD_DELAY >> $@; \
	fi
	$(Q)if [ ! -z $$VCD_DEPTH ]; \
		then echo "\`define VCD_DEPTH "$$VCD_DEPTH >> $@; \
	fi
	$(Q)if [ ! -z $$VCD_DELAY_INSNS ]; \
		then echo "\`define VCD_DELAY_INSNS "$$VCD_DELAY_INSNS >> $@; \
	fi
	$(Q)if [ ! -z $$END_TIME ]; \
		then echo "\`define END_TIME "$$END_TIME >> $@; \
	fi
	$(Q)if [ ! -z $$END_INSNS ]; \
		then echo "\`define END_INSNS "$$END_INSNS >> $@; \
	fi
	$(Q)if [ ! -z $$PRELOAD_RAM ]; \
		then echo "\`define PRELOAD_RAM "$$END_TIME >> $@; \
	fi
	$(Q)if [ -z $$DISABLE_PROCESSOR_LOGS ]; \
		then echo "\`define PROCESSOR_MONITOR_ENABLE_LOGS" >> $@; \
	fi
	$(Q)if [ ! -z $$VPI ]; \
		then echo "\`define VPI_DEBUG" >> $@; \
	fi
	$(Q)if [ ! -z $$SIM_QUIET ]; \
		then echo "\`define SIM_QUIET" >> $@; \
	fi
	$(Q)for module in $(GATELEVEL_MODULES); do echo "\`define "$$module"_IS_GATELEVEL " >> $@; done
	# for or1200_monitor setting ...
	$(Q)if [ ! -z $$OR1200_MONITOR_SPRS ]; \
		then echo "\`define OR1200_MONITOR_SPRS" >> $@; \
	fi
	$(Q)if [ ! -z $$OR1200_MONITOR_LOOKUP ]; \
		then echo "\`define OR1200_MONITOR_LOOKUP" >> $@; \
	fi
	$(Q)if [ ! -z $$OR1200_MONITOR_EXEC_STATE ]; \
		then echo "\`define OR1200_MONITOR_EXEC_STATE" >> $@; \
	fi
	$(Q)if [ ! -z $$OR1200_MONITOR_EXEC_LOG_DISASSEMBLY ]; \
		then echo "\`define OR1200_MONITOR_EXEC_LOG_DISASSEMBLY" >> $@; \
	fi
	$(Q)if [ ! -z $$OR1200_RFRAM_GENERIC ]; \
		then echo "\`define OR1200_RFRAM_GENERIC" >> $@; \
	fi
	$(Q)if [ ! -z $$OR1200_DISPLAY_EXECUTED ]; \
		then echo "\`define OR1200_DISPLAY_EXECUTED" >> $@; \
	fi
	$(Q)if [ ! -z $$OR1200_SYSTEM_CHECKER ]; \
		then echo "\`define OR1200_SYSTEM_CHECKER" >> $@; \
	fi
	$(Q)if [ ! -z $$OR1200_SYSTEM_CHECKER ]; \
		then echo "\`define OR1200_SYSTEM_CHECKER" >> $@; \
	fi
	$(Q)if [ ! -z $$OR1200_MONITOR_JUMPTRACK_DEBUG_OUTPUT ]; \
		then echo "\`define OR1200_MONITOR_JUMPTRACK_DEBUG_OUTPUT" >> $@; \
	fi
	$(Q)if [ ! -z $$OR1200 ]; \
		then echo "\`define OR1200" >> $@; \
	fi
	$(Q)if [ ! -z $$MOR1KX ]; \
		then echo "\`define MOR1KX" >> $@; \
	fi

