SW_ROOT=../../..

include $(SW_ROOT)/Makefile.inc

all: bootrom.v

/* or1k-basic.bin  */
%.v: %.bin $(SW_ROOT)/utils/bin2vlogarray
	$(Q)$(SW_ROOT)/utils/bin2vlogarray < $< > $@

%.dis: %.elf
	$(Q)$(OR32_OBJDUMP) -d $< > $@

%.bin: %.elf
	$(Q)$(OR32_OBJCOPY) -O binary $< $@

clean:
	$(Q)rm -f *.elf *.bin *.vmem *.flashin *.dis *.flash16

