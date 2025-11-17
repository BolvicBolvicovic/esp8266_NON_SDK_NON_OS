TARGET	= rtos.elf

PORT	= /dev/ttyUSB0
BAUDRATE= 115200
ESPTOOL	= esptool.py

CC_DIR	= ${HOME}/.esp/tools/xtensa-lx106-elf/bin
OBJCPY	= $(CC_DIR)/xtensa-lx106-elf-objcopy
CC	= $(CC_DIR)/xtensa-lx106-elf-gcc
CXX	= $(CC_DIR)/xtensa-lx106-elf-g++
CFLAGS	= \
	-Os \
	-g \
	-Wpointer-arith \
	-Wl,-EL \
	-nostdlib \
	-mlongcalls \
	-mtext-section-literals \
	-ffunction-sections \
	-fdata-sections \
	-fno-builtin-printf \
	-fno-jump-tables \
	-fno-guess-branch-probability \
	-freorder-blocks-and-partition \
	-fno-cse-follow-jumps \
	-Wall

LD_DIR	= ./ld
LD_FILE	= $(LD_DIR)/app.ld
LFLAGS	= \
	-nostdlib \
	-Wl,--no-check-sections \
	-Wl,-static \
	-Wl,--gc-sections \
	-L$(LD_DIR) \
	-T$(LD_FILE)

CSRCS	?= $(wildcard *.c ./user/*.c)
CXXSRCS	?= $(wildcard *.cpp ./user/*.cpp)
ASRCs	?= $(wildcard *.s ./user/*.s)
ASRCS	?= $(wildcard *.S ./user/*.S)

ODIR	:= .output

OBJDIR	:= $(ODIR)/obj
OBJS	:= $(addprefix $(OBJDIR)/, $(notdir $(CSRCS:.c=.o))) \
	$(addprefix $(OBJDIR)/, $(notdir $(CXXSRCS:.cpp=.o))) \
	$(addprefix $(OBJDIR)/, $(notdir $(ASRCs:.s=.o))) \
	$(addprefix $(OBJDIR)/, $(notdir $(ASRCS:.S=.o)))

all: $(OBJDIR) $(ODIR)/$(TARGET)-0x00000.bin

$(ODIR)/$(TARGET)-0x00000.bin: $(ODIR)/$(TARGET)
	$(ESPTOOL) --chip esp8266 elf2image $<

$(ODIR)/$(TARGET): $(OBJS)
	$(CC) $(LFLAGS) -o $@ $^

$(OBJDIR)/%.o: %.c
	$(CC) $(CFLAGS) -o $@ -c $<

$(OBJDIR)/%.o: %.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

$(OBJDIR)/%.o: %.s
	$(CC) $(CFLAGS) -o $@ -c $<

$(OBJDIR)/%.o: %.S
	$(CC) $(CFLAGS) -D__ASSEMBLER__ -o $@ -c $<

$(OBJDIR)/%.o: user/%.c
	$(CC) $(CFLAGS) -o $@ -c $<

$(OBJDIR)/%.o: user/%.cpp
	$(CC) $(CFLAGS) -o $@ -c $<

$(OBJDIR)/%.o: user/%.s
	$(CC) $(CFLAGS) -o $@ -c $<

$(OBJDIR)/%.o: user/%.S
	$(CC) $(CFLAGS) -D__ASSEMBLER__ -o $@ -c $<

$(OBJDIR): $(ODIR)
	@mkdir -p $(OBJDIR)

$(ODIR):
	@mkdir -p $(ODIR)

.PHONY: all re clean flash monitor

flash: $(ODIR)/$(TARGET)-0x00000.bin
	$(ESPTOOL) -c esp8266 -p $(PORT) -b $(BAUDRATE) \
	write_flash 0x00000 $<

serial:
	sudo minicom -D $(PORT) -b $(BAUDRATE)

clean:
	rm -rf $(ODIR)/

re: clean all

