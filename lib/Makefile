ARCHIVE	= librtos

CC_DIR	= ${HOME}/.esp/tools/xtensa-lx106-elf/bin
CC	= $(CC_DIR)/xtensa-lx106-elf-gcc
AR	= $(CC_DIR)/xtensa-lx106-elf-ar

INC	= -I.

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

CSRCS	= $(wildcard *.c)
OBJS	= $(CSRCS:.c=.o)

all: $(ARCHIVE).a

$(ARCHIVE).a: $(OBJS)
	$(AR) rcs $@ $^
%.o: %.c
	$(CC) $(INC) $(CFLAGS) -c $< -o $@

.PHONY: all clean tests

clean:
	rm -rf *.o
	rm -rf $(ARCHIVE).a

TARCHIVE= $(ARCHIVE)_tests
TAR	= ar
TCC	= gcc
TCFLAGS = \
	-DUNIT_TESTS \
	-m32 \
	-g \
	-Wall \
	-Wextra \
	$(INC)

TOBJS	= $(addprefix tests_, $(CSRCS:.c=.o))

tests: $(TARCHIVE).a

$(TARCHIVE).a: $(TOBJS)
	$(TAR) rcs $@ $^

tests_%.o: ./%.c
	$(TCC) $(INC) $(TCFLAGS) -c $< -o $@

clean_tests:
	rm -rf $(TARCHIVE).a $(TOBJS)

fclean: clean_tests clean
