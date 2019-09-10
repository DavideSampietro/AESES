NAME		=	tbox_generator.elf
CC			= gcc
CFLAGS	= -Wall -O3
LDFLAGS =
INCDIR	=
SRCDIR	= ./
SOURCES	= $(SRCDIR)/tbox.c
OBJECTS = $(SOURCES:.c=.o)

all: $(NAME)

$(NAME): $(OBJECTS)
	$(CC) $^ -o $@ $(LDFLAGS)

objects: $(OBJECTS)

$(SRCDIR)/%.o: $(SRCDIR)/%.c
	$(CC) -c $< -o $@ $(CFLAGS)

clean:
	rm -rf *.o tbox_generator
