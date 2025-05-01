CC = gcc
CFLAGS = -Wall -Wextra -std=c11 -D_GNU_SOURCE
LDFLAGS = -lm -lwiringPi -lgps
OBJDIR = obj
SRCDIR = src

# Source and object files
SOURCES = $(wildcard $(SRCDIR)/*.c)
OBJECTS = $(SOURCES:$(SRCDIR)/%.c=$(OBJDIR)/%.o)

# Executable
EXEC = moontracker

# Default target to build everything
all: $(EXEC)

# Rule to link the final executable
$(EXEC): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(EXEC) $(LDFLAGS)

# Rule to compile source files into object files
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up the build
clean:
	rm -rf $(OBJDIR)/*.o $(BINDIR)/$(EXEC)

.PHONY: all clean
