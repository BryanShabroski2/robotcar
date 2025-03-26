TARGET=hw8shabroski1

SOURCES=../include/import_registers.c \
	../include/enable_pwm_clock.c \
	../include/wait_period.c \
	video_interface.c \
	draw_bitmap_multiwindow.c \
	wait_key.c \
	scale_image_data.c \
	keypress.c \
	pwmsetup.c \
	hw8shabroski1.c

OBJECTS = $(patsubst %.cpp,%.o,$(patsubst %.c,%.o,$(SOURCES)))

CFLAGS = `pkg-config --cflags --libs gtk+-2.0` \
         -Wall \
         -Wno-deprecated-declarations \
         -g

all: $(TARGET)

clean:
	rm -f $(OBJECTS) $(TARGET)

$(TARGET): $(OBJECTS)
	gcc $^ $(CFLAGS) -o $@ -lm -lpthread -lswscale

%.o:%.c
	gcc $(CFLAGS) -c $< -o $@
