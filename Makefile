#LCCLIBS+= -larsal -lardiscovery -larcontroller -lncurses
LXXLIBS+= -lopencv_core -lopencv_highgui -lopencv_features2d -lopencv_imgproc -lopencv_imgcodecs -lopencv_videoio -lpthread -larsal -lardiscovery -larcontroller -lavcodec  -lswscale  -lavutil -lavformat -lopencv_core -lopencv_highgui -lopencv_features2d -lopencv_imgproc -lopencv_videoio -lpthread

OBJS=$(patsubst %.cpp,%.o,$(shell find src/ | grep .cpp))
#OBJS+=$(patsubst %.c,%.o,$(wildcard src/*.c) $(wildcard src/*/*.c))
TARGETDIR = ./bin
ROOT_DIR:=$(shell dirname $(realpath $(lastword $(MAKEFILE_LIST))))

all: dirs $(OBJS)
	@echo Linking everything together...
	@g++ $(OBJS) -o $(TARGETDIR)/main $(LCCLIBS) $(LXXLIBS)
	@cp Momentos.jpg ../bin
	@cp IzqCen.png ../bin
	@cp IzqAtr.png ../bin
	@cp DerCen.png ../bin
	@cp DerAtr.png ../bin
	@echo Done

dirs:
	@mkdir -p $(TARGETDIR)


#.c.o:
#	gcc -c $<  -o $@  $(LCCLIBS)

.cpp.o:
	@echo Compiling $<...
	@g++ -c $<  -o $@  $(LXXLIBS)

install:
	wget -O Installx64.zip "https://drive.google.com/uc?export=download&id=1ZiTAGkTFdDkEJ7CYhXHNIhfJgFtE_uw_"
	unzip Installx64.zip
	cp -r  Install/usr /
	rm -r Install
	rm Installx64.zip
	ldconfig

clean:
	@$(RM) -rf $(OBJS) bin
