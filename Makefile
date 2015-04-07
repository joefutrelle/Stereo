# apt packages required include libopencv-dev
CXXFLAGS=-ggdb -O3 -std=c++0x
LDLIBS=-lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_ml -lopencv_calib3d

SRCS1=mainCalibrate.cpp CalibrateStereoCamera.cpp FileIO.cpp
SRCS2=mainRectify.cpp RectifyImage.cpp Reconstruct3dImage.cpp FileIO.cpp

OBJS1=$(subst .cpp,.o,$(SRCS1))
OBJS2=$(subst .cpp,.o,$(SRCS2))

EXE1=StereoCalibrate
EXE2=StereoRectify

includes = $(wildcard *.hpp)

all: $(EXE1) $(EXE2)

StereoCalibrate: $(OBJS1)
	g++ $(LDFLAGS) -o $(EXE1) $(OBJS1) $(LDLIBS)
StereoRectify: $(OBJS2)
	g++ $(LDFLAGS) -o $(EXE2) $(OBJS2) $(LDLIBS)

%.o: %.cpp ${includes}
	g++ $(CXXFLAGS) -c $< -o $@

clean:
	$(RM) $(OBJS1)
	$(RM) $(EXE1)
	$(RM) $(OBJS2)
	$(RM) $(EXE2)

