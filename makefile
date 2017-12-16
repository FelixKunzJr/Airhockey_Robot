objects = obj/estimator.o obj/motor.o obj/transformation.o obj/player.o

# Use C++11 or newer to get pololu to work
CFLAGS += -std=c++11 -I"./inc" -o

# Import libraries we are using
LIBS = -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs -lopencv_calib3d \
-lopencv_dnn -lopencv_features2d -lopencv_flann -lopencv_ml -lopencv_photo -lopencv_shape \
-lopencv_stitching -lopencv_superres -lopencv_videoio -lopencv_video -lopencv_videostab \
-lflycapture -lpthread -lpololu-tic-1

estimator: $(objects)
	c++ $(objects) $(LIBS) $(CFLAGS) estimator

obj/estimator.o : src/estimator.cpp inc/estimator.h inc/transformation.h
	c++ -c src/estimator.cpp $(CFLAGS) obj/estimator.o

obj/motor.o : src/motor.cpp inc/motor.h
	c++ -c src/motor.cpp $(CFLAGS) obj/motor.o

obj/transformation.o : src/transformation.cpp inc/transformation.h
	c++ -c src/transformation.cpp $(CFLAGS) obj/transformation.o
	
obj/player.o : src/player.cpp inc/player.h inc/estimator.h
	c++ -c src/player.cpp $(CFLAGS) obj/player.o
	

