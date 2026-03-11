SDK_INC = orbbec_sdk/include
SDK_LIB = orbbec_sdk

depth_capture: depth_capture.cpp stb_image_write.h
	clang++ -std=c++14 -O2 -I. -I$(SDK_INC) -L$(SDK_LIB) -lOrbbecSDK \
		-Wl,-rpath,@executable_path/orbbec_sdk \
		-o depth_capture depth_capture.cpp

clean:
	rm -f depth_capture

.PHONY: clean
