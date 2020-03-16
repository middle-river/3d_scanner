all : 3d_scanner

clean :
	rm -f 3d_scanner 3d_scanner.o

3d_scanner : 3d_scanner.cc camera.h config.h pcd_writer.h
	g++ -Wno-psabi -Wall -O3 -I/opt/vc/include -L/opt/vc/lib/ -o $@ $< -pthread -lbcm_host -lmmal -lmmal_core -lmmal_util -lvcos -lpigpio -lopencv_calib3d -lopencv_core -lopencv_imgcodecs -lopencv_imgproc
