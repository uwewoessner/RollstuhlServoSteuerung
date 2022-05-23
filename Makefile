all: wheelchair calibrate

wheelchair: wheelchair.cpp FilterOnePole.o FilterTwoPole.o UDPComm.o myTime.h
	g++ wheelchair.cpp  FilterOnePole.o FilterTwoPole.o UDPComm.o /usr/local/lib/libethercat.a -I ../ethercat -o wheelchair

calibrate: calibrate.cpp FilterOnePole.o FilterTwoPole.o myTime.h
	g++ calibrate.cpp  FilterOnePole.o FilterTwoPole.o /usr/local/lib/libethercat.a -I ../ethercat -o calibrate

FilterOnePole.o: FilterOnePole.cpp myTime.h Filters.h
	g++ FilterOnePole.cpp -c
FilterTwoPole.o: FilterTwoPole.cpp myTime.h Filters.h
	g++ FilterTwoPole.cpp -c
UDPComm.o: UDPComm.cpp UDPComm.h
	g++ UDPComm.cpp -c
