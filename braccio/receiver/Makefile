all:
	g++ -lm -o receive receive.cpp RF24/*.o RF24Network/*.o

debug:
	g++ -DDEBUG -lm -o receive receive.cpp RF24/*.o RF24Network/*.o
	
verbose:
	g++ -DVERBOSE -lm -o receive receive.cpp RF24/*.o RF24Network/*.o

six:
	g++ -DSIX_EGGS -lm -o receive6 receive.cpp RF24/*.o RF24Network/*.o

six_debug:
	g++ -DSIX_EGGS -DDEUG -lm -o receive6 receive.cpp RF24/*.o RF24Network/*.o

six_verbose:
	g++ -DSIX_EGGS -DVERBOSE -lm -o receive6 receive.cpp RF24/*.o RF24Network/*.o