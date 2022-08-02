all:
	g++ -DSIX_EGGS -lm -o receive6 receive.cpp RF24/*.o RF24Network/*.o

debug:
	g++ -DSIX_EGGS -DDEBUG -lm -o receive6 receive.cpp RF24/*.o RF24Network/*.o
	
verbose:
	g++ -DSIX_EGGS -DVERBOSE -lm -o receive6 receive.cpp RF24/*.o RF24Network/*.o

three:
	g++ -lm -o receive receive.cpp RF24/*.o RF24Network/*.o

three_debug:
	g++ -DDEUG -lm -o receive receive.cpp RF24/*.o RF24Network/*.o

three_verbose:
	g++ -DVERBOSE -lm -o receive receive.cpp RF24/*.o RF24Network/*.o