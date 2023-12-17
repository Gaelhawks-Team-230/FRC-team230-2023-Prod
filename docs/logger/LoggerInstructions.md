1. Enabling the class
 a. In Loggers.h change #define LOGENABLED to false for no logging to occur

2. Instanciating the class
 a. You need to create a pointer to logger class to instanciate it e.g. Loggers2 *x in .h file for class that will use it
 b. In cpp you need to add [x = new Loggers2("x");]
 c. Need to do x -> Init(); after

3. Calling the function
 a. To actually log you need to write x->LogStrings({"", "", ""}) or LogDoubles
 b. If you want to have multiple prints of the log(maybe has changing values) you need to loop it if not in teleop