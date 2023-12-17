1. Enabling logger
 a. Go to logger.h and change constant LOGENABLED. 
 b. Setting true means that there is logging going to occur.
 c. Setting false means that there is no logging going to occur

2. Instantiating
 a. In your class .h create a pointer to logger class and create name for your instance. e.g. CLoggers *testlog
 b. In the cpp do: testlog = new CLoggers("testlog") this provides a row name in the csv
 c. Last thing you need to do is call init. testlog -> Init();

3. Call funcitons
 a. iI you need to log more than one value in a single variable or constant use the plurals of each log function. e.g LogStrings
 b. tT actually call just do testlog -> functionname(values/variable)

4. Exporting the file
 a. Open up data log tool 2023
 b. Connect to the roborio with correct team number
 c. Download the file that is told to you in the riolog that open upon deploying the code
 d. Input the files in the square that says input files
 e. Click export on the output button on the bottom right
 f. go to your file exlorer and open up the file that says csv.