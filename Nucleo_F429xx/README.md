
1. FPU Warning - > Right click on Project folder
	 ----------------------------------------------------------------------------------------------------------------------------------
	| Click on Properties-> C/C++ Build -> Settings -> Tool settings -> MCU/MPU Settings -> Select FPU as None and FP ABI as Software
	 ----------------------------------------------------------------------------------------------------------------------------------

2. Create Drivers folder unde project folder as below
	| Drivers
		| Inc
		| Src
	 ----------------------------------------------------------------------------------------------------------------------
	| Right click on Drivers folder and select Properties-> C/C++ Build -> Settings -> Uncheck Exclude resourcr from Build |
	 ----------------------------------------------------------------------------------------------------------------------
3. Include Path Settings
	Goto Project folder under Project Explorer and right click on that and follow the below steps
	 ----------------------------------------------------------------------------------------------------------------------------------
	| Click on Properties-> C/C++ Build -> Settings -> MCU GCC Compiler -> Include paths => Add the paths contains header files folder |
	 ----------------------------------------------------------------------------------------------------------------------------------