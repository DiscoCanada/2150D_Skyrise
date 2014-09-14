int count = 0;
//Wait for button release on the LCD.
void waitForRelease()
{
	while (nLCDButtons != 0){}
}
//Top menu for LCD
int top_menu()
{
	bool picked2 = false;
	int menu = -1;
	clearLCDLine(0);
	clearLCDLine(1);
	while (!picked2)
	{
		displayLCDCenteredString(0, "<Autonomous>");
		displayLCDString(1, 0, "<Sensor | Power>");
		if (nLCDButtons == 1)
		{
			waitForRelease();
			menu = 0;
			picked2 = true;
		}
		if (nLCDButtons == 4)
		{
			waitForRelease();
			menu = 2;
			picked2 = true;
		}
		if (nLCDButtons == 2)
		{
			waitForRelease();
			menu = 1;
			picked2 = true;
		}
	}
	return menu;
}

//Autonomous Menu
int auton_menu(){
	bool picked = false;
	bool canceled = false;
	clearLCDLine(0);
	clearLCDLine(1);
	while (!picked)
	{
		displayLCDCenteredString(1, "< Enter >");
		if (nLCDButtons == 1)
		{
			waitForRelease();
			count--;
			if (count < 0)
			{
				count = MAX_AUTONS;
			}
		}
		if (nLCDButtons == 4)
		{
			waitForRelease();
			count++;
			if (count > MAX_AUTONS)
			{
				count = 0;
			}
		}
		if (nLCDButtons == 2)
		{
			waitForRelease();
			picked = true;
		}

		switch (count)
		{
			case 0: {
				displayLCDCenteredString(0, "Blue Autoloader");
				break;
			}
			case 1: {
				displayLCDCenteredString(0, "Blue Normal");
					break;
			}
			case 2: {
				displayLCDCenteredString(0, "Red Autoloader");
				break;
			}
			case 3: {
				displayLCDCenteredString(0, "Red Normal");
				break;
			}
			case 4: {
				displayLCDCenteredString(0, "Deploy Only");
				break;
			}
			case 5: {
				displayLCDCenteredString(0, "1 Point");
				break;
			}
			case 6: {
				displayLCDCenteredString(0, "Only on Red");
				break;
			}
			case 7: {
				displayLCDCenteredString(0, "Only on Blue");
				break;
			}
			case 8:{
				displayLCDCenteredString(0, "Build 2 Skyrise");
				break;
			}
			case 9:{
				displayLCDCenteredString(0, "Stack 2 Cubes");
				break;
			}
			case 10:{
				displayLCDCenteredString(0, "Troll1");
				break;
			}
			case 11:{
				displayLCDCenteredString(0, "Troll2");
				break;
			}
			case 12:{
				displayLCDCenteredString(0, "Troll3");
				break;
			}
			case 13:{
				displayLCDCenteredString(0, "Troll4");
				break;
			}
			case 14:{
				displayLCDCenteredString(0, "Troll5");
				break;
			}
			case 15:{
				displayLCDCenteredString(0, "Troll6");
				break;
			}
			case 16: {
				displayLCDCenteredString(0, "Testing");
				break;
			}
			case 17: {
				displayLCDCenteredString(0, "Programming");
				break;
			}
		}
	}
	//clearLCDLine(1);

	while (!canceled)
	{
		displayLCDCenteredString(1, " < Cancel > ");
		if (nLCDButtons == 2)
		{
			waitForRelease();
			return -1;
		}
	}

	return 0;
}

//More sensor values
int moar_sensor_menu()
{
	bool picked2 = false;
	string rEncoder = "";
	string lEncoder = "";
	int menu = -1;
	clearLCDLine(0);
	clearLCDLine(1);
	while (!picked2)
	{
		rEncoder = "";
		lEncoder = "";
		displayLCDCenteredString(1, "<More | Back>");
		displayLCDString(0, 0, "L: ");
		sprintf(rEncoder, "%0.1f", rightdistance()); //Build the value to be displayed
		sprintf(lEncoder, "%0.1f", leftdistance()); //Build the value to be displayed
		displayLCDString(0, 2, lEncoder);
		displayLCDString(0, 9, "R: ");
		displayLCDString(0, 11, rEncoder);
		if (nLCDButtons == 4)
		{
			waitForRelease();
			menu = -1;
			picked2 = true;
		}
		if (nLCDButtons == 1)
		{
			waitForRelease();
			menu = 0;
			picked2 = true;
		}
	}
	return menu;
}

//Sensor values
int sensor_menu()
{
	bool picked2 = false;
	string liftHeight = "";
	string limitswitch = "";
	int menu = -1;
	clearLCDLine(0);
	clearLCDLine(1);
	while (!picked2)
	{
		liftHeight = "";
		limitswitch = "";
		if (SensorValue[rtLiftZero] == 1)
		{
			SensorValue[rtLiftPot] = 0;
		}
		displayLCDCenteredString(1, "<Moar | Back>");
		displayLCDString(0, 0, "lift: ");
		sprintf(liftHeight, "%d", nMotorEncoder[rtLiftPot]); //Build the value to be displayed
		displayLCDString(0, 4, liftHeight);
		sprintf(limitswitch, "%d", SensorValue[rtLiftZero]); //Build the value to be displayed
		displayLCDString(0, 10, " Lmt:");
		displayNextLCDString(limitswitch);
		if (nLCDButtons == 4)
		{
			waitForRelease();
			menu = -1;
			picked2 = true;
		}
		if (nLCDButtons == 1)
		{
			waitForRelease();
			menu = 3;
			picked2 = true;
		}
	}
	return menu;
}

//Power values
int power_menu()
{
	bool picked2 = false;
	string mainBattery = "";
	string secondary = "";
	int menu = -1;
	clearLCDLine(0);
	clearLCDLine(1);
	while (!picked2)
	{
		displayLCDString(0, 0, "P:");
		sprintf(mainBattery, "%1.2f%c", (float)nImmediateBatteryLevel / 1000.0, 'V'); //Build the value to be displayed
		displayNextLCDString(mainBattery);
		sprintf(secondary, "%1.2f%c", (float)BackupBatteryLevel / 1000.0, 'V');
		displayLCDString(0, 9, "B:");
		displayNextLCDString(secondary);
		displayLCDCenteredString(1, "<Back>");
		if (nLCDButtons == 2)
		{
			waitForRelease();
			menu = -1;
			picked2 = true;
		}
	}
	return menu;
}

//Menu task
task Menu()
{
	int whichMenu = -1;
	while (true)
	{
		if (whichMenu == -1)
		{
			whichMenu = top_menu();
		}
		if (whichMenu == 0)
		{
			whichMenu = sensor_menu();
		}
		if (whichMenu == 1)
		{
			whichMenu = auton_menu();
		}
		if (whichMenu == 2)
		{
			whichMenu = power_menu();
		}
		if (whichMenu == 3)
		{
			whichMenu = moar_sensor_menu();
		}
		wait1Msec(15);
	}
}
