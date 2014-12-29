bool picked = false;

task ProgramChooser()
{

  bLCDBacklight = true;
  clearLCDLine(0);
  clearLCDLine(1);

  int max = 12;
  int min = 0;

  while(!picked) {

    if(nLCDButtons == LCD_LEFT) {
      while(nLCDButtons != 0) {}
        auto --;
      }
      if(nLCDButtons == LCD_RIGHT) {
        while(nLCDButtons != 0) {}
          auto ++;
        }
        if(nLCDButtons == LCD_CENTER) {
          while(nLCDButtons != 0) {}
            picked = true;
          }

          if(auto < min) auto = max;
          if(auto > max) auto = min;
          string autoName = "";
          switch(auto) {
            case 0: autoName = "[HANGING][BLUE]0"; break;
            case 1: autoName = "[HANGING][BLUE]1"; break;
            case 2: autoName = "[HANGING][RED]0"; break;
            case 3: autoName = "[HANGING][RED]1"; break;
            case 4: autoName = "[MIDDLE][BLUE]0"; break;
            case 5: autoName = "[MIDDLE][BLUE]1"; break;
            case 6: autoName = "[MIDDLE][RED]0"; break;
            case 7: autoName = "[MIDDLE][RED]1"; break;
            case 8: autoName = "[HANG] [RED]"; break;
            case 9: autoName = "[HANG] [BLUE]"; break;
            case 10: autoName = "[Prgrm Skills]"; break;
            case 11: autoName = "[Block Red Mid]"; break;
            case 12: autoName = "[Block Blue Mid]"; break;
          }
          displayLCDCenteredString(0,autoName);
          displayLCDCenteredString(1,"<            >");
        }

void init()
{
  calibrateGyro();
  StartTask(handleEncoders);
  StartTask(gyroPID);
  StartTask(drivePID);
  StartTask(syncPID);
  StartTask(liftPID);
}

void auto1()
{
  drive(100, 1);
  waitForDrive();
  turn(90.0, 100);
  waitForTurn();
  lift(1000);
  waitForLift();
}

void auto2()
{

}

void auto3()
{

}

void auto4()
{

}
