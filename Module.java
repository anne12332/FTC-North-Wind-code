package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public abstract class Module {
     public abstract void init (HardwareMap hwd);

     public void delay (int millie){

         ElapsedTime t = new ElapsedTime();
         double t0 = t.milliseconds();
         while(t.milliseconds() - t0 < millie){}
     }
        }
