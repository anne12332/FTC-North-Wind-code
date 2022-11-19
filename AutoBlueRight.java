package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto_Blue_Right")
public class AutoBlueRight extends LinearOpMode {
    RobotNW Robot = new RobotNW();
    ElapsedTime timer = new ElapsedTime();

    public void runOpMode() throws InterruptedException{
        Robot.init(hardwareMap, telemetry, this);
        telemetry.addData("", "Init complited");
        telemetry.update();

        waitForStart();

        Robot.servo.setPosition(0.16);
        timer.reset();
        while(timer.milliseconds() < 2000 && !isStopRequested());
        Robot.WB.applyBreak();
        Robot.WB.applySpeed(0, 0.2, 0, 1);
        timer.reset();
        while(timer.milliseconds() < 300 && !isStopRequested());
        Robot.WB.applyBreak();


        Robot.WB.applySpeed(0, 0, 0.2, 1);
        while(Robot.IMU.getAngle() > -PI / 2. && !isStopRequested());
        Robot.WB.applyBreak();

        Robot.WB.applySpeed(0, 0.2, 0, 1);
        timer.reset();
        while(timer.milliseconds() < 1000 && !isStopRequested());
        Robot.servo.setPosition(0.8);
        Robot.WB.applySpeed(0, 0.2, 0, 1);
        timer.reset();
        while(timer.milliseconds() < 2000 && !isStopRequested());
        Robot.WB.applySpeed(0, -0.2, 0, 1);
        timer.reset();
        while(timer.milliseconds() < 500 && !isStopRequested());

        Robot.WB.applySpeed(0, 0, -0.2, 1);
        while(Robot.IMU.getAngle() < -PI / 16. && !isStopRequested());

        Robot.WB.applySpeed(0, 0.2, 0, 1);
        timer.reset();
        while(timer.milliseconds() < 2500 && !isStopRequested());
        Robot.WB.applySpeed(0, 0, -0.2, 1);
        while(Robot.IMU.getAngle() < 0 && !isStopRequested());
        Robot.WB.applyBreak();
    }


}
