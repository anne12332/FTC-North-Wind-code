package org.firstinspires.ftc.teamcode;

import static java.lang.Math.PI;
import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto_Blue_Left")
public class AutoBlueLeft extends LinearOpMode {
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
        Robot.WB.applySpeed(-0.4, 0, 0, 1);
        timer.reset();
        while(timer.milliseconds() < 640 && !isStopRequested());
        Robot.WB.applyBreak();
        go_to_angle(0);

        Robot.WB.applySpeed(0, 0.2, 0, 1);
        timer.reset();
        while(timer.milliseconds() < 900 && !isStopRequested());
        Robot.servo.setPosition(0.8);
        Robot.WB.applyBreak();
        timer.reset();
        while(timer.milliseconds() < 2000 && !isStopRequested());
        Robot.WB.applySpeed(0, 0.2, 0, 1);
        timer.reset();
        while(timer.milliseconds() < 150 && !isStopRequested());
        Robot.WB.applySpeed(0, -0.2, 0, 1);
        timer.reset();
        while(timer.milliseconds() < 500 && !isStopRequested());

        Robot.WB.applySpeed(-0.4, 0, 0, 1);
        timer.reset();
        while(timer.milliseconds() < 1200 && !isStopRequested());
        Robot.WB.applySpeed(0, 0.2, 0, 1);
        timer.reset();
        while(timer.milliseconds() < 2500 && !isStopRequested());
        Robot.WB.applyBreak();
    }

    public void go_to_angle(double targetAng) {
        while(abs(targetAng - Robot.IMU.getAngle()) > 1. / 30. * PI && !isStopRequested()){
            Robot.WB.applySpeed(0, 0, targetAng - Robot.IMU.getAngle(), 1);
        }
    }
}
