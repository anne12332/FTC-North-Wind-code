package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ServoMotor extends LinearOpMode {
    double pos1 = 0.5, pos0 = 0, posa = 0;
    Servo servo;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "S4");

          waitForStart();
         while (opModeIsActive()){
             telemetry.addData("OK", null);
             telemetry.addData("Position", servo.getPosition());
             telemetry.update();

             if(gamepad1.a) {
                 servo.setPosition(0.837);
                 telemetry.addData("Position", servo.getPosition());
                 telemetry.update();
             }

             if(gamepad1.b) {
                 servo.setPosition(pos1);
             }

             if(gamepad1.y){
                 servo.setPosition(-pos1);
             }

             if(gamepad1.x){
                 servo.setPosition(pos0);
             }

              if(gamepad1.right_trigger != 0){
                 servo.setPosition(gamepad1.right_trigger);
             }

             if(gamepad1.left_trigger != 0){
                 servo.setPosition(-gamepad1.left_trigger);
             }
         }
    }
}
