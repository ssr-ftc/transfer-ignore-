package org.firstinspires.ftc.teamcode;


import org.firstinspires.ftc.robotcore.external.Telemetry;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.roadrunner.util.Encoder;

@TeleOp(group = "drive")
@Config

public class Testwrist extends LinearOpMode {
    Wrist wirst;
    Servo s1;
    Servo s2;

    DcMotor arm;

    Encoder enc;
    public static double desiredAngle = 0;
    public static double desiredLift = 0;

    public static double ticks_in_degrees = (double) 90 /(-1900-200);

    public static int move = 1;

    double armangle;

    private FtcDashboard dashboard = FtcDashboard.getInstance();




    public void runOpMode() throws InterruptedException {
        s1 = hardwareMap.get(Servo.class, "servo_rotate");
        s2 = hardwareMap.get(Servo.class, "servo_lift");

        arm = hardwareMap.get(DcMotor.class, "motor_lever");


        wirst = new Wrist(s1, s2);

        Telemetry telemetry = new MultipleTelemetry(this.telemetry, dashboard.getTelemetry());


        waitForStart();

        while ((opModeIsActive())){
            armangle = (-1900-arm.getCurrentPosition())*ticks_in_degrees;

//            s2.setPosition(desiredLift);
//            s1.setPosition(desiredAngle);
            if (gamepad1.a) {
                wirst.setPositionofLiftServo(desiredAngle);
                wirst.setPositionofturnServo(-0.00454 * armangle + 0.5775); //works for any value
            }
            else{
                wirst.setPositionofturnServo(desiredLift);
            }





            telemetry.addData("WangleServoPos", wirst.liftServo.getPosition());
            telemetry.addData("posgoto", -0.00454*armangle + 0.5771);
            telemetry.addData("arm ticks", arm.getCurrentPosition());
            telemetry.addData("arm angle",armangle);
            telemetry.update();



        }

    }





}
