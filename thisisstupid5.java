package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorBNO055IMUCalibration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by IM on 9/16/2017.
 */




//r00d side




@Autonomous (name = "r00d side", group = "red")
//@Disabled

public class thisisstupid5 extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;
    public DcMotor rightSlide;

    public Servo lServo;
    public Servo rServo;
    public Servo arm;

    public ColorSensor color;
    public BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        double div = 0.75;

        // Define and initialize motors, servos, and sensors
        leftFrontMotor = hardwareMap.dcMotor.get("left_front_drive");
        rightFrontMotor = hardwareMap.dcMotor.get("right_front_drive");
        leftRearMotor = hardwareMap.dcMotor.get("left_rear_drive");
        rightRearMotor = hardwareMap.dcMotor.get("right_rear_drive");
        rightSlide = hardwareMap.dcMotor.get("right_slide");

        lServo = hardwareMap.servo.get("leftServo");
        rServo = hardwareMap.servo.get("rightServo");
        arm = hardwareMap.servo.get("arm");

        color = hardwareMap.get(ColorSensor.class, "color");

        float hsvValues[] = {0F, 0F, 0F};
        final float values[] = hsvValues;
        final double SCALE_FACTOR = 255;

        //set the right motors' directions to reverse
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //tell the wheels to run based off the encoders
        idle();

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);

        //lServo.setPosition(0);
        //rServo.setPosition(1);
        //arm.setPosition(1);


        // Wait for the game to start
        waitForStart();


        while (opModeIsActive()) {

            lServo.setPosition(0.55);
            rServo.setPosition(0.45);

            sleep(1550);

            rightSlide.setPower(0.45);

            sleep(550);

            rightSlide.setPower(0);

            sleep(250);

            arm.setPosition(0.45);
            sleep(200);
            arm.setPosition(0.77);
/*
            while(opModeIsActive()) {
                telemetry.addLine("blue: " + color.blue());
                telemetry.addLine("red: " + color.red());
                telemetry.addLine("green: " + color.green());
                telemetry.update();
            }
*/
            sleep(2050);

            telemetry.addLine("blue: " + color.blue());
            telemetry.addLine("red: " + color.red());
            telemetry.addLine("green: " + color.green());
            telemetry.update();

            int dist = 0;
            double pos;
            double angle = 0;

            Color.RGBToHSV((int) (color.red() * SCALE_FACTOR), (int) (color.green() * SCALE_FACTOR), (int) (color.blue() * SCALE_FACTOR), hsvValues);

            sleep(1050);

            if (color.red() > color.blue()) {
                //int start = leftFrontMotor.getCurrentPosition();

                leftFrontMotor.setPower(-0.05);
                leftRearMotor.setPower(-0.05);
                rightFrontMotor.setPower(-0.05);
                rightRearMotor.setPower(-0.05);

                //int end = leftFrontMotor.getCurrentPosition();

                telemetry.addLine("left is r00d");
                telemetry.update();

                //dist = end - start;

                sleep(850);

                leftFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightRearMotor.setPower(0);

                sleep(250);

            } else {
                leftFrontMotor.setPower(0.05);
                leftRearMotor.setPower(0.05);
                rightFrontMotor.setPower(0.05);
                rightRearMotor.setPower(0.05);

                telemetry.addLine("left is blu");
                telemetry.update();

                sleep(900);
                angle = 0.30;
                dist = 600;

                leftFrontMotor.setPower(0);
                leftRearMotor.setPower(0);
                rightFrontMotor.setPower(0);
                rightRearMotor.setPower(0);
            }

            arm.setPosition(0);

            sleep(1050);

            leftFrontMotor.setPower(0.15);
            leftRearMotor.setPower(0.15);
            rightFrontMotor.setPower(0.21 + angle);
            rightRearMotor.setPower(0.21 + angle);

            sleep(925 - dist);

            leftFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightRearMotor.setPower(0);

            sleep(500);

            leftFrontMotor.setPower(-0.2);
            leftRearMotor.setPower(-0.2);
            rightFrontMotor.setPower(0.2);
            rightRearMotor.setPower(0.2);

            sleep(800);

            leftFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightRearMotor.setPower(0);

            sleep(250);

            leftFrontMotor.setPower(0.2);
            leftRearMotor.setPower(0.2);
            rightFrontMotor.setPower(0.2);
            rightRearMotor.setPower(0.2);

            sleep(600);

            leftFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightRearMotor.setPower(0);

            sleep(250);

            leftFrontMotor.setPower(0.2);
            leftRearMotor.setPower(0.2);
            rightFrontMotor.setPower(-0.2);
            rightRearMotor.setPower(-0.2);

            sleep(800);

            leftFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightRearMotor.setPower(0);

            sleep(250);

            leftFrontMotor.setPower(0.1);
            leftRearMotor.setPower(0.1);
            rightFrontMotor.setPower(0.1);
            rightRearMotor.setPower(0.1);

            sleep(450);

            leftFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightRearMotor.setPower(0);

            sleep(250);

            lServo.setPosition(0);
            rServo.setPosition(1);

            sleep(250);

            leftFrontMotor.setPower(-0.1);
            leftRearMotor.setPower(-0.1);
            rightFrontMotor.setPower(-0.1);
            rightRearMotor.setPower(-0.1);

            sleep(300);

            leftFrontMotor.setPower(0);
            leftRearMotor.setPower(0);
            rightFrontMotor.setPower(0);
            rightRearMotor.setPower(0);

            sleep(250000000);

        }
    }
}
