package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**Controller Map - Controller #1
 * Joysticks - Tank Drive, Wheels
 * Y -
 * B - Servos, in and out
 * A -
 * X -
 * D-Pad Up -
 * D-Pad Right -
 * D-Pad Down -
 * D-Pad Left -
 * Left Bumper -
 * Left Trigger -
 * Right Bumper -
 * Right Trigger - Slow down
 */

/**Controller Map - Controller #2
 * Joysticks - Left: Glyph lift
 * Y -
 * B - Servos
 * A -
 * X -
 * D-Pad Up -
 * D-Pad Right -
 * D-Pad Down -
 * D-Pad Left -
 * Left Bumper -
 * Left Trigger -
 * Right Bumper -
 * Right Trigger -
 */

@TeleOp(name="Taco")
//@Disabled
public class Taako extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;
    public DcMotor rightSlide;

    public Servo lServo;
    public Servo rServo;
    public Servo arm;

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


        //set the right motors' directions to reverse
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set all motors to zero power
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightRearMotor.setPower(0);
        rightSlide.setPower(0);

        //lServo.setPosition(0);
        //rServo.setPosition(1);

        arm.setPosition(0);


        // Wait for the game to start
        waitForStart();


        // run until the end of the match
        while (opModeIsActive()) {
            int count = 0;

            //assign the x and y variables to the inputs from the joysticks
            float x = -gamepad1.left_stick_y; //left
            float y = -gamepad1.right_stick_y; //right
            float slide = -gamepad2.left_stick_y; //slide value

            //ensure the x and y values will not go over +-1 (a motors' max and min)
            x = Range.clip(x, -1, 1);
            y = Range.clip(y, -1, 1);
            slide = Range.clip(slide, -1, 1);

            arm.setPosition(0);

            //run the motors at their respective powers derived from the joysticks
            leftFrontMotor.setPower(x * div);
            rightFrontMotor.setPower(y * div);
            leftRearMotor.setPower(x * div);
            rightRearMotor.setPower(y * div);
            rightSlide.setPower(slide);

            if (gamepad2.x) {
                lServo.setPosition(0.55);
                rServo.setPosition(0.45);

                sleep(400);
            }

            if (gamepad2.b){
                lServo.setPosition(0);
                rServo.setPosition(1);

                sleep(400);
            }

            if(gamepad1.right_trigger > 0.35) {
                div = 0.2;
            }
            else{
                div = 0.75;
            }

        }

    }

}