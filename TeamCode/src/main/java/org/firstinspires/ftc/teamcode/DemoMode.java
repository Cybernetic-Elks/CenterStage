package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//TODO Redo for this year's robot (powerplay 2023)
//TODO Add limts to other parts of robot, possibly to main TeleOp
@TeleOp(name = "Demo TeleOp", group = "TeleOp")
/**
 * Programmer:    Sean Pakros & Kairon Johnson
 * Date Created:  5/28/22
 * Purpose: This is a TeleOp that we will use for demo and presentation purposes for others to try out.
 */

public class DemoMode extends LinearOpMode
{
    OpMode opmode;
    boolean pressed = false;

    @Override
    public void runOpMode() {
        Hardware h = new Hardware();


        try {
            h.init(hardwareMap, telemetry);
        } catch (Exception e) {
            telemetry.addData("Init Error:", "Something failed to initialize");
            e.printStackTrace();
        }

        telemetry.addData("Main Initialization ", "complete");
        telemetry.update();

        boolean pressedLastIterationIntake = false;
        boolean pressedLastIterationFast = false;
        boolean fastToggle = false;
        boolean pressedLastIterationWrist = false;
        boolean pressedLastIterationCarouselReverse = false;
        double armSpeedUp = -1;
        double armSpeedDown = .8;
        boolean limitSwitch = true;
        double wristPos = .5;
        boolean dpadDown = false, dpadUpPressed = false, dpadDownPressed = false;
        double winchPow = .5;

        waitForStart();
        while (opModeIsActive()) {
            boolean changed = false;
            boolean pressedIntake = gamepad1.x;
            boolean fast = gamepad2.right_stick_button;
            h.motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            h.motorBackRight.setDirection(DcMotorSimple.Direction.FORWARD);
            h.motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            h.motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            if(fast && !pressedLastIterationFast)
            {
                fastToggle = !fastToggle;
            }

            /**Start drive system**/
            if (fastToggle) {
                h.driveOmniDir(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, false, 2, 1);
            } else {
                h.driveOmniDir(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, false, 2, 2);
            }

            if(gamepad2.back) {
                requestOpModeStop();
            }

            /** These are what I call "Fine Tuning Controls" (FTC) which are used for slow accurate movements
             *  for actions such as placing the shipping element these would probably be the first thing I remove
             *  if I needed more controls
             **/
//            if (gamepad1.dpad_left || gamepad2.dpad_left) {
//                h.motorFrontLeft.setPower(-.2);
//                h.motorFrontRight.setPower(.2);
//                h.motorBackLeft.setPower(-.2);
//                h.motorBackRight.setPower(.2);
//            } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
//                h.motorFrontLeft.setPower(.2);
//                h.motorFrontRight.setPower(-.2);
//                h.motorBackLeft.setPower(.2);
//                h.motorBackRight.setPower(-.2);
//            }
//            if (gamepad1.dpad_up || gamepad2.dpad_up) {
//                h.motorFrontLeft.setPower(.2);
//                h.motorFrontRight.setPower(.2);
//                h.motorBackLeft.setPower(.2);
//                h.motorBackRight.setPower(.2);
//            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
//                h.motorFrontLeft.setPower(-.2);
//                h.motorFrontRight.setPower(-.2);
//                h.motorBackLeft.setPower(-.2);
//                h.motorBackRight.setPower(-.2);
//            }
            /** END Fine Tuning Controls**/

            telemetry.addData("motorFrontLeft encoder value: ", h.motorFrontLeft.getCurrentPosition());
            telemetry.addData("motorFrontRight encoder value: ", h.motorFrontRight.getCurrentPosition());
            telemetry.addData("motorBackLeft encoder value: ", h.motorBackLeft.getCurrentPosition());
            telemetry.addData("motorBackRight encoder value: ", h.motorBackRight.getCurrentPosition());
            telemetry.addData("slides encoder value: ", h.motorLift.getCurrentPosition());
            telemetry.addData("arm encoder value: ", h.servoArm.getPosition());
            telemetry.addData("outtake encoder value: ", h.servoClaw.getPosition());
            telemetry.update();

//
            if(gamepad1.left_trigger > 0.05){
                h.motorLift.setPower(gamepad1.left_trigger*.75);
            }else{
                h.motorLift.setPower(0);
            }
//
            if(gamepad1.x && pressed == false){
                pressed=true;
                h.servoClaw.setPosition(.43);//h.servoClaw.getPosition()+.01
            }
            if(gamepad1.y && pressed == false){
                pressed=true;
                h.servoClaw.setPosition(.1);//h.servoClaw.getPosition()-.01
            }


            if(gamepad1.dpad_up && pressed == false){
                pressed = true;
                h.servoArm.setPosition(.45);//.45
//                telemetry.addData("Status1", "Here");
            }

            if(gamepad1.dpad_down && pressed == false){
                pressed = true;
                h.servoArm.setPosition(.15);//.15
            }
            if(gamepad2.dpad_up){
                h.servoIntakeLift.setPosition(.67);//h.servoIntakeLift.getPosition()+.01
            }
            if(gamepad2.dpad_down){
                h.servoIntakeLift.setPosition(.76);//h.servoIntakeLift.getPosition()-.01
            }
            if(!gamepad1.x&&!gamepad1.y &&!gamepad1.dpad_up &&!gamepad1.dpad_down){
                pressed = false;
            }
//            if(h.servoIntakeLift>0)
            if(gamepad2.dpad_right){
                h.servoIntakeLift.setPosition(.72);//.77
            }
            if(gamepad1.right_trigger>.10) {
                h.motorIntake.setPower(gamepad1.right_trigger + .3);
            }
            if(gamepad1.b) {
                    h.motorIntake.setPower(0);
            }
            if(gamepad1.left_bumper) {
                h.motorLift.setPower(-.6);
            }
            if(gamepad1.right_bumper) {
                h.motorIntake.setPower(-.7);
            }
            telemetry.addData("Arm Position", h.servoArm.getPosition());
            telemetry.addData("Intake Position", h.servoIntakeLift.getPosition());
            telemetry.addData("Claw Position", h.servoClaw.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();


            /** Toggle code for opening and closing the claw, if you press x it will alternate between being closed and opened enough for one block
             *  If you press y it will open fully we rarely open it fully as it adds risk that we may grab two blocks
             **/
            //0 is opened-.57 is partial open, 1 is closed

            if(pressedIntake & !pressedLastIterationIntake)
            {
                /*if(h.servoIntake.getPosition() > .8)
                {
                    h.servoIntake.setPosition(.57); //0 .1
                }
                else
                {
                    h.servoIntake.setPosition(1); //1 .3
                }*/

            }
            if (gamepad1.y)
            {
                //h.servoIntake.setPosition(.45);
            }

            /** END CLAW CONTROL**/
            /** Simple controls for the carousel spin one way when 'b' is pressed another way when 'x' is pressed **/
            /** END CAROUSEL CONTROL **/
            /** Emergency switch to disable the limits on the arm in case we start the arm in the wrong position **/
            if(gamepad2.back)
            {
                limitSwitch = !limitSwitch;
            }

            /** Our arm controls, this rotates the arm so we can reach the different levels. If 'a' on gamepad1 is held while moving the arm
             * it will move at half speed for more precision. This is helpful for precision placing such as the team shipping element**/


            if(fastToggle)
            {
                armSpeedDown = .8;
                armSpeedUp = -1;
            }
            else
            {
                armSpeedDown = .5;
                armSpeedUp = -.7;
            }
            //TODO update winch limits

//            if(gamepad1.left_trigger > .01 && h.motorWinch.getCurrentPosition() < 380)
//            {
//                h.motorWinch.setPower(winchPow);
//            }
//            if (gamepad1.left_bumper && h.motorWinch.getCurrentPosition() >= 10)
//            {
//                h.motorWinch.setPower(-winchPow);
//            }
//            if((!gamepad1.left_bumper && gamepad1.left_trigger == 0) || (h.motorWinch.getCurrentPosition() > 380 || h.motorWinch.getCurrentPosition() <= 10))
//            {
//                h.motorWinch.setPower(0);
//            }


            pressedLastIterationIntake = pressedIntake;
            pressedLastIterationFast = fast;

        }
    }
}
