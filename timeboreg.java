/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */



package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.tree.JCTree;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name="no power", group="Linear Opmode")
//@Disabled
public class timeboreg extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    private Servo RightPlait;
    private Servo LeftPlait;

    Double power = 1.0;

    int Elev = 0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        RightPlait = hardwareMap.servo.get("RightPlait");
        LeftPlait = hardwareMap.servo.get("LeftPlait");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        while (opModeIsActive())
        {

            /*if (gamepad1.right_trigger > 0) //forward
            {
                RightBack.setPower(gamepad1.right_trigger);
                LeftBack.setPower(-gamepad1.right_trigger);
            }

            else if (gamepad1.left_trigger > 0) //backward
            {
                RightBack.setPower(-gamepad1.left_trigger);
                LeftBack.setPower(gamepad1.left_trigger);
            }

            else if (gamepad1.left_stick_x < 0) //turn right
            {
                RightBack.setPower(0);
                LeftBack.setPower(-power);
            }

            else if (gamepad1.left_stick_x > 0) //turn left
            {
                RightBack.setPower(power);
                LeftBack.setPower(0);
            }

            else if (gamepad1.dpad_right) //drive right
            {
                RightBack.setPower(-power);
                LeftBack.setPower(-power);
            }

            else if (gamepad1.dpad_left) // turn left
            {
                RightBack.setPower(power);
                LeftBack.setPower(power);
            }

            else if (gamepad2.dpad_up) {
                BringUpPump.setPower(0.5);
                sleep(1500);
                BringUpPump.setPower(0);
            }

            else if (gamepad2.dpad_down) {
                BringUpPump.setPower(-0.5);
                sleep(1500);
                BringUpPump.setPower(0);
            }

            else if (gamepad2.right_bumper){
                RightPlait.setPosition(0.2);
                LeftPlait.setPosition(0.8);
            }

            else if (gamepad2.left_bumper){
                RightPlait.setPosition(0.9);
                LeftPlait.setPosition(0.1);
            }

            else if (gamepad2.right_trigger > 0) OpenPump.setPower(-0.8);

            else if (gamepad2.left_trigger > 0) OpenPump.setPower(0.8);

            else if (gamepad2.a) Pumper.setPower(0.5);

            else if (gamepad2.x) Pumper.setPower(-0.5);

            else if (gamepad1.right_stick_y < 0 && Elev < 4) {
                Elev++;
                Elevator.setPower(-1);
                sleep(8120 / 4);
                Elevator.setPower(1);
            }

            if (Elev == 4){
                telemetry.addData("Elev: ", "Done");
            }

            else if (gamepad1.right_stick_y > 0) Elevator.setPower(0.8);

            else
            {
                RightBack.setPower(0);
                LeftBack.setPower(0);
                Pumper.setPower(0);
                Elevator.setPower(0);
                OpenPump.setPower(0);
            }*/

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}

