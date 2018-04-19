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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="R1", group ="7079")
public class R1 extends LinearOpMode {
    FaltechRobot robot = new FaltechRobot();
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        robot.init(hardwareMap, telemetry);

        while (!isStopRequested() && !robot.driveTrain.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robot.driveTrain.imu.getCalibrationStatus().toString());
        telemetry.update();

        robot.init(hardwareMap, telemetry);
        boolean finished = false;
        runtime.reset();
        int LCR = 0;
        waitForStart();

        if (opModeIsActive()) {

            telemetry.addData("Finished & Saw", "%s", LCR);
            telemetry.update();

            robot.jewelArm.moveJewelArmLeft(-1, 3000);
            robot.jewelArm.stop();
            sleep(250);

            if (robot.jewelArm.sensorRed.red() > robot.jewelArm.sensorRed.blue()) {
                robot.driveTrain.swivel(.25, 200);
                robot.jewelArm.moveJewelArmLeft(1, 2250);
                robot.driveTrain.swivel(-.25, 200);

            } else {
                robot.driveTrain.swivel(-.25, 200);
                robot.jewelArm.moveJewelArmLeft(1, 2550);
                robot.driveTrain.swivel(.25, 200);
            }

            robot.driveTrain.goInches((robot.glyphDistance[1 - LCR] * 1.25), .25, 10);

//            if (LCR == -1){
//                robot.driveTrain.goInches(robot.glyphDistance[0],.25,10);
//            }
//            else if (LCR == 0){
//                robot.driveTrain.goInches(robot.glyphDistance[1],.25,10);
//            }
//            else{
//                robot.driveTrain.goInches(robot.glyphDistance[2],.25,10);
//            }
//
            robot.driveTrain.turnDegreesRight(.3,75,5);
            robot.driveTrain.goInches(4, .25, 10);
//            robot.glyphColllection.elevatorUp(.8);
//            sleep(2000);


            telemetry.addData("Breaking Lock", "now");
            telemetry.update();
            sleep(1000);

            robot.glyphColllection.mtrHexFR.setPower(.5);
            sleep(1250);
            robot.glyphColllection.stop();

            telemetry.addData("Starting Flush", "now");

            robot.glyphColllection.collectionExpel(1);
            sleep(2500);

            telemetry.addData("Moving towards the cryptobox", "now");
            sleep(1000);

            robot.driveTrain.goInches(3, 0.25, 5);

            telemetry.addData("Moving out", "now");
            telemetry.addData("Testing Build", "now");
            telemetry.update();

            robot.driveTrain.goInches(-3,.25,5);
            //robot.driveTrain.goPulsate(12, 0.2, -1);
            sleep(500);
            robot.driveTrain.goInches(-3,.25,5);
            robot.glyphColllection.stop();
            robot.driveTrain.stop();
            finished = true;

        }
    }
}
//    String format(OpenGLMatrix transformationMatrix) {
//        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
//    }}