// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class AllianceShiftUtil {
    
    static String gameData = DriverStation.getGameSpecificMessage();

    private static Timer matchTimer = new Timer();

    private static Optional<DriverStation.Alliance> currentAlliance;

    private static Alliance firstShiftAlliance = currentAlliance.get();
    private static Alliance secondShiftAlliance = currentAlliance.get();

    public static void startMatch() {
        
        matchTimer.start();

        currentAlliance = DriverStation.getAlliance();

    }

    public static void setFirstShift(String gameData) {

        if (gameData.equals("B")) {
            
            firstShiftAlliance = Alliance.Blue;
            secondShiftAlliance = Alliance.Red;

        } else if (gameData.equals("R")) {
            
            firstShiftAlliance = Alliance.Red;
            secondShiftAlliance = Alliance.Blue;

        }
    }

    public static boolean ourAllianceShift() {

        double currentTime = matchTimer.get();

        if (currentAlliance.isEmpty()) {

            SmartDashboard.putString("Match Shift", "Empty Alliance");
            return true;

        } else if (currentTime < 20) {

            SmartDashboard.putString("Match Shift", "Auto");
            return true;

        } else if (currentTime < 30) {

            SmartDashboard.putString("Match Shift", "Transition");
            return true;

        } else if (currentTime < 30 + 25*1) {
            
            SmartDashboard.putString("Match Shift", "Shift 1: " + firstShiftAlliance.toString());
            return firstShiftAlliance == currentAlliance.get();

        } else if (currentTime < 30 + 25*2) {

            SmartDashboard.putString("Match Shift", "Shift 2: " + secondShiftAlliance.toString());
            return secondShiftAlliance == currentAlliance.get();

        } else if (currentTime < 30 + 25*3) {

            SmartDashboard.putString("Match Shift", "Shift 3: " + firstShiftAlliance.toString());
            return firstShiftAlliance == currentAlliance.get();

        } else if (currentTime < 30 + 25*4) {

            SmartDashboard.putString("Match Shift", "Shift 4: " + secondShiftAlliance.toString());
            return secondShiftAlliance == currentAlliance.get();

        } else {
            
            SmartDashboard.putString("Match Shift", "End Game");
            return true;

        }

    }

    public static void reset() {

        matchTimer.reset();

        firstShiftAlliance = currentAlliance.get();
        secondShiftAlliance = currentAlliance.get();

    }

}
