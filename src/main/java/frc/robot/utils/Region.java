package frc.robot.utils;

import edu.wpi.first.math.geometry.Rectangle2d;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;

public class Region {

    ArrayList<Rectangle2d> regions = new ArrayList<Rectangle2d>();

    public Region(Rectangle2d... regions) {
        
        for (Rectangle2d region : regions) {
            this.regions.add(region);
        }        

    }

    public boolean isInRegion(Pose2d pose) {
        for (Rectangle2d region : regions) {
            if (region.contains(pose.getTranslation())) {
                return true;
            }
        }
        return false;
    }

    
}