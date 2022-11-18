package frc.robot.subsystems.VisionSub;


import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 

import frc.robot.util.OrbitPID;


public class OrbitVision {

  // util class to handle bounding box data without manually creating a bunch of entries 
  public class BoundingBox {
    private NetworkTable table;
    private NetworkTableEntry c, tx1, ty1, tx2, ty2, tc;
    int index;

    BoundingBox(NetworkTable _table, int _index) {
      table = _table;
      index = _index;
    }

    
  }

  NetworkTable table = NetworkTableInstance.getDefault().getTable("orbitVision");

  public OrbitVision() {
    // pass
  }

  public int targetCount() { 
    return this.tc > 0;
  }
  
  

  public double getVertHeight() {
    return tvert.getDouble(Double.NaN);
  }

  // Advanced methods of calculations / robot actions and movement 

}
