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
    private NetworkTableEntry n, tx1, ty1, tx2, ty2, tc, tt;
    int index;

    BoundingBox(NetworkTable _table, int _index, NetworkTableEntry _n) {
      table = _table;
      index = _index;

      // create network table entries for target values
      n = _n; // number of detected BBs (shared between instances)
      tx1 = table.getEntry("tx1" + index); // tx1 of the index
      ty1 = table.getEntry("ty1" + index); // ty1 of the index
      tx2 = table.getEntry("tx2" + index); // tx2 of the index
      ty2 = table.getEntry("ty2" + index); // ty2 of the index
      tc = table.getEntry("tc" + index); // confidence of the box
      tt = table.getEntry("tt" + index); // type detected in the box
    }

    public double tx1() {return tx1.getDouble(0);}
    public double ty1() {return ty1.getDouble(0);}
    public double tx2() {return tx2.getDouble(0);}
    public double ty2() {return ty2.getDouble(0);}
    public double tc() {return tc.getDouble(0);}
    public double tt() {return tt.getDouble(0);}
  }

  NetworkTable table = NetworkTableInstance.getDefault().getTable("orbitVision");
  NetworkTableEntry n = table.getEntry("n");
  BoundingBox b1 = new BoundingBox(table, 1, n);

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
