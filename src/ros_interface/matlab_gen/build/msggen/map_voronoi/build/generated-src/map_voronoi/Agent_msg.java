package map_voronoi;

public interface Agent_msg extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "map_voronoi/Agent_msg";
  static final java.lang.String _DEFINITION = "int32[] col_det\nstring name\n\n";
  int[] getColDet();
  void setColDet(int[] value);
  java.lang.String getName();
  void setName(java.lang.String value);
}
