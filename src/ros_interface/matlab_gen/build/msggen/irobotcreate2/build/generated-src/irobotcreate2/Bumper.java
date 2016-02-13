package irobotcreate2;

public interface Bumper extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irobotcreate2/Bumper";
  static final java.lang.String _DEFINITION = "RoombaSwitch left\nRoombaSwitch right\n";
  irobotcreate2.RoombaSwitch getLeft();
  void setLeft(irobotcreate2.RoombaSwitch value);
  irobotcreate2.RoombaSwitch getRight();
  void setRight(irobotcreate2.RoombaSwitch value);
}
