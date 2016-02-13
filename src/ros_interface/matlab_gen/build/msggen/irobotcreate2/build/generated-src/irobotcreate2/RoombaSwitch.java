package irobotcreate2;

public interface RoombaSwitch extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irobotcreate2/RoombaSwitch";
  static final java.lang.String _DEFINITION = "Header header\nbool state\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  boolean getState();
  void setState(boolean value);
}
