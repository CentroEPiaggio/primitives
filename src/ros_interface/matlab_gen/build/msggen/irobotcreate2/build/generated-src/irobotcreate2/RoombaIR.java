package irobotcreate2;

public interface RoombaIR extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irobotcreate2/RoombaIR";
  static final java.lang.String _DEFINITION = "Header header\nbool state\nuint16 signal\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  boolean getState();
  void setState(boolean value);
  short getSignal();
  void setSignal(short value);
}
