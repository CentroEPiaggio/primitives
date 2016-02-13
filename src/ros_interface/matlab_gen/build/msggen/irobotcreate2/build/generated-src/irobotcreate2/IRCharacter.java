package irobotcreate2;

public interface IRCharacter extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irobotcreate2/IRCharacter";
  static final java.lang.String _DEFINITION = "Header header\nuint8 omni\nuint8 left\nuint8 right\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  byte getOmni();
  void setOmni(byte value);
  byte getLeft();
  void setLeft(byte value);
  byte getRight();
  void setRight(byte value);
}
