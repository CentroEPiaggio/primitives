package irobotcreate2;

public interface DigitLeds extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irobotcreate2/DigitLeds";
  static final java.lang.String _DEFINITION = "Header header\nuint8[] digits\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  org.jboss.netty.buffer.ChannelBuffer getDigits();
  void setDigits(org.jboss.netty.buffer.ChannelBuffer value);
}
