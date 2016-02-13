package irobotcreate2;

public interface PlaySong extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "irobotcreate2/PlaySong";
  static final java.lang.String _DEFINITION = "Header header\nuint8 song_number\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  byte getSongNumber();
  void setSongNumber(byte value);
}
