gst-launch-1.0 videotestsrc ! videorate ! video/x-raw,framerate=2/1,width=640,height=480 ! x264enc pass=qual quantizer=20 tune=zerolatency ! rtph264pay config-interval=10 pt=96 ! udpsink host=127.0.0.1 port=9000
