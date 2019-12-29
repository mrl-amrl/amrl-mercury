#!/bin/sh
docker run -name webstreamer --rm -v $PWD/config.json:/app/config.json -p 8000:8000 mpromonet/webrtc-streamer