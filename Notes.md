For FPS Tests

```sh
# Install dependencies (one-time)
sudo apt install ffmpeg -y

# Copy files
cp assets/video_to_frames.py ~/beepi_gc9a01a_nt/assets/
cp CMakeLists.txt ~/beepi_gc9a01a_nt/

# Put your video in assets/
cp /path/to/your/video.mp4 ~/beepi_gc9a01a_nt/assets/input.mp4

# Convert manually (recommended first time so you can see progress)
cd ~/beepi_gc9a01a_nt
python3 assets/video_to_frames.py assets/input.mp4 assets/video.bin --fps 50

# Build and run
cd build && make videoplay -j$(nproc)
sudo ./videoplay ../assets/video.bin
```