# GEMMA
Gecko Electro-Mechanical Motion Attraction (or just 'Gecko')

This project was made for a robotics course, and is posted here as more of an archive so some stuff might not even work. Originally I designed it to have electromagnets in the ends of the legs powered by a L298N motor driver, but I ended up having to trash that idea; the L298N mounting is still present in final prototype though.

I have included .stl, .f3d (Fusion 360), and .ino (Arduino) files and also a short .pdf outlining its basic operation.

Youtube video: https://www.youtube.com/watch?v=VduDyxTGmJU

## Materials/Components:
- 3D print filament for body
- super glue (or equivalent) for battery packs and servo horns
- Lots of wires
- PCB 
- DC Barrel jack
- 5V regulator for power supply
- Female and Male headers
- 2 AA battery packs
- Arduino Uno
- 4 servos (SG90) (with accompanying servo horns)
- Ultrasonic sensor



### Code Note(s)
Smoothing code for servos I found from **James Bruton**:
- https://www.youtube.com/watch?v=jsXolwJskKM
- https://github.com/XRobots/ServoSmoothing
