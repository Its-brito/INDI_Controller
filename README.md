#Thesis Documents

## tailsitter indi package
copy tailsitter_indi package/folder/directory to ~/PX4-Autopilot/src/modules/

```
cp -r tailsitter_indi ~/PX4-Autopilot/src/modules/
```

## Update modules in px4/sitl
move default.px4board to the respective directory

```
cp -f default.px4board ~/PX4-Autopilot/boards/px4/sitl/
```

## How to run tests
###1. Start PX4

```
cd ~/PX4-Autopilot/
make px4_sitl gz_quadtailsitter
```

###2. Initiate INDI/PID controller
Inside px4> start the module tailsitter_indi/pid
```
px4> tailsitter_indi start
```

###3. Run Custom scripts 
- Run Lissajous' FIgure-8 (in a new terminal)
```
python3 lissajous_trajectory.py
```

- Run Horizontal-Flight Doublet
```
python3 horizontal_doublet.py
```
## How to check logs

```
cd ~/PX4-Autopilot/build/px4_sitl_default/rootfs/logs/
ls -lt  # This lists files with the newest on top
mv session_00X.ulg pid_nowind.ulg
```