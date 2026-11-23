import os, sys, json, math
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import verify as V
from verify import run
C = lambda n: (V.BY[n]["c"][0], V.BY[n]["c"][1])
LIGHTS = [(16.8,0.0),(16.8,-4.0),(16.8,-8.0),(16.8,-12.0),(15.4,-11.6),(10.0,-19.4),(8.0,-19.4)]
def light(pts, label):
    print(f"  lighting {label}:")
    for (x,y) in pts:
        d = min(math.hypot(x-lx, y-ly) for lx,ly in LIGHTS)
        print(f"    ({x:6.2f},{y:7.2f}) nearest fill light {d:4.1f} m  {'OK' if d<3.0 else ('marginal' if d<5 else 'DARK - add a sphere light')}")

run("M-B  go straight at the wall, turn left and then approach the wall",
    (16.83,-9.87,180), [("straight_wall",3.0),("turn","left"),("straight_wall",2.0)])
light([(16.83,-9.87),(12.83,-9.87),(11.63,-11.07),(11.63,-21.6)], "M-B")

run("M-C  at the wall turn left, then go to the chair, then go straight 2 m",
    (16.83,-9.87,180), [("straight_wall",3.0),("turn","left"),
                        ("go_to","chair")+C("DeskChair_7"),("straight_dist",2.0)])
light([(11.63,-11.07),(11.05,-13.46)], "M-C")

run("M-D  at the wall turn left, then go to the person, then enter the first door on the right",
    (16.83,-9.87,180), [("straight_wall",3.0),("turn","left"),
                        ("go_to","person",11.63,-13.30),("enter_door",8.08,-13.07,"-x")])
light([(11.63,-13.30),(8.08,-13.07)], "M-D")

run("M-A  go straight, enter the first door on the left and then approach the chair",
    (9.33,-15.87,90), [("straight_dist",2.6),("enter_door",8.08,-13.07,"-x"),
                       ("go_to","chair")+C("ConfOfficeChairBlack5")])
light([(9.33,-15.87),(8.08,-13.07),(3.85,-14.01)], "M-A")

run("M-A' (variant) go straight, enter the first door on the right and then approach the chair (kitchen)",
    (17.33,-6.37,90), [("straight_dist",3.6),("enter_door",18.68,-2.52,"+x"),
                       ("go_to","chair")+C("WoodenChair_4")])
light([(17.33,-6.37),(18.68,-2.52),(23.36,-2.26)], "M-A'")
