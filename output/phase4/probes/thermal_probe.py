import glob, sys
for type_path in sorted(glob.glob('/sys/devices/virtual/thermal/thermal_zone*/type')):
    zone_dir = type_path.rsplit('/', 1)[0]
    try:
        with open(type_path) as f:
            zone_type = f.read().strip()
        with open(zone_dir + '/temp') as f:
            milli_c = int(f.read().strip())
        print(sys.version.split()[0], zone_type, milli_c)
    except (OSError, ValueError) as e:
        print(sys.version.split()[0], zone_type, 'caught', type(e).__name__)
    except Exception as e:
        print(sys.version.split()[0], zone_type, 'UNCAUGHT', type(e).__name__, e)
