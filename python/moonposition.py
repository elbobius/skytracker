import sys
from skyfield.api import Topos, load, Loader

def main():
    if len(sys.argv) != 4:
        print("Usage: moonposition.py <latitude> <longitude> <object>")
        return

    lat = float(sys.argv[1])
    lon = float(sys.argv[2])
    target_name = sys.argv[3].lower()

    # Load planetary ephemeris data
    load = Loader('~/skyfield-data')
    planets = load('de421.bsp')
    ts = load.timescale()
    t = ts.now()

    # Define observer location
    location = Topos(latitude_degrees=lat, longitude_degrees=lon)
    observer = planets['earth'] + location

    # Map object names to Skyfield targets
    object_map = {
        "moon": planets["moon"],
        "sun": planets["sun"],
        "mars": planets["mars"],
        "jupiter": planets["jupiter barycenter"],
        "saturn": planets["saturn barycenter"]
    }

    if target_name not in object_map:
        print("Error: Unknown object name. Use one of: moon, sun, mars, jupiter, saturn")
        return

    target = object_map[target_name]
    astrometric = observer.at(t).observe(target)
    alt, az, _ = astrometric.apparent().altaz()

    print(f"{alt.degrees:.4f},{az.degrees:.4f}")

if __name__ == "__main__":
    main()
