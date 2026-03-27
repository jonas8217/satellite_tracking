from pyorbital import orbital
import os.path
from datetime import datetime,timedelta,timezone
import numpy as np
import json
from matplotlib import pyplot as plt

project_dir = os.path.abspath(os.path.dirname(__file__) + "/..")

def main(argv):

    tle_file = os.path.abspath(project_dir + "/tle_files/sample_tles.txt")

    with open(tle_file,"r") as f:
        lines =  f.read().split("\n")

    tles = []
    for i,line in enumerate(lines):
        if line != "" and line[0] not in ("1","2"):
            if i + 3 <= len(lines):
                if lines[i+1].startswith("1 ") and lines[i+2].startswith("2 "):
                    tles.append((lines[i],lines[i+1],lines[i+2]))

    if len(argv) < 2:
        for i,tle in enumerate(tles):
            print(f" {i + 1}: {tle[0]}")

        n = int(input(f"Choose [1 - {len(tles)}]: ").strip()) - 1
        if not 0 <= n <= len(tles) - 1:
            print("Choice not in range of TLEs")
            exit()

        tle = tles[n]
    else:
        tle_options = []
        for tle in tles:
            if argv[1] in tle[0]:
                tle_options.append(tle)
        if len(tle_options) == 0:
            print(f"Could not find satellite '{argv[1]}'")
            exit()

        for i,tle in enumerate(tle_options):
            print(f" {i + 1}: {tle[0]}")

        n = int(input(f"Choose [1 - {len(tle_options)}]: ").strip()) - 1
        if not 0 <= n <= len(tle_options) - 1:
            print("Choice not in range of TLEs")
            exit()
        tle = tle_options[n]

    print(f"Using TLE: \n{tle[0]}\n{tle[1]}\n{tle[2]}\n")
    orb = orbital.Orbital(satellite=tle[0],line1=tle[1],line2=tle[2])

    # Ground station coordinates gathered from google maps
    # Altitude gotten from en-gb.topographic-map.com/map-z61h/Denmark/ and www.freemaptools.com/elevation-finder.htm
    # building height estimated by pythagoran therom

    fname = "gs_config_sdu_tek_roof.json"
    with open(fname, "r") as f:
        data = json.load(f)

    lat_gs = data["latitude"]
    long_gs = data["longitude"]
    alt_gs = data["altitude"]
    # az_offset = data["azimuth_offset"]
    # tilt_axis = data["tilt"]
    # tilt = data["tilt_axis"]

    rotor_min_el = -5
    rotor_max_el = 185
    rotor_min_az = -180
    rotor_max_az = 540

    rotor_az_max_speed = 2 # degrees per second
    rotor_el_max_speed = 2 # degrees per second


    hours = 24

    pass_times = orb.get_next_passes(datetime.now(timezone.utc), hours, long_gs, lat_gs, alt_gs)

    min_elevation = 15

    for p in pass_times:
        rise,fall,max_el_time = p
        # TODO maybe make this check take the time over the horizon into account, or maybe the signal strength of the satellite
        if orb.get_observer_look(max_el_time, long_gs, lat_gs, alt_gs)[1] < min_elevation:
            continue

        pass_time = fall-rise
        # print(pass_time.total_seconds())
        # print(dir(pass_time))
        # exit()

        point = 200 # total trajectory points

        ts,azs,els = [],[],[]

        for i in range(point):
            dt = timedelta(seconds=i * pass_time.total_seconds() / point)
            az,el = orb.get_observer_look(rise + dt, long_gs, lat_gs, alt_gs)
            ts.append(dt.total_seconds())
            azs.append(float(az))
            els.append(float(el))

        # TODO? resample to get samples spaced equally in angular distance instead of in time

        for i in range(len(azs)-1):
            if abs(azs[i] - azs[i+1]) > 180:
                sgn = int(azs[i] < azs[i+1])*2-1
                azs[i+1] -= sgn * 360

        # TODO make the derivative more accurate (shited half a sample forward in time)
        az_dots,el_dots = [],[]
        for i in range(len(ts)-1):
            az_dots.append((azs[i+1]-azs[i])/(ts[i+1]-ts[i]))
            el_dots.append((els[i+1]-els[i])/(ts[i+1]-ts[i]))
        az_dots.append(az_dots[-1])
        el_dots.append(el_dots[-1])

        # plt.plot(ts,az_dots)
        # plt.show()

        az_too_fast,el_too_fast = [],[]
        for az,el,az_dot,el_dot in zip(azs,els,az_dots,el_dots):
            if abs(az_dot) > rotor_az_max_speed:
                az_too_fast.append([az,el])
            if abs(el_dot) > rotor_el_max_speed:
                el_too_fast.append([az,el])

        if True:
            print(f"Rise: {rise.strftime('%x %X')} Fall: {fall.strftime('%x %X')} UTC")
            xs = np.array([azs,els])

            xs_fast_az = np.array(az_too_fast).T
            xs_fast_el = np.array(el_too_fast).T

            fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
            ax.set_rlim(bottom=90, top=0)
            ax.plot(xs[0,:]*np.pi/180, xs[1,:], marker=" ")
            # visualize problem areas?
            #   angular velocity greater than combined axes  velocity limits 
            if xs_fast_az.size != 0:
                ax.plot(xs_fast_az[0,:]*np.pi/180, xs_fast_az[1,:], marker=" ")
            if xs_fast_el.size != 0:
                ax.plot(xs_fast_el[0,:]*np.pi/180, xs_fast_el[1,:], marker=" ")

            plt.show()

        # TODO make an algorithm for producing a trajectory that:
        #   1. Does not violate the rotor max speed
        #   2. Is able to choose between going around and through the zenith

        # TODO Validate how good either of the two options from the above algorithm
        #   Integrate the expected Signal to noise ratio:
        #   Sources of gain loss:
        #     pointing error
        #     free space path loss
        #     ...

        # TODO
        #   Shorten the list to start and end at minimum elevation
        #   Check trajectory against rotor min and max azimuth, correct if possible (+/- 360 degrees) error otherwise


        ans = input("Create trajectory? [y/N]: ").lower()
        if ans in ["q","quit","exit"]:
            break

        if ans not in ["y","yes"]:
            continue


        # create trajectory
        if len(xs_fast_az) == 0 and len(xs_fast_el) == 0:
            with open(f"{project_dir}/trajectories/{tle[0].replace(' ','_')}_{rise.strftime('%Y_%m_%d_%H_%M_%S')}.csv", "w",newline="") as f:
                f.write("time [ms], azimuth [degrees], elevation [degrees]\n")
                for t,az,el in zip(ts,azs,els):
                    f.write(f"{rise.timestamp() + t}, {az}, {el}, \n")
        else:
            print("TODO :)")

        if input("Create another? [y/N]: ").lower() in ["y","yes"]:
            continue
        break


import sys
if __name__ == "__main__":
    main(sys.argv)