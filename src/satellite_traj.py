from pyorbital import orbital
import os.path
from datetime import datetime,timedelta,timezone
import numpy as np
import json

project_dir = os.path.abspath(os.path.dirname(__file__) + "/..")

def main(argv):

    if len(argv) < 2:
        print("No sattelite name given")
        return


    tle_file = os.path.abspath(project_dir + "/tle_files/sample_tles.txt")
    try:
        with open(tle_file,"r") as f:
            lines =  f.read().split("\n")

        idxs_start = []
        idxs_in = []
        for i,line in enumerate(lines):
            if line != "" and line not in ["1","2"]:
                if line.lower().startswith(argv[1].lower()):
                    idxs_start.append(i)
                elif argv[1].lower() in line.lower():
                    idxs_in.append(i)

        n = None
        if len(idxs_start) + len(idxs_in) == 0:
            print("could not find satellite")
            exit()
        elif len(idxs_start) == 1:
            n = idxs_start[0]
        elif len(idxs_in) == 1:
            n = idxs_in[0]
        elif len(idxs_start) != 0:
            for idx in idxs_start:
                if input(f"Choose: \"{lines[idx]}\" [y/N]").lower() in ["y","yes"]:
                    n = idx
                    break
        else:
            for idx in idxs_in:
                if input(f"Choose: \"{lines[idx]}\" [y/N]").lower() in ["y","yes"]:
                    n = idx
                    break

        if n == None:
            print("No satellite was chosen")
            exit()

        sat_name = lines[n]

        print(f"Using TLE: \n{sat_name}\n{lines[n+1]}\n{lines[n+2]}\n")

        orb = orbital.Orbital(satellite=sat_name,line1=lines[n+1],line2=lines[n+2])
    except Exception as e:
        print(e)
        return

    # from pyorbital import tlefile


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
        orb.get_position(t,)
        # TODO maybe make this check take the time over the horizon into account, or maybe the signal strength of the satellite
        if orb.get_observer_look(max_el_time, long_gs, lat_gs, alt_gs)[1] < min_elevation:
            continue

        pass_time = fall-rise
        # print(pass_time.total_seconds())
        # print(dir(pass_time))
        # exit()

        point = 100 # total trajectory points

        ts,azs,els = [],[],[]

        for i in range(point):
            dt = timedelta(seconds=i * pass_time.total_seconds() / point)
            az,el = orb.get_observer_look(rise + dt, long_gs, lat_gs, alt_gs)
            ts.append(dt.total_seconds())
            azs.append(float(az))
            els.append(float(el))

        # print(azs)
        # print(els)

        az_dots,el_dots = [],[]
        t_prev,az_prev,el_prev = ts[0],azs[0],els[0]
        for t,az,el in zip(ts[1:], azs[1:], els[1:]):
            az_dots.append((az-az_prev)/(t-t_prev))
            el_dots.append((el-el_prev)/(t-t_prev))

        az_dots.append(az_dots[-1])
        el_dots.append(el_dots[-1])

        # print(az_dots)

        az_too_fast,el_too_fast = [],[]
        for az,el,az_dot,el_dot in zip(azs,els,az_dots,el_dots):
            if abs(az_dot) > rotor_az_max_speed:
                az_too_fast.append([az,el])
            if abs(el_dot) > rotor_el_max_speed:
                el_too_fast.append([az,el])

        if True:
            xs = np.array([azs,els])

            xs_fast_az = np.array(az_too_fast).T
            xs_fast_el = np.array(el_too_fast).T
            
            from matplotlib import pyplot as plt

            print(xs_fast_az.shape,xs_fast_el.shape)

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

        ans = input("Create trajectory? [y/N]").lower()
        if ans in ["q","quit","exit"]:
            break

        if ans not in ["y","yes"]:
            continue


        # create trajectory
        if len(xs_fast_az) == 0 and len(xs_fast_el) == 0:
            with open(project_dir + f"/trajectories/{sat_name.replace(' ','_')}_{rise.strftime('%Y_%m_%d_%H_%M_%S')}.csv", "w",newline="") as f:
                f.write("time [ms], azimuth [degrees], elevation [degrees]\n")
                for t,az,el in zip(ts,azs,els):
                    f.write(f"{rise.timestamp() + t}, {az}, {el}, \n")
        else:
            print("TODO :)")

        if input("Create another? [y/N]").lower() in ["y","yes"]:
            continue
        break


import sys
if __name__ == "__main__":
    main(sys.argv)