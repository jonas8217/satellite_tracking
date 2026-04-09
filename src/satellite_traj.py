from pyorbital import orbital
import os.path
from datetime import datetime,timedelta,timezone
import numpy as np
import json
from matplotlib import pyplot as plt
import warnings

project_dir = os.path.abspath(os.path.dirname(__file__) + "/..")

def write_trajectory(tle,rise,ts,azs,els):
    with open(f"{project_dir}/trajectories/{tle[0].replace(' ','_')}_{rise.strftime('%Y_%m_%d_%H_%M_%S')}.csv", "w",newline="") as f:
        f.write("time [ms], azimuth [degrees], elevation [degrees]\n")
        for t,az,el in zip(ts,azs,els):
            f.write(f"{rise.timestamp() + t}, {az}, {el}, \n")

from utils import R_z,R_y

def polar_plot_setup(ax):
    ax.set_theta_offset(np.pi/2)
    ax.set_theta_direction(-1)
    ax.set_rlim(bottom=90, top=0)
    return ax

def add_aoslos(ax,aos,los):
    ax.plot(aos[0]*np.pi/180, aos[1], color="g", marker="o",clip_on=False, zorder=10, label="AOS", linestyle='None')
    ax.plot(los[0]*np.pi/180, los[1], color="r", marker="o",clip_on=False, zorder=10, label="LOS", linestyle='None')
    ax.legend()
    return ax

def add_aoslos_mod(ax,aos,los):
    ax.plot(aos[0]*np.pi/180, aos[1], color="c", marker="o",clip_on=False, zorder=10, label="track_AOS", linestyle='None')
    ax.plot(los[0]*np.pi/180, los[1], color="o", marker="o",clip_on=False, zorder=10, label="track_LOS", linestyle='None')
    ax.legend()
    return ax

def angular_dist(az1, el1, az2, el2):
    v1 = R_z(az1*np.pi/180) @ R_y(el1*np.pi/180) @ np.array([1,0,0])
    v2 = R_z(az2*np.pi/180) @ R_y(el2*np.pi/180) @ np.array([1,0,0])
    return np.arccos(min(np.dot(v1,v2),1))
angular_dist = np.vectorize(angular_dist)

def main(argv):

    tle_file = os.path.abspath(project_dir + "/tle_files/sample_tles.txt")

    with open(tle_file,"r") as f:
        lines =  f.read().split("\n")

    tles = []
    for i,line in enumerate(lines):
        if line != "" and line[0] not in ("1","2"):
            if i + 3 <= len(lines):
                if lines[i+1].startswith("1 ") and lines[i+2].startswith("2 "):
                    tles.append((lines[i].strip(),lines[i+1].strip(),lines[i+2].strip()))

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

        if len(tle_options) == 1:
            n = 0
        else:
            for i,tle in enumerate(tle_options):
                print(f" {i + 1}: {tle[0]}")

            n = int(input(f"Choose [1 - {len(tle_options)}]: ").strip()) - 1
            print()
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

    rotor_az_max_speed = 1.5 # degrees per second
    rotor_el_max_speed = 1.5 # degrees per second

    beam_width = 2.5 # "diameter" or 1.25 degrees as "radius"

    hours = 24*10

    warnings.filterwarnings("ignore",category=UserWarning)

    dt_now = datetime.now(timezone.utc)
    forward_delta = timedelta(hours=34,minutes=10)
    pass_times = orb.get_next_passes(dt_now + forward_delta, hours, long_gs, lat_gs, alt_gs)

    min_elevation = 15

    for p_idx,p in enumerate(pass_times):
        rise,fall,max_el_time = p
        # TODO maybe make this check take the time over the horizon into account, or maybe the signal strength of the satellite
        if orb.get_observer_look(max_el_time, long_gs, lat_gs, alt_gs)[1] < min_elevation:
            continue

        pass_time = fall-rise
        # print(pass_time.total_seconds())
        # print(dir(pass_time))
        # exit()

        points = 2000 # total trajectory points

        ts,azs,els = [],[],[]

        for i in range(points):
            dt = timedelta(seconds=i * pass_time.total_seconds() / points)
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

        xs = np.array([azs,els])

        azs_final = azs[:]
        els_final = els[:]

        # len(xs_fast_az) == 0 and len(xs_fast_el) == 0
        if True:
            gmt_p2 = timezone(timedelta(hours=2))
            delta : timedelta = rise - dt_now
            print(f"Rise in {delta.days * 24 + delta.seconds // 3600} hours and {(delta.seconds % 3600) // 60} minutes")
            print(f"Rise: {rise.strftime('%x %X')} Fall: {fall.strftime('%x %X')} UTC")
            print(f"Rise: {rise.astimezone(gmt_p2).strftime('%x %X')} Fall: {fall.astimezone(gmt_p2).strftime('%x %X')} UTC+2")

            xs_fast_az = np.array(az_too_fast).T
            xs_fast_el = np.array(el_too_fast).T

            fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
            ax = polar_plot_setup(ax)
            ax.plot(xs[0,:]*np.pi/180, xs[1,:], marker=" ")
            ax = add_aoslos(ax, xs[:,0],xs[:,-1])
            # visualize problem areas?
            #   angular velocity greater than combined axes  velocity limits 
            if xs_fast_az.size != 0:
                ax.plot(xs_fast_az[0,:]*np.pi/180, xs_fast_az[1,:], marker=" ")
            if xs_fast_el.size != 0:
                ax.plot(xs_fast_el[0,:]*np.pi/180, xs_fast_el[1,:], marker=" ")

            fig.set_size_inches(12,8)
            plt.show()

        # TODO make an algorithm for producing a trajectory that:
        #   1. Does not violate the rotor max speed
        #   2. Is able to choose between going around and through the zenith

        # TODO for polar plots:
        # add AOS and LOS for the satellite
        # add AOS and LOS due to inperfect tracking (use beam_width)
        # add original trajectory to compare

        if len(xs_fast_az) != 0 and len(xs_fast_el) == 0:
            print("The regular trajectory is faster than the rotor max azimuth speed, and therefore has to be optimized.\nShowing options")

            mid_idx = np.argmax(np.array(els))
            # Caluclate and show 2 possible options: through and around the zenith
            ## Around the Zenith:
            azs_around = np.array(azs)
            az_mid = azs[mid_idx]
            ts_mid = ts[mid_idx]
            sign = int(azs[mid_idx+1] > azs[mid_idx])*2-1
            i = 1
            while True:
                az_reach_max = abs(ts_mid - ts[mid_idx-i]) * rotor_az_max_speed
                az_diff = az_mid - azs[mid_idx-i]
                if az_reach_max > abs(az_diff):
                    break
                azs_around[mid_idx-i] = az_mid - sign * az_reach_max
                i += 1
            i = 1
            while True:
                az_reach_max = abs(ts_mid - ts[mid_idx+i]) * rotor_az_max_speed
                az_diff = az_mid - azs[mid_idx+i]
                if az_reach_max > abs(az_diff):
                    break
                azs_around[mid_idx+i] = az_mid + sign * az_reach_max
                i += 1

            # TODO recompute the elevation to minimize the error relative to the satellite (get close to 90 degrees elevation to minimize azimuth influence?)
            els_around = np.array(els)

            err_ang_around = angular_dist(azs,els,azs_around,els_around)

            ## Through the Zenith:
            azs_through = np.array(azs)
            azs_through[mid_idx:] = azs_through[mid_idx:] + (180 if azs[-1] < 180 else -180)
            azs_flip = azs_through.copy()

            az_mid_through = (azs_flip[0] + azs_flip[-1])/2
            sign = int(azs[-1] > azs[0])*2-1

            azs_through[mid_idx] = az_mid_through
            i = 1
            while True:
                az_reach_max = abs(ts_mid - ts[mid_idx-i]) * rotor_az_max_speed
                az_diff = az_mid_through - azs_flip[mid_idx-i]
                if az_reach_max > abs(az_diff):
                    break
                azs_through[mid_idx-i] = az_mid_through + sign * az_reach_max
                # print("az reach max:", az_reach_max)
                # print("az diff:", az_diff)
                i += 1
            i = 1
            while True:
                az_reach_max = abs(ts_mid - ts[mid_idx+i]) * rotor_az_max_speed
                az_diff = az_mid_through - azs_flip[mid_idx+i]
                if az_reach_max > abs(az_diff):
                    break
                azs_through[mid_idx+i] = az_mid_through - sign * az_reach_max
                # print("az reach max:", az_reach_max)
                # print("az diff:", az_diff)
                i += 1

            # Elevation can be further optimized to minimize error (would need to be numerically adjusted)
            el_mid = 90
            els_through = np.array(els)
            els_through[mid_idx:] = 180 - els_through[mid_idx:]
            els_flip = els_through.copy()

            els_through[mid_idx] = el_mid
            i = 1
            while True:
                el_reach_max = abs(ts_mid - ts[mid_idx-i]) * rotor_el_max_speed
                el_diff = el_mid - els_flip[mid_idx-i]
                if el_reach_max > abs(el_diff):
                    break
                els_through[mid_idx-i] = el_mid - el_reach_max
                i += 1
            i = 1
            while True:
                el_reach_max = abs(ts_mid - ts[mid_idx+i]) * rotor_el_max_speed
                el_diff = els_flip[mid_idx+i] - el_mid
                if el_reach_max > abs(el_diff):
                    break
                els_through[mid_idx+i] = el_mid + el_reach_max
                i += 1

            err_ang_through = angular_dist(azs,els,azs_through,els_through)

            debug = True

            fig = plt.figure()



            if debug:
                ax1_1 = fig.add_subplot(3,2,1,projection='polar')
            else:
                ax1_1 = fig.add_subplot(1,2,1,projection='polar')
            ax1_1 = polar_plot_setup(ax1_1)
            ax1_1.title.set_text("Around Zenith")
            ax1_1.plot(xs[0,:]*np.pi/180, xs[1,:], marker=" ",label="original")
            ax1_1.plot(azs_around*np.pi/180, els_around, marker=" ",label="modified")
            ax1_1 = add_aoslos(ax1_1, xs[:,0],xs[:,-1])
            if debug:
                ax1_2_1 = fig.add_subplot(3,2,3)
                ax1_2_1.set_ylabel("Azimuth")
                ax1_2_2 = ax1_2_1.twinx()
                ax1_2_2.set_ylabel("Elevation")
                ax1_2_2.set_yticks([0,15,30,45,60,75,90])
                l1_2_1_1 = ax1_2_1.plot(ts,azs,marker=" ",color="b",label="Az no compensation")
                l1_2_1_2 = ax1_2_1.plot(ts,azs_around,marker=" ",color="r",label="Az compensated")
                l1_2_2_1 = ax1_2_2.plot(ts,els,marker=" ",color="g",label="El no compensation")
                # l1_2_2_2 = ax1_2_2.plot(ts,els_around,marker=" ",color="c",label="El compensated")

                lines = l1_2_1_1+l1_2_1_2+l1_2_2_1#+l1_2_2_2
                ax1_2_1.legend(lines, [l.get_label() for l in lines])

                ax1_3 = fig.add_subplot(3,2,5)
                ax1_3.plot(ts, err_ang_around*180/np.pi, marker=" ", label="angular error")
                ax1_3.plot(ts,np.array([beam_width/2]*len(ts)), color="r", marker=" ", linestyle=(0,(3,6)), zorder=2, label="beam-width")
                ax1_3.legend()


            if debug:
                ax2_1 = fig.add_subplot(3,2,2,projection='polar')
            else:
                ax2_1 = fig.add_subplot(1,2,2,projection='polar')
            ax2_1 = polar_plot_setup(ax2_1)
            ax2_1.title.set_text("Through Zenith")
            # Reproject to the plotting space for the polar plot (+90 degrees does not work):
            els_through_polar = np.concatenate((els_through[:mid_idx],180 - els_through[mid_idx:]))
            azs_through_polar = np.concatenate((azs_through[:mid_idx],azs_through[mid_idx:] - (180 if azs[-1] < 180 else -180)))
            ax2_1.plot(xs[0,:]*np.pi/180, xs[1,:], marker=" ",label="original")
            ax2_1.plot(azs_through_polar*np.pi/180, els_through_polar, marker=" ",label="modified")
            ax2_1 = add_aoslos(ax2_1, xs[:,0],xs[:,-1])
            if debug:
                ax2_2_1 = fig.add_subplot(3,2,4)
                ax2_2_1.set_ylabel("Azimuth")
                ax2_2_2 = ax2_2_1.twinx()
                ax2_2_2.set_ylabel("Elevation")
                ax2_2_2.set_yticks([0,30,60,90,120,150,180])
                l2_2_1_1 = ax2_2_1.plot(ts,azs_flip,marker=" ",color="b",label="Az no compensation")
                l2_2_1_2 = ax2_2_1.plot(ts,azs_through,marker=" ",color="r",label="Az compensated")
                l2_2_2_1 = ax2_2_2.plot(ts,els_flip,marker=" ",color="g",label="El no compensation")
                l2_2_2_2 = ax2_2_2.plot(ts,els_through,marker=" ",color="c",label="El compensated")

                lines = l2_2_1_1+l2_2_1_2+l2_2_2_1+l2_2_2_2
                ax2_2_1.legend(lines, [l.get_label() for l in lines])

                ax2_3 = fig.add_subplot(3,2,6)
                ax2_3.plot(ts, err_ang_through*180/np.pi, marker=" ", label="angular error")
                ax2_3.plot(ts,np.array([beam_width/2]*len(ts)), color="r", marker=" ", linestyle=(0,(5,8)), zorder=2, label="beam-width")
                ax2_3.legend()


            # fig.subplots_adjust(0,0.055,0.96,0.933,0,0.15)
            fig.set_size_inches(16,8)
            plt.show()



        if len(xs_fast_el) != 0:
            print("This should generally not be possible...")
            exit()


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
            pass
        elif len(xs_fast_az) != 0 and len(xs_fast_el) == 0:
            try:
                choice = int(input(f"Choose Around[0] or Through[1] the Zenith: ").strip())
            except:
                exit()

            if choice == 0:
                azs_final,els_final = azs_around,els_around
            elif choice == 1:
                azs_final,els_final = azs_through,els_through
        else:
            print("This should already have exited, how did you get here? (len(xs_fast_el) != 0)")


        write_trajectory(tle,rise,ts,azs_final,els_final)
        if p_idx < len(pass_times)-1 and input("Create another? [y/N]: ").lower() in ["y","yes"]:
            continue
        break


import sys
if __name__ == "__main__":
    main(sys.argv)