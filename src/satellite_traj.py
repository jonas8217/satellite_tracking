from pyorbital import orbital
import os.path
from datetime import datetime,timedelta,timezone
import numpy as np
import json
from matplotlib import pyplot as plt

VISUALIZE = True

project_dir = os.path.abspath(os.path.dirname(__file__) + "/..")

rotor_min_el = -5
rotor_max_el = 185
rotor_min_az = -180
rotor_max_az = 540

rotor_az_max_speed = 1.5 # degrees per second
rotor_el_max_speed = 1.5 # degrees per second

def az_el_to_uvec(az,el):
    theta = (90 - el) * (np.pi / 180.0)
    phi = - az * (np.pi / 180.0)
    return np.array([
        np.sin(theta) * np.cos(phi),
        np.sin(theta) * np.sin(phi),
        np.cos(theta)
    ])

def angular_distance(v1,v2):
    return float(np.arccos(max(min(np.dot(v1,v2) / (np.linalg.norm(v1)*np.linalg.norm(v2)),1),-1))) * 180/np.pi

def around_keyhole_compensation(ts,azs,mid_point):
    az_mid = azs[mid_point]

    # compensate by going too fast earlier such that the trajectory matches at the max elevation point
    i = mid_point - 1
    new_azs = azs[:]
    direction = float(np.sign(azs[mid_point] - azs[mid_point - 1]))
    while True:
        t = abs(ts[i] - ts[mid_point])
        if t * rotor_az_max_speed >= abs(azs[i] - az_mid):
            break
        new_azs[i] = az_mid - direction * t * rotor_az_max_speed
        i -= 1

    j = mid_point + 1
    while True:
        t = abs(ts[j] - ts[mid_point])
        if t * rotor_az_max_speed >= abs(azs[j] - az_mid):
            break
        new_azs[j] = az_mid + direction * t * rotor_az_max_speed
        j += 1 # Note the +1 instead of -1

    return new_azs

def through_keyhole_compensation(ts,azs,els,mid_point):
    el_mid = 90

    # compensate by going through the keyhole to minimize how much the azimuth has to change in a short duration
    i = mid_point
    new_azs = azs[:]
    new_els = els[:]

    while True:
        t = abs(ts[i] - ts[mid_point])
        # print(el_mid - t * rotor_el_max_speed,els[i])
        if el_mid - t * rotor_el_max_speed <= els[i]:
            break
        new_els[i] = el_mid - t * rotor_el_max_speed
        # print(i,els[i],new_els[i])
        i -= 1
    
    new_els[mid_point] = 90.0

    for n in range(mid_point+1, len(els)):
        new_els[n] = 180 - els[n]

    j = mid_point + 1
    while True:
        t = abs(ts[j] - ts[mid_point])
        if el_mid + t * rotor_el_max_speed >= new_els[j]:
            break
        new_els[j] = 180 - (el_mid - t * rotor_el_max_speed)
        # print(j,els[j],new_els[j])
        j += 1 # Note the +1 instead of -1



    # print(new_els[:i+1])
    # print(new_els[i+1:mid_point])
    # print(new_els[mid_point:j+1])
    # print(new_els[j+1:])
    # print(i,j)
    # exit()

    az_mid = azs[mid_point]

    direction = -float(np.sign(az_mid - azs[mid_point-1]))

    new_az_mid = az_mid + direction * 90

    i = mid_point

    while True:
        t = abs(ts[i] - ts[mid_point])
        # print(i, new_az_mid, azs[i])
        # print(new_az_mid - direction * t * rotor_az_max_speed, direction * azs[i])
        if new_az_mid - direction * t * rotor_az_max_speed <= direction * azs[i]:
            break
        # print(new_az_mid + direction * t * rotor_az_max_speed)
        new_azs[i] = new_az_mid - direction * t * rotor_az_max_speed
        # print(i,azs[i],new_azs[i])
        i -= 1

    # print(new_azs[:i+1])
    # print(new_azs[i+1:mid_point])
    # print(i)
    # exit()

    for n in range(mid_point+1, len(azs)):
        new_azs[n] = azs[n] + 180

    j = mid_point + 1
    while True:
        t = abs(ts[j] - ts[mid_point])
        if new_az_mid - direction * t * rotor_az_max_speed <= direction * new_azs[i]:
            break 
        # print(new_az_mid, direction * t * rotor_az_max_speed)
        new_azs[j] = new_az_mid + direction * t * rotor_az_max_speed
        # print(j,azs[j],new_azs[j])
        j += 1 # Note the +1 instead of -1
        # exit()

    # print(new_azs[:i+1])
    # print(new_azs[i+1:mid_point])
    # print(new_azs[mid_point:j])
    # print(new_azs[j:])
    # print(i,j)
    # exit()

    return new_azs,new_els

def trapz_integral(xs, ys):
    I = 0
    for i in range(1, len(xs)):
        I += (xs[i]-xs[i-1])*(ys[i-1] + ys[i])/2
    return I


def optimize_trajectory(ts, sat_azs, sat_els):
    # try both methods (through or around keyhole)
    # determine which is best and output that one

    max_el_idx = sat_els.index(max(sat_els))
    new_azs_around = around_keyhole_compensation(ts,sat_azs,max_el_idx) # around keyhole
    new_azs_through, new_els_through = through_keyhole_compensation(ts,sat_azs,sat_els,max_el_idx) # through keyhole

    err_around = []
    for az,new_az,el in zip(sat_azs,new_azs_around,sat_els):
        err_around.append(angular_distance(az_el_to_uvec(az,el),az_el_to_uvec(new_az,el)))

    err_through = []
    for az,new_az,el,new_el in zip(sat_azs,new_azs_through,sat_els,new_els_through):
        err_through.append(angular_distance(az_el_to_uvec(az,el),az_el_to_uvec(new_az,new_el)))

    E_through = trapz_integral(ts, err_through)
    E_around = trapz_integral(ts, err_around)

    # TODO use a collection of more criteria to choose the method, e.g. the expected S/N (signal to noise ratio).
    # The method could even be chosen during the pass to make some measurements about the S/N
    print(f"Accumulative error through keyhole: {E_through}\nAccumulative error around  keyhole {E_around}")
    if E_through < E_around:
        print("Choosing through keyhole")
        return new_azs_through, new_els_through, err_through, 0
    else:
        print("Choosing around keyhole")
        return new_azs_around, sat_els, err_around, 1


def plot_polar_traj(azs, els, title):
    xs = np.array([azs,els])

    fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
    fig.suptitle(title)
    ax.set_xlabel("Azimuth [deg]")
    label_position=ax.get_rlabel_position() # type: ignore
    ax.text(np.radians(label_position-12),27+ax.get_rmax()/2.0,'Elevation [deg]', rotation=-label_position,ha='center',va='center')  # type: ignore
    ax.set_rlim(bottom=90, top=0) # type: ignore
    ax.set_theta_direction(-1) # type: ignore
    ax.plot(xs[0,:]*np.pi/180, xs[1,:], marker=" ",label="satellite trajectory",zorder=1)
    ax.scatter(xs[0,0]*np.pi/180, xs[1,0], marker="o",s=45,label="rise", color="g",zorder=2)
    ax.scatter(xs[0,-1]*np.pi/180, xs[1,-1], marker="o",s=45,label="fall", color="r",zorder=3)

def write_traj(ts,azs,els,rise_time,sat_name):
    with open(project_dir + f"/trajectories/{sat_name.replace(' ','_')}_{rise_time.strftime('%Y_%m_%d_%H_%M_%S')}.csv", "w",newline="") as f:
        f.write("time [s], azimuth [degrees], elevation [degrees]\n")
        for t,az,el in zip(ts,azs,els):
            f.write(f"{rise_time.timestamp() + t}, {az}, {el}, \n")

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

    hours = 24

    pass_times = []

    now = datetime.now(timezone.utc)

    with np.testing.suppress_warnings() as sup:
        sup.filter(UserWarning, ".+")
        pass_times : list[tuple[datetime,datetime,datetime]] = orb.get_next_passes(now + timedelta(days=1), hours, long_gs, lat_gs, alt_gs)

    min_elevation = 15

    for p in pass_times:
        rise,fall,max_el_time = p
        # TODO maybe make this check take the time over the horizon into account, or maybe the signal strength of the satellite
        with np.testing.suppress_warnings() as sup:
            sup.filter(UserWarning, ".+")
            if orb.get_observer_look(max_el_time, long_gs, lat_gs, alt_gs)[1] < min_elevation:
                continue
        pass_time = fall-rise

        points = 1000 # total trajectory points

        ts,azs,els = [],[],[]

        offset = 0
        for i in range(points):
            dt = timedelta(seconds=i * pass_time.total_seconds() / points)
            with np.testing.suppress_warnings() as sup:
                sup.filter(UserWarning, ".+")
                az,el = orb.get_observer_look(rise + dt, long_gs, lat_gs, alt_gs)
            ts.append(dt.total_seconds())
            az += offset
            if i > 0:
                if az - azs[-1] > 180:
                    offset -= 360
                    az += offset
                elif az - azs[-1] < -180:
                    offset += 360
                    az += offset

            azs.append(float(az))
            els.append(float(el))
        # print(azs)
        # print(els)

        az_dots,el_dots = [],[]
        for i in range(1,len(ts)):
            az_dots.append((azs[i]-azs[i-1])/(ts[i]-ts[i-1]))
            el_dots.append((els[i]-els[i-1])/(ts[i]-ts[i-1]))

        az_dots.append(az_dots[-1])
        el_dots.append(el_dots[-1])

        # print(az_dots)

        az_too_fast,el_too_fast = [],[]
        for az,el,az_dot,el_dot in zip(azs,els,az_dots,el_dots):
            if abs(az_dot) > rotor_az_max_speed:
                az_too_fast.append([az,el])
            if abs(el_dot) > rotor_el_max_speed:
                el_too_fast.append([az,el])

        if VISUALIZE:
            xs = np.array([azs,els])

            az_too_fast = np.array(az_too_fast).T
            el_too_fast = np.array(el_too_fast).T

            # print(az_too_fast.shape, el_too_fast.shape)

            title = f"Satellite trajectory {'near keyhole' if az_too_fast.size != 0 else ''} - polar plot"

            fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
            fig.suptitle(title)
            ax.set_xlabel("Azimuth [deg]")
            label_position=ax.get_rlabel_position() # type: ignore
            ax.text(np.radians(label_position-12),27+ax.get_rmax()/2.0,'Elevation [deg]', rotation=-label_position,ha='center',va='center')  # type: ignore
            ax.set_rlim(bottom=90, top=0) # type: ignore
            ax.set_theta_direction(-1) # type: ignore
            ax.plot(xs[0,:]*np.pi/180, xs[1,:], marker=" ",label="satellite trajectory",zorder=1)
            ax.scatter(xs[0,0]*np.pi/180, xs[1,0], marker="o",s=45,label="rise", color="g",zorder=2)
            ax.scatter(xs[0,-1]*np.pi/180, xs[1,-1], marker="o",s=45,label="fall", color="r",zorder=3)
            # visualize problem areas
            #   angular velocity greater than axes velocity limits 
            order = [0,1,2]
            if az_too_fast.size != 0:
                print("Azimuth too fast")
                order = [0,3,1,2]
                ax.plot(az_too_fast[0,:]*np.pi/180, az_too_fast[1,:], marker=" ",label="Az speed > 1.5°/s")
            if el_too_fast.size != 0:
                print("Elevation too fast")
                ax.plot(el_too_fast[0,:]*np.pi/180, el_too_fast[1,:], marker=" ")

            handles, labels = plt.gca().get_legend_handles_labels()

            ax.legend([handles[idx] for idx in order],[labels[idx] for idx in order], bbox_to_anchor=(0.7,0.9))



            # f_name = title.replace(",","").replace(" ","_").replace("-","").replace("__","_").replace("__","_") + ".png"
            # path = "plots/" + f_name
            # print(path)
            # plt.subplots_adjust(top=0.88,hspace=0.38)
            # plt.savefig(path)
            plt.show()

        print(rise)
        ans = input("Create trajectory? [y/N]: ").lower()
        if ans in ["q","quit","exit"]:
            break

        if ans not in ["y","yes"]:
            continue


        # create trajectory
        if len(az_too_fast) == 0 and len(el_too_fast) == 0:
           write_traj(ts, azs, els, rise, sat_name)
        else:
            new_azs, new_els, err, method = optimize_trajectory(ts, azs, els)

            if VISUALIZE:
                if method == 1:
                    title = "trajectory optimization around keyhole"

                    fig, ax = plt.subplots(3,1)
                    fig.suptitle(title,fontsize=14)
                    ax[0].set_xlabel("time [s]")
                    ax[0].set_ylabel("Azimuth [deg]")
                    ax[0].plot(ts,azs, marker="",color="b",label="satellite Azimuth")
                    ax[0].plot(ts,new_azs, marker="",color="r",label="trajectory Azimuth")
                    ax[0].legend()

                    ax[1].set_xlabel("time [s]")
                    ax[1].set_ylabel("Azimuth speed [deg/s]")
                    ax[1].plot(ts,az_dots, marker="",color="r")

                    ax[2].set_xlabel("time [s]")
                    ax[2].set_ylabel("angular error [deg]")
                    ax[2].plot(ts,err, marker="",color="b")

                    # f_name = title.replace(",","").replace(" ","_") + ".png"
                    # path = "plots/" + f_name
                    # print(path)
                    # fig.set_size_inches(6.4, 6.6)
                    # plt.subplots_adjust(top=0.92,hspace=0.38)
                    # plt.savefig(path)
                    plt.show()

                else:
                    mid = len(els)//2
                    azs_ = azs[:mid] + (180 + np.array(azs[mid:])).tolist()
                    els_ = els[:mid] + (180 - np.array(els[mid:])).tolist()
                    new_azs_ = new_azs[:mid] + (180 + np.array(new_azs[mid:])).tolist()
                    new_els_ = new_els[:mid] + (180 - np.array(new_els[mid:])).tolist()

                    plot_polar_traj(azs,els,"sat")
                    plot_polar_traj(new_azs_,new_els_, "traj")
                    plt.show()

                    title = "trajectory optimization through keyhole"

                    fig, ax = plt.subplots(3,1)
                    fig.suptitle(title,fontsize=14)
                    ax[0].set_xlabel("time [s]")
                    ax[0].set_ylabel("Azimuth [deg]")
                    ax[0].plot(ts,azs_, marker="",color="b",label="satellite Azimuth")
                    ax[0].plot(ts,new_azs, marker="",color="r",label="trajectory Azimuth")
                    ax[0].legend()

                    ax[1].set_xlabel("time [s]")
                    ax[1].set_ylabel("Elevation [deg]")
                    ax[1].plot(ts,els_, marker="",color="b",label="satellite Elevation")
                    ax[1].plot(ts,new_els, marker="",color="r",label="trajectory Elevation")
                    ax[1].legend()

                    ax[2].set_xlabel("time [s]")
                    ax[2].set_ylabel("angular error [deg]")
                    ax[2].plot(ts,err, marker="",color="b")

                    # f_name = title.replace(",","").replace(" ","_") + ".png"
                    # path = "plots/" + f_name
                    # print(path)
                    # fig.set_size_inches(6.4, 6.6)
                    # plt.subplots_adjust(top=0.92,hspace=0.38)
                    # plt.savefig(path)
                    plt.show()


            write_traj(ts, new_azs, new_els, rise, sat_name)

        # if input("Next pass? [y/N]: ").lower() in ["y","yes"]:
        #     continue
        break


import sys
if __name__ == "__main__":
    main(sys.argv)