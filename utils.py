import os
import subprocess
def read_from_file(filename):
    listed = []
    with open(filename,"r") as text:
        Line = text.readline()
        while Line!='':
            listed.append(Line.rstrip("\n"))
            Line = text.readline()

    return listed
def get_net_interfaces():
    # get the list of the network interfaces names
    proc = subprocess.Popen(["ls /sys/class/net"], stdout=subprocess.PIPE, shell=True)
    (out, err) = proc.communicate()
    out = out.decode().split("\n")
    # remove last empty element
    out.pop()
    out.insert(0, "default")
    return out
    