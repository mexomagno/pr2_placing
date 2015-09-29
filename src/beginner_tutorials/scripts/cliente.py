#!/usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('Suma_dos_enteros')
    try:
        sumalasweas = rospy.ServiceProxy('Suma_dos_enteros', sumaints)
        resp1 = sumalasweas(x, y)
        return resp1.suma
    except rospy.ServiceException, e:
        print "ERROR: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Pidiendo sumar %s con %s"%(x, y)
    print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
