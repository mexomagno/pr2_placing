#!/usr/bin/env python

from beginner_tutorials.srv import *
import rospy

def handle_add_two_ints(req):
    print "Retornando [%s + %s = %s]"%(req.primero, req.segundo, (req.primero + req.segundo))
    return sumaintsResponse(req.primero + req.segundo)

def add_two_ints_server():
    rospy.init_node("sumaints")
    s= rospy.Service("Suma_dos_enteros",sumaints, handle_add_two_ints)
    print "Listo para sumar enteros!"
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
