#!/usr/bin/env python

import wrap

if __name__ == '__main__':
    a = wrap.getMinJointPositions()
    a.printValues("JointPosition")
    print(wrap.min(a))
    a = a + a
    a.printValues("Operator add")
    a = a * 3.0
    a.printValues("Operator mul")

    g = wrap.GenericVector()
    g.setValues([1.2, 3.4, 1.2, 3.4, -1.2])
    g.printValues("GenericVector")
    print(g.max())
    print(g.min())
    g = g.abs()
    print(g.isValid())
    print(g.isZero())
    g.printValues("GenericVector")
    g *= 2
    g.printValues("GenericVector")
    g *= g
    g.printValues("GenericVector")

    j = wrap.JointPosition()
    print(type(j))
    j.setValues([1.2, 3.4, 1.2, 3.4, -1.2])
    j.printValues("JointPosition")
    print(j.max())
    print(j.min())
    print(j.isValid())
    print(j.isZero())
    print(j.q1())
    j.setQ1(5.0)
    print(j.q1())
    print(j.isReachable())

    c = wrap.CylindricPosition()
    c.setValues([1.34, 1.34, 2.09, 2.94, 2.94])
    c.printValues("CylindricPosition")
    d = c.toJointspace(True)
    d.printValues("JointPosition")
    d = c.toJointspace(False)
    d.printValues("JointPosition")
    d = c.toJointspace()
    d.printValues("JointPosition")
    d = c.toJointspace(j)
    d.printValues("JointPosition")
    
    s = wrap.StaticParameters()
    print(s.mass_5)
    s.mass_5 = 20.44
    print(s.mass_5)