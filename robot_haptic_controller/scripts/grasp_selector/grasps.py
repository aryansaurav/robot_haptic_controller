#!/usr/bin/env python


class grasp_primitives(object):
  """docstring for grasp_primitives"""
  def __init__(self):
    

    pOfs123 = dict()
    pOfs123['position'] = [0,   0.872664626,   0.872664626,   0.785398163,   0,   0.872664626,  0.872664626,
                           0.785398163,   0,  0.872664626,   0.872664626,   0.785398163,  0.279252680,   0,   0,  0]
    pOfs123['patch1'] = [0.,  0.,  0.,  0.,     0.,  0.,  0.,  0.,     0.,  0.,  0.,  0.,
                         1.1,  0.,  0.,  0.,     0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  1.]
    pOfs123['patch2'] = [0.,  0.7,  1.,  1.,  0.,  0.7,  1.,  1.1,  0.,  0.7,  1., 1.,
                         0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.]

    # OK (pb with this one)
    tsOfs123h = dict()
    # Changed thumb base position otherwise bug (from the hand ???)
    tsOfs123h['position'] = [0.,  0.83775804,  0.34906585,  0.12217305,  0.,  0.83775804,  0.34906585,
                             0.12217305,  0.,  0.83775804,  0.34906585,  0.12217305,  1.57881011, 0.06179939, 0.1453293,  0.17453293]
    # Disable thumb tip
    # tsOfs123h['patch1'] = [0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,
                           # 0., 0.,  1.1,  0,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]
    tsOfs123h['patch1'] = [0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,
                           0., 0.,  1.1,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]
    # This patch2 has first phalanxes disabled
    # tsOfs123h['patch2'] = [0.,  0.7,  1.,  1.,  0.,  0.7,  1.,  1.1,  0.,  0.7,  1., 1.,
                           # 0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.]
    # Removed base of finger contact (hard to reach something with it ...)
    tsOfs123h['patch2'] = [0.,  0.0,  1.,  1.,  0.,  0.0,  1.,  1.1,  0.,  0.0,  1., 1.,
                           0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.]

    my_full_grasp_no_index = dict()

    my_full_grasp_no_index['position'] = [0.,  0.0,  0.0,  0.0,
                                             0.,  0.83775804,  0.34906585, 0.12217305,
                                             0.,  0.83775804,  0.34906585,  0.12217305,
                                             1.67881011, 0.06179939, -0.17453293,  0.17453293]
    my_full_grasp_no_index['patch1']  =   [0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,
                                           0.,  0.,  1.,  1.,

                                           0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,  0.]

    my_full_grasp_no_index['patch2'] =    [0.,  0.,  0.,  0.0,
                                           0.,  0.,  1.,  1.1,
                                           0.,  0.,  1.,  1.1,
                                           0.,  0.,  0.,  0.,

                                           0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,  0.]

#############################################3

    three_per_finger = dict()

    three_per_finger['position'] = my_full_grasp_no_index['position'] 
    three_per_finger['patch1']  =   [0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,
                                           0.,  0.,  1.,  1.,

                                           0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,  0.]
    three_per_finger['patch2'] =    [0.,  1.,  1.,  1.0,
    # three_per_finger['patch2'] =    [0.,  0.,  0.,  0.0,  # when index finger was broken
                                           0.,  1.,  1.,  1.1,
                                           0.,  10.,  1.,  1.1,
                                           0.,  0.,  0.,  0.,

                                           0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,  0.]

#############################################3

    zero_contact = dict()

    zero_contact['position'] = my_full_grasp_no_index['position'] 
    zero_contact['patch1']  =   [0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,

                                           0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,  0.]
    zero_contact['patch2'] =    [0.,  0.,  0.,  0.0,
                                           0.,  0.,  0.,  0.0,
                                           0.,  0.,  0.,  0.0,
                                           0.,  0.,  0.,  0.,

                                           0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,
                                           0.,  0.,  0.,  0.,  0.]
#############################################3
    tsOfs123l = dict()
    tsOfs123l['position'] = [0.,  0.83775804,  0.34906585,  0.12217305,  0.,  0.83775804,  0.34906585,
                             0.12217305,  0.,  0.83775804,  0.34906585,  0.12217305,  1.37881011, -0.26179939, -0.17453293,  0.17453293]
    tsOfs123l['patch1'] = [0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,
                           0.,  0.,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]
    tsOfs123l['patch2'] = [0.,  1.,  1.,  0.,  0.,  1.1,  1.,  0.,  0.,  1.,  1.,  0.,
                           0., 0.,  0.,  0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]

    # OK
    tsOp = dict()
    tsOp['position'] = [0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,
                        0.,  0.,  0.,  1.37881011, -0.1981317, -0.17453293,  1.04719755]
    tsOp['patch1'] = [0.,  0.,  0.,  0.,      0.,  0.,  0.,  0.,      0.,  0.,  0., 0.,
                      0.,  0.7,  1.,  1.1,      0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.]
    tsOp['patch2'] = [0.,  0.,  0.,  0.,          0.,  0.,  0.,  0.,          0.,  0.,  0.,  0.,
                      0., 0.,  0.,  0.,       0., 0.,  0.0,  0.0,     0.,  0.,  0.,  0.,     1.1,  0., 0.,  0.,     1.]
    # OK
    tsOs12 = dict()
    tsOs12['position'] = [0.30142573,  0.78539816,  1.04719755,  0.78539816,  0.30142573,
                          1.50098316,  0.87266463,  0.29670597,  0.,  0.2,  0.1,  0.1,  0.28, 0.22, -0.09, 0.5]
    tsOs12['patch1'] = [0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,
                        0.7,  1.1,  1.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.]
    tsOs12['patch2'] = [0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,
                        0.,  0.,  0.,  0.,  0.7,  1.1,  1.,  0.,  0.7, 1.,  1.,  0.,  0.,  0.,  0.,  0.]


    my_full_grasp = dict()
    my_full_grasp['position'] = [0.,  0.83775804,  0.34906585,  0.12217305,
                                 0.,  0.83775804,  0.34906585, 0.12217305,
                                 0.,  0.83775804,  0.34906585,  0.12217305,
                                 1.37881011, -0.26179939, -0.17453293,  0.17453293]
    
    thumb_tip = [0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,
                          0., 0.,  0.,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]

    fingers_tip =   [0.,  0.,  0.,  1.,  
    0.,  0.,  0.,  1.1,  
    0.,  0.,  0.,  1.,
                          0., 0.,  0.,  0.,  
                          0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]


    # OK
    ttOft123 = dict()
    ttOft123['position'] = [0.,  0.83775804,  0.34906585,  0.12217305,  0.,  0.83775804,  0.34906585,
                            0.12217305,  0.,  0.83775804,  0.34906585,  0.12217305,  1.37881011, -0.26179939, -0.17453293,  0.17453293]
    # ttOft123['patch1'] = [0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,
    #                       0., 0.,  0.,  1.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]
    # ttOft123['patch2'] = [0.,  0.,  0.,  1.,  0.,  0.,  0.,  1.,  0.,  0.,  0.,  1.1,
    #                       0., 0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.]
    ttOft123['patch1'] = thumb_tip
    ttOft123['patch2'] = fingers_tip

    finger3 = [0.,  1.,  1.,  1.1,
               0.,  1.,  1.,  1.1,
               0.,  1.,  1.,  1.1,
               0.,  0.,  0.,  0.,

               0.,  0.,  0.,  0.,
               0.,  0.,  0.,  0.,
               0.,  0.,  0.,  0.,  0.]
    finger3nopink = [0.,  1.,  1.,  1.1,
               0.,  1.,  1.,  1.1,
               0.,  0.,  0.,  0.0,
               0.,  0.,  0.,  0.,

               0.,  0.,  0.,  0.,
               0.,  0.,  0.,  0.,
               0.,  0.,  0.,  0.,  0.]

    finger3index = [0.,  1.,  1.,  1.1,
               0.,  0.,  1.,  1.1,
               0.,  0.,  1.,  1.1,
               0.,  0.,  0.,  0.,

               0.,  0.,  0.,  0.,
               0.,  0.,  0.,  0.,
               0.,  0.,  0.,  0.,  0.]

    finger2 = [0.,  0.,  1.,  1.1,
               0.,  0.,  1.,  1.1,
               0.,  0.,  1.,  1.1,
               0.,  0.,  0.,  0.,

               0.,  0.,  0.,  0.,
               0.,  0.,  0.,  0.,
               0.,  0.,  0.,  0.,  0.]


    # same as tsofsH                                
    my_full_grasp['position'] =[0.,  0.83775804,  0.34906585,  0.12217305,  0.,  0.83775804,  0.34906585,
                             0.12217305,  0.,  0.83775804,  0.34906585,  0.12217305,  1.57881011, 0.06179939, 0.1453293,  0.17453293]

    my_full_grasp['patch1'] =  [0.,  0.,  0.,  0.,
                               0.,  0.,  0.,  0.,
                               0.,  0.,  0.,  0.,
                               0.,  0.,  5.,  5.,
                               0.,  0.,  0.,  0.,
                               0.,  0.,  0.,  0.,
                               0.,  0.,  0.,  0.,  0.]
                               
    my_full_grasp['patch2'] = [0.,  0.,  1.,  1.1,
               0.,  0.,  1.,  1.2,
               0.,  0.,  1.,  1.1,
               0.,  0.,  0.,  0.,

               0.,  0.,  0.,  0.,
               0.,  0.,  0.,  0.,
               0.,  0.,  0.,  0.,  0.]
    # [0.,  1.,  1.,  1.1,
    #            0.,  1.,  1.,  1.2,
    #            0.,  1.,  1.,  1.1,
    #            0.,  0.,  0.,  0.,

    #            0.,  0.,  0.,  0.,
    #            0.,  0.,  0.,  0.,
    #            0.,  0.,  0.,  0.,  0.]

    # Tip grasp.
    precision_disk = dict()
    precision_disk['patch1'] = thumb_tip
    precision_disk['patch2'] = fingers_tip
    precision_disk['position'] = [0.24466872925561411, 0.7516013693590897, 0.7185794199868916, 0.7256737800835952, 0.16167471104868714, 0.8604280810313537, 0.8219938471469248, 0.6551926033013512, 0.2478912057281758, 0.7725392530837093, 0.7823391936311186, 0.504936954013225, 1.434908878919885, 0.09785298928954592, 0.28692295651512306, 0.9835755299629617]

    # [ 0.,  0.83775804,  0.34906585,  0.12217305,  
    #                           0.,  0.83775804,  0.34906585,0.12217305,  
    #                           0.,  0.83775804,  0.34906585,  0.12217305,  
    #                           1.80881011, 0.10179939, 0.17453293,  0.17453293]

    # Tripod side grasp
    tripod = dict()
    tripod['position'] = [0.2667272950390217, 0.7481597017302286, 1.0773778087423926, 0.2920762285055284, 
    0.1681621361470414, 0.9368356540192291, 0.9450684080533669, 0.4171753499172109, 
    0.1788256951260598, 0.0869167666405636, 0.0829013725767343, 0.10625126554307424, 
    1.2661472732635355, 0.027460594500212796, -0.1297994655501145, 0.7324531249851967]

    # [0.30142573,  0.78539816,  1.04719755,  0.78539816,  
    #                         0.30142573, 1.50098316,  0.87266463,  0.29670597,  
    #                         0.,  0.2,  0.1,  0.1,  
    #                         0.88, 0.32, 0.29, 0.5]
    tripod['patch1'] = [0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,
                        0.0,  1.1,  1.2,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,  0.,  0.,  0.,  0.,  0.]
    tripod['patch2'] = [0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0.,  0., 0.,  0.,
                        0.,  0.,  0.,  0.,  0.0,  1.1,  1.2,  0.,  0.0, 1.,  1.2,  0.,  0.,  0.,  0.,  0.]

    # grasp_primitives = []
    # grasp_primitives.append(ttOft123)   # 0 TIP
    # grasp_primitives.append(tsOfs123h)  # 1 SURF H
    # grasp_primitives.append(tsOfs123l)  # 2 SURF L
    # grasp_primitives.append(pOfs123)    # 3 PALM 123
    # grasp_primitives.append(tsOp)       # 4 PALM T
    # grasp_primitives.append(tsOs12)     # 5 SIDE
    # grasp_primitives.append(my_full_grasp)     # 5 SIDE
    # grasp_primitives.append(my_full_grasp_no_index)       # 6 PALM T (again)

    f2oppos = dict()

    f2oppos['position'] =[0.,  0.83775804,  0.34906585,  0.12217305,  
    0.,  0.83775804,  0.34906585,0.12217305,  
    0.,  0.03775804,  0.04906585,  0.02217305,  
    1.57881011, 0.06179939, 0.1453293,  0.17453293]

    f2oppos['patch1'] = [0.,  0.,  0.,  0.,
                               0.,  0.,  0.,  0.,
                               0.,  0.,  0.,  0.,
                               0.,  0.,  1.,  1.,
                               0.,  0.,  0.,  0.,
                               0.,  0.,  0.,  0.,
                               0.,  0.,  0.,  0.,  0.]
    f2oppos['patch2'] = [0.,  0.,  1.,  1.1,
               0.,  0.,  1.,  1.1,
               0.,  0.,  0.,  .0,
               0.,  0.,  0.,  0.,

               0.,  0.,  0.,  0.,
               0.,  0.,  0.,  0.,
               0.,  0.,  0.,  0.,  0.]


    ulnar = dict()

    ulnar['position'] =[0.,  0.03775804,  0.04906585,  0.12217305,  
    0.,  0.03775804,  0.04906585,0.12217305,  
    0.,  0.83775804,  0.34906585,  0.02217305,  
    1.57881011, 0.06179939, 0.1453293,  0.17453293]

    ulnar['patch1'] = [0.,  0.,  0.,  0.,
                               0.,  0.,  0.,  0.,
                               0.,  0.,  0.,  0.,
                               0.,  0.,  1.,  1.,
                               0.,  0.,  0.,  0.,
                               0.,  0.,  0.,  0.,
                               0.,  0.,  0.,  0.,  0.]
    ulnar['patch2'] = [0.,  0.,  0.,  0.,
               0.,  0.,  0.,  0.,
               0.,  0.,  1.,  1.1,
               0.,  0.,  0.,  0.,

               0.,  0.,  0.,  0.,
               0.,  0.,  0.,  0.,
               0.,  0.,  0.,  0.,  0.]

    grasp_primitives = []
    grasp_primitives.append(precision_disk)   # 0 TIP
    grasp_primitives.append(tripod)     # 5 SIDE
    grasp_primitives.append(my_full_grasp)     # 5 SIDE
    grasp_primitives.append(f2oppos)  # 1 SURF H
    grasp_primitives.append(ulnar)    # 3 PALM 123
    
    grasp_primitives.append(tsOfs123h)  # 2 SURF L
    grasp_primitives.append(tsOfs123l)  # 2 SURF L
    grasp_primitives.append(pOfs123)    # 3 PALM 123
    grasp_primitives.append(tsOp)       # 4 PALM T
    grasp_primitives.append(tsOs12)     # 5 SIDE
    grasp_primitives.append(my_full_grasp_no_index)       # 6 PALM T (again)
    grasp_primitives.append(three_per_finger)       # 6 PALM T (again)
    grasp_primitives.append(zero_contact)       # 6 PALM T (again)


    self.grasp_primitives = grasp_primitives
