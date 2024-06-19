def setSup(v1, v2, v3, v4, v5, v6, v7, v8, s1, s2, s3, s4, s5, s6, ser):

    # Servo Range 1 2 3: 300--------------450-------------------900
    range_min = 600
    range_max = 300
    s1_calibration = -35-18
    s2_calibration = 45-3
    s3_calibration = 30+4
    s1 += s1_calibration
    s2 += s2_calibration
    s3 += s3_calibration
    s1_range_max = range_max + s1_calibration
    s2_range_max = range_max + s2_calibration
    s3_range_max = range_max + s3_calibration
    

    if s1 < s1_range_max:
        s1 = s1_range_max
    if s1 > range_min:
        s1 = range_min
        #msgbox('s1,out of range')

    s1_temp = '{:016b}'.format(s1)
    s1_h = int(s1_temp[:8], 2)
    s1_l = int(s1_temp[8:], 2)

    if s2 < s2_range_max:
        s2 = s2_range_max
    if s2 > range_min:
        s2 = range_min
        #msgbox('s2,out of range')

    s2_temp = '{:016b}'.format(s2)
    s2_h = int(s2_temp[:8], 2)
    s2_l = int(s2_temp[8:], 2)

    if s3 < s3_range_max:
        s3 = s3_range_max
    if s3 > range_min:
        s3 = range_min

    s3_temp = '{:016b}'.format(s3)
    s3_h = int(s3_temp[:8], 2)
    s3_l = int(s3_temp[8:], 2)

    # Servo Range 4 5 6: 230--------------450-------------------900
    range_max_1 = 230
    range_min_1 = 600
    s4_calibration = -148-105+148-20
    s5_calibration = -197-105+50-23
    s6_calibration = -118-80+118+25
    s4 += s4_calibration
    s5 += s5_calibration
    s6 += s6_calibration
    s4_range_max = range_max_1 + s4_calibration
    s5_range_max = range_max_1 + s5_calibration
    s6_range_max = range_max_1 + s6_calibration

    if s4 < s4_range_max:
        s4 = s4_range_max
    if s4 > range_min_1:
        s4 = range_min_1
        #msgbox('s4,out of range')

    s4_temp = '{:016b}'.format(s4)
    s4_h = int(s4_temp[:8], 2)
    s4_l = int(s4_temp[8:], 2)

    if s5 < s5_range_max:
        s5 = s5_range_max
    if s5 > range_min_1:
        s5 = range_min_1
        #msgbox('s5,out of range')

    s5_temp = '{:016b}'.format(s5)
    s5_h = int(s5_temp[:8], 2)
    s5_l = int(s5_temp[8:], 2)

    if s6 < s6_range_max:
        s6 = s6_range_max
    if s5 > range_min_1:
        s6 = range_min_1
        #msgbox('s5,out of range')

    s6_temp = '{:016b}'.format(s6)
    s6_h = int(s6_temp[:8], 2)
    s6_l = int(s6_temp[8:], 2)


    ############# Chambers range################
    #0--------------------------------------255#
    ############################################

    # data to be sent
    data = bytearray([106, v1, v2, v3, v4, v5, v6, v7, v8, s1_l, s1_h, s2_l, s2_h, s3_l, s3_h, s4_l, s4_h, s5_l, s5_h, s6_l, s6_h])
    try:
        # send the data via serial
        ser.write(data)
    except:
        print('error com port')
