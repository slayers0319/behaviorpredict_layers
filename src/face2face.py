#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import String

R_L_dic = {'personR':'personL','personL':'personR'}
Timer = 0
flag = 0

def face_to_face(data):
    global pub_point
    global Timer
    global flag
    data_splited = data.data.replace(',B','').split(',')
    print("="*20)
    result = []
    for i in range(len(data_splited)/3-1):
        if data_splited[i*3] not in R_L_dic:
            continue
        
        for j in range(i+1,len(data_splited)/3):
            if data_splited[j*3] not in R_L_dic:
                continue

            if data_splited[j*3]==data_splited[i*3]:
                continue
            
            if abs(float(data_splited[i*3+2])-float(data_splited[j*3+2]))<0.5:
                if data_splited[i*3]=='personR' and float(data_splited[i*3+1]) < float(data_splited[j*3+1]):
                    result.append([data_splited[i*3:i*3+3],data_splited[j*3:j*3+3]])
                elif data_splited[i*3]=='personL' and float(data_splited[i*3+1]) > float(data_splited[j*3+1]):
                    result.append([data_splited[i*3:i*3+3],data_splited[j*3:j*3+3]])

    print(result)
    if len(result)==0:
        return

    d = 100000
    output = []
    for point in result:
        if float(point[0][2])<d:
            d = point[0][2]
            output = point
    output = "P1,{},{},P2,{},{}".format(output[0][1],output[0][2],output[1][1],output[1][2])
    print(output)
    Timer = time.time()
    flag = 1
    pub_point.publish(output)
    
rospy.init_node('face2face',anonymous=True)
rospy.Subscriber('/fusion_data', String, face_to_face, queue_size=1)
pub_point = rospy.Publisher('/face2face_point', String, queue_size=1)


r = rospy.Rate(10)
print("start")
while not rospy.is_shutdown():
    if (time.time()-Timer)>16 and flag==1:
        flag = 0
        pub_point.publish("clear")
    r.sleep()
