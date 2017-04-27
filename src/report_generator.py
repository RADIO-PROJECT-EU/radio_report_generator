#!/usr/bin/env python
import os
import rospy
import rospkg
import smtplib
import numpy as np
from datetime import datetime
from datetime import timedelta
from std_msgs.msg import Int32
from kobuki_msgs.msg import Sound
from email.mime.text import MIMEText
from email.MIMEMultipart import MIMEMultipart
from email.mime.application import MIMEApplication

sound_pub = None

def generateReport(msg):
    global sound_pub
    files_to_move = []
    fromaddr = "roboskelncsr@gmail.com"
    toaddr = ["gstavrinos@iit.demokritos.gr", "gs.juggle@gmail.com"]
    subject = "Medical Report as of "+datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    msg = MIMEMultipart()
    msg['From'] = fromaddr
    msg['To'] = ", ".join(toaddr)
    msg['Bcc'] = "stavrinosgeo@gmail.com"
    msg['Subject'] = subject

    body = subject+"\n\n\n"

    rospack = rospkg.RosPack()

    radio_logs = '~/radio_logs'
    rmax = 1
    if not os.path.exists(os.path.expanduser(radio_logs)):
        os.makedirs(os.path.expanduser(radio_logs))
    else:
        for i in os.listdir(os.path.expanduser(radio_logs)):
            if 'report' in i:
                i = i.replace('report','')
                i = i.replace('.csv','')
                if int(i) >= rmax:
                    rmax = int(i) + 1
    with open(os.path.expanduser(radio_logs)+'/'+'report'+str(rmax)+'.csv','w+') as report_file:
        path = rospack.get_path('motion_analysis_wrapper')+'/logs/'
        files = []
        found_file = False
        for i in os.listdir(path):
            for d in range(2, -1, -1):
                if os.path.isfile(os.path.join(path,i)) and 'official_log_bed_'+(datetime.today()-timedelta(d)).strftime("%d-%m-%Y") in i:
                    found_file = True
                    files.append(i)
                    files_to_move.append(path+i)

        counter = 0
        repetition = 'a'
        annotations1 = ['4', '38', '57']
        annotations2 = ['V2', 'V15', 'V25']
        if found_file:
            files.sort()
            for f in files:
                content = np.genfromtxt(path+f, delimiter=',')
                content = content[1]
                report_file.write(annotations1[counter]+','+annotations2[counter]+','+repetition+','+str(content[0])+','+str(content[1])+'\n')
                counter += 1
                if counter == len(annotations1):
                    counter = 0
                    repetition = 'b'

        files = []
        found_file = False

        for i in os.listdir(path):
            for d in range(2, -1, -1):
                if os.path.isfile(os.path.join(path,i)) and 'official_log_pill_'+(datetime.today()-timedelta(d)).strftime("%d-%m-%Y") in i:
                    found_file = True
                    files.append(i)
                    files_to_move.append(path+i)

        counter = 0
        repetition = 'a'
        annotations1 = ['10', '22', '44', '63']
        annotations2 = ['V5', 'V7', 'V18', 'V28']
        if found_file:
            files.sort()
            for f in files:
                content = np.genfromtxt(path+f, delimiter=',',dtype=None)
                content = content[1]
                report_file.write(annotations1[counter]+','+annotations2[counter]+','+repetition+','+str(content[0])+','+str(content[1])+'\n')
                counter += 1
                if counter == len(annotations1):
                    counter = 0
                    repetition = 'b'
        files = []
        found_file = False

        path = rospack.get_path('hpr_wrapper')+'/logs/'
        for i in os.listdir(path):
            for d in range(2, -1, -1):
                if os.path.isfile(os.path.join(path,i)) and 'official_log_walk_'+(datetime.today()-timedelta(d)).strftime("%d-%m-%Y") in i:
                    found_file = True
                    files.append(i)
                    files_to_move.append(path+i)

        counter = 0
        repetition = 'a'
        annotations1 = ['12', '24', '30', '50', '65', '69', '75']
        annotations2 = ['V6', 'V8', 'V11', 'V21', 'V29', 'V31', 'V34']
        if found_file:
            files.sort()
            for f in files:
                content = []
                filtered = []
                title = []
                content = np.genfromtxt(path+f, delimiter=',')
                content = content[1:-1]
                filtered = [x for x in content if x[2] > 2]
                filtered.sort(key=lambda x: x[2])
                filtered = filtered[len(filtered)/2]
                report_file.write(annotations1[counter]+','+annotations2[counter]+','+repetition+',')
                report_file.write(str(filtered[2])+'\n')
                counter += 1
                if counter == len(annotations1):
                    counter = 0
                    repetition = 'b'

        files = []
        found_file = False

        path = rospack.get_path('ros_visual_wrapper')+'/logs/'
        for i in os.listdir(path):
            for d in range(2, -1, -1):
                if os.path.isfile(os.path.join(path,i)) and 'official_log_chair_'+(datetime.today()-timedelta(d)).strftime("%d-%m-%Y") in i:
                    found_file = True
                    files.append(i)
                    files_to_move.append(path+i)

        counter = 0
        repetition = 'a'
        annotations1 = ['8', '28', '34', '48', '61']
        annotations2 = ['V4', 'V10', 'V13', 'V20', 'V27']
        if found_file:
            files.sort()
            for f in files:
                content = np.genfromtxt(path+f, delimiter=',')
                content = content[1:-1]
                content.sort()
                content = content[len(content)/2]
                report_file.write(annotations1[counter]+','+annotations2[counter]+','+repetition+',')
                report_file.write(str(content)+"\n")
                counter += 1
                if counter == len(annotations1):
                    counter = 0
                    repetition = 'b'

    with open(os.path.expanduser(radio_logs)+'/report'+str(rmax)+'.csv', 'rb') as myfile:
        part = MIMEApplication(myfile.read(), Name=f)
        part['Content-Disposition'] = 'attachment; filename=%s' % 'report'+str(rmax)+'.csv'
        msg.attach(part)
    msg.attach(MIMEText(body, 'plain'))

    server = smtplib.SMTP('smtp.gmail.com', 587)
    server.starttls()
    server.login(fromaddr, "reportt0d0cs")
    text = msg.as_string()
    server.sendmail(fromaddr, toaddr, text)
    server.quit()
    for f in files_to_move:
        #os.rename(os.path.expanduser(f),os.path.expanduser(radio_logs)+'/'+f.rpartition('/')[-1])
        print 'Moved',f
    sound_msg = Sound()
    sound_msg.value = 0
    sound_pub.publish(sound_msg)

def init():
    global sound_pub
    rospy.init_node('radio_report_generator')
    sound_pub = rospy.Publisher('mobile_base/commands/sound', Sound, queue_size=1)
    rospy.Subscriber("radio_generate_report", Int32, generateReport)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    init()