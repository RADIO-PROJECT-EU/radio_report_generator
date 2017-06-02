#!/usr/bin/env python
import os
import rospy
import rospkg
import smtplib
import numpy as np
from datetime import datetime
from datetime import timedelta
from email.mime.text import MIMEText
from email.MIMEMultipart import MIMEMultipart
from email.mime.application import MIMEApplication
from radio_services.srv import InstructionWithAnswer


def generateReport(msg):
    files_to_move = []
    fromaddr = "roboskelncsr@gmail.com"
    toaddr = ["mratera@fhag.es", "sarino@fhag.es"]
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
        annotations = ['V4', 'V224', 'V38', 'V57']
        if found_file:
            files.sort()
            for f in files:
                content = np.genfromtxt(path+f, delimiter=',')
                if len(content) > 0:
                    #content = content[1]
                    report_file.write(annotations[counter]+','+repetition+','+str(content[0])+'\n')
                else:
                    report_file.write(annotations[counter]+','+repetition+', , \n')
                counter += 1
                if counter == len(annotations):
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
        annotations = ['V10', 'V210', 'V22', 'V44', 'V63']
        if found_file:
            files.sort()
            for f in files:
                content = np.genfromtxt(path+f, delimiter=',',dtype=None)
                if len(content) > 1:
                    content = content[1]
                    report_file.write(annotations[counter]+','+repetition+','+str(content[0])+','+str(content[1])+'\n')
                else:
                    report_file.write(annotations[counter]+','+repetition+', , \n')
                counter += 1
                if counter == len(annotations):
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
        annotations = ['V12', 'V212', 'V24', 'V30', 'V50', 'V65', 'V69', 'V75']
        if found_file:
            files.sort()
            for f in files:
                content = []
                filtered = []
                title = []
                content = np.genfromtxt(path+f, delimiter=',')
                if len(content) > 1:
                    content = content[1:]
                    filtered = [x for x in content if x[2] > 2]
                    if len(filtered) < 1:
                        filtered = [x for x in content]
                    filtered.sort(key=lambda x: x[2])
                    filtered = filtered[len(filtered)/2]
                    report_file.write(annotations[counter]+','+repetition+',')
                    report_file.write(str(filtered[2])+'\n')
                else:
                    report_file.write(annotations[counter]+','+repetition+', \n')
                counter += 1
                if counter == len(annotations):
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
        annotations = ['V8', 'V228', 'V28', 'V34', 'V48', 'V61']
        if found_file:
            files.sort()
            for f in files:
                content = np.genfromtxt(path+f, delimiter=',')
                if len(content) > 1:
                    content = content[1:]
                    content.sort()
                    content = content[len(content)/2]
                    report_file.write(annotations[counter]+','+repetition+',')
                    report_file.write(str(content)+"\n")
                else:
                    report_file.write(annotations[counter]+','+repetition+', \n')
                counter += 1
                if counter == len(annotations):
                    counter = 0
                    repetition = 'b'

        files = []
        found_file = False

        path = rospack.get_path('snc_events_wrapper')+'/logs/'
        for i in os.listdir(path):
            for d in range(2, -1, -1):
                if os.path.isfile(os.path.join(path,i)) and 'official_log_tv_'+(datetime.today()-timedelta(d)).strftime("%d-%m-%Y") in i:
                    found_file = True
                    files.append(i)
                    files_to_move.append(path+i)

        counter = 0
        repetition = 'a'
        annotations = ['V39']
        if found_file:
            files.sort()
            for f in files:
                content = np.genfromtxt(path+f, delimiter=',', dtype=None)
                content = content[1]
                report_file.write(annotations[counter]+','+repetition+',')
                report_file.write(str(content)+"\n")
                counter += 1
                if counter == len(annotations):
                    counter = 0
                    repetition = 'b'

        files = []
        found_file = False

        path = rospack.get_path('snc_events_wrapper')+'/logs/'
        for i in os.listdir(path):
            for d in range(2, -1, -1):
                if os.path.isfile(os.path.join(path,i)) and 'official_log_presence_'+(datetime.today()-timedelta(d)).strftime("%d-%m-%Y") in i:
                    found_file = True
                    files.append(i)
                    files_to_move.append(path+i)

        counter = 0
        repetition = 'a'
        annotations = ['V42']
        if found_file:
            files.sort()
            for f in files:
                content = np.genfromtxt(path+f, delimiter=',', dtype=None)
                content = content[1]
                report_file.write(annotations[counter]+','+repetition+',')
                report_file.write(str(content)+"\n")
                counter += 1
                if counter == len(annotations):
                    counter = 0
                    repetition = 'b'

        files = []
        found_file = False

        path = rospack.get_path('snc_events_wrapper')+'/logs/'
        for i in os.listdir(path):
            for d in range(2, -1, -1):
                if os.path.isfile(os.path.join(path,i)) and 'official_log_cooking_'+(datetime.today()-timedelta(d)).strftime("%d-%m-%Y") in i:
                    found_file = True
                    files.append(i)
                    files_to_move.append(path+i)

        counter = 0
        repetition = 'a'
        annotations = ['V52', 'V67']
        if found_file:
            files.sort()
            for f in files:
                content = np.genfromtxt(path+f, delimiter=',', dtype=None)
                content = content[1]
                report_file.write(annotations[counter]+','+repetition+',')
                report_file.write(str(content)+"\n")
                counter += 1
                if counter == len(annotations):
                    counter = 0
                    repetition = 'b'

        files = []
        found_file = False

        path = rospack.get_path('snc_events_wrapper')+'/logs/'
        for i in os.listdir(path):
            for d in range(2, -1, -1):
                if os.path.isfile(os.path.join(path,i)) and 'official_log_coming_'+(datetime.today()-timedelta(d)).strftime("%d-%m-%Y") in i:
                    found_file = True
                    files.append(i)
                    files_to_move.append(path+i)

        counter = 0
        repetition = 'a'
        annotations = ['V70']
        if found_file:
            files.sort()
            for f in files:
                content = np.genfromtxt(path+f, delimiter=',', dtype=None)
                content = content[1]
                report_file.write(annotations[counter]+','+repetition+',')
                report_file.write(str(content)+"\n")
                counter += 1
                if counter == len(annotations):
                    counter = 0
                    repetition = 'b'

        files = []
        found_file = False

        path = rospack.get_path('snc_events_wrapper')+'/logs/'
        for i in os.listdir(path):
            for d in range(2, -1, -1):
                if os.path.isfile(os.path.join(path,i)) and 'official_log_going_'+(datetime.today()-timedelta(d)).strftime("%d-%m-%Y") in i:
                    found_file = True
                    files.append(i)
                    files_to_move.append(path+i)

        counter = 0
        repetition = 'a'
        annotations = ['V71']
        if found_file:
            files.sort()
            for f in files:
                content = np.genfromtxt(path+f, delimiter=',', dtype=None)
                content = content[1]
                report_file.write(annotations[counter]+','+repetition+',')
                report_file.write(str(content)+"\n")
                counter += 1
                if counter == len(annotations):
                    counter = 0
                    repetition = 'b'

    with open(os.path.expanduser(radio_logs)+'/report'+str(rmax)+'.csv', 'rb') as myfile:
        part = MIMEApplication(myfile.read(), Name='report'+str(rmax)+'.csv')
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
        os.rename(os.path.expanduser(f),os.path.expanduser(radio_logs)+'/'+f.rpartition('/')[-1])
        print 'Moved',f
    return True

def init():
    rospy.init_node('radio_report_generator')
    rospy.Service('radio_report_generation', InstructionWithAnswer, generateReport)
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    init()
