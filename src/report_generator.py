#!/usr/bin/env python
import os
import rospy
import rospkg
import smtplib
from datetime import datetime
from std_msgs.msg import Int32
from kobuki_msgs.msg import Sound
from email.mime.text import MIMEText
from email.MIMEMultipart import MIMEMultipart
from email.mime.application import MIMEApplication

sound_pub = None

def generateReport(msg):
    global sound_pub
    files_to_remove = []
    fromaddr = "roboskelncsr@gmail.com"
    toaddr = ["gstavrinos@iit.demokritos.gr", "gs.juggle@gmail.com"]
    subject = "Medical Report as of "+datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    msg = MIMEMultipart()
    msg['From'] = fromaddr
    msg['To'] = ", ".join(toaddr)
    msg['BCC'] = "stavrinosgeo@gmail.com"
    msg['Subject'] = subject

    body = subject+"\n\n\n"

    rospack = rospkg.RosPack()

    path = rospack.get_path('motion_analysis_wrapper')+'/logs/'
    files = []
    found_file = False
    for i in os.listdir(path):
        if os.path.isfile(os.path.join(path,i)) and 'official_log_bed_'+datetime.today().strftime("%d-%m-%Y") in i:
            found_file = True
            files.append(i)
            files_to_remove.append(path+i)

    if found_file:
        body += "Lying-Standing file attached.\n"
        body += "---\n\n\n"
        for f in files:
            with open(path+f, 'rb') as myfile:
                part = MIMEApplication(myfile.read(), Name=f)
                part['Content-Disposition'] = 'attachment; filename="%s"' % f
                msg.attach(part)
    files = []
    found_file = False

    for i in os.listdir(path):
        if os.path.isfile(os.path.join(path,i)) and 'official_log_pill_'+datetime.today().strftime("%d-%m-%Y") in i:
            found_file = True
            files.append(i)
            files_to_remove.append(path+i)

    if found_file:
        body += "Pill intake file attached.\n"
        body += "---\n\n\n"
        for f in files:
            with open(path+f, 'rb') as myfile:
                part = MIMEApplication(myfile.read(), Name=f)
                part['Content-Disposition'] = 'attachment; filename="%s"' % f
                msg.attach(part)
    files = []
    found_file = False


    path = rospack.get_path('hpr_wrapper')+'/logs/'
    for i in os.listdir(path):
        if os.path.isfile(os.path.join(path,i)) and 'official_log_walk_'+datetime.today().strftime("%d-%m-%Y") in i:
            found_file = True
            files.append(i)
            files_to_remove.append(path+i)

    if found_file:
        body += "4 meters walk file attached.\n"
        body += "---\n\n\n"
        for f in files:
            with open(path+f, 'rb') as myfile:
                part = MIMEApplication(myfile.read(), Name=f)
                part['Content-Disposition'] = 'attachment; filename="%s"' % f
                msg.attach(part)

    files = []
    found_file = False

    path = rospack.get_path('ros_visual_wrapper')+'/logs/'
    for i in os.listdir(path):
        if os.path.isfile(os.path.join(path,i)) and 'official_log_chair_'+datetime.today().strftime("%d-%m-%Y") in i:
            found_file = True
            files.append(i)
            files_to_remove.append(path+i)

    if found_file:
        body += "Sitting-Standing file attached."
        body += "---\n\n\n"
        for f in files:
            with open(path+f, 'rb') as myfile:
                part = MIMEApplication(myfile.read(), Name=f)
                part['Content-Disposition'] = 'attachment; filename="%s"' % f
                msg.attach(part)

    '''
    files = []
    found_file = False

    body += "Gym info:\n"

    path = rospack.get_path('radio_node_manager')+'/logs/'
    for i in os.listdir(path):
        if os.path.isfile(os.path.join(path,i)) and 'official_log_'+datetime.today().strftime("%d-%m-%Y") in i:
            found_file = True
            files.append(i)
            files_to_remove.append(path+i)

    if found_file:
            for f in files:
                with open(path+f, 'r') as myfile:
                        body += myfile.read()

    body += "---\n\n\n"
    '''

    msg.attach(MIMEText(body, 'plain'))

    server = smtplib.SMTP('smtp.gmail.com', 587)
    server.starttls()
    server.login(fromaddr, "reportt0d0cs")
    text = msg.as_string()
    server.sendmail(fromaddr, toaddr, text)
    server.quit()
    #for f in files_to_remove:
    #    os.remove(f)
    #    print 'Deleted',f
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