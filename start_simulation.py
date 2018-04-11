#!/usr/bin/env python

import Tkinter as tk
import tkMessageBox
import ConfigParser
import thread
from Tkinter import *
from ttk import *
import Image, tkFileDialog
import numpy as np
import sys, time, os, glob, shutil, math, datetime

Default_Maps_and_InitPoses = { 'DIS_first_floor': ['11.8 2 270', '2 2 0'], 'DISB1': ['11.8 2 270', '2 2 0'],  \
  'DISlabs': ['9.7 7.3 90', '10.7 7.3 0'], 'dis_underground': ['16 42 270'], \
  'Rive1': ['10 18 0'], 'Rockin2014': ['-3.5 6.2 270', '0 0 0'],  
  'peccioli@Home' : ['10.3 5.0 135'], 'peccioli@Work' : ['1.25 0.25 90'],
  'explorer1' : ['25.0 20.0 90'], 'saman': ['-2 8 90'], 'samanx': ['11 3 90']  }

Robots = [ 'diago', 'sapienzbot', 'errazio' ]

Nrobots = ['1','2','3','4']

Localization = ['none', 'amcl', 'thin_localizer', 'srrg_localizer', 'gmapping', 'cg_mrslam']

Navigation = ['none','gradient_based_nav','move_base','spqrel_planner','explorer']

Demo = ['none', 'hri_pnp', 'rcathome','saman2']

LASER_TOPIC = 'scan' 
LASER_TOPIC_STAGE = 'scan' # for using laser_filters
LASER_FRAME = 'laser_frame'


def start_robot(robot_name, robot_id, nrobots, map_name, localization, navigation, INITPOSE_X,INITPOSE_Y,INITPOSE_TH_RAD, rviz, demo, joystick, keyboard):

  global run_gradient_based_navigation

  no_localizer=False
  run_amcl=False
  run_thin_localizer=False
  run_srrg_localizer=False
  run_gmapping=False
  run_cg_mrslam=False
  run_gradient_based_navigation=False
  run_move_base=False
  run_thin_planner=False
  run_spqrel_planner=False
  run_explorer=False

  if (localization=='amcl'):
    run_amcl=True
  elif (localization=='thin_localizer'):
    run_thin_localizer=True
  elif (localization=='srrg_localizer'):
    run_srrg_localizer=True
  elif (localization=='gmapping'):
    run_gmapping=True
  elif (localization=='cg_mrslam'):
    run_cg_mrslam=True
  else:
    no_localizer=True
    
  if (navigation=='gradient_based_nav'):
    run_gradient_based_navigation=True
  elif (navigation=='move_base'):
    run_move_base=True
  elif (navigation=='thin_planner'):
    run_thin_planner=True
  elif (navigation=='spqrel_planner'):
    run_spqrel_planner=True
  elif (navigation=='explorer'):
    run_explorer=True
    run_move_base=True


  amcl_str = ''
  if (run_amcl):
    amcl_str = "use_amcl:=true"

  thin_localizer_str = ''
  if (run_thin_localizer):
    thin_localizer_str = "use_thin_localizer:=true"

  srrg_localizer_str = ''
  if (run_srrg_localizer):
    srrg_localizer_str = "use_srrg_localizer:=true"

  no_localizer_str = ''
  if (no_localizer):
    no_localizer_str = "use_map_server:=false"

  gmapping_str = ''
  if (run_gmapping):
    gmapping_str = "use_gmapping:=true"
    no_localizer_str = "use_map_server:=false"

  cg_mrslam_str = ''
  if (run_cg_mrslam):
    cg_mrslam_str = "use_cg_mrslam:=true"
    no_localizer_str = "use_map_server:=false"


  gbn_str = ''
  if (run_gradient_based_navigation):
    gbn_str = "use_gradient_based_navigation:=true"

  move_base_str = ''
  if (run_move_base):
    move_base_str = "use_move_base:=true"

  thin_planner_str = ''
#  if (run_thin_planner):
#    thin_planner_str = "use_thin_planner:=true"

  spqrel_planner_str = ''
  if (run_spqrel_planner):
    spqrel_planner_str = "use_spqrel_planner:=true"

  explorer_str = ''
  if (run_explorer):
    explorer_str = "use_explorer:=true"


  if (localization!='none' or navigation!='none'):
    cmd = 'xterm -hold -e "roslaunch stage_environments stage_robot.launch robotname:=%s robotid:=%s nrobots:=%s mapname:=%s initial_pose_x:=%f initial_pose_y:=%f initial_pose_a:=%f laser_topic:=%s laser_frame:=%s %s %s %s %s %s %s %s %s %s %s %s" &' % (robot_name, robot_id, nrobots, map_name, INITPOSE_X, INITPOSE_Y, INITPOSE_TH_RAD, LASER_TOPIC, LASER_FRAME, amcl_str, thin_localizer_str, srrg_localizer_str, gmapping_str, cg_mrslam_str, gbn_str, move_base_str, thin_planner_str, spqrel_planner_str, explorer_str, no_localizer_str )
    print cmd
    os.system(cmd)
    os.system('sleep 10')

  # Start rviz
  if (rviz==1):
    cmd = 'xterm -hold -e "roslaunch tfpose_ros demo_video.launch robot_name:=%s ' % (robot_name)
    if (not run_gradient_based_navigation):
      cmd = cmd + ' cmd_vel_topic:=cmd_vel ' 
    cmd = cmd + '" &'
    print cmd
    os.system(cmd)
    os.system('sleep 1')
 #   cmd = 'xterm -e rosrun rviz rviz -d `rospack find stage_environments`/config/%s/rviz/%s.rviz &' % (robot_name,robot_name)
  #  os.system(cmd)

  # Start the demo
  if (demo=='hri_pnp'):
    cmd = 'xterm -e "roslaunch hri_pnp hri_pnpas.launch robot_name:=%s" &' % (robot_name)
    print cmd
    os.system(cmd)
    os.system('sleep 1')
  elif (demo=='rcathome'):
    cmd = 'xterm -e "roslaunch robocupathome_pnp rcathome_pnp.launch robot_name:=%s" &' % (robot_name)
    print cmd
    os.system(cmd)
    os.system('sleep 3')
    cmd = 'xterm -e "roslaunch rosplan_demos pnp_conditional.launch" &'
    print cmd
    os.system(cmd)
    os.system('sleep 1')
  elif (demo=='saman2'):
    cmd = 'xterm -e "roslaunch saman2_pnpas saman2_pnp.launch robot_name:=%s" &' % (robot_name)
    print cmd
    os.system(cmd)
    os.system('sleep 1')


  # Drive the robot with joystick or keyboard
  if (joystick==1) or (keyboard==1):
    cmd = 'xterm -e "roslaunch stage_environments joystick.launch robot_name:=%s ' % (robot_name)
    if (keyboard==1):
      cmd = cmd + ' use_keyboard:=true '
    if (not run_gradient_based_navigation):
      cmd = cmd + ' cmd_vel_topic:=cmd_vel ' 
    cmd = cmd + '" &'
    print cmd
    os.system(cmd)
    os.system('sleep 1')




def run_simulation(map_name,robot,nrobots, initpose,localization,navigation,demo,joystick,keyboard,rviz):

  global run_gradient_based_navigation

  print 'Run simulation: map:%s robot:%s nrobots:%s init:%s loc: %s nav: %s demo: %s joy:%s keyb:%s rviz:%s' % (map_name,robot,nrobots, initpose,localization,navigation,demo,joystick,keyboard,rviz)
  
  output_worldfile="../maps/AUTOGEN_%s_%s.world" % (map_name,robot)

  if (robot=='sapienzbot'):
    robottype='erratic'
  if (robot=='diago' ):
    robottype='segway'
  if (robot=='errazio' ):
    robottype='marrtino'

  pp = initpose.split()
  INITPOSE_X=float(pp[0])
  INITPOSE_Y=float(pp[1])
  INITPOSE_TH=float(pp[2]) # deg
  INITPOSE_TH_RAD=INITPOSE_TH*math.pi/180.0 # rad

  # Create the world
  cmd = './create_world.py %s %s %s %s %f %f %f %s' % (map_name,robottype,robot,nrobots, INITPOSE_X,INITPOSE_Y,INITPOSE_TH,output_worldfile)
  print cmd
  os.system(cmd)

  # Start the simulated environment

  cmd = 'xterm -hold -e roslaunch stage_environments stage_map.launch world_file:=%s robot_type:=%s laser_topic:=%s laser_frame:=%s &' % (output_worldfile, robottype, LASER_TOPIC_STAGE, LASER_FRAME)
  print cmd
  os.system(cmd)
  os.system('sleep 5')
  
  
  # Start the robot(s)

  for i in range(0,int(nrobots)):
    robot_name = robot+'_'+str(i)
    robot_id = i
    
    ix = INITPOSE_X + i%2;
    iy = INITPOSE_Y + i/2;

    start_robot(robot_name, robot_id, nrobots, map_name, localization, navigation, ix,INITPOSE_Y,INITPOSE_TH_RAD, rviz, demo, joystick, keyboard)


##############
#    GUI
##############

class DIP(tk.Frame):
    def __init__(self, parent):
        tk.Frame.__init__(self, parent) 
        self.parent = parent        
        self.first_map_selected = True
        self.initUI()
        


    def map_selected(self, *args):
	current_map = self.map_name_ddm.get()
	#print 'Current map: ',current_map
	self.initpose_list = Default_Maps_and_InitPoses[current_map]
	#print self.initpose_list
	if (self.first_map_selected):
	  self.first_map_selected = False
	  try:
	    last_init=self.oldConfigs["initpose"]
	  except:
	    last_init=self.initpose_list[0]
	else:
	    last_init=self.initpose_list[0]
	self.initpose_ddm.set(last_init)
	_row = 1
	_col = 2
	self.opt_poses = tk.OptionMenu(self, self.initpose_ddm, *self.initpose_list)
	self.opt_poses.grid(sticky=W, row=_row, column=_col, pady=4, padx=5)

	
    def initUI(self):
        self.loadOldConfig()

        self.parent.title("Simulation Launcher")
        self.style = Style()
        self.style.theme_use("alt")
        self.parent.resizable(width=FALSE, height=FALSE)
        self.pack(fill=BOTH, expand=1)
        
        #self.columnconfigure(1, weight=1)
        #self.columnconfigure(3, pad=7)
        #self.rowconfigure(3, weight=1)
        #self.rowconfigure(7, pad=7)
        
        self.map_name_ddm = StringVar(self)
        self.robot_ddm = StringVar(self)
        self.nrobots_ddm = StringVar(self)
        self.initpose_ddm = StringVar(self)

        Maps = sorted(Default_Maps_and_InitPoses.keys())

        _row = 0
        # Map
        lbl = Label(self, text="Map")
        lbl.grid(sticky=W, row=_row, column=0, pady=4, padx=5)                
        self.map_name_list = Maps
        self.map_name_ddm.trace("w", self.map_selected)
        try:
            lastmap_name=self.oldConfigs["map"]
        except:
            lastmap_name=self.map_name_list[0]
        self.map_name_ddm.set(lastmap_name)
        tk.OptionMenu(self, self.map_name_ddm, *self.map_name_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)


	_row = _row + 1
	# Robot type
        lbl = Label(self, text="Robot & Pose")
        lbl.grid(sticky=W, row = _row, column= 0, pady=4, padx=5)
        self.robot_list = Robots
        try:
            lastrobot=self.oldConfigs["robot"]
        except:
            lastrobot=self.robot_list[0]
        self.robot_ddm.set(lastrobot)
        tk.OptionMenu(self, self.robot_ddm, *self.robot_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)

	_row = _row + 1
	# N. Robots
        lbl = Label(self, text="Number of Robots")
        lbl.grid(sticky=W, row = _row, column= 0, pady=4, padx=5)
        self.nrobots_list = Nrobots
        try:
            lastnrobots=self.oldConfigs["nrobots"]
        except:
            lastnrobots=self.nrobots_list[0]
        self.nrobots_ddm.set(lastnrobots)
        tk.OptionMenu(self, self.nrobots_ddm, *self.nrobots_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)


	_row = _row + 1
	# Localization
        lbl = Label(self, text="Localization")
        lbl.grid(sticky=W, row = _row, column= 0, pady=4, padx=5)
        self.loc_list = Localization
        self.loc_ddm = StringVar(self)
        try:
            lastlocmode=self.oldConfigs["localization"]
        except:
            lastlocmode=self.loc_list[0]
        self.loc_ddm.set(lastlocmode)
        tk.OptionMenu(self, self.loc_ddm, *self.loc_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)


	_row = _row + 1
	# Navigation
        lbl = Label(self, text="Navigation")
        lbl.grid(sticky=W, row=_row, column=0, pady=4, padx=5)
        self.nav_list = Navigation
        self.nav_ddm = StringVar(self)
        try:
            lastterm=self.oldConfigs["navigation"]
        except:
            lastterm=self.nav_list[0]
        self.nav_ddm.set(lastterm)
        tk.OptionMenu(self, self.nav_ddm, *self.nav_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)


	_row = _row + 1
	# Demo
        lbl = Label(self, text="Demo")
        lbl.grid(sticky=W, row=_row, column=0, pady=4, padx=5)
        self.demo_list = Demo
        self.demo_ddm = StringVar(self)
        try:
            lastterm=self.oldConfigs["demo"]
        except:
            lastterm=self.demo_list[0]
        self.demo_ddm.set(lastterm)
        tk.OptionMenu(self, self.demo_ddm, *self.demo_list).grid(sticky=W, row=_row, column=1, pady=4, padx=5)


	_row = _row + 1
	# Joystick
	self.joystick_ddm = IntVar(self)
	try:
	  last_joystick=self.oldConfigs["joystick"]
	except:
	  last_joystick=0
	self.joystick_ddm.set(last_joystick)
	self.Chk_joystick = tk.Checkbutton(self, text='Joystick', variable=self.joystick_ddm)      
	self.Chk_joystick.grid(sticky=W, row=_row, column=0, pady=10, padx=5)

	# Keyboard
	self.keyboard_ddm = IntVar(self)
	try:
	  last_keyboard=self.oldConfigs["keyboard"]
	except:
	  last_keyboard=0
	self.keyboard_ddm.set(last_keyboard)
	self.Chk_keyboard = tk.Checkbutton(self, text='Keyboard', variable=self.keyboard_ddm)      
	self.Chk_keyboard.grid(sticky=W, row=_row, column=1, pady=10, padx=5)

	# Rviz
	self.rviz_ddm = IntVar(self)
	try:
	  last_rviz=self.oldConfigs["rviz"]
	except:
	  last_rviz=0
	self.rviz_ddm.set(last_rviz)
	self.Chk_rviz = tk.Checkbutton(self, text='Gestures', variable=self.rviz_ddm)      ##text='Rviz'
	self.Chk_rviz.grid(sticky=W, row=_row, column=2, pady=10, padx=5)
      
      
	_row = _row + 1
	# Buttons
        launchButton = Button(self, text="Start",command=self.launch_script)
        launchButton.grid(sticky=W, row=_row, column=0, pady=4, padx=5)
        
        launchButton = Button(self, text="Quit",command=self.kill_demo)
        launchButton.grid(sticky=W, row=_row, column=1, pady=4, padx=5)
        
    
    def launch_script(self):
        self.saveConfigFile();
        thread.start_new_thread( run_simulation, (self.map_name_ddm.get(), self.robot_ddm.get(), self.nrobots_ddm.get(), self.initpose_ddm.get(), self.loc_ddm.get(), self.nav_ddm.get(), self.demo_ddm.get(), self.joystick_ddm.get(), self.keyboard_ddm.get(), self.rviz_ddm.get()) )

    
    def quit(self):
      self.saveConfigFile()
      #self.parent.destroy()
      
    def kill_demo(self):
      os.system("./quit.sh &")
      
      
    def saveConfigFile(self):
      f = open('lastConfigUsed', 'w')
      f.write("[Config]\n")
      f.write("map: %s\n"%self.map_name_ddm.get())
      f.write("robot: %s\n"%self.robot_ddm.get())
      f.write("nrobots: %s\n"%self.nrobots_ddm.get())
      f.write("initpose: %s\n"%self.initpose_ddm.get())
      f.write("localization: %s\n"%self.loc_ddm.get())
      f.write("navigation: %s\n"%self.nav_ddm.get())
      f.write("demo: %s\n"%self.demo_ddm.get())
      f.write("joystick: %s\n"%self.joystick_ddm.get())
      f.write("keyboard: %s\n"%self.keyboard_ddm.get())
      f.write("rviz: %s\n"%self.rviz_ddm.get())
      f.close()


    def loadOldConfig(self):
      try:
        self.oldConfigs = {}
        self.Config = ConfigParser.ConfigParser()
        self.Config.read("lastConfigUsed")
        for option in self.Config.options("Config"):
          self.oldConfigs[option] = self.Config.get("Config", option)
      except:
        print "Could not load config file"


    


def main():

  if (len(sys.argv)==1):
    root = tk.Tk()
    f = DIP(root)
    root.geometry("360x320+0+0")
    root.mainloop()
    f.quit()

  elif (len(sys.argv)>=10):
    map_name = sys.argv[1]
    robot_name = sys.argv[2]
    nrobots = sys.argv[3]
    init_pose = sys.argv[4]+' '+sys.argv[5]+' '+sys.argv[6]
    localization = sys.argv[7]
    navigation = sys.argv[8]
    demo = sys.argv[9]
    i=10
    joystick=0
    keyboard=0
    rviz=0
    while (i<len(sys.argv)):
      if (sys.argv[i]=='joystick'):
        joystick=1
      if (sys.argv[i]=='keyboard'):
        keyboard=1
      if (sys.argv[i]=='rviz'):
        rviz=1
      i=i+1
    print "Starting the simulation..."
    run_simulation(map_name, robot_name, nrobots, init_pose, localization, navigation, demo, joystick, keyboard, rviz)

    print "Use the following command to quit the simulation"
    print "  `rospack find stage_environments`/scripts/quit.sh"
    
  else:
    print "Use: ",sys.argv[0]
    print " or  ",sys.argv[0],' <map_name> <robot_name> <num robots> <init_pose> <localization> <navigation> <demo> [joystick] [keyboard] [rviz]'
  


if __name__ == '__main__':
    main()

