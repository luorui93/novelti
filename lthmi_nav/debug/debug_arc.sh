#!/bin/bash

# ===SETUP====

# sudo apt-get install wmctrl terminator

# Set default terminal:
# gsettings set org.gnome.desktop.default-applications.terminal exec 'terminator'

# GDB config file:
# $ cd ~/ws/src/lthmi_nav/lthmi_nav/tools
# $ ln -s .gdbinit ~

# If GDB reports smth like "ptrace: Operation not permitted", issue
# # echo 0 > /proc/sys/kernel/yama/ptrace_scope
# as root(!).
# To make it permanent in file /etc/sysctl.d/10-ptrace.conf set kernel.yama.ptrace_scope to 0 (zero)
# More on this here:
#   http://askubuntu.com/questions/41629/after-upgrade-gdb-wont-attach-to-process



d=`pwd`
prog=test_big_bang #test_best_pose_finder  #test_space_divider  #test_big_bang #test_big_bang #test_star_with_obstacles #arc
#prog=test_star_with_obstacles
source ~/ws/devel/setup.bash
roscd 
cd ../build/lthmi_nav/lthmi_nav
make
if [ $? == 0 ]; then
    NAME="my_main_terminal"; echo -en "\033]0;$NAME\a" #needed to keep the focus on the current terminal
    gnome-terminal --geometry=165x80+5000+40  -x sh -c "rosrun lthmi_nav $prog" & #pid_prog=$!
    sleep 1 
    wmctrl -a "my_main_terminal"
    #echo "Term PID=$pid_prog"
    sleep 1 
    pid_prog=`pidof $prog`
    pid_sh=`ps -o ppid= $pid_prog`
    pid_term=`ps -o ppid= $pid_sh`
    trap "kill $pid_term; cd $d" SIGINT SIGTERM
    echo "$pid_prog -> $pid_sh -> $pid_term"
    
    gdb -p `pidof $prog`
    kill $pid_term
fi

cd $d


# ===to debug macros:===
#cd ~/ws/src/lthmi_nav/lthmi_nav/src
#gcc -Wall -O2 -g fast_dist.cpp -o qq -std=c++0x -save-temps -I../include
#IGNORE errors after preprocessing. You just need a file with .ii extension
#from fast_dist.ii copy relevant piece
#replace { with {\n , } with }\n and ; with ;\n
#align using some automatic cpp alignment tool
#paste to original source file


# ===plan:===
#++obstacle at origin
#++order of walk, overshadowing of NBPs
#++corner point calculation
#*corner point initialization (angle) 
#*diagonal nbps
#*****new star points on the wave
#++star creation/iteration
#++*remove multiplication from non-zero radius boundary creation
#optimization
#stars on top of each other
#++novas under reg+nbp pairs
#rosify


#problems:
# 1)SOLVED nova star next to reg+nbp pairs: test_big_bang, test 5, point right above the top wall of inner П (under 1)
# 2)SOLVED non correct distances when a beam is expanding in an angled area limited at least on one side by another beam: area to the left of point from problem 1
# 3)SOLVED empty cones when two angled beams (beams adjucent to each other): test_big_bang, test 5, right under the bottom wall of inner П 
# 4)SOLVED  star initialization radius or init distance seems offest: same point as in problem 1
# 5)SOLVED nova stars which in the next cycle are overlapped by the next arc of the same epxanding star: same area as in problem 1
# 6)FROZEN point (-3,-3) on start boundary => octant=5 => corner point is at (x-1,y), should be at (x,y-1). See test_big_bang, test 9, upper room, when entering the door
#   #Will be solved by uncommenting line 405 in fast_dist.cpp (if (abs(x)==abs(y)) oct--;), once problem 7 is solved 
# 7) walk thorugh NBPs located on main diagonals
# 8)SOLVED empty corner in test_big_bang, test 8
# 9)SOLVED In test_big_bang, test 7, bottom right area, above room 2 has seemngly incorrectly placed nova stars
# 10)SOLVED Empty corner in bottom left in test 10
# 11)SOLVED Empty corner again, now in test 11
# 12) Another small corner in test 12

# to build rviz plugin:
# sudo apt-get install ros-hydro-cmake-modules
