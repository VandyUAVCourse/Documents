This is the README.txt of the openquadrotor.org system.
openquadrotor.org Copyright (c) 2009 Slawomir Grzonka, Giorgio Grisetti, and Wolfram Burgard (University of Freiburg, Germany)

=============
L I C E N S E
=============
This software is licenced under the Common Creative License,
Attribution-NonCommercial-ShareAlike 3.0

You are free:
- to Share - to copy, distribute and transmit the work
- to Remix - to adapt the work

Under the following conditions:
- Attribution. You must attribute the work in the manner specified
  by the author or licensor (but not in any way that suggests that
  they endorse you or your use of the work).
- Noncommercial. You may not use this work for commercial purposes.
- Share Alike. If you alter, transform, or build upon this work,
  you may distribute the resulting work only under the same or
  similar license to this one.

Any of the above conditions can be waived if you get permission
from the copyright holder.  Nothing in this license impairs or
restricts the author's moral rights.

this software is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied
warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
-------------------------------------------------------------------

Please also note, that this software uses some third-party libraries like
IPC, libserial , qglviewer, qt etc...
These might have different licences and ARE their licence is NOT modified.
 This is noted in their own descriptions and/or header files.
The above license only corresponds to OUR OWN parts of the
openquadrotor-project.

==============
How to compile
==============
Go to your openquadrotor.org diretory (we will call it $QUAD)
cd $QUAD
./configure
. setlibpath
make

Note that some test-programs need qt3 and libqglviewer. If it wont complile, 
simply remove the "*_test" in the Makefile under APPS of the corresponding director.
For debugging and visualization, plz install qglviewer. Note that you must eventually adjust the 
libpath to qglviewer manually in the corresponding makefiles.

=====================
What should I do now?
======================

Eventually you have to cross-compile the code for your specific system (e.g. the linux version on the gumstix).
In this case, change the symlink in build_tools from Makefile.generic-shared-object->Makefile.generic-shared-object-test to
Makefile.generic-shared-object->Makefile.generic-shared-object-notest
if you want to run something do it the following way:

1) start central
2) start configSender -f <configfile> -g <groupname> (e.g. configSender -f ../config/quadcopter.conf -g quadcopter)
3) start laser + imu on your robot
4) now you can already to scanmatching via odometry_laser_imu_sender 
note that this program in general takes some arguments, type -h for help. If this programs gives a segfault, then its highly probable that the worldsize was too small.
5) log your stuff via logger [-g for generating a filename for you], please create a directory called "logs" in your bin director
6) if you have qglviewer, view your map by odometry_laser_imu_sender_test (over ipc)

please note that all of this is a preliminary version and neither 100% stable nor complete :) I am still working on improving the code etc.


