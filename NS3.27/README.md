## Quick Start

#### Dependencies Installation

```

sudo apt-get install gcc-5 g++-5 python python3 python3-dev

sudo apt-get install python3-setuptools git mercurial

sudo apt-get install qt5-default mercurial

sudo apt-get install gir1.2-goocanvas-2.0 python-gi python-gi-cairo python-pygraphviz python3-gi python3-gi-cairo python3-pygraphviz gir1.2-gtk-3.0 ipython ipython3  

sudo apt-get install openmpi-bin openmpi-common openmpi-doc libopenmpi-dev

sudo apt-get install autoconf cvs bzr unrar

sudo apt-get install gdb valgrind 

sudo apt-get install uncrustify

sudo apt-get install doxygen graphviz imagemagick

sudo apt-get install texlive texlive-extra-utils texlive-latex-extra texlive-font-utils dvipng latexmk

sudo apt-get install python3-sphinx dia 

sudo apt-get install gsl-bin libgsl-dev libgsl23 libgslcblas0

sudo apt-get install tcpdump

sudo apt-get install sqlite sqlite3 libsqlite3-dev

sudo apt-get install libxml2 libxml2-dev

sudo apt-get install cmake libc6-dev libc6-dev-i386 libclang-6.0-dev llvm-6.0-dev automake 

sudo apt-get install libgtk2.0-0 libgtk2.0-dev

sudo apt-get install vtun lxc uml-utilities

sudo apt-get install libboost-signals-dev libboost-filesystem-dev

sudo apt-get install fftw3 fftw3-dev
```

#### Build

CC='gcc-5' CXX='g++-5' ./waf configure

./waf

#### Simple run of ICC

./waf --run "periodcCCEvaDumbbell"

Outputs including RTT, RTT_standing (Stand), Queuing delay (Qd), Time stamp (now), cwnd, Target rate (Trate) and so on, which are represented as bellow, indicates that the simulation is run successfully.

`0x[...] rttI [...] Stand [...] Old [...] RTTmin [...] Qd [...] now [...] ......`




-----------------------------------------------------

    The Network Simulator, Version 3
    --------------------------------

Table of Contents:
------------------

1) An overview
2) Building ns-3
3) Running ns-3
4) Getting access to the ns-3 documentation
5) Working with the development version of ns-3

Note:  Much more substantial information about ns-3 can be found at
http://www.nsnam.org

1) An Open Source project
-------------------------

ns-3 is a free open source project aiming to build a discrete-event
network simulator targeted for simulation research and education.   
This is a collaborative project; we hope that
the missing pieces of the models we have not yet implemented
will be contributed by the community in an open collaboration
process.

The process of contributing to the ns-3 project varies with
the people involved, the amount of time they can invest
and the type of model they want to work on, but the current
process that the project tries to follow is described here:
http://www.nsnam.org/developers/contributing-code/

This README excerpts some details from a more extensive
tutorial that is maintained at:
http://www.nsnam.org/documentation/latest/

2) Building ns-3
----------------

The code for the framework and the default models provided
by ns-3 is built as a set of libraries. User simulations
are expected to be written as simple programs that make
use of these ns-3 libraries.

To build the set of default libraries and the example
programs included in this package, you need to use the
tool 'waf'. Detailed information on how to use waf is
included in the file doc/build.txt

However, the real quick and dirty way to get started is to
type the command
  ./waf configure --enable-examples
followed by
  ./waf 
in the directory which contains
this README file. The files built will be copied in the
build/ directory.

The current codebase is expected to build and run on the
set of platforms listed in the RELEASE_NOTES file.

Other platforms may or may not work: we welcome patches to 
improve the portability of the code to these other platforms. 

3) Running ns-3
---------------

On recent Linux systems, once you have built ns-3 (with examples
enabled), it should be easy to run the sample programs with the
following command, such as:

  ./waf --run simple-global-routing

That program should generate a simple-global-routing.tr text 
trace file and a set of simple-global-routing-xx-xx.pcap binary
pcap trace files, which can be read by tcpdump -tt -r filename.pcap
The program source can be found in the examples/routing directory.

4) Getting access to the ns-3 documentation
-------------------------------------------

Once you have verified that your build of ns-3 works by running
the simple-point-to-point example as outlined in 3) above, it is
quite likely that you will want to get started on reading
some ns-3 documentation. 

All of that documentation should always be available from
the ns-3 website: http:://www.nsnam.org/documentation/.

This documentation includes:

  - a tutorial

  - a reference manual

  - models in the ns-3 model library

  - a wiki for user-contributed tips: http://www.nsnam.org/wiki/

  - API documentation generated using doxygen: this is
    a reference manual, most likely not very well suited 
    as introductory text:
    http://www.nsnam.org/doxygen/index.html

5) Working with the development version of ns-3
-----------------------------------------------

If you want to download and use the development version 
of ns-3, you need to use the tool 'mercurial'. A quick and
dirty cheat sheet is included in doc/mercurial.txt but
reading through the mercurial tutorials included on the
mercurial website is usually a good idea if you are not
familiar with it.

If you have successfully installed mercurial, you can get
a copy of the development version with the following command:
"hg clone http://code.nsnam.org/ns-3-dev"
