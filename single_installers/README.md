# vagrant-rss/single_installers
installation shell scripts for Vagrantfile(s) for setting up a VM for the RSE implementation

Additional Notes (umask 027)
============================

If you are running these scripts outside of a Vagrantfile setup, and the `umask` value for your (Ubuntu) system is 027 (instead of 022), then you may need to run the following command post-installation for some of the compiled (python) libraries to be usable / findable by non-root users:
```
    sudo chmod -R o+rx /usr/local/lib/python2.7/dist-packages
```

If the `umask` value for your (Ubuntu) system is 027 (instead of 022), you may also need to install OMPL quasi-manually. Try running the `install_ompl.sh` script first, to get all the dependencies installed and the files downloaded, then try:
```
    sudo su
    cd /root/initdeps/omplapp-1.1.1-Source
    gedit CMakeLists.txt
```
and then add the following lines at line 9 (below `project(omplapp CXX C)`):
```
# Check the architecture and set CMAKE_LIBRARY_ARCHITECTURE accordingly
#if(UNIX)
#  if(CMAKE_SIZEOF_VOID_P EQUAL 8)
#    set(CMAKE_LIBRARY_ARCHITECTURE "x86_64-linux-gnu")
#  endif()
#endif()
set(CMAKE_LIBRARY_ARCHITECTURE "x86_64-linux-gnu")
set(BOOST_INCLUDEDIR /usr/include)
set(BOOST_LIBRARYDIR /usr/lib/x86_64-linux-gnu)
```
Also add the above lines to ompl/CMakeLists.txt at line 6 (below `project(omplapp CXX C)`), using:
```
    gedit ompl/CMakeLists.txt
```

Once you've added those lines, too, run the following commands:
```
    cd build/Release
    cmake ../..
    make installpyplusplus
    sudo chmod -R o+rx /usr/local/lib/python2.7/dist-packages
    cmake .
    make update_bindings
    make
    make test
    make doc
    sudo make install
```

If you run into issues at the `make update_bindings` step (e.g. it errors and says that it isn't a valid option/found), then the problem may be the import order of your python path (e.g., the version of `pygccxml` being used). Try the following:
```
    cd /root/initdeps/omplapp-1.1.1-Source/py-bindings
    ./generate-bindings.py
```
If you get the following:
```
File "/usr/local/lib/python2.7/dist-packages/pyplusplus/code_creators/declaration_based.py", line 8, in <module>
    from pygccxml.utils import utils
ImportError: cannot import name utils
```
then the issue is likely that a different (older) version of the `pygccxml` library is being used from another directory (e.g., `/usr/lib/pymodules/python2.7`).
You can check the path order via:
```
    python -c "import sys; print '\n'.join(sys.path)"
```
You can likely fix this by opening the `easy-install.pth` file that loads into python before all-else:
```
    sudo gedit /usr/local/lib/python2.7/dist-packages/easy-install.pth
```
and adding the following line at the top of the file (before the `/usr/lib/pymodules/python2.7` entry):
```
    /usr/local/lib/python2.7/dist-packages
```
This will force `/usr/local/lib/python2.7/dist-packages/pygccxml` (among other things in that directory) to be imported first.
You can then test to make sure that this worked by trying:
```
    python -c "from pygccxml.utils import utils"
```
If you don't get an import error, then finish compiling+installing OMPL by running the following lines:
```
    cd /root/initdeps/omplapp-1.1.1-Source/build/Release
    cmake ../..
    cmake .
    make update_bindings
    make
    make test
    make doc
    sudo make install
```

License
=======

This is free software released under the terms of the BSD 3-Clause License. There is no warranty; not even for merchantability or fitness for a particular purpose. Consult LICENSE for copying conditions.

When code is modified or re-distributed, the LICENSE file should accompany the code or any subset of it, however small. As an alternative, the LICENSE text can be copied within files, if so desired.

Contact
=======

If you have any questions regarding the contents of this repository, please email Catharine McGhan at <cat.mcghan@uc.edu>.

-EOF-
