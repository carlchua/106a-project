#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/mnt/c/final_project/106a-project/a1_soccer_vision/src/ros_numpy"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/mnt/c/final_project/106a-project/a1_soccer_vision/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/mnt/c/final_project/106a-project/a1_soccer_vision/install/lib/python2.7/dist-packages:/mnt/c/final_project/106a-project/a1_soccer_vision/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/mnt/c/final_project/106a-project/a1_soccer_vision/build" \
    "/usr/bin/python2" \
    "/mnt/c/final_project/106a-project/a1_soccer_vision/src/ros_numpy/setup.py" \
     \
    build --build-base "/mnt/c/final_project/106a-project/a1_soccer_vision/build/ros_numpy" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/mnt/c/final_project/106a-project/a1_soccer_vision/install" --install-scripts="/mnt/c/final_project/106a-project/a1_soccer_vision/install/bin"
