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

echo_and_run cd "/home/linzp/catkin_op/src/navigation/base_local_planner"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/linzp/catkin_op/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/linzp/catkin_op/install/lib/python3/dist-packages:/home/linzp/catkin_op/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/linzp/catkin_op/build" \
    "/usr/bin/python3" \
    "/home/linzp/catkin_op/src/navigation/base_local_planner/setup.py" \
     \
    build --build-base "/home/linzp/catkin_op/build/navigation/base_local_planner" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/linzp/catkin_op/install" --install-scripts="/home/linzp/catkin_op/install/bin"
