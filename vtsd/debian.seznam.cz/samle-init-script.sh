#!/bin/sh
### BEGIN INIT INFO
# Provides:          vtsd
# Required-Start:    $network $local_fs
# Required-Stop:
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: VTS daemon sample init script
# Description:       VTS daemon sample init script
### END INIT INFO

# Author: Vaclav Blazek <vaclav.blazek@citationtech.net>

# PATH should only include /usr/* if it runs after the mountnfs.sh script
PATH=/sbin:/usr/sbin:/bin:/usr/bin:/opt/vts-seznamcz/bin
DESC=vtsd                # Introduce a short description here
NAME=vtsd                # Introduce the short server's name here
DAEMON=/opt/vts-seznamcz/bin/vtsd   # Introduce the server's location here
DAEMON_ARGS="-f ./opt/vts-seznamcz/etc/vtsd.conf --daemonize" # Arguments to run the daemon with
PIDFILE=/var/run/$NAME.pid
CTRL=/var/run/$NAME.ctrl
SCRIPTNAME=/etc/init.d/$NAME
STOPTIMEOUT=5 # number of seconds to try to stop process before giving up

# Exit if the package is not installed
[ -x $DAEMON ] || exit 0

# Read configuration variable file if it is present
[ -r /etc/default/$NAME ] && . /etc/default/$NAME

# Load the VERBOSE setting and other rcS variables
. /lib/init/vars.sh

# Define LSB log_* functions.
# Depend on lsb-base (>= 3.0-6) to ensure that this file is present.
. /lib/lsb/init-functions

#
# Function that starts the daemon/service
#
do_start()
{
	# Return
	#   0 if daemon has been started
	#   1 if daemon was already running
	#   2 if daemon could not be started
	$DAEMON --pidfile $PIDFILE $DAEMON_ARGS --signal status
	RETVAL="$?"
	[ "$RETVAL" = 0 ] && return 0

	$DAEMON --pidfile $PIDFILE --ctrl $CTRL $DAEMON_ARGS  \
		|| return 2
	# Add code here, if necessary, that waits for the process to be ready
	# to handle requests from services started subsequently which depend
	# on this one.  As a last resort, sleep for some time.
}

#
# Function that stops the daemon/service
#
do_stop()
{
	# Return
	#   0 if daemon has been stopped
	#   1 if daemon was already stopped
	#   2 if daemon could not be stopped
	#   other if a failure occurred
	$DAEMON --pidfile $PIDFILE $DAEMON_ARGS --signal stop/${STOPTIMEOUT}
	RETVAL="$?"
	[ "$RETVAL" = 0 ] && return 0
	[ "$RETVAL" = 1 ] && return 1
	rm -f $PIDFILE
	return "$RETVAL"
}

#
# Function that requests statistics of the daemon/service
#
do_stat()
{
	# Return
	#   0 if daemon is running
	#   1 if daemon is not running
	#   other if a failure occurred
	$DAEMON --pidfile $PIDFILE $DAEMON_ARGS --signal stat
	RETVAL="$?"
	return "$RETVAL"
}

#
# Function that sends a SIGHUP to the daemon/service
#
do_reload() {
	#
	# If the daemon can reload its configuration without
	# restarting (for example, when it is sent a SIGHUP),
	# then implement that here.
	#
	$DAEMON --pidfile $PIDFILE $DAEMON_ARGS --signal logrotate
	return 0
}

do_monitor() {
    echo "monitor" | socat -T2 - UNIX-CONNECT:$CTRL
}

case "$1" in
  start)
    [ "$VERBOSE" != no ] && log_daemon_msg "Starting $DESC " "$NAME"
    do_start
    case "$?" in
		0|1) [ "$VERBOSE" != no ] && log_end_msg 0 ;;
		2) [ "$VERBOSE" != no ] && log_end_msg 1 ;;
	esac
  ;;

  stop)
	[ "$VERBOSE" != no ] && log_daemon_msg "Stopping $DESC" "$NAME"
	do_stop
	case "$?" in
		0|1) [ "$VERBOSE" != no ] && log_end_msg 0 ;;
		2) [ "$VERBOSE" != no ] && log_end_msg 1 ;;
	esac
	;;

  stat)
	[ "$VERBOSE" != no ] && log_daemon_msg "Requesting statistics $DESC" "$NAME"
	do_stat
	;;

  status)
       status_of_proc -p "$PIDFILE" "$DAEMON" "$NAME" && exit 0 || exit $?
       ;;
  reload|force-reload)
	#
	# If do_reload() is not implemented then leave this commented out
	# and leave 'force-reload' as an alias for 'restart'.
	#
	log_daemon_msg "Reloading $DESC" "$NAME"
	do_reload
	log_end_msg $?
	;;
  restart)
	#
	# If the "reload" option is implemented then remove the
	# 'force-reload' alias
	#
	log_daemon_msg "Restarting $DESC" "$NAME"
	do_stop
	case "$?" in
	  0|1)
		do_start
		case "$?" in
			0) log_end_msg 0 ;;
			1) log_end_msg 1 ;; # Old process is still running
			*) log_end_msg 1 ;; # Failed to start
		esac
		;;
	  *)
	  	# Failed to stop
		log_end_msg 1
		;;
	esac
	;;
  monitor)
    do_monitor
    ;;
  *)
	echo "Usage: $SCRIPTNAME {start|stop|status|restart|reload|force-reload|monitor}" >&2
	exit 3
	;;
esac

:
