#!/bin/sh
#
# multi.sh - run one raytracer arrangement interactively.
#
# See ./testall.sh to run every arrangement and check the output against the
# reference images.
#
if [ $# -ne 1 -a $# -ne 2 ];
then
  echo "usage: multi.sh [-sl] <#processors>"
  exit 1
fi

SHLINK="-sl"
while :
do
  case "$1" in
    -sl)
      SHLINK="-sl"
      shift 1
      ;;
    *)
      N=${1}
      break
      ;;
  esac
done

MAPFILE="raytrace${N}.map"
BTLFILE="raytrace${N}.btl"

if [ ! -f ${MAPFILE} ];
then
  echo "${MAPFILE} does not exists!"
  exit 1
fi
if [ ! -f ${BTLFILE} ];
then
  echo "${BTLFILE} does not exists!"
  exit 1
fi

nworkers=`expr ${N} - 1`
echo "Workers = ${nworkers}"

export SPYNET=${MAPFILE}

# A run killed before it could clean up leaves its SysV segment behind, and
# node 1 then refuses to create a new one ("SHMLinks already exists"), which
# fails the whole network.  The key is ftok(mapfile, 1984): proj_id in the top
# byte, then the low byte of st_dev, then the low 16 bits of st_ino.
eval `stat -c 'dev=%d ino=%i' ${MAPFILE}`
KEY=`printf '0x%08x' $(( (192 << 24) | ((dev & 255) << 16) | (ino & 65535) ))`
STALE=`ipcs -m | awk -v k="${KEY}" '$1 == k { print $2 }'`
if [ -n "${STALE}" ];
then
  echo "Removing stale shared segment ${KEY} (shmid ${STALE})"
  ipcrm -m ${STALE}
fi

PIDS=""
cleanup() {
  for p in ${PIDS}; do kill -9 ${p} 2>/dev/null; done
  STALE=`ipcs -m | awk -v k="${KEY}" '$1 == k { print $2 }'`
  [ -n "${STALE}" ] && ipcrm -m ${STALE} 2>/dev/null
  return 0
}
trap 'echo; cleanup; exit 130' INT TERM

if [ 1 -le ${nworkers} ];
then
  # Node 1 creates the shared segment; the others attach to it and exit at
  # once if it is not there yet.  So node 1 goes first, alone, and we wait
  # for the segment to appear before starting anyone else.  Starting every
  # worker in one loop is a race that kills node 2 at three processors and up.
  echo "Starting worker1"
  t4 -s8 ${SHLINK} -sn 1 -si &
  PIDS="${PIDS} $!"

  waited=0
  while [ ${waited} -lt 100 ];
  do
    ipcs -m | awk -v k="${KEY}" '$1 == k { found = 1 } END { exit !found }' && break
    sleep 0.1
    waited=`expr ${waited} + 1`
  done

  i=2
  while [ ${i} -le ${nworkers} ];
  do
    echo "Starting worker${i}"
    t4 -s8 ${SHLINK} -sn ${i} -si &
    PIDS="${PIDS} $!"
    i=`expr ${i} + 1`
  done
  sleep 1
fi

echo "Starting HOST"
t4 -s8 ${SHLINK} -sn 0 -sb ${BTLFILE}

# Only the emulators this script started - never `pkill t4`, which would also
# kill any unrelated emulator on the machine, and a stray one burns a core and
# makes the next run look like it hung.
cleanup
