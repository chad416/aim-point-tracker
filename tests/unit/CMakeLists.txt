cmake_minimum_required(VERSION 3.20)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(aimtrk_unit)

set(AIMTRK ${CMAKE_CURRENT_SOURCE_DIR}/../../aimtrk)

target_include_directories(app PRIVATE src ${AIMTRK}/include)

target_sources(app PRIVATE
	src/test_centroid.c
	src/test_detect.c
	src/test_geom.c
	src/test_calib.c
	src/test_zero.c
	src/test_proto.c
	src/test_timesync.c
	src/test_track.c
	src/test_match.c
	${AIMTRK}/src/linalg.c
	${AIMTRK}/src/centroid.c
	${AIMTRK}/src/detect.c
	${AIMTRK}/src/geom.c
	${AIMTRK}/src/calib.c
	${AIMTRK}/src/zero.c
	${AIMTRK}/src/proto.c
	${AIMTRK}/src/timesync.c
	${AIMTRK}/src/track.c
	${AIMTRK}/src/match.c
)
