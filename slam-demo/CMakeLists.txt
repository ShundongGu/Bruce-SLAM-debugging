# --------------------------------------------------------------------------
#   			Cmake compiles slam demo examples
#   Functional description：test
#   Author                ：Bruce
# --------------------------------------------------------------------------

ADD_DEFINITIONS(-w -Wno-psabi -Wno-dev)
SET(TARGET_NAME "slam-demo")

# --------------------------------------------------------------------------
#  				            SELECT COMPILER
# --------------------------------------------------------------------------
OPTION(Platform "Please select the target platform?" "Rockchip")
MESSAGE(STATUS "################## Platform = ${Platform}")
if(${Platform} STREQUAL "RK")
    SET(CMAKE_C_COMPILER "arm-buildroot-linux-gnueabihf-gcc")
    SET(CMAKE_CXX_COMPILER "arm-buildroot-linux-gnueabihf-g++")
	MESSAGE(STATUS "------------------")
	MESSAGE(STATUS "    Bruce  test   ")
	MESSAGE(STATUS "------------------")
    SET(TOOLCHAIN_LIB /opt/aarch32-rockchip-linux-toolchain/)
    SET(DEMO_LIB /home/gusd1784/learning/test_code/slambook/ch2/libhello.a)
    SET(GLOG_LIB ${TOOLCHAIN_LIB}/3rdParty/lib/libglog.so)

else(${Platform} STREQUAL "x86")
	SET(TOOLCHAIN_LIB /opt/orb-x86-toolchain/)
	MESSAGE(STATUS "------------------")
	MESSAGE(STATUS "     Ubuntu       ")
	MESSAGE(STATUS "------------------")
	SET(RT_PTHREAD_LIB pthread rt)
	SET(GNUSTL_SHARED_LIB stdc++)

 	SET(DEMO_LIB ${TOOLCHAIN_LIB}/lib/libhello.a)

	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pedantic")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lsupc++")

endif()

# ------------------------------------------------------------------------
#   				       PROJECT NAME
# ------------------------------------------------------------------------
MESSAGE(STATUS "-------------------------------------------------------")
MESSAGE(STATUS "-------------------SLAM DEMO START---------------------")
MESSAGE(STATUS "-------------------------------------------------------")

# 声明要求的 cmake 最低版本
cmake_minimum_required( VERSION 2.8 )

# 声明一个 cmake 工程
project( HelloSLAM )

SET(CMAKE_SYSTEM_NAME Linux)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lstdc++")

# 设置编译模式
set( CMAKE_BUILD_TYPE "Debug" )

# 四核Rockchip ARM平台
if(${Platform} STREQUAL "RK")
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pedantic")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lsupc++")

	MESSAGE(STATUS "-----------------------------------")
	MESSAGE(STATUS "   Cortex-A35 FOR aarch32 speed    ")
	MESSAGE(STATUS "-----------------------------------")
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -mcpu=cortex-a35")
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a")
	SET(LIB_OUT_DIR /home/gusd1784/learning/test_code/slambook/ch2/install/slam/lib)
	SET(BIN_OUT_DIR /home/gusd1784/learning/test_code/slambook/ch2/install/slam/bin)
	LINK_DIRECTORIES(${TOP_LEVEL_SRC_DIR}/install/slam/lib)

# ubuntu X86运行平台
else(${Platform} STREQUAL "x86")
		SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -pedantic")
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pedantic")
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
		SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lsupc++")
		SET(ELSE_LIB_PATH "${TOP_LEVEL_SRC_DIR}/otherlibs/platform/lib/linux")

		SET(LIB_OUT_DIR ${TOP_LEVEL_SRC_DIR}/install/x86/lib)
		SET(BIN_OUT_DIR ${TOP_LEVEL_SRC_DIR}/install/x86/bin)
		LINK_DIRECTORIES(${TOP_LEVEL_SRC_DIR}/install/x86/lib)
endif()

# 引入相关库编译头文件路径
INCLUDE_DIRECTORIES(${TOOLCHAIN_LIB}/3rdParty/include/)

# 生成一个静态库
add_library( hello libHelloSLAM.cpp )

# 生成一个动态库
add_library( hello_shared SHARED libHelloSLAM.cpp )

# 添加一个可执行程序
# 语法：add_executable( 程序名 源代码文件 ）
add_executable( ${TARGET_NAME} CChristmasHappy.cpp )

# 将静态库文件链接到可执行程序上
target_link_libraries( slam-demo ${DEMO_LIB} ${GLOG_LIB})

# 设置可执行程序和库的输出路径
SET(LIBRARY_OUTPUT_PATH ${LIB_OUT_DIR})
SET(EXECUTABLE_OUTPUT_PATH ${BIN_OUT_DIR})

MESSAGE(STATUS "-------------------------------------------------------")
MESSAGE(STATUS "---------------SLAM DEMO CMAKE END --------------------")
MESSAGE(STATUS "-------------------------------------------------------")



