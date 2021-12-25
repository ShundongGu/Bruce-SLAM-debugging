#!/bin/bash
echo 1
if [ ! -d "../build/"  ];then
  mkdir ../build
else
  echo build exist
fi

filename=`date +%m%d`

rm ../install/slam/bin/*

cd ../build
cmake .. -Wno-dev -DPlatform=RK
free -h

make -j8

if [ $? -eq 0 ];then
  cd ../install/slam/bin
  arm-linux-gnueabihf-strip *
#  cp * /mnt/hgfs/up_load/$filename
  echo -e "\033[32m===============================\033[0m"
  echo -e "\033[32m     ┏┛ ┻━━━━━┛  ┻┓            \033[0m "
  echo -e "\033[32m     ┃            ┃            \033[0m "
  echo -e "\033[32m     ┃　　　━　   ┃               \033[0m"
  echo -e "\033[32m     ┃　┳┛　  ┗┳  ┃              \033[0m"
  echo -e "\033[32m     ┃　　　　　  ┃              \033[0m"
  echo -e "\033[32m     ┃　　　┻　   ┃              \033[0m"
  echo -e "\033[32m     ┃　　　　　  ┃              \033[0m"
  echo -e "\033[32m     ┗━┓　　　┏━━━┛              \033[0m"
  echo -e "\033[32m       ┃　　　┃   神兽保佑        \033[0m"
  echo -e "\033[32m       ┃　　　┃   代码无BUG！     \033[0m"
  echo -e "\033[32m       ┃　　　┗━━━━━━━━━┓        \033[0m"
  echo -e "\033[32m       ┃　　　　　　　  ┣┓      \033[0m"
  echo -e "\033[32m       ┃　　　　       ┏┛      \033[0m"
  echo -e "\033[32m       ┗━┓ ┓ ┏━━━┳ ┓ ┏━┛       \033[0m "
  echo -e "\033[32m         ┃ ┫ ┫   ┃ ┫ ┫         \033[0m "
  echo -e "\033[32m         ┗━┻━┛   ┗━┻━┛         \033[0m "
  echo -e "\033[32m===============================\033[0m"
  eval "md5sum *"
  eval "ls -l"
  date +%Y/%m/%d%t%H:%M:%S
else
  echo -e "\033[31m=============================================\033[0m"
  echo -e "\033[31m  ┏┛ ┻━━━━━┛  ┻┓      ┃　　　┃   代码有BUG!       \033[0m"
  echo -e "\033[31m  ┃            ┃      ┃　　　┃   神兽GG！     \033[0m"
  echo -e "\033[31m  ┃　　　━　   ┃      ┃　　　┗━━━━━━━━━┓        \033[0m"
  echo -e "\033[31m  ┃_-┛　  ┗-_ ┃      ┃　　　　　　　  ┣┓      \033[0m"
  echo -e "\033[31m  ┃　　　　　  ┃      ┃　　　　       ┏┛      \033[0m"
  echo -e "\033[31m  ┃　　　┻　   ┃      ┗━┓ ┓ ┏━━━┳ ┓ ┏━┛       \033[0m "
  echo -e "\033[31m  ┃　　　　　  ┃        ┃ ┫ ┫   ┃ ┫ ┫         \033[0m "
  echo -e "\033[31m  ┗━┓　　　┏━━━┛        ┗━┻━┛   ┗━┻━┛         \033[0m "
  echo -e "\033[31m=============================================\033[0m"
fi
