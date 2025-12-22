# igh-dm3e-556
两个雷赛从站dm3e-556进行csp模式，并且到达指定位置后停止旋转

1.先启动igh主站

sudo /usr/local/etc/init.d/ethercat start

2.输入以下四个命令就可以执行程序了

mkdir build && cd build 

cmake ..

make 

sudo ./ excutable

3.修改从站的Vendor Id和Product code，需要打开终端输入

sudo /usr/local/bin/ethercat cstruct -p 0






