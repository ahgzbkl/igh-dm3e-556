# igh-dm3e-556
两个雷赛从站dm3e-556进行csp模式，并且到达指定位置后停止旋转

1.先启动igh主站
sudo /usr/local/etc/init.d/ethercat start

2.输入以下四个命令就可以执行程序了，但是还得
mkdir build && cd build 
cmake ..
make 
sudo ./ excutable

3.修改从站的Vendor Id和Product code，需要打开终端输入
sudo /usr/local/bin/ethercat cstruct -p 0
信息如下：
/* Master 0, Slave 0, "DM3E-556"
 * Vendor ID:       0x00004321
 * Product code:    0x00008100
 * Revision number: 0x00000001
 */

ec_pdo_entry_info_t slave_0_pdo_entries[] = {
    {0x6040, 0x00, 16},
    {0x6060, 0x00, 8},
    {0x607a, 0x00, 32},
    {0x6041, 0x00, 16},
    {0x606c, 0x00, 32},
    {0x6064, 0x00, 32},
    {0x6061, 0x00, 8},
};

ec_pdo_info_t slave_0_pdos[] = {
    {0x1600, 3, slave_0_pdo_entries + 0},
    {0x1a00, 4, slave_0_pdo_entries + 3},
};

ec_sync_info_t slave_0_syncs[] = {
    {0, EC_DIR_OUTPUT, 0, NULL, EC_WD_DISABLE},
    {1, EC_DIR_INPUT, 0, NULL, EC_WD_DISABLE},
    {2, EC_DIR_OUTPUT, 1, slave_0_pdos + 0, EC_WD_ENABLE},
    {3, EC_DIR_INPUT, 1, slave_0_pdos + 1, EC_WD_DISABLE},
    {0xff}
};






