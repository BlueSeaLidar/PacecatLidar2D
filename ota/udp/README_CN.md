# UDP 固件升级工具使用说明

## 使用步骤

1. 确保雷达与主机在同一网络。
2. 进入源码目录：

```bash
cd src
```

3. 编译：

```bash
./build.sh
```

或手动编译：

```bash
gcc upgrade.c -o lanhai
```

4. 赋予脚本执行权限：

```bash
chmod +x build.sh start_upgrade.sh
```

5. 修改脚本并运行升级：

编辑 [start_upgrade.sh](src/start_upgrade.sh)，修改 `./lanhai` 命令中的 `-d`（雷达 IP）、`-p`（端口）、`-f`（固件文件名）为实际值：

脚本内容示例：

```bash
./lanhai -d 192.168.0.92 -p 6543 -f LDS-E330-S_V1_20241009_192644.lhl
```

固件文件需放在 `src` 目录下（与 `upgrade.c`、`build.sh`、`start_upgrade.sh` 同一层）。

然后运行：

```bash
./start_upgrade.sh
```

出现 `iap return 0 write IAP ok` 说明升级成功，不用管后续的 udp 报错（升级后会重启雷达，无法读取数据是正常现象）。

升级协议文档见 [IAP升级流程.pdf](docs/IAP升级流程.pdf)。

