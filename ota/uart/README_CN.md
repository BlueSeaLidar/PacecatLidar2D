# 串口固件升级工具使用说明

## 使用步骤

1. 连接设备到串口。
2. 进入源码目录：

```bash
cd src
```

3. 编译：

```bash
./build.sh
```

4. 串口赋权：

```bash
sudo chmod 777 /dev/ttyUSB*
```

5. 修改脚本并运行升级：

编辑 [run.sh](src/run.sh)，修改 `./LiUpCon` 命令中的串口设备、波特率和固件文件名为实际值：

脚本内容示例：

```bash
./LiUpCon  /dev/ttyUSB0  921600 LDS-50C-R_V4.9_20240515.bin
```

然后运行：

```bash
./run.sh
```

