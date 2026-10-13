# Serial Firmware Upgrade Tool

## Usage

1. Connect the device to the serial port.
2. Enter the source directory:

```bash
cd src
```

3. Build:

```bash
./build.sh
```

4. Grant serial port permission:

```bash
sudo chmod 777 /dev/ttyUSB*
```

5. Edit the script and run the upgrade:

Edit [run.sh](src/run.sh) and change the serial port, baud rate, and firmware file name in the `./LiUpCon` command to the actual values:

Script content example:

```bash
./LiUpCon  /dev/ttyUSB0  921600 LDS-50C-R_V4.9_20240515.bin
```

Then run:

```bash
./run.sh
```

