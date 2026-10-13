# UDP Firmware Upgrade Tool

## Usage

1. Make sure the lidar and the host are on the same network.
2. Enter the source directory:

```bash
cd src
```

3. Build:

```bash
./build.sh
```

Or build manually:

```bash
gcc upgrade.c -o lanhai
```

4. Grant execute permission to the scripts:

```bash
chmod +x build.sh start_upgrade.sh
```

5. Edit the script and run the upgrade:

Edit [start_upgrade.sh](src/start_upgrade.sh) and change the `-d` (lidar IP), `-p` (port), and `-f` (firmware file name) in the `./lanhai` command to the actual values:

Script content example:

```bash
./lanhai -d 192.168.0.92 -p 6543 -f LDS-E330-S_V1_20241009_192644.lhl
```

Place the firmware file in the `src` directory (the same level as `upgrade.c`, `build.sh`, and `start_upgrade.sh`).

Then run:

```bash
./start_upgrade.sh
```

When you see `iap return 0 write IAP ok`, the upgrade succeeded. Ignore any subsequent UDP errors (the lidar restarts after upgrade, so failing to read data is normal).

See [IAP升级流程.pdf](docs/IAP升级流程.pdf) for the upgrade protocol.

