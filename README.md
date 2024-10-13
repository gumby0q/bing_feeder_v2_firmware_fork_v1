# bing_feeder_v2_firmware_fork_v1

better sensor contol for bing feeders

---
#### Install tools

1. Install `xpm` if you don't have it: https://xpack.github.io/xpm/install/
2. Run `xpm install` command

---
#### build

```sh
make
```
---
#### flash

flashing using openocd provided by xpacks

```sh
make flash
```

### feeder step configuration

in `main.c` change folowing define

```c
#define SENSOR_COUNTER_MAX        2  /* defines step 1 = 2mm; 2 = 4mm ... */
```

### Features

- better filtered sensor readings avoids faulty triggers
- long press moves feeder back for 4 mm

## Optional

---
### Install VSCode Extensions

1. [C/C++](https://github.com/Microsoft/vscode-cpptools)
2. [Cortex Debug](https://github.com/Marus/cortex-debug)
3. [Makefile Tools](https://github.com/Microsoft/vscode-makefile-tools)

---
#### Install [stlink](https://github.com/stlink-org/stlink#installation)

1. Go to releases and download `*.deb` package
2. Install it with `sudo dpkg -i stlink_*.deb`
3. In case of missing packages run this command `sudo apt-get -f install`
4. Check installation: `st-info --version`

* `export LD_LIBRARY_PATH=/usr/local/lib` - for `libstlink-shared.so` related error

