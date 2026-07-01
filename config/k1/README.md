# Creality K1 - Klipper

Configuration and micro-controller build settings for the Creality K1.

Bed probing uses `[load_cell_probe]`: the four HX711 load cells on the
leveling MCU are summed into one signal, so the probe triggers on a real
force in grams and is calibrated against a known mass.

## Files

- `printer.cfg` - main printer configuration (includes `macros.cfg`)
- `macros.cfg` - sensorless XY homing and load-cell Z homing
- `main-mcu.config`, `nozzle-mcu.config`, `leveling-mcu.config` - MCU build
  settings (`GD32F303RET6`, `GD32F303CBT6`, `GD32E230F8P6`)

## Building the firmware

For each micro-controller, build with its config:

```sh
cp config/k1/leveling-mcu.config .config
make olddefconfig
make
# out/klipper.bin  -> leveling MCU
```

Repeat with `main-mcu.config` and `nozzle-mcu.config`.

Flashing note: this produces a plain `klipper.bin`.  The factory MCU loader
expects a binary with its own header/checksum, so flash over SWD or add
that loader header before flashing through the factory update path.

## Host

`[load_cell_probe]` needs the Python `numpy` module in the Klipper
environment.

## Calibrating the probe (force in grams)

```
LOAD_CELL_DIAGNOSTIC        ; check sample rate / saturation
LOAD_CELL_CALIBRATE         ; then: TARE
                            ;       (apply a known mass, e.g. 500 g)
                            ;       CALIBRATE GRAMS=500
                            ;       ACCEPT
SAVE_CONFIG
```

This writes `counts_per_gram` and `reference_tare_counts`.  Afterwards
`LOAD_CELL_READ` reports force in grams and the probe triggers at
`trigger_force` grams.  Verify with `PROBE_ACCURACY`, then run
`BED_MESH_CALIBRATE`.
