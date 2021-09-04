# Newsched ATSC OOT

## Dependencies

```
meson
ninja
newsched
```

## Building
1. Build and install `newsched`
2. Build and install `ns-atsc`

```
meson setup build --buildtype=debugoptimized --prefix=[PREFIX]
cd build
ninja
ninja install
```

## Running
This OOT includes an example flowgraph under `/blocklib/atsc/test/test_atsc_receive.py`
```
python3 test_atsc_receive.py [atsc_captured_file].sc16
```
Where `[atsc_captured_file].sc16` is an RF capture (interleaved shorts) of an ATSC signal at 2x the ATSC symbol rate
