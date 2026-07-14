# Four physical tools contract fixture

This fixture describes four independent physical tools using deliberately
synthetic identities and values. It exists to detect cross-tool profile leakage
and to freeze the repo-owned compatibility surface during the refactor.

It is not a complete Klipper printer configuration. It has no MCU, pins,
serial path, real dock geometry, actuator movement, or production calibration.
All machine actions emit `KTCC_EVENT` records. Do not install it on a printer.

The fixture intentionally excludes virtual tools, remapping, and `[ktcclog]`.
It includes the independent statistics persistence keys with zeroed synthetic
values. Active and standby temperatures live in a non-heating recording macro
because job temperatures must not be persisted.
