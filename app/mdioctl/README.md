# mdioctl

A [phytool](https://github.com/wkz/phytool)-style command-line tool for the Tapioca
**MDIO master** firmware (`-D RUN_MDIO_MASTER`). It drives a PHY's management
registers over the device's USB-CDC port, using the same verbs and output as
phytool — minus the interface field, since there's only one USB port, not a NIC.

```sh
./mdioctl read  1/2          # 0x0007
./mdioctl write 1/4 0x01e1   # silent on success
./mdioctl read  1:31/0x0300  # MMD / Clause-45-style path
./mdioctl write --verify 0:31/0x0302 0x3e80  # also silent when verified
./mdioctl print 1            # pretty-dump the standard registers
./mdioctl print --mmd 31 --start 0x0300 --count 16 1
```

## Usage

| command | does | stdout |
|---|---|---|
| `read <phy>/<reg>` | read one register | the bare value, `0x%04x` |
| `read <phy>:<mmd>/<reg>` | read one MMD register | the bare value, `0x%04x` |
| `write [--verify] <phy>/<reg> <val>` | write one register | *(nothing — success is exit 0)* |
| `write [--verify] <phy>:<mmd>/<reg> <val>` | write one MMD register | *(nothing — success is exit 0)* |
| `print <phy>` | bulk-read regs 0..31 and pretty-print | the decoded table |
| `print --mmd <mmd> [--start <reg>] [--count <n>] <phy>` | read an MMD register window | raw register table |

`mdioctl` follows `phytool`'s output style: **a successful write prints nothing**.
That is true for both plain writes and `write --verify`. Check `$?` if a script
or shell session needs to test success explicitly:

```sh
./mdioctl write --verify 0:31/0x0302 0x3e80
echo $?   # 0 means the write was accepted and the readback matched
```

- Numbers are **base-0**: `0x..` hex or plain decimal (`mdioctl read 1/2` == `mdioctl read 0x1/0x2`).
- C22 `phy`/`reg` and MMD `mmd` are `0..31`; MMD `reg` and `val` are `0..0xffff`.
- MMD paths follow `phytool` shape without the Linux interface field: `<phy>:<mmd>/<reg>`.
- `write --verify` is an add-on to the normal write path. Internally, it waits for
  the device-side `write ... ok`, then issues a read of the same register and
  compares the exact 16-bit value. Success is still silent. A failed write, read
  `noresp`/`timeout`, or readback mismatch prints an error to stderr and exits
  non-zero.
- Plain MDIO writes cannot directly detect a missing PHY, because Clause-22 writes
  have no PHY-driven TA response. `--verify` detects absence through the follow-up
  read, and also catches registers that did not retain the requested value.
- The CDC port is resolved as **`--port`**, else the **`MDIO_PORT`** env var, else
  **auto-detected** when a single device is plugged in (with several, it lists the
  candidates and asks you to pick). The flag wins over the env var:
  ```sh
  export MDIO_PORT=/dev/cu.usbmodemXXXX   # set once for the session
  ./mdioctl read 1/2
  ```
- **Errors go to stderr with a non-zero exit** (`noresp` = no PHY answered on a read,
  `timeout` = the blob stalled, verify mismatch = readback differed), so it pipes
  and scripts like phytool:
  ```sh
  v=$(./mdioctl read 1/2) || exit 1
  ```

`print` decodes the IEEE 802.3 Clause-22 registers (`BMCR`, `BMSR`, `PHYID1/2`,
`ANAR`, `ANLPAR`, `ANER`); vendor registers 7..31 are shown raw. The register
*naming* is entirely host-side — the device only ever returns raw 16-bit values.

`print --mmd` is also host-side: it issues one MMD read per register through the
same `phy:mmd/reg` path. By default it reads 32 registers starting at `0`; use
`--start` and `--count` for sparse vendor windows:

```sh
./mdioctl print --mmd 31 --start 0x0300 --count 16 0
```

## The wire protocol (drive it by hand)

`mdioctl` is only a convenience wrapper. The firmware speaks a plain-ASCII line
protocol over CDC, so **any terminal works** — handy for debugging:

```
!read  <phy>/<reg>          ->  read  <phy>/<reg> 0xXXXX   | read  <phy>/<reg> err <why>
!write <phy>/<reg> <val>    ->  write <phy>/<reg> ok       | write <phy>/<reg> err <why>
!read  <phy>:<mmd>/<reg>    ->  read  <phy>:<mmd>/0xRRRR 0xXXXX | read  <phy>:<mmd>/0xRRRR err <why>
!write <phy>:<mmd>/<reg> <val> -> write <phy>:<mmd>/0xRRRR ok   | write <phy>:<mmd>/0xRRRR err <why>
!print <phy>                ->  32x  read <phy>/<reg> 0xXXXX
<invalid command>           ->  err invalid
```

e.g. `screen /dev/cu.usbmodemXXXX`, then type `!read 1/2`. The `#`-prefixed banner
line at boot is informational; `mdioctl` skips it. Over-long input is rejected as
one complete line through the next CR/LF, so its suffix is never interpreted.
`mdioctl` reports the resulting `err invalid` directly and exits non-zero.

## Clause 45 / MMD

MMD access is implemented through the standard Clause-22 REGCR/ADDAR indirect
sequence. Native Clause-45 frames are still a later firmware backend; the CLI syntax
is already the intended `phytool`-style form.
