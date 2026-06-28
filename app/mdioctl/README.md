# mdioctl

A [phytool](https://github.com/wkz/phytool)-style command-line tool for the Tapioca
**MDIO master** firmware (`-D RUN_MDIO_MASTER`). It drives a PHY's management
registers over the device's USB-CDC port, using the same verbs and output as
phytool — minus the interface field, since there's only one USB port, not a NIC.

```sh
./mdioctl read  1/2          # 0x0007
./mdioctl write 1/4 0x01e1   # silent on success
./mdioctl print 1            # pretty-dump the standard registers
```

## Usage

| command | does | stdout |
|---|---|---|
| `read <phy>/<reg>` | read one register | the bare value, `0x%04x` |
| `write <phy>/<reg> <val>` | write one register | *(nothing — success is exit 0)* |
| `print <phy>` | bulk-read regs 0..31 and pretty-print | the decoded table |

- Numbers are **base-0**: `0x..` hex or plain decimal (`mdioctl read 1/2` == `mdioctl read 0x1/0x2`).
- `phy` and `reg` are `0..31`; `val` is `0..0xffff`.
- The CDC port is resolved as **`--port`**, else the **`MDIO_PORT`** env var, else
  **auto-detected** when a single device is plugged in (with several, it lists the
  candidates and asks you to pick). The flag wins over the env var:
  ```sh
  export MDIO_PORT=/dev/cu.usbmodemXXXX   # set once for the session
  ./mdioctl read 1/2
  ```
- **Errors go to stderr with a non-zero exit** (`noresp` = no PHY answered, `timeout` =
  the blob stalled), so it pipes and scripts like phytool:
  ```sh
  v=$(./mdioctl read 1/2) || exit 1
  ```

`print` decodes the IEEE 802.3 Clause-22 registers (`BMCR`, `BMSR`, `PHYID1/2`,
`ANAR`, `ANLPAR`, `ANER`); vendor registers 7..31 are shown raw. The register
*naming* is entirely host-side — the device only ever returns raw 16-bit values.

## The wire protocol (drive it by hand)

`mdioctl` is only a convenience wrapper. The firmware speaks a plain-ASCII line
protocol over CDC, so **any terminal works** — handy for debugging:

```
!read  <phy>/<reg>          ->  read  <phy>/<reg> 0xXXXX   | read  <phy>/<reg> err <why>
!write <phy>/<reg> <val>    ->  write <phy>/<reg> ok       | write <phy>/<reg> err <why>
!print <phy>                ->  32x  read <phy>/<reg> 0xXXXX
```

e.g. `screen /dev/cu.usbmodemXXXX`, then type `!read 1/2`. The `#`-prefixed banner
line at boot is informational; `mdioctl` skips it.

## Clause 45

Not implemented yet (Clause-22 only). C45 is a later, CPU-side-only addition to the
firmware; the CLI would gain a `<phy>:<dev>/<reg>` path form to match phytool.
