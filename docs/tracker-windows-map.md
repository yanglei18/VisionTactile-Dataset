# VIVE Ultimate Tracker Windows Mapping

Windows mapping is mandatory before Linux pose validation. Windows is used
only to pair, initialize, and map the Trackers; formal pose capture remains on
Linux.

## Requirements

- Windows 10 or Windows 11.
- Current SteamVR and VIVE Hub.
- SteamVR null driver as described by the external PyVUT project when no
  physical headset is available.
- One VIVE Wireless Dongle and three charged Ultimate Trackers.
- A normally lit, textured room with minimal glare.

The null-driver route is a community workflow, not official HTC no-headset
support. Stop if VIVE Hub refuses the virtual headset.

## Hardware setup

1. Disconnect all RealSense cameras.
2. Connect the Dongle directly through its dedicated extension or cradle.
3. Do not use a generic USB hub or splitter.
4. Keep all three Trackers charged and initially powered off.

## Mapping sequence

1. Start SteamVR and verify that the null driver supplies a usable virtual
   headset state.
2. Start VIVE Hub and open the Ultimate Tracker setup flow.
3. Pair one Tracker at a time and privately record its serial-to-role mapping.
4. Create one tracking map for the fixed experiment room.
5. Move and rotate each Tracker until all three show Ready or the current VIVE
   Hub equivalent.
6. Power-cycle all three normally and confirm that all return to Ready without
   creating another map.

## Stop conditions

- Stop if VIVE Hub rejects the SteamVR null driver.
- Stop before any firmware update, factory reset, map clear, pairing clear,
  bootloader operation, or device recovery operation.
- Do not mark mapping complete if any Tracker fails to relocalize after a
  normal power cycle.

## Private evidence

Write screenshots and text evidence only below `C:\vut-validation\`. Record
Windows, SteamVR, VIVE Hub, Dongle, and Tracker firmware versions. Keep device
serials and role mappings out of Git.

The Windows gate passes only when `null_driver_accepted`, `all_three_paired`,
`map_completed`, `all_three_ready`, and `relocalized_after_power_cycle` are
true, and `firmware_update_performed` is false.
