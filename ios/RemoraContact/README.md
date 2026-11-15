# RemoraContact (iPhone)

Sends the phone's GPS fixes to the MOOS shoreside so the vessel carrying it (the skiff) shows
up as a contact for the boat's helm. The receiver is `iContactUDP` in
[`moos-ivp-remora`](../../moos-ivp-remora), used by the
[`joust`](../../moos-ivp-remora/missions/joust) mission.

- SwiftUI, iOS 17 or later, no dependencies. `FixSender.swift` is all of the logic.
- One UDP datagram per fix (about 1 Hz), repeated once a second while the phone is not moving:
  `NAME=skiff,LAT=..,LON=..,SPD=..,HDG=..,TIME=..,ACC=..` (`HDG=-1` when there is no course).
- Keeps sending with the screen off (background location). The blue location indicator stays on.

## Build and run

1. Open `RemoraContact.xcodeproj` in Xcode.
2. Target RemoraContact > Signing & Capabilities: pick your team. Change the bundle identifier
   (`org.projectremora.RemoraContact`) if Xcode says it is taken.
3. Run on the phone. Allow location "While Using the App"; that is enough, the background
   mode keeps it going once started.
4. On the phone: connect Tailscale, enter the laptop's Tailscale address and port 9300, a
   vessel name, Start sending.

Built here for the simulator and for a device without signing (2026-11-15). Not yet run on a
phone, so permissions, background behaviour and battery use are untested.
