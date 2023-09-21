# About
I wanted to see if I could experience any VR games using a standard controller and the XREAL Air glasses with head
tracking. I came across an [OpenVR HMD library](https://github.com/r57zone/OpenVR-ArduinoHMD) that build out the basic
HMD integration that would work with SteamVR and a 
[Linux USB driver](https://gitlab.com/TheJackiMonster/nrealAirLinuxDriver) that provided the necessary outputs to plug
into the former. I'm not a C++ developer, so it took many days of tinkering and fighting with CMake to get this to work
(I'm sure my CMake setup could be greatly cleaned up and simplified, so I welcome some PRs here) and many more days of
tinkering to link to two libraries together and get the compiled driver working in SteamVR.

If you're looking for general, lightweight head-tracking support for all games (including non-VR), check out my other [XREAL Air Linux Game Driver](https://github.com/wheaney/xrealAirLinuxDriver).

[![ko-fi](https://ko-fi.com/img/githubbutton_sm.svg)](https://ko-fi.com/U7U8OVC0L)

# Build
Building requires cmake 3.21 or later.

To build, just run `bin/package.sh`.

# SteamVR Usage
## Installing
You'll need to install [SteamVR](https://store.steampowered.com/app/250820/SteamVR/) before you can install this driver.

From there:
1. [Download the latest build](https://github.com/wheaney/OpenVR-xrealAirGlassesHMD/releases/latest/download/driver_air_glasses.tar.gz) or run the build instructions above
2. Extract the resulting tar gzip file to your home directory or wherever your prefer; from this you should have an `air_glasses` directory (the one containing the directories `resources` and `bin`). Then use `vrpathreg` to register your driver with SteamVR. This looks something like: `~/.steam/steam/steamapps/common/SteamVR/bin/vrpathreg.sh adddriver ~/air_glasses`.
   * Alternatively, extract the tar gzip file to your SteamVR drivers directory. This should be something like 
`~/.steam/steam/steamapps/common/SteamVR/drivers/`. Be sure this extracts as the directory `air_glasses`, which should
contain the directories `resources` and `bin`.
3. Copy the `60-xreal-air.rules` file from the `udev` directory to `/etc/udev/rules.d/`
   * For Steam Deck, you may need to disable the readonly file system: `sudo steamos-readonly disable`
   * Reload the udev rules using something like `udevadm control --reload`
4. Restart Steam (not usually necessary)

## Using XREAL Air glasses as a HMD VR device

If you're on the Steam Deck, switch to desktop mode. Launch SteamVR. Run the room configuration and set it up for 
sitting mode, with a non-zero height (e.g. 60 inches or a reasonable height). You can hold the brightness up button on 
your glasses for a few seconds to enable side-by-side 3d.

So far I've gotten this to work as a 3DoF HMD device in Steam Home (I don't have a SteamVR compatible controller so I couldn't move around) and in games that support VR like Subnautica (a standard controller worked with this! But it was super zoomed in despite having what I believe to be the correct FoV setting). I've only had mixed success as it doesn't work every time. 

A couple things to look out for: you need to launch SteamVR first. I've either launched it from Steam directly or via the startup script (e.g. `~/.steam/steam/steamapps/common/SteamVR/bin/vrstartup.sh`). SteamVR should show a green headset icon to indicate that the HMD is recognized and plugged in, in addition Steam should show a headset icon up in the top-right corner by the minimize/close buttons. If you've gotten that far, you'll want to get out of SteamVR Home (I haven't figure out how to disable this yet in Linux), leave SteamVR launched (make sure the green headset icon is still shown), then you can launch the VR game of your choice from Steam. Theoretically, any non-VR game should work through the desktop theatre mode that SteamVR supports, and other SteamVR-compatible applications like [SimulaVR](https://github.com/SimulaVR/Simula) should work, but, again, it seems pretty finicky.

Sometimes SteamVR decides to block the headset, as it thinks it's the cause of a crash. To fix this, I edit `~/.steam/steam/config/steamvr.vrsettings` and remove the safe-mode configuration for `air_glasses`.

Good luck!
