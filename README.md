# About
I wanted to see if I could experience any VR games using a standard controller and the xReal Air glasses with head
tracking. I came across an [OpenVR HMD library](https://github.com/r57zone/OpenVR-ArduinoHMD) that build out the basic
HMD integration that would work with SteamVR and a 
[Linux USB driver](https://gitlab.com/TheJackiMonster/nrealAirLinuxDriver) that provided the necessary outputs to plug
into the former. I'm not a C++ developer, so it took many days of tinkering and fighting with CMake to get this to work
(I'm sure my CMake setup could be greatly cleaned up and simplified, so I welcome some PRs here) and many more days of
tinkering to link to two libraries together and get the compiled driver working in SteamVR.

So far I've only gotten this to work as a HMD in SteamVR Home. I don't have any SteamVR compatible controllers, so I
haven't been able to move around, only use the head tracking to look around. My only testing has been on a Steam Deck
which fails to launch any other applications that are supposedly SteamVR capable. I have Subnautica and two other
SteamVR compatible games, but they all launch in non-VR mode despite changing their Launch Options in the game
properties prior to launching, this might be due to missing a VR compatible controller.

# Build
Building requires cmake 3.21 or later.

To build, just run `bin/package.sh`.

# Installing
You'll need to install [SteamVR](https://store.steampowered.com/app/250820/SteamVR/) before you can install this driver.

From there:
1. Run the build instructions above or download the latest build
2. Extract the resulting tar gzip file to your SteamVR drivers directory. This should be something like 
`~/.steam/steam/steamapps/common/SteamVR/drivers/`. Be sure this extracts as the directory `air_glasses`, which should
contain the file `driver_air_glasses.so`.
3. Copy the `nreal_air.rules` file from the `udev` directory to `/etc/udev/rules.d/`
4. Restart Steam (not usually necessary)

# Using xReal Air glasses as a HMD VR device
If you're on the Steam Deck, switch to desktop mode. Launch SteamVR. Run the room configuration and set it up for 
sitting mode, with a non-zero height (e.g. 60 inches or a reasonable height). You can hold the brightness up button on 
your glasses for a few seconds to enable side-by-side 3d.
