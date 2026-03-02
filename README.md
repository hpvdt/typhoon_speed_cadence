# Typhoon Speed and Cadence

This is the amazing repository for the amazing Speed and Cadance team.
This embedded system uses an STM32 and a hall effect setup to accurately calculate the speed of the bike in real time, displaying the information to an OLED
HOE: Oliver Cai, Juston Lo
Lead: Daniel Dot
Members: Luca M, Rena, Noah, Iris

MEMBERS ADD YOUR LAST NAMES PLEASE I DONT KNOW THEM :(

## Hardware Setup

We must all operate on the same main version of KiCad, 9.0.
Ensure you are always working on the project file, never open the files standalone.

## Firmware Setup

### THIS PART IS VERY IMPORTANT PLEASE READ

We must all operate on the same versions, if we do not then stuff will break. We are currently using:
- STM32CubeIDE 2.0.0
- STM32CubeMX 6.16.0

You must download both to program as the IDE is used oto program while the CubeMX is for the .ioc file to configure the board pinout.

When writing code in the main.c, main.h, or any program where code is overwritten from the .ioc save, PLEASE ENSURE YOU ARE WRITING IN A USER CODE BLOCK, or else your code will be removed.

The main code is found in main.c in the src folder, and whenever working on the code always work in the project folder, dont open the file standalone. 
The logic for the code is commented in code, and any changes MUST also be commented for others to work simultaneously on it.

If there are any questions about the project, message me on disc dot2944, or email daniel.dot@mail.utoronto.ca

## Organization

Pretty self-explanitory, put hardware related files into the `hardware` folder, firmware in the `firmware` folder. Other folders can be added as needed: for example a `media` folder is handy for storing pictures related to a project which can be used in documentation.

## Conventions

Partly for consistency, partly for functionality; please follow the following conventions:

- Try to keep folder names to single words and all lowercase
- Use underscores to separate words in names (e.g. `board_rev1`). **DO NOT USE SPACES! ESPECIALLY FOR FOLDERS CONTAINING CODE!** Spaces in file paths may cause issues in compilers and such so best to avoid them entirely.
