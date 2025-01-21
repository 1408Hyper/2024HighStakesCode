![1408Hyper 2024-25 repo banner](https://raw.githubusercontent.com/1408Hyper/creative-assets/refs/heads/main/logos/final/SWED/2024HighStakesCode%20Banner%202%20Light%20Only.PNG)

<center><h1> ⚡<em><strong>1408H</em></strong>yper @ VRC 2024-25</center></h1>

[![Github Actions Makefile CI](https://github.com/helloworld3200/1408Hyper-2024VRC-Code/actions/workflows/c-cpp.yml/badge.svg)](https://github.com/helloworld3200/1408Hyper-2024VRC-Code/actions/workflows/c-cpp.yml)

> 1408H's source code for the 2024-25 "High Stakes" season of VRC.

### Sponsored by [Teehee Dental Works](https://teehee.sg/)

**2024HighStakesCode** was designed with portability in mind - we understand that situations can quickly change at any competition.  

Our code is hot-swappable, making heavy use of **abstract classes** and **templates**
to allow for us to rapidly change and test different pieces of code when time is of the essence.

Additionally, our code features a [**custom built CI/CD testing suite via GitHub Actions.**](https://github.com/helloworld3200/1408Hyper-2024VRC-Code/actions/workflows/c-cpp.yml) - the only purpose-made one that exists in the entire world at the time of writing.

## Chain of Command

_This list does not contain all roles, only those most relevant to programming._

1. **Principal _Software Engineer_** > [helloworld3200](https://github.com/helloworld3200)
2. **Senior _Software Engineer_** > [Aadi-L](https://github.com/Aadi-L)
3. **Assistant _Software Engineer_** > [krishma2](https://github.com/krishma2)

-  **Principal _Mechanical Engineer_** > [1408H-Builder](https://github.com/1408H-Builder)
- **Senior _Mechanical Engineer_** > [YuvrajVerma09](https://github.com/YuvrajVerma09)

## Included Libraries

- Created with [`PROS` API](https://github.com/purduesigbots/pros)
as this has better documentation than the official VEX API.

## Project Structure

To run Make, first install [ARM G++](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads).

- **Main file** at `src/main.cpp`.
- **Includes** in `include/main.h`.
- **Options _(e.g. motor ports)_** in `include/options.h`
- **Legacy/unused code _(e.g. old LiftMech)_** in `legacy/` directory
