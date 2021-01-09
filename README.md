# Little Free Library Monitor

Using a Particle Photon to monitor the use of our little free library. A magnetic contact sensor on the door triggers
the Photon to send updated via MQTT to a Home Assistant service.
[Photos of Setup](https://photos.google.com/share/AF1QipN5xh3hu_HO4zXRZLLBMUYIvoRR8gYFrqdjjhFlartGJoHF_7nDE2l5T03hNQTMsQ?key=NlczaDVhcS1wTExfV0xsc3lONEVtWWJIeG9KbnhR)

## Configure and Build

### Submodule Libs

The libs are imported via git submodules, after checking populate them via:

```bash
git submodule init
git submodule update
```

### Configuration

Copy `credentials.h.SAMPLE` to `credentials.h` and update the constants for your environment. Initial flash and testing
should set `MQTT_TESTING` to true to avoid polluting Home Assistant with incorrect sensors.

### Building

Use the [Particle CLI](https://docs.particle.io/tutorials/developer-tools/cli/) `particle compile photon` or
[Particle Workbench](https://docs.particle.io/tutorials/developer-tools/workbench/).


## Bill of Materials

* [Particle Photon](https://store.particle.io/collections/gen-2/products/photon)
* [SparkFun Photon Battery Shield](https://www.sparkfun.com/products/13626)
* [3Ah LiPo Battery](https://www.amazon.com/gp/product/B07TVDPC21)
* [Magnetic Reed Switch](https://www.amazon.com/gp/product/B07YVG94SF)
* [10 Watt, 6 Volt Solar Panel](https://www.amazon.com/gp/product/B085W9KCZ8)
* [Project Box](https://www.amazon.com/gp/product/B073Y7FW1Q)
* [Breadboard](https://www.amazon.com/gp/product/B07PCJP9DY)

