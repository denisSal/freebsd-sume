# NetFPGA SUME Reference NIC device driver

## Project description
This Google Summer of Code project enables NetFPGA SUME 4x10 Gbps FPGA board to work as a NIC on FreeBSD by creating a driver based on the existing Linux driver for the 'Reference NIC' design using the RIFFA DMA engine from the private NetFPGA-SUME-live repository.

The SUME hardware design also offers communication with internal registers belonging to different modules of the design over a second channel. This is used as a way to obtain packet statistics and link status from nf_10g_interface modules connected to the board's SFP+ modules.

Original GSoC proposal: https://summerofcode.withgoogle.com/projects/#4932584418574336

FreeBSD Wiki project page: https://wiki.freebsd.org/SummerOfCode2020Projects/NetFPGA_SUME_Driver

## Preparing the NetFPGA SUME
To flash the board with the 'Reference NIC' project, one must first obtain the reference_nic bitstream. The build instructions are available on the [NetFPGA SUME Wiki page](https://github.com/NetFPGA/NetFPGA-SUME-public/wiki/NetFPGA-SUME-Reference-NIC), but it is also possible to download the pre-built bitstream from [the University of Cambridge](http://www.cl.cam.ac.uk/research/srg/netos/projects/netfpga/bitfiles/NetFPGA-SUME-live/1.9.0/reference_nic/reference_nic.bit).

**NOTE**: on my setup, the pre-built bitstream wasn't working correctly - at higher incoming rates, the newly incoming packets would overwrite the ones in some internal FIFO so the packets would come out of the board scrambled until physical reset of the board. If you want to skip building the bitstream and don't want to use the one from Cambridge, I'll soon provide the one I built myself.

For the next step I used Linux, as the appropriate JTAG programming tools are not available for FreeBSD. It is also possible to flash the board using the Vivado's *xmd* tool, as explained in the Reference NIC wiki page.

Install Digilent Adept Tools (Runtime and Utilities) from [Digilent](https://reference.digilentinc.com/reference/software/adept/start), connect your machine with the NetFPGA via USB and flash the board with:
```
 # dsumecfg -d NetSUME write -verbose -s 2 -f reference_nic.bit # flash to flash section 2
 # dsumecfg -d NetSUME setbootsec -s 2 # load flash section 2 on board boot-up
 # dsumecfg -d NetSUME reconfig -s 2 # reconfigure the board from section 2
```

## Instructions to load the driver
After downloading the code, run:
```
 # make
 # kldload ./if_sume.ko
```
The driver should load and create 4 interfaces (named *sume0*-*sume3*).

## Additional information
The driver in the current state is merged to the FreBSD head repository in [this](https://reviews.freebsd.org/rS364973) commit. I may use this Github repository to further improve and fix the code before merging to the FreBSD release. Additionally, I also started the work on the SUME NICv2 driver [here](https://github.com/denisSal/freebsd-sume-nic_v2). 

The original Linux drivers for both the reference NIC and the NICv2 drivers can be found in the NetFPGA SUME live repo [here](
https://github.com/NetFPGA/NetFPGA-SUME-live/tree/master/lib/sw/std/driver/sume_riffa_v1_0_0) and [here](https://github.com/NetFPGA/NetFPGA-SUME-live/tree/master/contrib-projects/nic_v2/sw/sume_uam_v1_0_0) (registration needed to view the code).

You can find more details and information about the project and code in the driver's man page, as well as on the GSoC project page and FreeBSD wiki page linked above.

## Useful links
 - https://reference.digilentinc.com/sume:sume
 - https://github.com/NetFPGA/NetFPGA-SUME-live
 - https://github.com/NetFPGA/NetFPGA-SUME-public/wiki
