# NetFPGA SUME reference NIC device driver

## Project description
NetFPGA is an open-source project aimed to foster networking hardware prototyping. Their latest and de-facto standard development board is NetFPGA SUME with a Virtex-7 FPGA and 4 SFP+ (10 Gbps) ports. The board’s open-source repository includes 14 designs: 'reference' projects to be used as a first step to building customized NICs, switches, routers, etc., as well as community member contributed projects. The NetFPGA software ecosystem, including the device drivers, is unfortunately Linux-centric, with no support for FreeBSD.

This project will enable using NetFPGA SUME as a 4x10 Gbps NIC on hosts running FreeBSD. There are already FreeBSD drivers for older (now obsolete) versions of NetFPGA 10G (if_nf10bmac(4)) and NetFPGA 1G (https://github.com/wkoszek/freebsd_netfpga), both incompatible with the SUME board but could provide some valuable insight.

By researching iflib(9) framework and by consulting with my mentors, I have come to the conclusion that iflib is not the right tool for this task. NetFPGA SUME is a network device with 4 physical 10G interfaces connected to the host with only one PCI endpoint. The existing SUME Linux driver allocates four netdevs, essentially creating four different 10G 'virtual' interfaces in the OS. It then multiplexes packets based on the header meta-data generated by hardware. It seems that iflib cannot provide neccessary functionalities needed for NetFPGA SUME hardware without additional hacking, so I will completely ditch the idea and create the device driver without iflib.

Writing a device driver on top of the iflib would inherently provide support for netmap(4), but by writing a standalone driver, adding Netmap support will have to be done differently. Netmap support will enable high-speed traffic exchange between (custom) NetFPGA designs and userspace network applications, paving the way for experimentation with new hardware / software network processing paradigms.

## Deliverables
 - NetFPGA SUME driver skeleton with basic PIO register communication
 - NetFPGA SUME reference NIC device driver for FreeBSD with basic NIC functionalities (sending/receiving packets via DMA)
 - netmap(4) integration with the developed device driver

## Milestones and test plan
  - June 1st: Start of coding
    - Start writing a generic driver for basic communication between the OS and NetFPGA, mostly to learn more about the process of kernel driver programming and inner workings of host-NetFPGA communication (device interrupts, PCIe PIO and DMA). This will serve as a base for the next step. Improve the driver communication with FPGA registers, enable reading/writing to/from specific memory locations, gain control over DMA transfers. NetFPGA register communication is enabled.
  - July 3rd: Mid-term Evaluations #1
    - Enable sending/receiving network packets to/from NetFPGA/OS. Implement interrupt handlers. Start integrating Netmap specifics to the driver.
  - July 31st: Mid-term Evaluations #2
    - Full Netmap integration. Testing with different Netmap applications (for sending/receiving traffic). This will almost certainly reveal bugs in the driver so fixing them will inevitably take away some time. Driver optimizations for better traffic throughput. Last minute minor fixes and finishing touches. Documentation finishing up.
  - August 31st: Final Evaluations

## What works?
  - Driver loads.
  - Driver initializes DMA segments, 4 network interfaces.
  - Simple read/write operations from/to SUME specific memory regions (registers).

## What doesn't work (yet)?
  - Interrupt driven read/write operations.
  - Resource locking.
  - Network packets RX/TX.
  - Netmap integration.
