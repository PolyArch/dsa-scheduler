# Stream Specialization Scheduler

## About

- This library contains two major components, the config library which acts as
   a hardware description language, and the scheduler library for describing and
   scheduling dataflow graphs (DFG files) to the hardware.
- The simulator for the dataflow graph is currently embedded within the scheduler library.

## Installation

### Prerequisites

This repository requires a Python Version.

### Building this repository

To build this repository, please follow the following steps:

First source the setup script from the main repo.

```bash
source ../setup.sh
```

Make the library by running:

```bash
make all
```

## Command Reference

All scheduling functions are found through `ss_sched` command.

To run the scheduler, mapping a given dfg file to an adg file, run:

```bash
ss_sched [adg] [dfg]
```

To run the Design Space Explorer, run the following command:
```bash
ss_sched [adg] [dfg] -x
```


For a full reference of available options, run:

```bash
ss_sched --help
```

ADG Rules
==================================

Nodes
-----

1. A spatial node's granularity must be [8, 16, 32, 64]

2. VectorPorts always have a granularity of 8 (byte-accessible).

3. A spatial node's datawidth must be a power of 2.

4. A vectorports datawidth is implicitly defined as the spatial links connected to it.

> Example:
> VectorPort A has three links connecting to Switches X, Y, and Z. Switches X, Y, and Z have respective widths of 32, 32, and 64.
> Thus VectorPort A has a width of 32 + 32 + 64 = 128.

5. A node's granularity must be smaller than the spatial nodes datawidth
  
6. The number of slots of a node is defined as Node DataWidth / Node Granularity.

Links
----

1. A links granularity is max(source granularity, sink granularity)

2. The DataWidth of a link between two spatial nodes is max(source DataWidth, sink DataWidth).

3. The DataWidth of a link between vectorport and spatial node is the spatial nodes DataWidth.

4. The number of slots on a link is defined as Link DataWidth / Link Granularity

Link Connectivity
----

1. Two Nodes can't have two links in same direction between them.

> Example:
>
> - Invalid: NodeA->NodeB and NodeA->NodeB
> - Valid: NodeA->NodeB and NodeB->NodeA
  
2. Two ADG Vector Ports can't have a link between each other.

3. An ADG Node can't have a recursive link.

4. A Spatial Node (Switch/PE) and Data Node (DMA/SPM) can't have a link between each other.

Mapping
----

1. Vertex slots must be mapped to even slots.

> Example:
>
> - Invalid:    | ___| OPA | OPA |___ |
> - Valid:      | OPA | OPA | ___|___ |
> - Valid:      | ___|___ | OPA | OPA |

VectorPort Mapping
----

1. Two DFG ports can not be mapped to a single VectorPort.

2. The stated dfg edge is always 8 bits wide.

3. A stated DFG port can not be mapped to a non-stated vectorport.

4. The stated dfg edge is always mapped to a stated VectorPort on bits [0-8]. The first link of a vector port only includes the stated edge.

5. The other links of the vectorport are statically assigned according to their value id. A InputPort value can not stradle two different links.

Routing
----

1. Memory DFG Edges, or those with either their source or destination vertex being a data node, can't utilize switches.

> Exampe:
>
> - Invalid: SPM0 -> SWITCH0 -> IVP0

2. A dfg edge entering a non-switch must come in at an even slot

> Example:
>
> - Invalid:    | ___| OPA | OPA |___ |
> - Valid:      | OPA | OPA | ___|___ |
> - Valid:      | ___|___ | OPA | OPA |

3. A switch can map to any contiguous slot.

> Example:
>
> - Valid:      | ___| OPA | OPA |___ |
> - Valid:      | OPA | OPA | ___|___ |
> - Valid:      | OPA | ___|___ | OPA |
> - Invalid:    | OPA | ___| OPA |___ |

4. A lower bitwidth edge must always be mapped to the lower bits of a granularity.

> Example:
> A 16 bit edge mapped to a Node with granularity 32 and datawidth 64 bit granularity
>
> - Valid: Mapping edge to bits: [0:16] or [32:48]
> - Invalid: Mapping edge to bits: [16:32] or [48:64]

---
