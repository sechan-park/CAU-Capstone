# CAU-Capstone
> **Target Board**: PYNQ-Z2 (Zynq-7000)  
> **Protocol**: AXI4-Lite (Control) + AXI4-Full (Memory)

## File Hierarchy
```
.
├── README.md
├── python
│   ├── CAU_Capstone_Baseline.ipynb
│   └── CAU_Capstone_ESG.ipynb
└── rtl
    ├── Baseline
    │   ├── axi
    │   │   ├── axi_dma_ctrl.sv
    │   │   ├── dma_read.sv
    │   │   ├── dma_write.sv
    │   │   ├── sa_engine_ip_v1_0_M00_AXI.v
    │   │   └── sa_engine_ip_v1_0_S00_AXI.v
    │   ├── core
    │   │   ├── sa_core.sv
    │   │   ├── sa_core_pipeline.sv
    │   │   └── sa_engine_top.sv
    │   ├── mem
    │   │   └── dpram_wrapper.sv
    │   ├── pe
    │   │   ├── hPE.sv
    │   │   ├── sa_controller.sv
    │   │   ├── sa_PE_wrapper.sv
    │   │   ├── sa_RF.sv
    │   │   ├── sa_unit.sv
    │   │   └── X_REG.sv
    │   └── top
    │       └── sa_engine_ip_v1_0.v
    └── ESG
        ├── axi
        │   ├── axi_addr_gen.sv
        │   ├── axi_dma_ctrl.sv
        │   ├── dma_read.sv
        │   ├── dma_write.sv
        │   ├── sa_engine_ip_v1_0_M00_AXI.v
        │   └── sa_engine_ip_v1_0_S00_AXI.v
        ├── core
        │   ├── sa_core.sv
        │   ├── sa_engine_top.sv
        │   ├── tile_compute.sv
        │   ├── tile_controller.sv
        │   ├── tile_loader.sv
        │   └── tile_store.sv
        ├── include
        │   ├── addr_map.svh
        │   └── sa_defs.svh
        ├── mem
        │   ├── bram_pingpong.sv
        │   └── dpram_wrapper.sv
        ├── pe
        │   ├── hPE.sv
        │   ├── pe_array_8x8.sv
        │   ├── pe_int8_dsp.sv
        │   ├── pe_int8_lut.sv
        │   ├── sa_controller.sv
        │   ├── sa_PE_wrapper.sv
        │   ├── sa_RF.sv
        │   ├── sa_unit.sv
        │   └── X_REG.sv
        ├── pkg
        │   ├── axi_regs_pkg.sv
        │   └── sa_params_pkg.sv
        └── top
            └── sa_engine_ip_v1_0.v
```
