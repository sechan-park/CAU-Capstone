//----------------------------------------------------------------+
// Project: Capstone Design for graduation
// Module: addr_map (SystemVerilog header file)
// Description:
//		Address map for the system
//
// Last Updated: 2025-11-06 (by Jimin Hwang)
//----------------------------------------------------------------+
`ifndef SA_ADDR_MAP_SVH
`define SA_ADDR_MAP_SVH

//------------------------------------------------------------------------------
// Legacy-compatible base map (do NOT break current TB)
//------------------------------------------------------------------------------
// Note:
//   - The current testbench writes START at 0x00, READ_BASE at 0x04,
//     and WRITE_BASE at 0x08, and polls DONE from 0x00.
//   - To preserve compatibility, we keep these offsets as-is and place
//     new controls at higher addresses.

// AXI-Lite register offsets (byte addresses)
`define SA_REG_CONTROL         32'h0000_0000  // CTRL write, STATUS read (shared reg0)
`define SA_REG_READ_BASE       32'h0000_0004  // legacy READ base address
`define SA_REG_WRITE_BASE      32'h0000_0008  // legacy WRITE base address

// Extended controls (start at higher offsets to avoid collision)
`define SA_REG_N               32'h0000_0010
`define SA_REG_K               32'h0000_0014
`define SA_REG_M               32'h0000_0018
`define SA_REG_TILE_SIZE       32'h0000_001C
`define SA_REG_BLOCK_M         32'h0000_0020
`define SA_REG_BASE_A          32'h0000_0024
`define SA_REG_BASE_B          32'h0000_0028
`define SA_REG_BASE_C          32'h0000_002C
`define SA_REG_STRIDE_A        32'h0000_0030
`define SA_REG_STRIDE_B        32'h0000_0034
`define SA_REG_STRIDE_C        32'h0000_0038
// Optional perf/debug counters
`define SA_REG_PERF0           32'h0000_003C
`define SA_REG_PERF1           32'h0000_0040

//------------------------------------------------------------------------------
// Control/Status bitfields (reg0 interpretation)
//------------------------------------------------------------------------------
// CTRL bits (write)
`define SA_CTRL_START_BIT      0
`define SA_CTRL_UPDATE_A_BIT   1
`define SA_CTRL_IRQ_EN_BIT     2

`define SA_CTRL_START          (32'h1 << `SA_CTRL_START_BIT)
`define SA_CTRL_UPDATE_A       (32'h1 << `SA_CTRL_UPDATE_A_BIT)
`define SA_CTRL_IRQ_EN         (32'h1 << `SA_CTRL_IRQ_EN_BIT)

// STATUS bits (read)
`define SA_STAT_BUSY_BIT       0
`define SA_STAT_DONE_BIT       1
`define SA_STAT_ERROR_BIT      2

`define SA_STAT_BUSY           (32'h1 << `SA_STAT_BUSY_BIT)
`define SA_STAT_DONE           (32'h1 << `SA_STAT_DONE_BIT)
`define SA_STAT_ERROR          (32'h1 << `SA_STAT_ERROR_BIT)

//------------------------------------------------------------------------------
// Defaults (may be mirrored into S00_AXI reset values as needed)
//------------------------------------------------------------------------------
`define SA_DEF_TILE_SIZE       32'd8
`define SA_DEF_BLOCK_M         32'd64
`define SA_DEF_N               32'd8
`define SA_DEF_K               32'd8
`define SA_DEF_M               32'd8

`endif // SA_ADDR_MAP_SVH

