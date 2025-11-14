//----------------------------------------------------------------+
// Project: Capstone Design for graduation
// Module: axi_regs_pkg (SystemVerilog package file)
// Description:
//		Package for AXI registers
//
// Last Updated: 2025-11-06 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns/1ps

package axi_regs_pkg;

  // Include address/bit positions
  `include "addr_map.svh"

  // ---------------------------------------------------------------------------
  // Mask constants for control & status (derived from bit positions)
  // ---------------------------------------------------------------------------
  localparam logic [31:0] CTRL_START_MASK    = (32'h1 << `SA_CTRL_START_BIT);
  localparam logic [31:0] CTRL_UPDATE_A_MASK = (32'h1 << `SA_CTRL_UPDATE_A_BIT);
  localparam logic [31:0] CTRL_IRQ_EN_MASK   = (32'h1 << `SA_CTRL_IRQ_EN_BIT);

  localparam logic [31:0] STAT_BUSY_MASK     = (32'h1 << `SA_STAT_BUSY_BIT);
  localparam logic [31:0] STAT_DONE_MASK     = (32'h1 << `SA_STAT_DONE_BIT);
  localparam logic [31:0] STAT_ERROR_MASK    = (32'h1 << `SA_STAT_ERROR_BIT);

  // ---------------------------------------------------------------------------
  // Default values
  // ---------------------------------------------------------------------------
  localparam logic [31:0] CONTROL_RESET = 32'h0000_0000;
  localparam logic [31:0] STATUS_RESET  = 32'h0000_0000;

  localparam logic [31:0] DEFAULT_TILE_SIZE = `SA_DEF_TILE_SIZE;
  localparam logic [31:0] DEFAULT_BLOCK_M   = `SA_DEF_BLOCK_M;
  localparam logic [31:0] DEFAULT_N         = `SA_DEF_N;
  localparam logic [31:0] DEFAULT_K         = `SA_DEF_K;
  localparam logic [31:0] DEFAULT_M         = `SA_DEF_M;

  // ---------------------------------------------------------------------------
  // Address aliases (optional; use macros directly if preferred)
  // ---------------------------------------------------------------------------
  localparam logic [31:0] REG_CONTROL    = `SA_REG_CONTROL;
  localparam logic [31:0] REG_READ_BASE  = `SA_REG_READ_BASE;
  localparam logic [31:0] REG_WRITE_BASE = `SA_REG_WRITE_BASE;
  localparam logic [31:0] REG_N          = `SA_REG_N;
  localparam logic [31:0] REG_K          = `SA_REG_K;
  localparam logic [31:0] REG_M          = `SA_REG_M;
  localparam logic [31:0] REG_TILE_SIZE  = `SA_REG_TILE_SIZE;
  localparam logic [31:0] REG_BLOCK_M    = `SA_REG_BLOCK_M;
  localparam logic [31:0] REG_BASE_A     = `SA_REG_BASE_A;
  localparam logic [31:0] REG_BASE_B     = `SA_REG_BASE_B;
  localparam logic [31:0] REG_BASE_C     = `SA_REG_BASE_C;
  localparam logic [31:0] REG_STRIDE_A   = `SA_REG_STRIDE_A;
  localparam logic [31:0] REG_STRIDE_B   = `SA_REG_STRIDE_B;
  localparam logic [31:0] REG_STRIDE_C   = `SA_REG_STRIDE_C;

  // ---------------------------------------------------------------------------
  // Helpers: pack/unpack/check for control & status words
  // ---------------------------------------------------------------------------
  function automatic logic [31:0] control_pack(
      input logic start,
      input logic update_a,
      input logic irq_en
  );
    logic [31:0] w;
    w  = CONTROL_RESET;
    w |= start    ? CTRL_START_MASK    : 32'h0;
    w |= update_a ? CTRL_UPDATE_A_MASK : 32'h0;
    w |= irq_en   ? CTRL_IRQ_EN_MASK   : 32'h0;
    return w;
  endfunction

  function automatic logic [31:0] status_pack(
      input logic busy,
      input logic done,
      input logic error
  );
    logic [31:0] w;
    w  = STATUS_RESET;
    w |= busy  ? STAT_BUSY_MASK  : 32'h0;
    w |= done  ? STAT_DONE_MASK  : 32'h0;
    w |= error ? STAT_ERROR_MASK : 32'h0;
    return w;
  endfunction

  function automatic logic status_is_busy(input logic [31:0] w);
    return (w & STAT_BUSY_MASK)  != 32'h0;
  endfunction

  function automatic logic status_is_done(input logic [31:0] w);
    return (w & STAT_DONE_MASK)  != 32'h0;
  endfunction

  function automatic logic status_is_error(input logic [31:0] w);
    return (w & STAT_ERROR_MASK) != 32'h0;
  endfunction

endpackage : axi_regs_pkg

