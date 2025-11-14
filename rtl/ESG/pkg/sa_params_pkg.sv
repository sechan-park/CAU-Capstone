//----------------------------------------------------------------+
// Project: Capstone Design for graduation
// Module: sa_params_pkg (SystemVerilog package file)
// Description:
//		Package for system parameters
//
// Last Updated: 2025-11-06 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns/1ps

package sa_params_pkg;

  // ---------------------------------------------------------------------------
  // Base parameters (override via module parameters if needed)
  // ---------------------------------------------------------------------------
  parameter int SIDE            = 8;    // systolic array side length
  parameter int TILE_SIZE       = 8;    // tile dimension (rows/cols)
  parameter int BLOCK_M         = 64;   // column block span (in elements)

  parameter int ELEM_BITS       = 8;    // input element width (INT8)
  parameter int ACC_BITS        = 32;   // accumulator/output width (INT32)

  parameter int AXI_ADDR_WIDTH  = 32;   // AXI address width
  parameter int AXI_DATA_WIDTH  = 32;   // AXI data width (bits per beat)

  parameter bit LITTLE_ENDIAN   = 1;    // 1: little-endian lane order
  parameter bit USE_DSP         = 0;    // 0: LUT PE, 1: DSP PE (for synthesis)

  // ---------------------------------------------------------------------------
  // Derived constants
  // ---------------------------------------------------------------------------
  localparam int BYTES_PER_ELEM   = (ELEM_BITS + 7) / 8;     // 1 for INT8
  localparam int BYTES_PER_BEAT   = AXI_DATA_WIDTH / 8;      // 4 for 32b AXI
  localparam int ELEMS_PER_BEAT   = (BYTES_PER_ELEM > 0) ? (BYTES_PER_BEAT / BYTES_PER_ELEM) : 0;

  localparam int TILE_ELEMS       = TILE_SIZE * TILE_SIZE;   // e.g., 64 for 8x8
  localparam int A_TILE_BYTES     = TILE_ELEMS * BYTES_PER_ELEM;   // 64B (INT8)
  localparam int B_TILE_BYTES     = TILE_ELEMS * BYTES_PER_ELEM;   // 64B (INT8)
  localparam int C_TILE_BYTES     = TILE_ELEMS * (ACC_BITS / 8);   // 256B (INT32)

  localparam int MAX_BURST_BEATS  = 16;                      // AXI4 typical max
  localparam int BYTES_PER_BURST  = BYTES_PER_BEAT * MAX_BURST_BEATS; // 64B @32b AXI
  localparam int ALIGN_MASK       = BYTES_PER_BEAT - 1;      // 4B alignment mask

  // Systolic array pipeline latency without load/store (approximate)
  localparam int SA_PIPE_LAT      = (2 * SIDE) - 1;

  // ---------------------------------------------------------------------------
  // Helpers
  // ---------------------------------------------------------------------------
  function automatic int ceil_div(input int a, input int b);
    if (b <= 0) return 0;
    return (a + b - 1) / b;
  endfunction

  function automatic int round_up(input int val, input int align);
    if (align <= 0) return val;
    return ((val + align - 1) / align) * align;
  endfunction

endpackage : sa_params_pkg
