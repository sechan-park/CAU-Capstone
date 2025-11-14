//----------------------------------------------------------------+
// Project: Capstone Design
// Module : pe_int8_lut (SystemVerilog module)
// Description:
//   INT8xINT8->INT32 PE (LUT-based implementation).
//   - Signed int8 operands (two's complement) with 32-bit accumulation.
//   - Single-cycle MADD when en=1: acc_out <= acc_in + a*b.
//
// Last Updated: 2025-11-10 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns/1ps

module pe_int8_lut #(
  parameter int ELEM_BITS = 8,
  parameter int ACC_BITS  = 32
)(
  input  logic                         clk,
  input  logic                         rstn,
  input  logic                         clr,       // clear accumulator
  input  logic                         shift_en,  // shift/forward enable
  input  logic signed [ELEM_BITS-1:0]  a_in,
  input  logic signed [ELEM_BITS-1:0]  b_in,
  input  logic                         a_v_in,
  input  logic                         b_v_in,
  output logic signed [ELEM_BITS-1:0]  a_out,
  output logic signed [ELEM_BITS-1:0]  b_out,
  output logic                         a_v_out,
  output logic                         b_v_out,
  output logic        [ACC_BITS-1:0]   c_out
);

  // Combinational product uses current-cycle inputs
  logic signed [2*ELEM_BITS-1:0] prod_s;
  assign prod_s = a_in * b_in;

  // Shift/forward and accumulate
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      a_out <= '0; b_out <= '0; a_v_out <= 1'b0; b_v_out <= 1'b0; c_out <= '0;
    end else begin
      // accumulate first (uses current inputs and valids)
      if (clr) c_out <= '0;
      else if (a_v_in && b_v_in) c_out <= $signed(c_out) + $signed({{(ACC_BITS-(2*ELEM_BITS)){prod_s[2*ELEM_BITS-1]}}, prod_s});

      // one-cycle forward path for data/valid
      if (shift_en) begin
        a_out <= a_in; b_out <= b_in; a_v_out <= a_v_in; b_v_out <= b_v_in;
      end
    end
  end

endmodule
