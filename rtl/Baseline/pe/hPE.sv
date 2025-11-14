//----------------------------------------------------------------+
// Project: Capstone Design for graduation
// Module: hPE (SystemVerilog version)
// Description:
//		PE for systolic array
// Last Updated: 2025-10-21 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns / 1ps

module hPE (
    input logic RST,
    input logic CLK,
    input logic EN,
    input logic signed[7:0] A,
    input logic signed[7:0] B,
    input logic A_valid_in,
    input logic B_valid_in,
    output logic signed[7:0] A_out,
    output logic signed[7:0] B_out,
    output logic A_valid_out,
    output logic B_valid_out,
    output logic C_valid_out,
    output logic signed [18:0] C
);

  logic signed [15:0] mult_s;
  (* use_dsp = "yes" *) logic signed [18:0] acc_q;

  // Combinational logic for multiplication
  always_comb begin
    mult_s = $signed(A_data) * $signed(B_data);
  end

  // Parameters
  localparam int K_DIM = 8; // dot-product length

  // Internal signals
  logic signed [7:0] A_data;
  logic signed [7:0] B_data;
  logic       A_valid;
  logic       B_valid;
  logic signed [18:0] ACC;
  logic       acc_en;
  logic [3:0] k_cnt;
  logic       cvalid_d;

  // Accumulate enable based on registered valids
  always_comb begin
    acc_en = A_valid && B_valid;
  end

  // Main logic
  always_ff @(posedge CLK) begin
    if (!RST) begin
      ACC      <= 0;
      A_data   <= 0;
      B_data   <= 0;
      A_valid  <= 0;
      B_valid  <= 0;
      k_cnt    <= '0;
      cvalid_d <= 1'b0;
    end else if (EN) begin
      // register inputs down the pipeline
      A_data  <= A;
      B_data  <= B;
      A_valid <= A_valid_in;
      B_valid <= B_valid_in;

      // accumulate only when both inputs are valid in the same cycle
      if (acc_en) begin
        ACC   <= (k_cnt == 0) ? mult_s : (ACC + mult_s);
        k_cnt <= (k_cnt == K_DIM-1) ? '0 : (k_cnt + 1'b1);
      end

      // assert C_valid one cycle after the last accumulation of a dot-product
      cvalid_d <= acc_en && (k_cnt == K_DIM-1);
    end
  end

  // Output assignments
  always_comb begin
    A_out        = A_data;
    B_out        = B_data;
    A_valid_out  = A_valid;
    B_valid_out  = B_valid;
    C            = ACC;
    C_valid_out  = cvalid_d;
  end

endmodule

