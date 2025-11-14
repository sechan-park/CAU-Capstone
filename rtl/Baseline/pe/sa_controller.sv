//----------------------------------------------------------------+
// Project: Capstone Design for graduation
// Module: sa_controller (SystemVerilog version)
// Description:
//		Controller for 8x8 Systolic Array
//
// Last Updated: 2025-10-21 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns / 1ps
module sa_controller (
    input logic RST,
    input logic CLK,
    input logic EN,  // EN only for controller --> disable when changing inputs before load or read!
    input logic WRITE,  // WRITE TO INT_MEM
    input logic [2:0] IDX,  // index of the element
    input logic [3:0] REG_SELECT,  // which register
    input logic LOAD,  // Load to REGISTER FILE
    input logic [7:0] DATA_IN,
    output logic DATA_OUTPUT_EN,  // Enable output
    output logic [18:0] DATA_OUT,
    output logic        B_load_done_o, // when the last write operation for Matrix B is done (from dpram to controller)
    output logic        register_load_done_o,  // when the last write operation for registers is done (from MAT registers to X_REG)
    output logic        A_load_done_o, // when the last write operation for Matrix A is done (from dpram to controller)
    output logic        matmul_done_o // when the matmul is done
);
  ///////////////////////////////////////////////////////////

  //                      SYSTOLIC ARRAY

  // reg [4:0]   IDX = 0;

  reg [7:0] DATA_0 = 0;
  reg [7:0] DATA_1 = 0;
  reg [7:0] DATA_2 = 0;
  reg [7:0] DATA_3 = 0;
  reg [7:0] DATA_4 = 0;
  reg [7:0] DATA_5 = 0;
  reg [7:0] DATA_6 = 0;
  reg [7:0] DATA_7 = 0;
  reg [7:0] DATA_8 = 0;
  reg [7:0] DATA_9 = 0;
  reg [7:0] DATA_10 = 0;
  reg [7:0] DATA_11 = 0;
  reg [7:0] DATA_12 = 0;
  reg [7:0] DATA_13 = 0;
  reg [7:0] DATA_14 = 0;
  reg [7:0] DATA_15 = 0;


  /////////////////////////////////////////////////////////////

  //                  CONTROLLER


  reg [18:0] DATA_OUT_BUFFER = 0;

  // Matrix 1
  reg [7:0] MAT_A0[0:7];
  reg [7:0] MAT_A1[0:7];
  reg [7:0] MAT_A2[0:7];
  reg [7:0] MAT_A3[0:7];
  reg [7:0] MAT_A4[0:7];
  reg [7:0] MAT_A5[0:7];
  reg [7:0] MAT_A6[0:7];
  reg [7:0] MAT_A7[0:7];


  // Matrix 2
  reg [7:0] MAT_B0[0:7];
  reg [7:0] MAT_B1[0:7];
  reg [7:0] MAT_B2[0:7];
  reg [7:0] MAT_B3[0:7];
  reg [7:0] MAT_B4[0:7];
  reg [7:0] MAT_B5[0:7];
  reg [7:0] MAT_B6[0:7];
  reg [7:0] MAT_B7[0:7];


  // Result Matrix 8*8
  reg [18:0] MAT_C0[0:7];
  reg [18:0] MAT_C1[0:7];
  reg [18:0] MAT_C2[0:7];
  reg [18:0] MAT_C3[0:7];
  reg [18:0] MAT_C4[0:7];
  reg [18:0] MAT_C5[0:7];
  reg [18:0] MAT_C6[0:7];
  reg [18:0] MAT_C7[0:7];


  reg [2:0] IDX_COUNTER = 0;
  reg [2:0] IDX_IN_BUFFER = 0;
  reg [7:0] DATA_IN_BUFFER = 0;

  reg SA_EN = 0;
  reg SA_WRITE = 0;
  reg OUTPUT_EN = 0;

  logic B_load_done_pulse;
  assign B_load_done_o = B_load_done_pulse;

  logic A_load_done_pulse;
  assign A_load_done_o = A_load_done_pulse;

  logic register_load_done_pulse;
  assign register_load_done_o = register_load_done_pulse;

  // valid outputs from systolic array - only need the last one
  logic matmul_done_pulse;
  assign matmul_done_o = matmul_done_pulse;

  // Lock further writes after completion to avoid overwriting banks
  logic a_locked;
  logic b_locked;

  always_ff @(posedge CLK) begin
    if (!RST) begin
      B_load_done_pulse <= 1'b0;
      A_load_done_pulse <= 1'b0;
      register_load_done_pulse <= 1'b0;
      a_locked <= 1'b0;
      b_locked <= 1'b0;
    end else begin

      if (EN) begin
        // default pulses low (one-cycle pulses)
        B_load_done_pulse <= 1'b0;
        A_load_done_pulse <= 1'b0;
        register_load_done_pulse <= 1'b0;
        // Always connect results register!
        if (LOAD) begin
          // Load to registers (constant addresses)
          if (!WRITE) begin
            SA_EN <= 1;
            // SA_WRITE is the 'valid' signal generator now.
            // It should be high only when we are feeding valid data.
            if (IDX_COUNTER < 8) begin
              SA_WRITE <= 1;
            end else begin
              SA_WRITE <= 0;
            end
            OUTPUT_EN <= 0;

            DATA_0 <= MAT_A0[IDX_COUNTER];
            DATA_1 <= MAT_A1[IDX_COUNTER];
            DATA_2 <= MAT_A2[IDX_COUNTER];
            DATA_3 <= MAT_A3[IDX_COUNTER];
            DATA_4 <= MAT_A4[IDX_COUNTER];
            DATA_5 <= MAT_A5[IDX_COUNTER];
            DATA_6 <= MAT_A6[IDX_COUNTER];
            DATA_7 <= MAT_A7[IDX_COUNTER];

            DATA_8 <= MAT_B0[IDX_COUNTER];
            DATA_9 <= MAT_B1[IDX_COUNTER];
            DATA_10 <= MAT_B2[IDX_COUNTER];
            DATA_11 <= MAT_B3[IDX_COUNTER];
            DATA_12 <= MAT_B4[IDX_COUNTER];
            DATA_13 <= MAT_B5[IDX_COUNTER];
            DATA_14 <= MAT_B6[IDX_COUNTER];
            DATA_15 <= MAT_B7[IDX_COUNTER];

            if (IDX_COUNTER == 7) begin
              register_load_done_pulse <= 1'b1;
            end

            IDX_COUNTER   <= IDX_COUNTER + 1;
            IDX_IN_BUFFER <= IDX_COUNTER;
            // clear write locks when entering LOAD-read path
            a_locked <= 1'b0;
            b_locked <= 1'b0;
          end  // LOAD && WRITE == READ!
          else if (WRITE) begin
            SA_EN <= 0;
            SA_WRITE <= 0;
            OUTPUT_EN <= 1;

            case (REG_SELECT)
              0: DATA_OUT_BUFFER <= MAT_C0[IDX];
              1: DATA_OUT_BUFFER <= MAT_C1[IDX];
              2: DATA_OUT_BUFFER <= MAT_C2[IDX];
              3: DATA_OUT_BUFFER <= MAT_C3[IDX];
              4: DATA_OUT_BUFFER <= MAT_C4[IDX];
              5: DATA_OUT_BUFFER <= MAT_C5[IDX];
              6: DATA_OUT_BUFFER <= MAT_C6[IDX];
              7: DATA_OUT_BUFFER <= MAT_C7[IDX];
            endcase
          end
        end  // If not loading, then 
        else if (!LOAD) begin

          // Writing to Seqeunce Registers (outside the systolic array)
          // can safely turn SA off; for now.
          if (WRITE) begin
            SA_WRITE <= 0;
            SA_EN <= 0;
            OUTPUT_EN <= 0;

            // Detect the last write for Matrix A
            if (REG_SELECT == 4'h7 && IDX == 3'h7) begin
              A_load_done_pulse <= 1'b1; // pulse
              a_locked <= 1'b1;          // lock A banks against further writes
            end

            // Detect the last write operation for the load sequence (Matrix B)
            if (REG_SELECT == 4'hF && IDX == 3'h7) begin
              B_load_done_pulse <= 1'b1; // pulse
              b_locked <= 1'b1;          // lock B banks against further writes
            end

            case (REG_SELECT)
              4'd0: if (!a_locked) MAT_A0[IDX] <= DATA_IN;
              4'd1: if (!a_locked) MAT_A1[IDX] <= DATA_IN;
              4'd2: if (!a_locked) MAT_A2[IDX] <= DATA_IN;
              4'd3: if (!a_locked) MAT_A3[IDX] <= DATA_IN;
              4'd4: if (!a_locked) MAT_A4[IDX] <= DATA_IN;
              4'd5: if (!a_locked) MAT_A5[IDX] <= DATA_IN;
              4'd6: if (!a_locked) MAT_A6[IDX] <= DATA_IN;
              4'd7: if (!a_locked) MAT_A7[IDX] <= DATA_IN;
              4'd8: if (!b_locked) MAT_B0[IDX] <= DATA_IN;
              4'd9: if (!b_locked) MAT_B1[IDX] <= DATA_IN;
              4'hA: if (!b_locked) MAT_B2[IDX] <= DATA_IN;
              4'hB: if (!b_locked) MAT_B3[IDX] <= DATA_IN;
              4'hC: if (!b_locked) MAT_B4[IDX] <= DATA_IN;
              4'hD: if (!b_locked) MAT_B5[IDX] <= DATA_IN;
              4'hE: if (!b_locked) MAT_B6[IDX] <= DATA_IN;
              4'hF: if (!b_locked) MAT_B7[IDX] <= DATA_IN;
            endcase
          end  // DO MATMUL IF NOT LOADING AND WRITING (IE IDLE!)
          else if (!WRITE) begin
            SA_WRITE <= 0;
            SA_EN <= 1;
            OUTPUT_EN <= 0;
          end
        end
      end
    end
  end

  ///////////////////////////////////////////////////////////////
  // Only when !WRITE (!SA_WRITE), will the matmul begin
  ///////////////////////////////////////////////////////////////

  sa_unit u_sa_unit (
      .RST(RST),
      .CLK(CLK),
      .EN(SA_EN),
      .WRITE(SA_WRITE),
      .IDX(IDX_IN_BUFFER),

      // Inputs to registers
      .DIN_0 (DATA_0),
      .DIN_1 (DATA_1),
      .DIN_2 (DATA_2),
      .DIN_3 (DATA_3),
      .DIN_4 (DATA_4),
      .DIN_5 (DATA_5),
      .DIN_6 (DATA_6),
      .DIN_7 (DATA_7),
      .DIN_8 (DATA_8),
      .DIN_9 (DATA_9),
      .DIN_10(DATA_10),
      .DIN_11(DATA_11),
      .DIN_12(DATA_12),
      .DIN_13(DATA_13),
      .DIN_14(DATA_14),
      .DIN_15(DATA_15),

      // Systolic Array Outputs
      .Y_00(MAT_C0[0]), .Y_10(MAT_C0[1]), .Y_20(MAT_C0[2]), .Y_30(MAT_C0[3]), .Y_40(MAT_C0[4]), .Y_50(MAT_C0[5]), .Y_60(MAT_C0[6]), .Y_70(MAT_C0[7]),
      .Y_01(MAT_C1[0]), .Y_11(MAT_C1[1]), .Y_21(MAT_C1[2]), .Y_31(MAT_C1[3]), .Y_41(MAT_C1[4]), .Y_51(MAT_C1[5]), .Y_61(MAT_C1[6]), .Y_71(MAT_C1[7]),
      .Y_02(MAT_C2[0]), .Y_12(MAT_C2[1]), .Y_22(MAT_C2[2]), .Y_32(MAT_C2[3]), .Y_42(MAT_C2[4]), .Y_52(MAT_C2[5]), .Y_62(MAT_C2[6]), .Y_72(MAT_C2[7]),
      .Y_03(MAT_C3[0]), .Y_13(MAT_C3[1]), .Y_23(MAT_C3[2]), .Y_33(MAT_C3[3]), .Y_43(MAT_C3[4]), .Y_53(MAT_C3[5]), .Y_63(MAT_C3[6]), .Y_73(MAT_C3[7]),
      .Y_04(MAT_C4[0]), .Y_14(MAT_C4[1]), .Y_24(MAT_C4[2]), .Y_34(MAT_C4[3]), .Y_44(MAT_C4[4]), .Y_54(MAT_C4[5]), .Y_64(MAT_C4[6]), .Y_74(MAT_C4[7]),
      .Y_05(MAT_C5[0]), .Y_15(MAT_C5[1]), .Y_25(MAT_C5[2]), .Y_35(MAT_C5[3]), .Y_45(MAT_C5[4]), .Y_55(MAT_C5[5]), .Y_65(MAT_C5[6]), .Y_75(MAT_C5[7]),
      .Y_06(MAT_C6[0]), .Y_16(MAT_C6[1]), .Y_26(MAT_C6[2]), .Y_36(MAT_C6[3]), .Y_46(MAT_C6[4]), .Y_56(MAT_C6[5]), .Y_66(MAT_C6[6]), .Y_76(MAT_C6[7]),
      .Y_07(MAT_C7[0]), .Y_17(MAT_C7[1]), .Y_27(MAT_C7[2]), .Y_37(MAT_C7[3]), .Y_47(MAT_C7[4]), .Y_57(MAT_C7[5]), .Y_67(MAT_C7[6]), .Y_77(MAT_C7[7]),

      .Y_77_valid_out(matmul_done_pulse)
  );

  assign DATA_OUT = DATA_OUT_BUFFER;
  assign DATA_OUTPUT_EN = OUTPUT_EN;

endmodule
