//----------------------------------------------------------------+
// Project: Capstone Design for graduation
// Module: sa_RF (SystemVerilog version)
// Description:
//		Register File for 8x8 Systolic Array
//
// Last Updated: 2025-10-21 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns / 1ps

// RegisterFile & Internal Controller

// Assumption:
//      The SW part already transposed the weight matrix.
//      
//      Future Work: TRANSPOSER.sv

module sa_RF(
    input logic CLK,
    input logic RST,
    input logic RF_EN,
    input logic WRITE,
    input logic [2:0] IDX,
    
    input logic [7:0] DATA_IN_0,
    input logic [7:0] DATA_IN_1,
    input logic [7:0] DATA_IN_2,
    input logic [7:0] DATA_IN_3,
    input logic [7:0] DATA_IN_4,
    input logic [7:0] DATA_IN_5,
    input logic [7:0] DATA_IN_6,
    input logic [7:0] DATA_IN_7,
    input logic [7:0] DATA_IN_8,
    input logic [7:0] DATA_IN_9,
    input logic [7:0] DATA_IN_A,
    input logic [7:0] DATA_IN_B,
    input logic [7:0] DATA_IN_C,
    input logic [7:0] DATA_IN_D,
    input logic [7:0] DATA_IN_E,
    input logic [7:0] DATA_IN_F,

    output logic [7:0] X_OUT [0:7],
    output logic [7:0] W_OUT [0:7],
    output logic X_VALID_OUT [0:7],
    output logic W_VALID_OUT [0:7]
);

    
    // Registers for A Matrix
    X_REG X_REG_0(.CLK(CLK), .RST(RST), .EN(RF_EN), .WRITE(WRITE),
        .IDX({2'b0,IDX}), .DIN(DATA_IN_0), .DOUT(X_OUT[0]), .VALID_IN(WRITE), .VALID_OUT(X_VALID_OUT[0]));

    X_REG X_REG_1(.CLK(CLK), .RST(RST), .EN(RF_EN), .WRITE(WRITE),
        .IDX({2'b0,IDX}+1), .DIN(DATA_IN_1), .DOUT(X_OUT[1]), .VALID_IN(WRITE), .VALID_OUT(X_VALID_OUT[1]));

    X_REG X_REG_2(.CLK(CLK), .RST(RST), .EN(RF_EN), .WRITE(WRITE),
        .IDX({2'b0,IDX}+2), .DIN(DATA_IN_2), .DOUT(X_OUT[2]), .VALID_IN(WRITE), .VALID_OUT(X_VALID_OUT[2]));

    X_REG X_REG_3(.CLK(CLK), .RST(RST), .EN(RF_EN), .WRITE(WRITE),
        .IDX({2'b0,IDX}+3), .DIN(DATA_IN_3), .DOUT(X_OUT[3]), .VALID_IN(WRITE), .VALID_OUT(X_VALID_OUT[3]));

    X_REG X_REG_4(.CLK(CLK), .RST(RST), .EN(RF_EN), .WRITE(WRITE),
        .IDX({2'b0,IDX}+4), .DIN(DATA_IN_4), .DOUT(X_OUT[4]), .VALID_IN(WRITE), .VALID_OUT(X_VALID_OUT[4]));

    X_REG X_REG_5(.CLK(CLK), .RST(RST), .EN(RF_EN), .WRITE(WRITE),
        .IDX({2'b0,IDX}+5), .DIN(DATA_IN_5), .DOUT(X_OUT[5]), .VALID_IN(WRITE), .VALID_OUT(X_VALID_OUT[5]));

    X_REG X_REG_6(.CLK(CLK), .RST(RST), .EN(RF_EN), .WRITE(WRITE),
        .IDX({2'b0,IDX}+6), .DIN(DATA_IN_6), .DOUT(X_OUT[6]), .VALID_IN(WRITE), .VALID_OUT(X_VALID_OUT[6]));

    X_REG X_REG_7(.CLK(CLK), .RST(RST), .EN(RF_EN), .WRITE(WRITE),
        .IDX({2'b0,IDX}+7), .DIN(DATA_IN_7), .DOUT(X_OUT[7]), .VALID_IN(WRITE), .VALID_OUT(X_VALID_OUT[7]));



    // Registers for B Matrix    
    X_REG X_REG_8(.CLK(CLK), .RST(RST), .EN(RF_EN), .WRITE(WRITE),
        .IDX({2'b0,IDX}), .DIN(DATA_IN_8), .DOUT(W_OUT[0]), .VALID_IN(WRITE), .VALID_OUT(W_VALID_OUT[0]));

    X_REG X_REG_9(.CLK(CLK), .RST(RST), .EN(RF_EN), .WRITE(WRITE),
        .IDX({2'b0,IDX}+1), .DIN(DATA_IN_9), .DOUT(W_OUT[1]), .VALID_IN(WRITE), .VALID_OUT(W_VALID_OUT[1]));

    X_REG X_REG_A(.CLK(CLK), .RST(RST), .EN(RF_EN), .WRITE(WRITE),
        .IDX({2'b0,IDX}+2), .DIN(DATA_IN_A), .DOUT(W_OUT[2]), .VALID_IN(WRITE), .VALID_OUT(W_VALID_OUT[2]));

    X_REG X_REG_B(.CLK(CLK), .RST(RST), .EN(RF_EN), .WRITE(WRITE),
        .IDX({2'b0,IDX}+3), .DIN(DATA_IN_B), .DOUT(W_OUT[3]), .VALID_IN(WRITE), .VALID_OUT(W_VALID_OUT[3]));

    X_REG X_REG_C(.CLK(CLK), .RST(RST), .EN(RF_EN), .WRITE(WRITE),
        .IDX({2'b0,IDX}+4), .DIN(DATA_IN_C), .DOUT(W_OUT[4]), .VALID_IN(WRITE), .VALID_OUT(W_VALID_OUT[4]));

    X_REG X_REG_D(.CLK(CLK), .RST(RST), .EN(RF_EN), .WRITE(WRITE),
        .IDX({2'b0,IDX}+5), .DIN(DATA_IN_D), .DOUT(W_OUT[5]), .VALID_IN(WRITE), .VALID_OUT(W_VALID_OUT[5]));

    X_REG X_REG_E(.CLK(CLK), .RST(RST), .EN(RF_EN), .WRITE(WRITE),
        .IDX({2'b0,IDX}+6), .DIN(DATA_IN_E), .DOUT(W_OUT[6]), .VALID_IN(WRITE), .VALID_OUT(W_VALID_OUT[6]));

    X_REG X_REG_F(.CLK(CLK), .RST(RST), .EN(RF_EN), .WRITE(WRITE),
        .IDX({2'b0,IDX}+7), .DIN(DATA_IN_F), .DOUT(W_OUT[7]), .VALID_IN(WRITE), .VALID_OUT(W_VALID_OUT[7]));


endmodule

