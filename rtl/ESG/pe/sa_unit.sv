//----------------------------------------------------------------+
// Project: Capstone Design for graduation
// Module: sa_unit (SystemVerilog version)
// Description:
//		Top Unit for 8x8 Systolic Array
//
// Last Updated: 2025-10-21 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns / 1ps

module sa_unit(
    input logic RST,
    input logic CLK,
    input logic EN,
    input logic WRITE,
    input logic [2:0] IDX,
    
    input logic [7:0] DIN_0,
    input logic [7:0] DIN_1,
    input logic [7:0] DIN_2,
    input logic [7:0] DIN_3,
    input logic [7:0] DIN_4,
    input logic [7:0] DIN_5,
    input logic [7:0] DIN_6,
    input logic [7:0] DIN_7,
    input logic [7:0] DIN_8,
    input logic [7:0] DIN_9,
    input logic [7:0] DIN_10,
    input logic [7:0] DIN_11,
    input logic [7:0] DIN_12,
    input logic [7:0] DIN_13,
    input logic [7:0] DIN_14,
    input logic [7:0] DIN_15,
    
    output logic [18:0] Y_00,
    output logic [18:0] Y_10,
    output logic [18:0] Y_20,
    output logic [18:0] Y_30,
    output logic [18:0] Y_40,
    output logic [18:0] Y_50,
    output logic [18:0] Y_60,
    output logic [18:0] Y_70,

    output logic [18:0] Y_01,
    output logic [18:0] Y_11,
    output logic [18:0] Y_21,
    output logic [18:0] Y_31,
    output logic [18:0] Y_41,
    output logic [18:0] Y_51,
    output logic [18:0] Y_61,
    output logic [18:0] Y_71,

    output logic [18:0] Y_02,
    output logic [18:0] Y_12,
    output logic [18:0] Y_22,
    output logic [18:0] Y_32,
    output logic [18:0] Y_42,
    output logic [18:0] Y_52,
    output logic [18:0] Y_62,
    output logic [18:0] Y_72,

    output logic [18:0] Y_03,
    output logic [18:0] Y_13,
    output logic [18:0] Y_23,
    output logic [18:0] Y_33,
    output logic [18:0] Y_43,
    output logic [18:0] Y_53,
    output logic [18:0] Y_63,
    output logic [18:0] Y_73,

    output logic [18:0] Y_04,
    output logic [18:0] Y_14,
    output logic [18:0] Y_24,
    output logic [18:0] Y_34,
    output logic [18:0] Y_44,
    output logic [18:0] Y_54,
    output logic [18:0] Y_64,
    output logic [18:0] Y_74,

    output logic [18:0] Y_05,
    output logic [18:0] Y_15,
    output logic [18:0] Y_25,
    output logic [18:0] Y_35,
    output logic [18:0] Y_45,
    output logic [18:0] Y_55,
    output logic [18:0] Y_65,
    output logic [18:0] Y_75,

    output logic [18:0] Y_06,
    output logic [18:0] Y_16,
    output logic [18:0] Y_26,
    output logic [18:0] Y_36,
    output logic [18:0] Y_46,
    output logic [18:0] Y_56,
    output logic [18:0] Y_66,
    output logic [18:0] Y_76,

    output logic [18:0] Y_07,
    output logic [18:0] Y_17,
    output logic [18:0] Y_27,
    output logic [18:0] Y_37,
    output logic [18:0] Y_47,
    output logic [18:0] Y_57,
    output logic [18:0] Y_67,
    output logic [18:0] Y_77,

    output logic        Y_77_valid_out
);
    
    // FOR REGISTER FILE // 

    logic [7:0] DATA_BUFFER [0:15];
    logic [2:0]   IDX_BUFFER;

    // These connect to the edges registers of Systolic Array
    logic [7:0] X_OUT [0:7];
    logic [7:0] W_OUT [0:7];
    logic X_VALID_OUT [0:7];
    logic W_VALID_OUT [0:7];

    // Unused Edges
    logic  [18:0] N_C0Y, N_C1Y, N_C2Y, N_C3Y, N_C4Y, N_C5Y, N_C6Y, N_C7Y;
    logic  [18:0] N_R0Y, N_R1Y, N_R2Y, N_R3Y, N_R4Y, N_R5Y, N_R6Y, N_R7Y;

    always_ff @(posedge CLK) begin
        if (!RST) begin
            for(integer i = 0; i < 16; i = i + 1) begin
                DATA_BUFFER[i[3:0]] <= 8'b0;
            end
            IDX_BUFFER <= 0;
        end else begin

        // If this module is enabled...
            if(EN && WRITE) begin

                DATA_BUFFER[0] <= DIN_0;
                DATA_BUFFER[1] <= DIN_1;
                DATA_BUFFER[2] <= DIN_2;
                DATA_BUFFER[3] <= DIN_3;
                DATA_BUFFER[4] <= DIN_4;
                DATA_BUFFER[5] <= DIN_5;
                DATA_BUFFER[6] <= DIN_6;
                DATA_BUFFER[7] <= DIN_7;
                DATA_BUFFER[8] <= DIN_8;
                DATA_BUFFER[9] <= DIN_9;
                DATA_BUFFER[10] <= DIN_10;
                DATA_BUFFER[11] <= DIN_11;
                DATA_BUFFER[12] <= DIN_12;
                DATA_BUFFER[13] <= DIN_13;
                DATA_BUFFER[14] <= DIN_14;
                DATA_BUFFER[15] <= DIN_15;

                IDX_BUFFER <= IDX;
            end
        end
    end


    ////////////////////// ARCHITECTURE //////////////////////


// Systolic Array Tile
    sa_PE_wrapper u_sa_PE_wrapper(
            // AUX
            .RST(RST),
            .CLK(CLK), .EN(EN),
            

            // Edge Registers Inputs
            .N_R0X(X_OUT[0]), .N_R1X(X_OUT[1]), .N_R2X(X_OUT[2]), .N_R3X(X_OUT[3]), .N_R4X(X_OUT[4]), .N_R5X(X_OUT[5]), .N_R6X(X_OUT[6]), .N_R7X(X_OUT[7]), 
            .N_C0X(W_OUT[0]), .N_C1X(W_OUT[1]), .N_C2X(W_OUT[2]), .N_C3X(W_OUT[3]), .N_C4X(W_OUT[4]), .N_C5X(W_OUT[5]), .N_C6X(W_OUT[6]), .N_C7X(W_OUT[7]),
            
            .N_R0X_valid_in(X_VALID_OUT[0]), .N_R1X_valid_in(X_VALID_OUT[1]), .N_R2X_valid_in(X_VALID_OUT[2]), .N_R3X_valid_in(X_VALID_OUT[3]), .N_R4X_valid_in(X_VALID_OUT[4]), .N_R5X_valid_in(X_VALID_OUT[5]), .N_R6X_valid_in(X_VALID_OUT[6]), .N_R7X_valid_in(X_VALID_OUT[7]),
            .N_C0X_valid_in(W_VALID_OUT[0]), .N_C1X_valid_in(W_VALID_OUT[1]), .N_C2X_valid_in(W_VALID_OUT[2]), .N_C3X_valid_in(W_VALID_OUT[3]), .N_C4X_valid_in(W_VALID_OUT[4]), .N_C5X_valid_in(W_VALID_OUT[5]), .N_C6X_valid_in(W_VALID_OUT[6]), .N_C7X_valid_in(W_VALID_OUT[7]),

            // Systolic Array Outputs
            .Y_00(Y_00), .Y_10(Y_10), .Y_20(Y_20), .Y_30(Y_30), .Y_40(Y_40), .Y_50(Y_50), .Y_60(Y_60), .Y_70(Y_70),
            .Y_01(Y_01), .Y_11(Y_11), .Y_21(Y_21), .Y_31(Y_31), .Y_41(Y_41), .Y_51(Y_51), .Y_61(Y_61), .Y_71(Y_71),
            .Y_02(Y_02), .Y_12(Y_12), .Y_22(Y_22), .Y_32(Y_32), .Y_42(Y_42), .Y_52(Y_52), .Y_62(Y_62), .Y_72(Y_72),
            .Y_03(Y_03), .Y_13(Y_13), .Y_23(Y_23), .Y_33(Y_33), .Y_43(Y_43), .Y_53(Y_53), .Y_63(Y_63), .Y_73(Y_73),
            .Y_04(Y_04), .Y_14(Y_14), .Y_24(Y_24), .Y_34(Y_34), .Y_44(Y_44), .Y_54(Y_54), .Y_64(Y_64), .Y_74(Y_74),
            .Y_05(Y_05), .Y_15(Y_15), .Y_25(Y_25), .Y_35(Y_35), .Y_45(Y_45), .Y_55(Y_55), .Y_65(Y_65), .Y_75(Y_75),
            .Y_06(Y_06), .Y_16(Y_16), .Y_26(Y_26), .Y_36(Y_36), .Y_46(Y_46), .Y_56(Y_56), .Y_66(Y_66), .Y_76(Y_76),
            .Y_07(Y_07), .Y_17(Y_17), .Y_27(Y_27), .Y_37(Y_37), .Y_47(Y_47), .Y_57(Y_57), .Y_67(Y_67), .Y_77(Y_77), 

            .Y_77_valid_out(Y_77_valid_out),

            // Unused Edge Pins
            .N_C0Y(N_C0Y), .N_C1Y(N_C1Y), .N_C2Y(N_C2Y), .N_C3Y(N_C3Y), .N_C4Y(N_C4Y), .N_C5Y(N_C5Y), .N_C6Y(N_C6Y), .N_C7Y(N_C7Y), 
            .N_R0Y(N_R0Y), .N_R1Y(N_R1Y), .N_R2Y(N_R2Y), .N_R3Y(N_R3Y), .N_R4Y(N_R4Y), .N_R5Y(N_R5Y), .N_R6Y(N_R6Y), .N_R7Y(N_R7Y)
    );


// Register File
    sa_RF u_sa_RF(
        .CLK(CLK),
        .RST(RST),
        .RF_EN(EN),
                
        .WRITE(WRITE),

        .IDX(IDX_BUFFER),
        
        .DATA_IN_0(DATA_BUFFER[0]),
        .DATA_IN_1(DATA_BUFFER[1]),
        .DATA_IN_2(DATA_BUFFER[2]),
        .DATA_IN_3(DATA_BUFFER[3]),
        .DATA_IN_4(DATA_BUFFER[4]),
        .DATA_IN_5(DATA_BUFFER[5]),
        .DATA_IN_6(DATA_BUFFER[6]),
        .DATA_IN_7(DATA_BUFFER[7]),
        .DATA_IN_8(DATA_BUFFER[8]),
        .DATA_IN_9(DATA_BUFFER[9]),
        .DATA_IN_A(DATA_BUFFER[10]),
        .DATA_IN_B(DATA_BUFFER[11]),
        .DATA_IN_C(DATA_BUFFER[12]),
        .DATA_IN_D(DATA_BUFFER[13]),
        .DATA_IN_E(DATA_BUFFER[14]),
        .DATA_IN_F(DATA_BUFFER[15]),

    // ...existing code...

        .X_OUT(X_OUT),
        .W_OUT(W_OUT),
        .X_VALID_OUT(X_VALID_OUT),
        .W_VALID_OUT(W_VALID_OUT)
    );


endmodule
