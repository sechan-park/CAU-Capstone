//----------------------------------------------------------------+
// Project: Capstone Design for graduation
// Module: sa_PE_wrapper (SystemVerilog version)
// Description:
//		Wrapper for 8x8 Systolic Array Processing Elements
//
// Last Updated: 2025-10-21 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns / 1ps

module sa_PE_wrapper(
    // Aux
    input logic RST,
    input logic CLK,
    input logic EN,

    // Edge Registers
    input logic [7:0]  N_R0X, N_R1X, N_R2X, N_R3X, N_R4X, N_R5X, N_R6X, N_R7X, 
    input logic [7:0]  N_C0X, N_C1X, N_C2X, N_C3X, N_C4X, N_C5X, N_C6X, N_C7X,
    input logic          N_R0X_valid_in, N_R1X_valid_in, N_R2X_valid_in, N_R3X_valid_in, N_R4X_valid_in, N_R5X_valid_in, N_R6X_valid_in, N_R7X_valid_in,
    input logic          N_C0X_valid_in, N_C1X_valid_in, N_C2X_valid_in, N_C3X_valid_in, N_C4X_valid_in, N_C5X_valid_in, N_C6X_valid_in, N_C7X_valid_in,


    // Out Edges
    output logic [18:0]  N_C0Y, N_C1Y, N_C2Y, N_C3Y, N_C4Y, N_C5Y, N_C6Y, N_C7Y,
    output logic [18:0]  N_R0Y, N_R1Y, N_R2Y, N_R3Y, N_R4Y, N_R5Y, N_R6Y, N_R7Y,

    // Direct Outputs (Debugging)
    output logic [18:0]  Y_00, Y_10, Y_20, Y_30, Y_40, Y_50, Y_60, Y_70,
    output logic [18:0]  Y_01, Y_11, Y_21, Y_31, Y_41, Y_51, Y_61, Y_71,
    output logic [18:0]  Y_02, Y_12, Y_22, Y_32, Y_42, Y_52, Y_62, Y_72,
    output logic [18:0]  Y_03, Y_13, Y_23, Y_33, Y_43, Y_53, Y_63, Y_73,
    output logic [18:0]  Y_04, Y_14, Y_24, Y_34, Y_44, Y_54, Y_64, Y_74,
    output logic [18:0]  Y_05, Y_15, Y_25, Y_35, Y_45, Y_55, Y_65, Y_75,
    output logic [18:0]  Y_06, Y_16, Y_26, Y_36, Y_46, Y_56, Y_66, Y_76,
    output logic [18:0]  Y_07, Y_17, Y_27, Y_37, Y_47, Y_57, Y_67, Y_77,
    
    output logic         Y_77_valid_out
    
);
    // Internal valid wires
    logic Y_00_valid_out, Y_10_valid_out, Y_20_valid_out, Y_30_valid_out, Y_40_valid_out, Y_50_valid_out, Y_60_valid_out, Y_70_valid_out;
    logic Y_01_valid_out, Y_11_valid_out, Y_21_valid_out, Y_31_valid_out, Y_41_valid_out, Y_51_valid_out, Y_61_valid_out, Y_71_valid_out;
    logic Y_02_valid_out, Y_12_valid_out, Y_22_valid_out, Y_32_valid_out, Y_42_valid_out, Y_52_valid_out, Y_62_valid_out, Y_72_valid_out;
    logic Y_03_valid_out, Y_13_valid_out, Y_23_valid_out, Y_33_valid_out, Y_43_valid_out, Y_53_valid_out, Y_63_valid_out, Y_73_valid_out;
    logic Y_04_valid_out, Y_14_valid_out, Y_24_valid_out, Y_34_valid_out, Y_44_valid_out, Y_54_valid_out, Y_64_valid_out, Y_74_valid_out;
    logic Y_05_valid_out, Y_15_valid_out, Y_25_valid_out, Y_35_valid_out, Y_45_valid_out, Y_55_valid_out, Y_65_valid_out, Y_75_valid_out;
    logic Y_06_valid_out, Y_16_valid_out, Y_26_valid_out, Y_36_valid_out, Y_46_valid_out, Y_56_valid_out, Y_66_valid_out, Y_76_valid_out;
    logic Y_07_valid_out, Y_17_valid_out, Y_27_valid_out, Y_37_valid_out, Y_47_valid_out, Y_57_valid_out, Y_67_valid_out;

    // Interconnect (ROW) - Data
    logic    [7:0] N_R00, N_R10, N_R20, N_R30, N_R40, N_R50, N_R60;
    logic    [7:0] N_R01, N_R11, N_R21, N_R31, N_R41, N_R51, N_R61;
    logic    [7:0] N_R02, N_R12, N_R22, N_R32, N_R42, N_R52, N_R62;
    logic    [7:0] N_R03, N_R13, N_R23, N_R33, N_R43, N_R53, N_R63;
    logic    [7:0] N_R04, N_R14, N_R24, N_R34, N_R44, N_R54, N_R64;
    logic    [7:0] N_R05, N_R15, N_R25, N_R35, N_R45, N_R55, N_R65;
    logic    [7:0] N_R06, N_R16, N_R26, N_R36, N_R46, N_R56, N_R66;
    logic    [7:0] N_R07, N_R17, N_R27, N_R37, N_R47, N_R57, N_R67;

    // Interconnect (ROW) - Valid
    logic    N_R00_valid, N_R10_valid, N_R20_valid, N_R30_valid, N_R40_valid, N_R50_valid, N_R60_valid;
    logic    N_R01_valid, N_R11_valid, N_R21_valid, N_R31_valid, N_R41_valid, N_R51_valid, N_R61_valid;
    logic    N_R02_valid, N_R12_valid, N_R22_valid, N_R32_valid, N_R42_valid, N_R52_valid, N_R62_valid;
    logic    N_R03_valid, N_R13_valid, N_R23_valid, N_R33_valid, N_R43_valid, N_R53_valid, N_R63_valid;
    logic    N_R04_valid, N_R14_valid, N_R24_valid, N_R34_valid, N_R44_valid, N_R54_valid, N_R64_valid;
    logic    N_R05_valid, N_R15_valid, N_R25_valid, N_R35_valid, N_R45_valid, N_R55_valid, N_R65_valid;
    logic    N_R06_valid, N_R16_valid, N_R26_valid, N_R36_valid, N_R46_valid, N_R56_valid, N_R66_valid;
    logic    N_R07_valid, N_R17_valid, N_R27_valid, N_R37_valid, N_R47_valid, N_R57_valid, N_R67_valid;

    // Interconnect (COL) - Data
    logic    [7:0] N_C00, N_C10, N_C20, N_C30, N_C40, N_C50, N_C60, N_C70; 
    logic    [7:0] N_C01, N_C11, N_C21, N_C31, N_C41, N_C51, N_C61, N_C71;
    logic    [7:0] N_C02, N_C12, N_C22, N_C32, N_C42, N_C52, N_C62, N_C72;
    logic    [7:0] N_C03, N_C13, N_C23, N_C33, N_C43, N_C53, N_C63, N_C73;
    logic    [7:0] N_C04, N_C14, N_C24, N_C34, N_C44, N_C54, N_C64, N_C74;
    logic    [7:0] N_C05, N_C15, N_C25, N_C35, N_C45, N_C55, N_C65, N_C75;
    logic    [7:0] N_C06, N_C16, N_C26, N_C36, N_C46, N_C56, N_C66, N_C76;

    // Interconnect (COL) - Valid
    logic    N_C00_valid, N_C10_valid, N_C20_valid, N_C30_valid, N_C40_valid, N_C50_valid, N_C60_valid, N_C70_valid;
    logic    N_C01_valid, N_C11_valid, N_C21_valid, N_C31_valid, N_C41_valid, N_C51_valid, N_C61_valid, N_C71_valid;
    logic    N_C02_valid, N_C12_valid, N_C22_valid, N_C32_valid, N_C42_valid, N_C52_valid, N_C62_valid, N_C72_valid;
    logic    N_C03_valid, N_C13_valid, N_C23_valid, N_C33_valid, N_C43_valid, N_C53_valid, N_C63_valid, N_C73_valid;
    logic    N_C04_valid, N_C14_valid, N_C24_valid, N_C34_valid, N_C44_valid, N_C54_valid, N_C64_valid, N_C74_valid;
    logic    N_C05_valid, N_C15_valid, N_C25_valid, N_C35_valid, N_C45_valid, N_C55_valid, N_C65_valid, N_C75_valid;
    logic    N_C06_valid, N_C16_valid, N_C26_valid, N_C36_valid, N_C46_valid, N_C56_valid, N_C66_valid, N_C76_valid;

    // Architecture

// ROW 0 
    hPE C00(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_00), .A(N_R0X), .B(N_C0X), .A_out(N_R00), .B_out(N_C00), .A_valid_in(N_R0X_valid_in), .B_valid_in(N_C0X_valid_in), .A_valid_out(N_R00_valid), .B_valid_out(N_C00_valid), .C_valid_out(Y_00_valid_out));
    hPE C10(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_10), .A(N_R00), .B(N_C1X), .A_out(N_R10), .B_out(N_C10), .A_valid_in(N_R00_valid), .B_valid_in(N_C1X_valid_in), .A_valid_out(N_R10_valid), .B_valid_out(N_C10_valid), .C_valid_out(Y_10_valid_out));
    hPE C20(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_20), .A(N_R10), .B(N_C2X), .A_out(N_R20), .B_out(N_C20), .A_valid_in(N_R10_valid), .B_valid_in(N_C2X_valid_in), .A_valid_out(N_R20_valid), .B_valid_out(N_C20_valid), .C_valid_out(Y_20_valid_out));
    hPE C30(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_30), .A(N_R20), .B(N_C3X), .A_out(N_R30), .B_out(N_C30), .A_valid_in(N_R20_valid), .B_valid_in(N_C3X_valid_in), .A_valid_out(N_R30_valid), .B_valid_out(N_C30_valid), .C_valid_out(Y_30_valid_out));
    hPE C40(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_40), .A(N_R30), .B(N_C4X), .A_out(N_R40), .B_out(N_C40), .A_valid_in(N_R30_valid), .B_valid_in(N_C4X_valid_in), .A_valid_out(N_R40_valid), .B_valid_out(N_C40_valid), .C_valid_out(Y_40_valid_out));
    hPE C50(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_50), .A(N_R40), .B(N_C5X), .A_out(N_R50), .B_out(N_C50), .A_valid_in(N_R40_valid), .B_valid_in(N_C5X_valid_in), .A_valid_out(N_R50_valid), .B_valid_out(N_C50_valid), .C_valid_out(Y_50_valid_out));
    hPE C60(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_60), .A(N_R50), .B(N_C6X), .A_out(N_R60), .B_out(N_C60), .A_valid_in(N_R50_valid), .B_valid_in(N_C6X_valid_in), .A_valid_out(N_R60_valid), .B_valid_out(N_C60_valid), .C_valid_out(Y_60_valid_out));
    hPE C70(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_70), .A(N_R60), .B(N_C7X), .A_out(N_R0Y), .B_out(N_C70), .A_valid_in(N_R60_valid), .B_valid_in(N_C7X_valid_in), .A_valid_out(), .B_valid_out(N_C70_valid), .C_valid_out(Y_70_valid_out));


// ROW 1
    hPE C01(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_01), .A(N_R1X), .B(N_C00), .A_out(N_R01), .B_out(N_C01), .A_valid_in(N_R1X_valid_in), .B_valid_in(N_C00_valid), .A_valid_out(N_R01_valid), .B_valid_out(N_C01_valid), .C_valid_out(Y_01_valid_out));
    hPE C11(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_11), .A(N_R01), .B(N_C10), .A_out(N_R11), .B_out(N_C11), .A_valid_in(N_R01_valid), .B_valid_in(N_C10_valid), .A_valid_out(N_R11_valid), .B_valid_out(N_C11_valid), .C_valid_out(Y_11_valid_out));
    hPE C21(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_21), .A(N_R11), .B(N_C20), .A_out(N_R21), .B_out(N_C21), .A_valid_in(N_R11_valid), .B_valid_in(N_C20_valid), .A_valid_out(N_R21_valid), .B_valid_out(N_C21_valid), .C_valid_out(Y_21_valid_out));
    hPE C31(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_31), .A(N_R21), .B(N_C30), .A_out(N_R31), .B_out(N_C31), .A_valid_in(N_R21_valid), .B_valid_in(N_C30_valid), .A_valid_out(N_R31_valid), .B_valid_out(N_C31_valid), .C_valid_out(Y_31_valid_out));
    hPE C41(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_41), .A(N_R31), .B(N_C40), .A_out(N_R41), .B_out(N_C41), .A_valid_in(N_R31_valid), .B_valid_in(N_C40_valid), .A_valid_out(N_R41_valid), .B_valid_out(N_C41_valid), .C_valid_out(Y_41_valid_out));
    hPE C51(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_51), .A(N_R41), .B(N_C50), .A_out(N_R51), .B_out(N_C51), .A_valid_in(N_R41_valid), .B_valid_in(N_C50_valid), .A_valid_out(N_R51_valid), .B_valid_out(N_C51_valid), .C_valid_out(Y_51_valid_out));
    hPE C61(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_61), .A(N_R51), .B(N_C60), .A_out(N_R61), .B_out(N_C61), .A_valid_in(N_R51_valid), .B_valid_in(N_C60_valid), .A_valid_out(N_R61_valid), .B_valid_out(N_C61_valid), .C_valid_out(Y_61_valid_out));
    hPE C71(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_71), .A(N_R61), .B(N_C70), .A_out(N_R1Y), .B_out(N_C71), .A_valid_in(N_R61_valid), .B_valid_in(N_C70_valid), .A_valid_out(), .B_valid_out(N_C71_valid), .C_valid_out(Y_71_valid_out));


// ROW 2
    hPE C02(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_02), .A(N_R2X), .B(N_C01), .A_out(N_R02), .B_out(N_C02), .A_valid_in(N_R2X_valid_in), .B_valid_in(N_C01_valid), .A_valid_out(N_R02_valid), .B_valid_out(N_C02_valid), .C_valid_out(Y_02_valid_out));
    hPE C12(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_12), .A(N_R02), .B(N_C11), .A_out(N_R12), .B_out(N_C12), .A_valid_in(N_R02_valid), .B_valid_in(N_C11_valid), .A_valid_out(N_R12_valid), .B_valid_out(N_C12_valid), .C_valid_out(Y_12_valid_out));
    hPE C22(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_22), .A(N_R12), .B(N_C21), .A_out(N_R22), .B_out(N_C22), .A_valid_in(N_R12_valid), .B_valid_in(N_C21_valid), .A_valid_out(N_R22_valid), .B_valid_out(N_C22_valid), .C_valid_out(Y_22_valid_out));
    hPE C32(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_32), .A(N_R22), .B(N_C31), .A_out(N_R32), .B_out(N_C32), .A_valid_in(N_R22_valid), .B_valid_in(N_C31_valid), .A_valid_out(N_R32_valid), .B_valid_out(N_C32_valid), .C_valid_out(Y_32_valid_out));
    hPE C42(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_42), .A(N_R32), .B(N_C41), .A_out(N_R42), .B_out(N_C42), .A_valid_in(N_R32_valid), .B_valid_in(N_C41_valid), .A_valid_out(N_R42_valid), .B_valid_out(N_C42_valid), .C_valid_out(Y_42_valid_out));
    hPE C52(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_52), .A(N_R42), .B(N_C51), .A_out(N_R52), .B_out(N_C52), .A_valid_in(N_R42_valid), .B_valid_in(N_C51_valid), .A_valid_out(N_R52_valid), .B_valid_out(N_C52_valid), .C_valid_out(Y_52_valid_out));
    hPE C62(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_62), .A(N_R52), .B(N_C61), .A_out(N_R62), .B_out(N_C62), .A_valid_in(N_R52_valid), .B_valid_in(N_C61_valid), .A_valid_out(N_R62_valid), .B_valid_out(N_C62_valid), .C_valid_out(Y_62_valid_out));
    hPE C72(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_72), .A(N_R62), .B(N_C71), .A_out(N_R2Y), .B_out(N_C72), .A_valid_in(N_R62_valid), .B_valid_in(N_C71_valid), .A_valid_out(), .B_valid_out(N_C72_valid), .C_valid_out(Y_72_valid_out));


// Row 3
    hPE C03(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_03), .A(N_R3X), .B(N_C02), .A_out(N_R03), .B_out(N_C03), .A_valid_in(N_R3X_valid_in), .B_valid_in(N_C02_valid), .A_valid_out(N_R03_valid), .B_valid_out(N_C03_valid), .C_valid_out(Y_03_valid_out));
    hPE C13(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_13), .A(N_R03), .B(N_C12), .A_out(N_R13), .B_out(N_C13), .A_valid_in(N_R03_valid), .B_valid_in(N_C12_valid), .A_valid_out(N_R13_valid), .B_valid_out(N_C13_valid), .C_valid_out(Y_13_valid_out));
    hPE C23(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_23), .A(N_R13), .B(N_C22), .A_out(N_R23), .B_out(N_C23), .A_valid_in(N_R13_valid), .B_valid_in(N_C22_valid), .A_valid_out(N_R23_valid), .B_valid_out(N_C23_valid), .C_valid_out(Y_23_valid_out));
    hPE C33(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_33), .A(N_R23), .B(N_C32), .A_out(N_R33), .B_out(N_C33), .A_valid_in(N_R23_valid), .B_valid_in(N_C32_valid), .A_valid_out(N_R33_valid), .B_valid_out(N_C33_valid), .C_valid_out(Y_33_valid_out));
    hPE C43(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_43), .A(N_R33), .B(N_C42), .A_out(N_R43), .B_out(N_C43), .A_valid_in(N_R33_valid), .B_valid_in(N_C42_valid), .A_valid_out(N_R43_valid), .B_valid_out(N_C43_valid), .C_valid_out(Y_43_valid_out));
    hPE C53(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_53), .A(N_R43), .B(N_C52), .A_out(N_R53), .B_out(N_C53), .A_valid_in(N_R43_valid), .B_valid_in(N_C52_valid), .A_valid_out(N_R53_valid), .B_valid_out(N_C53_valid), .C_valid_out(Y_53_valid_out));
    hPE C63(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_63), .A(N_R53), .B(N_C62), .A_out(N_R63), .B_out(N_C63), .A_valid_in(N_R53_valid), .B_valid_in(N_C62_valid), .A_valid_out(N_R63_valid), .B_valid_out(N_C63_valid), .C_valid_out(Y_63_valid_out));
    hPE C73(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_73), .A(N_R63), .B(N_C72), .A_out(N_R3Y), .B_out(N_C73), .A_valid_in(N_R63_valid), .B_valid_in(N_C72_valid), .A_valid_out(), .B_valid_out(N_C73_valid), .C_valid_out(Y_73_valid_out));


// Row 4
    hPE C04(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_04), .A(N_R4X), .B(N_C03), .A_out(N_R04), .B_out(N_C04), .A_valid_in(N_R4X_valid_in), .B_valid_in(N_C03_valid), .A_valid_out(N_R04_valid), .B_valid_out(N_C04_valid), .C_valid_out(Y_04_valid_out));
    hPE C14(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_14), .A(N_R04), .B(N_C13), .A_out(N_R14), .B_out(N_C14), .A_valid_in(N_R04_valid), .B_valid_in(N_C13_valid), .A_valid_out(N_R14_valid), .B_valid_out(N_C14_valid), .C_valid_out(Y_14_valid_out));
    hPE C24(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_24), .A(N_R14), .B(N_C23), .A_out(N_R24), .B_out(N_C24), .A_valid_in(N_R14_valid), .B_valid_in(N_C23_valid), .A_valid_out(N_R24_valid), .B_valid_out(N_C24_valid), .C_valid_out(Y_24_valid_out));
    hPE C34(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_34), .A(N_R24), .B(N_C33), .A_out(N_R34), .B_out(N_C34), .A_valid_in(N_R24_valid), .B_valid_in(N_C33_valid), .A_valid_out(N_R34_valid), .B_valid_out(N_C34_valid), .C_valid_out(Y_34_valid_out));
    hPE C44(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_44), .A(N_R34), .B(N_C43), .A_out(N_R44), .B_out(N_C44), .A_valid_in(N_R34_valid), .B_valid_in(N_C43_valid), .A_valid_out(N_R44_valid), .B_valid_out(N_C44_valid), .C_valid_out(Y_44_valid_out));
    hPE C54(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_54), .A(N_R44), .B(N_C53), .A_out(N_R54), .B_out(N_C54), .A_valid_in(N_R44_valid), .B_valid_in(N_C53_valid), .A_valid_out(N_R54_valid), .B_valid_out(N_C54_valid), .C_valid_out(Y_54_valid_out));
    hPE C64(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_64), .A(N_R54), .B(N_C63), .A_out(N_R64), .B_out(N_C64), .A_valid_in(N_R54_valid), .B_valid_in(N_C63_valid), .A_valid_out(N_R64_valid), .B_valid_out(N_C64_valid), .C_valid_out(Y_64_valid_out));
    hPE C74(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_74), .A(N_R64), .B(N_C73), .A_out(N_R4Y), .B_out(N_C74), .A_valid_in(N_R64_valid), .B_valid_in(N_C73_valid), .A_valid_out(), .B_valid_out(N_C74_valid), .C_valid_out(Y_74_valid_out));


// Row 5
    hPE C05(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_05), .A(N_R5X), .B(N_C04), .A_out(N_R05), .B_out(N_C05), .A_valid_in(N_R5X_valid_in), .B_valid_in(N_C04_valid), .A_valid_out(N_R05_valid), .B_valid_out(N_C05_valid), .C_valid_out(Y_05_valid_out));
    hPE C15(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_15), .A(N_R05), .B(N_C14), .A_out(N_R15), .B_out(N_C15), .A_valid_in(N_R05_valid), .B_valid_in(N_C14_valid), .A_valid_out(N_R15_valid), .B_valid_out(N_C15_valid), .C_valid_out(Y_15_valid_out));
    hPE C25(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_25), .A(N_R15), .B(N_C24), .A_out(N_R25), .B_out(N_C25), .A_valid_in(N_R15_valid), .B_valid_in(N_C24_valid), .A_valid_out(N_R25_valid), .B_valid_out(N_C25_valid), .C_valid_out(Y_25_valid_out));
    hPE C35(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_35), .A(N_R25), .B(N_C34), .A_out(N_R35), .B_out(N_C35), .A_valid_in(N_R25_valid), .B_valid_in(N_C34_valid), .A_valid_out(N_R35_valid), .B_valid_out(N_C35_valid), .C_valid_out(Y_35_valid_out));
    hPE C45(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_45), .A(N_R35), .B(N_C44), .A_out(N_R45), .B_out(N_C45), .A_valid_in(N_R35_valid), .B_valid_in(N_C44_valid), .A_valid_out(N_R45_valid), .B_valid_out(N_C45_valid), .C_valid_out(Y_45_valid_out));
    hPE C55(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_55), .A(N_R45), .B(N_C54), .A_out(N_R55), .B_out(N_C55), .A_valid_in(N_R45_valid), .B_valid_in(N_C54_valid), .A_valid_out(N_R55_valid), .B_valid_out(N_C55_valid), .C_valid_out(Y_55_valid_out));
    hPE C65(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_65), .A(N_R55), .B(N_C64), .A_out(N_R65), .B_out(N_C65), .A_valid_in(N_R55_valid), .B_valid_in(N_C64_valid), .A_valid_out(N_R65_valid), .B_valid_out(N_C65_valid), .C_valid_out(Y_65_valid_out));
    hPE C75(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_75), .A(N_R65), .B(N_C74), .A_out(N_R5Y), .B_out(N_C75), .A_valid_in(N_R65_valid), .B_valid_in(N_C74_valid), .A_valid_out(), .B_valid_out(N_C75_valid), .C_valid_out(Y_75_valid_out));


// Row 6
    hPE C06(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_06), .A(N_R6X), .B(N_C05), .A_out(N_R06), .B_out(N_C06), .A_valid_in(N_R6X_valid_in), .B_valid_in(N_C05_valid), .A_valid_out(N_R06_valid), .B_valid_out(N_C06_valid), .C_valid_out(Y_06_valid_out));
    hPE C16(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_16), .A(N_R06), .B(N_C15), .A_out(N_R16), .B_out(N_C16), .A_valid_in(N_R06_valid), .B_valid_in(N_C15_valid), .A_valid_out(N_R16_valid), .B_valid_out(N_C16_valid), .C_valid_out(Y_16_valid_out));
    hPE C26(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_26), .A(N_R16), .B(N_C25), .A_out(N_R26), .B_out(N_C26), .A_valid_in(N_R16_valid), .B_valid_in(N_C25_valid), .A_valid_out(N_R26_valid), .B_valid_out(N_C26_valid), .C_valid_out(Y_26_valid_out));
    hPE C36(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_36), .A(N_R26), .B(N_C35), .A_out(N_R36), .B_out(N_C36), .A_valid_in(N_R26_valid), .B_valid_in(N_C35_valid), .A_valid_out(N_R36_valid), .B_valid_out(N_C36_valid), .C_valid_out(Y_36_valid_out));
    hPE C46(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_46), .A(N_R36), .B(N_C45), .A_out(N_R46), .B_out(N_C46), .A_valid_in(N_R36_valid), .B_valid_in(N_C45_valid), .A_valid_out(N_R46_valid), .B_valid_out(N_C46_valid), .C_valid_out(Y_46_valid_out));
    hPE C56(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_56), .A(N_R46), .B(N_C55), .A_out(N_R56), .B_out(N_C56), .A_valid_in(N_R46_valid), .B_valid_in(N_C55_valid), .A_valid_out(N_R56_valid), .B_valid_out(N_C56_valid), .C_valid_out(Y_56_valid_out));
    hPE C66(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_66), .A(N_R56), .B(N_C65), .A_out(N_R66), .B_out(N_C66), .A_valid_in(N_R56_valid), .B_valid_in(N_C65_valid), .A_valid_out(N_R66_valid), .B_valid_out(N_C66_valid), .C_valid_out(Y_66_valid_out));
    hPE C76(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_76), .A(N_R66), .B(N_C75), .A_out(N_R6Y), .B_out(N_C76), .A_valid_in(N_R66_valid), .B_valid_in(N_C75_valid), .A_valid_out(), .B_valid_out(N_C76_valid), .C_valid_out(Y_76_valid_out));


// Row 7
    hPE C07(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_07), .A(N_R7X), .B(N_C06), .A_out(N_R07), .B_out(N_C0Y), .A_valid_in(N_R7X_valid_in), .B_valid_in(N_C06_valid), .A_valid_out(N_R07_valid), .B_valid_out(), .C_valid_out(Y_07_valid_out));
    hPE C17(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_17), .A(N_R07), .B(N_C16), .A_out(N_R17), .B_out(N_C1Y), .A_valid_in(N_R07_valid), .B_valid_in(N_C16_valid), .A_valid_out(N_R17_valid), .B_valid_out(), .C_valid_out(Y_17_valid_out));
    hPE C27(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_27), .A(N_R17), .B(N_C26), .A_out(N_R27), .B_out(N_C2Y), .A_valid_in(N_R17_valid), .B_valid_in(N_C26_valid), .A_valid_out(N_R27_valid), .B_valid_out(), .C_valid_out(Y_27_valid_out));
    hPE C37(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_37), .A(N_R27), .B(N_C36), .A_out(N_R37), .B_out(N_C3Y), .A_valid_in(N_R27_valid), .B_valid_in(N_C36_valid), .A_valid_out(N_R37_valid), .B_valid_out(), .C_valid_out(Y_37_valid_out));
    hPE C47(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_47), .A(N_R37), .B(N_C46), .A_out(N_R47), .B_out(N_C4Y), .A_valid_in(N_R37_valid), .B_valid_in(N_C46_valid), .A_valid_out(N_R47_valid), .B_valid_out(), .C_valid_out(Y_47_valid_out));
    hPE C57(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_57), .A(N_R47), .B(N_C56), .A_out(N_R57), .B_out(N_C5Y), .A_valid_in(N_R47_valid), .B_valid_in(N_C56_valid), .A_valid_out(N_R57_valid), .B_valid_out(), .C_valid_out(Y_57_valid_out));
    hPE C67(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_67), .A(N_R57), .B(N_C66), .A_out(N_R67), .B_out(N_C6Y), .A_valid_in(N_R57_valid), .B_valid_in(N_C66_valid), .A_valid_out(N_R67_valid), .B_valid_out(), .C_valid_out(Y_67_valid_out));
    hPE C77(.RST(RST), .CLK(CLK),  .EN(EN), .C(Y_77), .A(N_R67), .B(N_C76), .A_out(N_R7Y), .B_out(N_C7Y), .A_valid_in(N_R67_valid), .B_valid_in(N_C76_valid), .A_valid_out(), .B_valid_out(), .C_valid_out(Y_77_valid_out));

endmodule
