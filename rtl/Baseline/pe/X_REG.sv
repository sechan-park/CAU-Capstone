//----------------------------------------------------------------+
// Project: Capstone Design for graduation
// Module: X_REG (SystemVerilog version)
// Description:
//		Register for 8x8 Systolic Array
//
// Last Updated: 2025-10-21 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns / 1ps

// Parallel Shifting Register --> Replace FIFOs

module X_REG(
    input logic CLK,
    input logic RST,
    input logic EN,
    input logic WRITE,
    input logic [4:0] IDX,
    input logic [7:0] DIN,
    output logic [7:0] DOUT,
    input logic VALID_IN,
    output logic VALID_OUT
);
    
    logic     [7:0]  PIPE    [0:31];
    logic            VALID_PIPE [0:31];
    logic     [7:0]  DOUT_BUFFER;
    logic            VOUT_BUFFER;

    

    // Data Sampling (IN REG)
    always_ff @(posedge CLK) begin
        if (!RST) begin
            for(integer i = 0; i < 32; i = i + 1) begin
                PIPE[i] <= 0;
                VALID_PIPE[i] <= 0;
            end
            DOUT_BUFFER <= 0;
            VOUT_BUFFER <= 0;
        end else begin
            if (EN) begin
                if(WRITE) begin
                    PIPE[IDX] <= DIN;
                    VALID_PIPE[IDX] <= VALID_IN;
                end

                else if (!WRITE) begin
                    DOUT_BUFFER <= PIPE[0];
                    VOUT_BUFFER <= VALID_PIPE[0];

                    for(integer i = 0; i < 31; i = i + 1) begin
                        PIPE[i] <= PIPE[i+1];
                        VALID_PIPE[i] <= VALID_PIPE[i+1];
                    end

                    PIPE[31] <= 0;
                    VALID_PIPE[31] <= 0;
                end
            end
        end
    end

    // OUT REG
    assign DOUT = DOUT_BUFFER;
    assign VALID_OUT = VOUT_BUFFER;

endmodule
