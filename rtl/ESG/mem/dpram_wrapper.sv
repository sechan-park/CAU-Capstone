//------------------------------------------------------------+
//------------------------------------------------------------+
// Project: Deep Learning Hardware Design Contest
// Module: dpram_wrapper (SystemVerilog version)
// Description:
//		Dual-port RAM wrapper
//		FPGA = 1: Use the generated RAM 
//		Otherwise: Use a RAM modeling
//
// History: 2021.09.01 by NXT (truongnx@capp.snu.ac.kr)
//          2025.10.11 Ported to SystemVerilog for Capstone Design
//------------------------------------------------------------+

`timescale 1ns/1ps

//`define FPGA 1
module dpram_wrapper #(
    parameter DW = 24,              // data bit-width per word
    parameter AW = 8,               // address bit-width
    parameter DEPTH = 921600,       // depth, word length
    parameter N_DELAY = 1
)(
    input  logic               clk,
    // Port A (Write)
    input  logic               ena,
    input  logic [AW-1:0]      addra,
    input  logic               wea,
    input  logic [DW-1:0]      dia,
    // Port B (Read)
    input  logic               enb,
    input  logic [AW-1:0]      addrb,
    output logic [DW-1:0]      dob
);

//------------------------------------------------------------------------+
// Internal signals
//------------------------------------------------------------------------+
logic [DW-1:0] rdata;

`ifdef FPGA
    //------------------------------------------------------------------------+
    // implement generate block ram
    //------------------------------------------------------------------------+
    generate
        if((DEPTH == 512) && (DW == 72)) begin: gen_dpram_512x72
            dpram_512x72 u_dpram_512x72 (
                // write ports
                .clka    (clk),
                .ena     (ena),
                .wea     (wea),
                .addra   (addra),
                .dina    (dia),
                // read ports 
                .clkb    (clk),
                .enb     (enb),
                .addrb   (addrb),
                .doutb   (dob)
            );
        end
        else if((DEPTH == 26) && (DW == 32)) begin: gen_dpram_26x32
            dpram_26x32 u_dpram_26x32 (
                // write ports
                .clka    (clk),
                .ena     (ena),
                .wea     (wea),
                .addra   (addra),
                .dina    (dia),
                // read ports 
                .clkb    (clk),
                .enb     (enb),
                .addrb   (addrb),
                .doutb   (dob)
            );
        end
        else if((DEPTH == 13) && (DW == 256)) begin: gen_dpram_13x256
            dpram_13x256 u_dpram_13x256 (
                // write ports
                .clka    (clk),
                .ena     (ena),
                .wea     (wea),
                .addra   (addra),
                .dina    (dia),
                // read ports 
                .clkb    (clk),
                .enb     (enb),
                .addrb   (addrb),
                .doutb   (dob)
            );
        end
        else if((DEPTH == 208) && (DW == 256)) begin: gen_dpram_208x256
            dpram_208x256 u_dpram_208x256 (
                // write ports
                .clka    (clk),
                .ena     (ena),
                .wea     (wea),
                .addra   (addra),
                .dina    (dia),
                // read ports 
                .clkb    (clk),
                .enb     (enb),
                .addrb   (addrb),
                .doutb   (dob)
            );
        end
        else if((DEPTH == 208) && (DW == 32)) begin: gen_dpram_208x32
            dpram_208x32 u_dpram_208x32 (
                // write ports
                .clka    (clk),
                .ena     (ena),
                .wea     (wea),
                .addra   (addra),
                .dina    (dia),
                // read ports 
                .clkb    (clk),
                .enb     (enb),
                .addrb   (addrb),
                .doutb   (dob)
            );
        end
        else if((DEPTH == 16384) && (DW == 32)) begin: gen_dpram_16384x32
            dpram_16384x32 u_dpram_16384x32 (
                // write ports
                .clka    (clk),
                .ena     (ena),
                .wea     (wea),
                .addra   (addra),
                .dina    (dia),
                // read ports 
                .clkb    (clk),
                .enb     (enb),
                .addrb   (addrb),
                .doutb   (dob)
            );
        end
        else if((DEPTH == 1280) && (DW == 32)) begin: gen_dpram_1280x32
            // Capstone Design: Generic DPRAM for sa_core (11-bit address)
            dpram_1280x32 u_dpram_1280x32 (
                // write ports
                .clka    (clk),
                .ena     (ena),
                .wea     (wea),
                .addra   (addra),
                .dina    (dia),
                // read ports 
                .clkb    (clk),
                .enb     (enb),
                .addrb   (addrb),
                .doutb   (dob)
            );
        end
    endgenerate
`else
    //------------------------------------------------------------------------+
    // Memory modeling (Simulation)
    //------------------------------------------------------------------------+
    logic [DW-1:0] ram [0:DEPTH-1]; // Memory cell
    
    // Write
    always_ff @(posedge clk) begin
        if(ena) begin
            if(wea) ram[addra] <= dia;
        end
    end	
    
    // Read
    generate 
       if(N_DELAY == 1) begin: delay_1
          logic [DW-1:0] rdata_reg;   //Primary Data Output
          //Read port
          always_ff @(posedge clk) begin: read
             if(enb)
                rdata_reg <= ram[addrb];
          end
          assign dob = rdata_reg;
       end
       else begin: delay_n
          logic [N_DELAY*DW-1:0] rdata_r;
          always_ff @(posedge clk) begin: read
             if(enb)
                rdata_r[0+:DW] <= ram[addrb];
          end

          always_ff @(posedge clk) begin: delay
             for(int i = 0; i < N_DELAY-1; i++)
                if(enb)
                   rdata_r[(i+1)*DW+:DW] <= rdata_r[i*DW+:DW];
          end
          assign dob = rdata_r[(N_DELAY-1)*DW+:DW];
       end
    endgenerate
`endif

endmodule

