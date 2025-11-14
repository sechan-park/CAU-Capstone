//----------------------------------------------------------------+
//----------------------------------------------------------------+
// Project: Deep Learning Hardware Design Contest
// Module: dma_read (ported from axi_dma_rd)
// Description:
//		Load parameters and input feature map from DRAM via AXI4
//
// Ported for Capstone Design Project
// Last Updated: 2025-10-11 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns/1ps

module dma_read #(
    // Parameters
    parameter BITS_TRANS = 18,
    parameter OUT_BITS_TRANS = 13,
    parameter AXI_WIDTH_USER = 1,               // Master ID
    parameter AXI_WIDTH_ID   = 4,               // ID width in bits
    parameter AXI_WIDTH_AD   = 32,              // address width
    parameter AXI_WIDTH_DA   = 32,              // data width
    parameter AXI_WIDTH_DS   = (AXI_WIDTH_DA/8),// data strobe width
    // AXI4 User signal widths (for compatibility with M00_AXI)
    parameter AXI_WIDTH_ARUSER = 0,             // Read address user signal width
    parameter AXI_WIDTH_RUSER  = 0              // Read data user signal width
)(
    //AXI Master Interface - AXI4 Compatible
    //Read address channel
    output logic                      M_AXI_ARVALID,    // address/control valid handshake
    input  logic                      M_AXI_ARREADY,    // Read addr ready
    output logic [AXI_WIDTH_AD-1:0]   M_AXI_ARADDR,     // Address Read 
    output logic [AXI_WIDTH_ID-1:0]   M_AXI_ARID,       // Read addr ID
    output logic [7:0]                M_AXI_ARLEN,      // Transfer length
    output logic [2:0]                M_AXI_ARSIZE,     // Transfer width
    output logic [1:0]                M_AXI_ARBURST,    // Burst type
    output logic                      M_AXI_ARLOCK,     // Atomic access (AXI4: 1-bit)
    output logic [3:0]                M_AXI_ARCACHE,    // Cachable/bufferable infor
    output logic [2:0]                M_AXI_ARPROT,     // Protection info
    output logic [3:0]                M_AXI_ARQOS,      // Quality of Service
    output logic [AXI_WIDTH_ARUSER-1:0] M_AXI_ARUSER,  // User defined signal (parameterized)
 
    //Read data channel
    input  logic                      M_AXI_RVALID,     // Read data valid 
    output logic                      M_AXI_RREADY,     // Read data ready (to Slave)
    input  logic [AXI_WIDTH_DA-1:0]   M_AXI_RDATA,      // Read data bus
    input  logic                      M_AXI_RLAST,      // Last beat of a burst transfer
    input  logic [AXI_WIDTH_ID-1:0]   M_AXI_RID,        // Read ID
    input  logic [AXI_WIDTH_RUSER-1:0] M_AXI_RUSER,     // User defined signal (parameterized)
    input  logic [1:0]                M_AXI_RRESP,      // Read response
     
    //Functional Ports (original naming from axi_dma_rd)
    input  logic                      start_dma,
    input  logic [BITS_TRANS-1:0]     num_trans,        // Number of 32-bit words transferred
    input  logic [AXI_WIDTH_AD-1:0]   start_addr,
    output logic [AXI_WIDTH_DA-1:0]   data_o,
    output logic                      data_vld_o,
    output logic [BITS_TRANS-1:0]     data_cnt_o,
    output logic                      done_o,

    //Global signals
    input  logic                      clk,
    input  logic                      rstn
);

    localparam  FIXED_BURST_SIZE = 256; //Change if you want, 256 is possible, but it can be dangerous
    localparam  LOG_BURST_SIZE = $clog2(FIXED_BURST_SIZE);

//------------------------------------------------------------------------------
// Internal Signals - AXI4 Compatible
//------------------------------------------------------------------------------
    assign M_AXI_ARID = 3'd0;           // Read addr ID
    assign M_AXI_ARLOCK = 1'b0;         // Atomic access (AXI4: single bit, normal access)
    assign M_AXI_ARCACHE = 4'b0011;     // Bufferable & Cacheable (VIP narrow-cache check friendly)
    assign M_AXI_ARPROT = 3'd0;         // Protection info
    assign M_AXI_ARQOS = 4'b1111;       // Highest priority
    assign M_AXI_ARUSER = 'b0;          // User defined signal (parameterized width)
        
    logic [AXI_WIDTH_AD-1:0]  ext_araddr;
    logic [AXI_WIDTH_AD-1:0]  ext_arlen;
    logic [2:0]               ext_arsize;
    logic [1:0]               ext_arburst;
    logic                     ext_arvalid;
    logic                     ext_arready;
    logic [AXI_WIDTH_DA-1:0]  ext_rdata;
    logic [1:0]               ext_rresp;
    logic                     ext_rlast;
    logic                     ext_rvalid;
    logic                     ext_rready;

    assign  M_AXI_ARVALID = ext_arvalid;
    assign  M_AXI_ARADDR  = ext_araddr;
    assign  M_AXI_ARLEN   = ext_arlen;
    assign  M_AXI_ARSIZE  = ext_arsize;
    assign  M_AXI_ARBURST = ext_arburst;
    assign  ext_arready = M_AXI_ARREADY;

    assign  ext_rdata   = M_AXI_RDATA;
    assign  ext_rresp   = M_AXI_RRESP;
    assign  ext_rlast   = M_AXI_RLAST;
    assign  ext_rvalid  = M_AXI_RVALID;
    assign  M_AXI_RREADY  = ext_rready;

    logic ext_rlast_r;
    logic [1:0] ext_rresp_r;
    logic last_trans;
    logic [7:0] q_burst_size_rd;
    logic [8:0] q_burst_size_rd_1;
    logic [AXI_WIDTH_AD-1:0] q_ext_addr_rd;
    logic [2:0] st_rdaxi, next_st_rdaxi;
    logic [BITS_TRANS-1:0] num_trans_d;
    logic [BITS_TRANS-1:0] data_cnt;
    logic [BITS_TRANS-1:0] d_burst_cnt_rd, q_burst_cnt_rd;
    logic start_dma_d;
   
    always_ff @(posedge clk or negedge rstn)
        if(!rstn)   ext_rlast_r <= 'h0;
        else        ext_rlast_r <= ext_rlast & ext_rvalid & ext_rready;
   
    always_ff @(posedge clk or negedge rstn)
        if(!rstn)   ext_rresp_r <= 'h0;
        else        ext_rresp_r <= ext_rresp;

//------------------------------------------------------------------------------
// Main body of code
//------------------------------------------------------------------------------
   //FSM for Read from AXI
    localparam RD_IDLE = 0, RD_PRE = 1, RD_START = 2, RD_SEQ = 3, RD_WAIT = 4;

    always_ff @(posedge clk or negedge rstn)
        if(!rstn)         st_rdaxi <= RD_IDLE;
        else              st_rdaxi <= next_st_rdaxi;//flow setting


    assign ext_rready = 1'b1;
    
    always_ff @(posedge clk or negedge rstn)
        if(!rstn) start_dma_d <= 'b0;
        else start_dma_d <= start_dma;

    always_ff @(posedge clk or negedge rstn)
        if(!rstn) num_trans_d <= 'h0;
        else if(start_dma) num_trans_d <= num_trans; //start_dma access only 1time?


    always_ff @(posedge clk or negedge rstn)
        if(!rstn)   q_burst_cnt_rd <= 0;
        else        q_burst_cnt_rd <= d_burst_cnt_rd;

    always_ff @(posedge clk or negedge rstn) begin
        if(!rstn) begin
            q_burst_size_rd <= 0;
            q_burst_size_rd_1 <= 0;
            last_trans <= 1'b0;
        end
        else if(q_burst_cnt_rd + FIXED_BURST_SIZE > num_trans_d) begin
            q_burst_size_rd <= num_trans_d[LOG_BURST_SIZE - 1 :0] - 1;
            q_burst_size_rd_1 <= num_trans_d[LOG_BURST_SIZE - 1 :0];    //The # of remaining transaction
            if(st_rdaxi == RD_SEQ) last_trans <= 1'b1;
            else last_trans <= 1'b0;
        end
        else if(q_burst_cnt_rd + FIXED_BURST_SIZE == num_trans_d) begin
            q_burst_size_rd <= FIXED_BURST_SIZE-1;
            q_burst_size_rd_1 <= FIXED_BURST_SIZE;    //The # of remaining transaction
            if(st_rdaxi == RD_SEQ) last_trans <= 1'b1;
            else last_trans <= 1'b0;
        end
        else begin
            q_burst_size_rd <= FIXED_BURST_SIZE-1;
            q_burst_size_rd_1 <= FIXED_BURST_SIZE;
        end
    end


    always_ff @(posedge clk or negedge rstn) begin
        if(!rstn)
            q_ext_addr_rd <= 0;
        else if(start_dma)
            q_ext_addr_rd <= start_addr;
        else if((st_rdaxi == RD_WAIT) && (next_st_rdaxi == RD_PRE))
            q_ext_addr_rd <= q_ext_addr_rd + {FIXED_BURST_SIZE,{2'b00}};//4B 
    end


    always_comb begin
        next_st_rdaxi = st_rdaxi;
        d_burst_cnt_rd = q_burst_cnt_rd;

        //AXI read addr channel
        ext_araddr = 0;
        ext_arlen = 0;
        ext_arsize = 0;
        ext_arburst = 0;
        ext_arvalid = 1'b0;

        case(st_rdaxi)
            default: next_st_rdaxi = RD_IDLE;
            RD_IDLE: begin
                if(start_dma_d)
                next_st_rdaxi = RD_PRE;
            end
            RD_PRE: begin
                if(q_burst_cnt_rd == num_trans_d) begin //end of blk read
                d_burst_cnt_rd = 0;
                next_st_rdaxi = RD_IDLE;
                end
                else next_st_rdaxi = RD_START;
            end
            RD_START: begin   //start burst read
                if(ext_arready) begin
                ext_arvalid = 1'b1;
                ext_araddr = q_ext_addr_rd;
                ext_arlen = q_burst_size_rd;
                ext_arsize = 3'b010;
                ext_arburst = 2'b01;// BURST_INCR

                next_st_rdaxi = RD_SEQ;
                end
            end
            RD_SEQ: begin  //currently, wait until finishing reading data
                if(ext_rlast_r) begin //last beat of a burst
                    if(ext_rresp_r == 2'b00)
                        next_st_rdaxi = RD_WAIT;
                    else     //error, restart the burst transfer
                        next_st_rdaxi = RD_START;
                end
            end
            RD_WAIT: begin //wait data written to ofifo
                d_burst_cnt_rd = q_burst_cnt_rd + q_burst_size_rd_1;
                next_st_rdaxi = RD_PRE;
            end
        endcase

    end


    always_ff @(posedge clk or negedge rstn) begin
        if(!rstn) begin
            data_cnt_o <= 'h0;
            data_o <= 'h0;
            data_vld_o <= 'h0;
        end
        else begin
            data_cnt_o <= data_cnt;
            data_o <= ext_rdata;
            data_vld_o <= ext_rvalid;
        end
    end
  
    always_ff @(posedge clk or negedge rstn) begin
        if(!rstn) begin
            done_o <= 'h0;
        end
        else begin
            done_o <= last_trans && ext_rlast && (ext_rresp == 2'b00);
        end
    end // Timing check needed


    always_ff @(posedge clk or negedge rstn) begin
        if(!rstn)               data_cnt <= 'h0;
        else begin
            if(st_rdaxi == RD_START) data_cnt <= q_burst_cnt_rd;
            else if(ext_rvalid) data_cnt <= data_cnt + 'h1;
        end
    end
    
endmodule

