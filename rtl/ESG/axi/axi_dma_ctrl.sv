//----------------------------------------------------------------+
// Project: Deep Learning Hardware Design Contest
// Module: axi_dma_ctrl
// Description:
//   DMA control FSM for block-wise data transfer
//   Manages Read/Write address calculation and sequencing
//
// Original: Contest verified code
// Ported for Capstone Design Project
// Note: Control logic only, no AXI interface
// Last Updated: 2025-10-11 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns/1ps

module axi_dma_ctrl #(
   parameter AXI_WIDTH_AD  = 32,
   parameter BIT_TRANS     = 18
)(
input  logic                  clk, 
input  logic                  rstn,
input  logic [1:0]            i_start,
input  logic [31:0]           i_base_address_rd,
input  logic [31:0]           i_base_address_wr,
input  logic [BIT_TRANS-1:0]  i_num_trans,
 input  logic [15:0]           i_max_req_blk_idx_rd,
 input  logic [15:0]           i_max_req_blk_idx_wr,

input  logic [10:0]           row_cnt,
// DMA Read
input  logic                  i_read_done,
output logic                  o_ctrl_read,
output logic [31:0]           o_read_addr,
// DMA Write
input  logic                  i_write_done,
input  logic                  i_indata_req_wr,
output logic                  o_ctrl_write,
output logic [31:0]           o_write_addr,
output logic [BIT_TRANS-1:0]  o_write_data_cnt,
output logic                  o_ctrl_write_done   
);

// Internal Signals
// FSM
localparam ST_IDLE         = 0;
localparam ST_DMA          = 1;
localparam ST_DMA_WAIT     = 2;
localparam ST_DMA_SYNC     = 3;
localparam ST_DMA_DONE     = 4;

logic [2:0] cstate_rd, nstate_rd;
logic [2:0] cstate_wr, nstate_wr;

// dma read
logic ctrl_read;
logic ctrl_read_wait;
logic ctrl_read_sync;
logic ctrl_read_done;
logic [AXI_WIDTH_AD-1:0] read_addr;
logic [15:0] req_blk_idx_rd;

// dma write
logic ctrl_write;
logic ctrl_write_wait;
logic ctrl_write_sync;
logic ctrl_write_done;
logic [AXI_WIDTH_AD-1:0] write_addr;
logic [BIT_TRANS-1:0] write_data_cnt;
logic [15:0] req_blk_idx_wr;

logic [BIT_TRANS-1:0] num_trans;
logic [15:0] max_req_blk_idx_rd;
logic [15:0] max_req_blk_idx_wr;
logic [31:0] dram_base_addr_rd;
logic [31:0] dram_base_addr_wr;
logic read_done;
logic write_done;
logic indata_req_wr;

assign num_trans = i_num_trans;
assign max_req_blk_idx_rd = i_max_req_blk_idx_rd;
assign max_req_blk_idx_wr = i_max_req_blk_idx_wr;
assign dram_base_addr_rd = i_base_address_rd;
assign dram_base_addr_wr = i_base_address_wr;
assign read_done = i_read_done;
assign write_done = i_write_done;
assign indata_req_wr = i_indata_req_wr;

assign o_write_data_cnt     = write_data_cnt;
assign o_ctrl_write         = ctrl_write;
assign o_ctrl_read          = ctrl_read;
assign o_read_addr          = read_addr;
assign o_write_addr         = write_addr;
assign o_ctrl_write_done    = ctrl_write_done;

//----------------------------------------------------------------
// FSM for DMA Read
//----------------------------------------------------------------
always_ff @(posedge clk or negedge rstn) begin
    if(~rstn) begin
        cstate_rd <= ST_IDLE;
    end
    else begin
        cstate_rd <= nstate_rd;
    end
end

always_comb begin
    ctrl_read = 0;
    ctrl_read_wait = 0;
    ctrl_read_sync = 0;
    ctrl_read_done = 0;
    nstate_rd = cstate_rd;
    case(cstate_rd)
        ST_IDLE: begin
            if(i_start == 2'b10) 
                nstate_rd = ST_DMA;
            else
                nstate_rd = ST_IDLE;
        end
        ST_DMA: begin
            nstate_rd = ST_DMA_WAIT;
            ctrl_read = 1;
        end
        ST_DMA_WAIT: begin
            ctrl_read_wait = 1;
            if(read_done) begin 
                if (req_blk_idx_rd == max_req_blk_idx_rd - 1)
                    nstate_rd = ST_DMA_DONE;
                else                 
                    nstate_rd = ST_DMA_SYNC;
            end 
        end 
        ST_DMA_SYNC: begin 
            ctrl_read_sync = 1;
            nstate_rd = ST_DMA;         
        end 
        ST_DMA_DONE: begin
            ctrl_read_done = 1;
            nstate_rd = ST_IDLE;
        end
        default: nstate_rd = ST_IDLE;
    endcase 
end 

always_ff @(posedge clk or negedge rstn) begin
    if(~rstn) begin
        req_blk_idx_rd <= 0;
    end
    else begin
        if(read_done) begin 
           if(req_blk_idx_rd == max_req_blk_idx_rd - 1)
                req_blk_idx_rd <= 0;                // Reset the counter
            else 
                req_blk_idx_rd <= req_blk_idx_rd + 1;   // Up-Counter    
        end 
    end
end

assign read_addr = dram_base_addr_rd + {req_blk_idx_rd, 6'b0};
// + {row_cnt*80, 6'b0}; 

//----------------------------------------------------------------
// FSM for DMA Write
//----------------------------------------------------------------
always_ff @(posedge clk or negedge rstn) begin
    if(~rstn) begin
        cstate_wr <= ST_IDLE;
    end
    else begin
        cstate_wr <= nstate_wr;
    end
end

always_comb begin
    ctrl_write = 0;
    ctrl_write_wait = 0;
    ctrl_write_sync = 0;
    ctrl_write_done = 0;
    nstate_wr = cstate_wr;
    case(cstate_wr)
        ST_IDLE: begin
            if(i_start == 2'b11)
                nstate_wr = ST_DMA;
            else 
                nstate_wr = ST_IDLE;
        end
        ST_DMA: begin
            nstate_wr = ST_DMA_WAIT;
            ctrl_write = 1;
        end
        ST_DMA_WAIT: begin
            ctrl_write_wait = 1;
            if(write_done) begin 
                if (req_blk_idx_wr == (max_req_blk_idx_wr) - 1)
                    nstate_wr = ST_DMA_DONE;
                else 
                    nstate_wr = ST_DMA_SYNC;
            end 
        end
        ST_DMA_SYNC: begin 
            ctrl_write_sync = 1;
            nstate_wr = ST_DMA;
        end 
        ST_DMA_DONE: begin
            ctrl_write_done = 1;
            nstate_wr = ST_IDLE;
        end
        default: nstate_wr = ST_IDLE;
    endcase 
end 

always_ff @(posedge clk or negedge rstn) begin
    if(~rstn) begin
        req_blk_idx_wr <= 0;
    end
    else begin
        if(write_done) begin 
           if(req_blk_idx_wr == (max_req_blk_idx_wr) - 1)
                req_blk_idx_wr <= 0;                // Reset the counter
            else 
                req_blk_idx_wr <= req_blk_idx_wr + 1;   // Up-Counter    
        end 
    end
end

always_ff @(posedge clk or negedge rstn) begin
    if(~rstn) begin
        write_data_cnt <= 0;
    end
    else begin
        if(write_done)
            write_data_cnt <= 0;
        else if (indata_req_wr) begin
            if(write_data_cnt == num_trans - 1)
                write_data_cnt <= 0;
            else 
                write_data_cnt <= write_data_cnt + 1;
        end
    end
end

// always_ff @(posedge clk or negedge rstn) begin
//     if(~rstn) begin
//         write_data_cnt <= 0;
//     end
//     else begin
//         if(ctrl_write)
//             write_data_cnt <= 0;
//         else if (indata_req_wr) begin 
//             if(write_data_cnt == num_trans - 1)
//                 write_data_cnt <= 0;
//             else 
//                 write_data_cnt <= write_data_cnt + 1;
//         end 
//     end
// end

assign write_addr = dram_base_addr_wr + {req_blk_idx_wr, 6'b0} + {write_data_cnt, 2'b0}; 

endmodule


