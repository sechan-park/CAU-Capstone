//----------------------------------------------------------------+
//----------------------------------------------------------------+
// Project: Deep Learning Hardware Design Contest
// Module: dma_write (ported from axi_dma_wr)
// Description:
//		Write the final output feature map to DRAM by AXI4
//
// Ported for Capstone Design Project
// Last Updated: 2025-10-11 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns/1ps

module dma_write #(
    // Parameters
    parameter BITS_TRANS = 18,
    parameter OUT_BITS_TRANS = 13,
    parameter AXI_WIDTH_USER = 1,              // Master ID
    parameter AXI_WIDTH_ID   = 4,              // ID width in bits
    parameter AXI_WIDTH_AD   = 32,             // address width
    parameter AXI_WIDTH_DA   = 32,             // data width
    parameter AXI_WIDTH_DS   = (AXI_WIDTH_DA/8),// data strobe width
    // AXI4 User signal widths (for compatibility with M00_AXI)
    parameter AXI_WIDTH_AWUSER = 0,            // Write address user signal width
    parameter AXI_WIDTH_WUSER  = 0,            // Write data user signal width
    parameter AXI_WIDTH_BUSER  = 0             // Write response user signal width
)(
    //AXI Master Interface - AXI4 Compatible
    //Write address channel
    output logic                     M_AXI_AWVALID,   // address/control valid handshake
    input  logic                     M_AXI_AWREADY,
    output logic [AXI_WIDTH_AD-1:0]  M_AXI_AWADDR,    // Address Write
    output logic [AXI_WIDTH_ID-1:0]  M_AXI_AWID,      // Address ID
    output logic [7:0]               M_AXI_AWLEN,     // Transfer length
    output logic [2:0]               M_AXI_AWSIZE,    // Transfer width
    output logic [1:0]               M_AXI_AWBURST,   // Burst type
    output logic                     M_AXI_AWLOCK,    // Atomic access (AXI4: 1-bit)
    output logic [3:0]               M_AXI_AWCACHE,   // Cachable/bufferable infor
    output logic [2:0]               M_AXI_AWPROT,    // Protection info
    output logic [3:0]               M_AXI_AWQOS,
    output logic [AXI_WIDTH_AWUSER-1:0] M_AXI_AWUSER, // User defined (parameterized)
                           
    //Write data channel       
    output logic                     M_AXI_WVALID,    // Write data valid
    input  logic                     M_AXI_WREADY,    // Write data ready
    output logic [AXI_WIDTH_DA-1:0]  M_AXI_WDATA,     // Write Data bus
    output logic [AXI_WIDTH_DS-1:0]  M_AXI_WSTRB,     // Write Data byte lane strobes
    output logic                     M_AXI_WLAST,     // Last beat of a burst transfer
    // Note: M_AXI_WID removed - deprecated in AXI4
    output logic [AXI_WIDTH_WUSER-1:0] M_AXI_WUSER,   // User defined (parameterized)

    //Write response channel
    input  logic                     M_AXI_BVALID,    // Response info valid
    output logic                     M_AXI_BREADY,    // Response info ready (to slave)
    input  logic [1:0]               M_AXI_BRESP,     // Buffered write response
    input  logic [AXI_WIDTH_ID-1:0]  M_AXI_BID,       // buffered response ID
    input  logic [AXI_WIDTH_BUSER-1:0] M_AXI_BUSER,   // User defined (parameterized)

    //Functional Ports (original naming from axi_dma_wr)
    input  logic                       start_dma,
    output logic                       done_o,
    input  logic [OUT_BITS_TRANS-1:0]  num_trans,      // Number of 32-bit words
    input  logic [AXI_WIDTH_DA-1:0]    start_addr,
    input  logic [AXI_WIDTH_DA-1:0]    indata,
    output logic                       indata_req_o,
    output logic                       fail_check,      // For debugging

    //Global signals
    input  logic                       clk,
    input  logic                       rstn
);

//---------------------------------------------------------------------
// parameter definitions 
//---------------------------------------------------------------------
   localparam FIXED_BURST_SIZE = 256;  //It can be ~256
   localparam DEFAULT_ID = 0;
   //AXI data width: number of bytes
   localparam  SIZE_1B     = 3'b000;
   localparam  SIZE_2B     = 3'b001;
   localparam  SIZE_4B     = 3'b010;
   localparam  SIZE_8B     = 3'b011;
   localparam  SIZE_16B    = 3'b100;      // not supported    
   localparam  SIZE_32B    = 3'b101;      // not supported
   localparam  SIZE_64B    = 3'b110;      // not supported
   localparam  SIZE_128B   = 3'b111;      // not supported
   
   localparam  RESP_OKAY   = 2'b00;
   localparam  RESP_EXOKAY = 2'b01;
   localparam  RESP_SLVERR = 2'b10;
   localparam  RESP_DECERR = 2'b11;
   localparam LOG_BURST_SIZE = $clog2(FIXED_BURST_SIZE);

//---------------------------------------------------------------------
// Internal signals - AXI4 Compatible
//---------------------------------------------------------------------
  logic [AXI_WIDTH_AD-1:0] ext_awaddr;
  logic [7:0]              ext_awlen;
  logic [2:0]              ext_awsize;
  logic                    ext_awvalid;
  logic                    ext_awready;
  logic [AXI_WIDTH_DA-1:0] ext_wdata;
  logic [AXI_WIDTH_DS-1:0] ext_wstrb;
  logic                    ext_wlast;
  logic                    ext_wvalid;
  logic                    ext_wready;
  logic [AXI_WIDTH_ID-1:0] ext_bid;
  logic [1:0]              ext_bresp;
  logic                    ext_bvalid;
  logic                    ext_bready;

   // AXI4 signal assignments
   assign M_AXI_AWID = DEFAULT_ID;
   // Note: M_AXI_WID removed - deprecated in AXI4
   assign M_AXI_AWBURST = 2'b01;  //Increase mode
   assign M_AXI_AWLOCK = 1'b0;    // AXI4: single bit, normal access
   assign M_AXI_AWCACHE = 4'b0011;  // Bufferable & Cacheable (VIP narrow-cache check friendly)
   assign M_AXI_AWPROT = 3'b000;
   assign M_AXI_AWQOS = 4'b1111;
   assign M_AXI_AWUSER = 'b0;     // Parameterized width
   assign M_AXI_WUSER = 'b0;      // Parameterized width

   assign  M_AXI_AWVALID = ext_awvalid;
   assign  M_AXI_AWADDR  = ext_awaddr;
   assign  M_AXI_AWLEN   = ext_awlen;
   assign  M_AXI_AWSIZE  = ext_awsize;
   assign  ext_awready = M_AXI_AWREADY;
               
   assign  M_AXI_WVALID  = ext_wvalid;
   assign  M_AXI_WDATA   = ext_wdata;
   assign  M_AXI_WSTRB   = ext_wstrb;
   assign  M_AXI_WLAST   = ext_wlast;
   assign  ext_wready = M_AXI_WREADY;
               
   assign  ext_bid     = M_AXI_BID;
   assign  ext_bresp   = M_AXI_BRESP;
   assign  ext_bvalid  = M_AXI_BVALID;
   assign  M_AXI_BREADY    = ext_bready;

   logic [OUT_BITS_TRANS-1:0] num_trans_d;
   logic [7:0] d_beat_cnt_wr, q_beat_cnt_wr;
   logic [OUT_BITS_TRANS-1:0] d_burst_cnt_wr, q_burst_cnt_wr;
   logic [7:0] q_burst_size_wr;
   logic [8:0] q_burst_size_wr_1;   //added 1 to q_burst_size_wr
   logic [AXI_WIDTH_AD-1:0] q_ext_addr_wr; //current AXI address for Write

   //FSM for Write to axi
   logic [2:0] st_wr2axi, next_st_wr2axi;
   localparam  WR_IDLE = 0,
               WR_PRE = 1,
               WR_START = 2,
               WR_BUFF_WAIT =3,	// Reserved
               WR_SEQ = 4,
               WR_WAIT = 5;

//---------------------------------------------------------------------
// Module designs 
//---------------------------------------------------------------------
   //---------------------------------------------------------------
   // FSM for Write data to AXI Interface 
   //---------------------------------------------------------------
   always_ff @(posedge clk or negedge rstn) begin
      if(!rstn) num_trans_d <= 'h0;
      else if(start_dma) num_trans_d <= num_trans;
   end
   
   always_ff @(posedge clk or negedge rstn) begin
      if(!rstn) begin
         q_beat_cnt_wr <= 0;
         q_burst_cnt_wr <= 0;
      end
      else begin
         q_beat_cnt_wr <= d_beat_cnt_wr;
         q_burst_cnt_wr <= d_burst_cnt_wr;
      end
   end
   
   always_ff @(posedge clk or negedge rstn)
   begin
      if(!rstn) begin
         q_burst_size_wr <= 0; 
         q_burst_size_wr_1 <= 0; 
      end
      else if(q_burst_cnt_wr + FIXED_BURST_SIZE > num_trans_d)
      begin
         q_burst_size_wr <= num_trans_d[LOG_BURST_SIZE-1:0] - 1;
         q_burst_size_wr_1 <= num_trans_d[LOG_BURST_SIZE-1:0];
      end
      else begin
         q_burst_size_wr <= FIXED_BURST_SIZE-1;
         q_burst_size_wr_1 <= FIXED_BURST_SIZE;
      end
   end
   
   always_ff @(posedge clk or negedge rstn) begin
      if(!rstn) 
         q_ext_addr_wr <= 0;
      else if(start_dma)
         q_ext_addr_wr <= start_addr;
      else if((st_wr2axi == WR_WAIT) && (next_st_wr2axi == WR_PRE) && (ext_bresp == RESP_OKAY))
         q_ext_addr_wr <= q_ext_addr_wr + {q_burst_size_wr_1, {2'b00}}; //4 Byte
   end

   always_ff @(posedge clk or negedge rstn)
      if(!rstn)  st_wr2axi <= WR_IDLE;
      else       st_wr2axi <= next_st_wr2axi;

   always_comb begin
      next_st_wr2axi = st_wr2axi;
      d_beat_cnt_wr = q_beat_cnt_wr;
      d_burst_cnt_wr = q_burst_cnt_wr;
      
      indata_req_o = 1'b0;
      //AXI signals for write
      ext_awvalid = 1'b0;
      ext_awaddr = 0;
      ext_awlen = 0;
      ext_awsize = 0;
      ext_wvalid = 1'b0;
      ext_wdata = 0;
      ext_wstrb = 0;
      ext_wlast = 1'b0;
      ext_bready = 1'b0;
      done_o = 1'b0;
      fail_check = 1'b0;
      
      case(st_wr2axi)
         default: next_st_wr2axi = WR_IDLE;
         WR_IDLE: begin
            if(start_dma)
               next_st_wr2axi = WR_PRE;
         end
         WR_PRE: begin
            if(q_burst_cnt_wr == num_trans_d) begin   //end of blk transfer
               d_burst_cnt_wr = 0;
               next_st_wr2axi = WR_IDLE;
               done_o = 1'b1;
            end
            else
               next_st_wr2axi = WR_START;
         end
         WR_START: begin   //start burst transfer
               ext_awvalid = 1'b1; //start write cmd
               ext_awaddr = q_ext_addr_wr;
               ext_awlen = q_burst_size_wr;
               ext_awsize = SIZE_4B;      //data width is 32bit
            if(ext_awready) begin //valid data and axiwr is ready
               // indata_req_o = 1'b1;
				   next_st_wr2axi = WR_SEQ;
            end
         end
         WR_SEQ: begin
            if(ext_wready) begin
               ext_wvalid = 1'b1;   //start output data
               ext_wdata = indata;
               ext_wstrb = {AXI_WIDTH_DS{1'b1}};  //no support for narrow transfer

               indata_req_o = 1'b1;

               if(q_burst_size_wr == q_beat_cnt_wr) begin //last beat of burst
                  d_beat_cnt_wr = 'h0;
                  ext_wlast = 1'b1;
                  next_st_wr2axi = WR_WAIT;
               end
               else begin
                  // indata_req_o = 1'b1; (try to fix! - jimin)
                  d_beat_cnt_wr = q_beat_cnt_wr + 1'b1;
               end
            end
         end
         WR_WAIT: begin //wait bresp from AXI
            ext_bready = 1'b1;
            if(ext_bvalid) begin
               if((ext_bresp == RESP_OKAY)) begin //TODO: to compatible with AXI IC
                  d_burst_cnt_wr = q_burst_cnt_wr + q_burst_size_wr_1;
                  d_beat_cnt_wr = 0;
                  next_st_wr2axi = WR_PRE;   
               end  //when bresp is wrong, back to start state and repeat recent burst 
               else begin 
                  d_beat_cnt_wr = 0;
                  next_st_wr2axi = WR_PRE;
                  fail_check = 1'b1;
               end            
            end
         end
      endcase
   end

endmodule
