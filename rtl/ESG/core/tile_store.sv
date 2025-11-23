//----------------------------------------------------------------+
// Project: Capstone Design
// Module : tile_store (SystemVerilog)
// Description:
//   Capture C tile stream (row-major) into ping-pong BRAM and write
//   back to DRAM using AXI4 via existing dma_write.
//   - Per-row bursts for correctness (handles edges naturally).
//   - Full tile (8x8): 8 bursts of 8 beats (32B/row).
//   - Edge tile: n_eff rows, AWLEN=m_eff-1.
//
// Notes:
//   - Exposes AXI master signals from internal dma_write instance.
//   - stride_c_row_bytes should be M*4 (bytes per C row).
// Last Updated: 2025-11-11 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns/1ps

import sa_params_pkg::*;
`include "sa_defs.svh"

module tile_store #(
  parameter int TILE_SIZE       = 8,
  parameter int DATA_W          = 32,
  // AXI USER width parameterized for clean integration with dma_write and TB
  parameter int AXI_USER_WIDTH  = 1
 )(
  input  logic                         clk,
  input  logic                         rstn,

  // Tile meta
  input  logic [AXI_ADDR_WIDTH-1:0]    base_c_addr,       // byte addr base_C
  input  logic [31:0]                  stride_c_row_bytes,// M*4
  input  logic [15:0]                  i0,                // tile row origin
  input  logic [15:0]                  j0,                // tile col origin
  input  logic [3:0]                   n_eff,             // 1..8
  input  logic [3:0]                   m_eff,             // 1..8

  // Store control
  input  logic                         start_store,       // pulse to begin write-back for one tile bank

  // External C buffer (ping-pong) consumer interface
  output logic                         cbuf_consume_req,  // pulse: arm read bank
  input  logic                         cbuf_consume_busy, // optional observability
  output logic [$clog2(64)-1:0]        cbuf_rd_addr,      // word address within tile bank
  input  logic [DATA_W-1:0]            cbuf_rd_data,      // registered data (1-cycle)
  output logic                         cbuf_rd_en,        // assert per beat when data consumed (USE_CONS_COMMIT=0)
  output logic                         cbuf_cons_commit,  // pulse: release bank after done

  // Status
  output logic                         wr_busy,
  output logic                         wr_done,           // pulse per tile write-back

  // AXI4 Master (connect upward)
  output logic                         M_AXI_AWVALID,
  input  logic                         M_AXI_AWREADY,
  output logic [AXI_ADDR_WIDTH-1:0]    M_AXI_AWADDR,
  output logic [3:0]                   M_AXI_AWID,
  output logic [7:0]                   M_AXI_AWLEN,
  output logic [2:0]                   M_AXI_AWSIZE,
  output logic [1:0]                   M_AXI_AWBURST,
  output logic                         M_AXI_AWLOCK,
  output logic [3:0]                   M_AXI_AWCACHE,
  output logic [2:0]                   M_AXI_AWPROT,
  output logic [3:0]                   M_AXI_AWQOS,
  output logic [AXI_USER_WIDTH-1:0]    M_AXI_AWUSER,

  output logic                         M_AXI_WVALID,
  input  logic                         M_AXI_WREADY,
  output logic [DATA_W-1:0]            M_AXI_WDATA,
  output logic [(DATA_W/8)-1:0]        M_AXI_WSTRB,
  output logic                         M_AXI_WLAST,
  output logic [AXI_USER_WIDTH-1:0]    M_AXI_WUSER,

  input  logic                         M_AXI_BVALID,
  output logic                         M_AXI_BREADY,
  input  logic [1:0]                   M_AXI_BRESP,
  input  logic [3:0]                   M_AXI_BID,
  input  logic [AXI_USER_WIDTH-1:0]    M_AXI_BUSER
);

  // Control FSM (single always_ff style)
  typedef enum logic [2:0] { S_IDLE, S_PREP_ROW, S_REQ, S_DMA, S_NEXT_ROW, S_DONE } state_e;
  state_e state;

  logic [3:0] row_idx;     // 0..7
  logic [3:0] col_idx;     // 0..7

  // DMA write instance and parameters
  logic        start_dma, done_dma, indata_req_o;
  // Per-burst parameters
  logic [12:0] dma_words_q;                    // beats in current burst
  logic [AXI_ADDR_WIDTH-1:0] dma_start_addr_q; // start addr of current burst
  // Per-row parameters (for addr_gen)
  logic [12:0] row_words_q;                    // words per row (= m_eff)
  logic [AXI_ADDR_WIDTH-1:0] row_base_q;       // base addr of current row
  // Track whether the accepted burst is the last one for this row
  logic        burst_last_q;
  // Track first WVALID handshake within S_DMA to open a read window
  //   logic wvalid_q;           // previous WVALID level

  // try a new thing!! (jimin-251114)
  // logic rd_window_q;        // latched window after first WVALID handshake
  // logic w_fire;             // AXI W channel handshake (VALID && READY)
  // assign w_fire = M_AXI_AWVALID & M_AXI_AWREADY;

  // try a new thing!! (jimin-251114)
  logic rd_window_q;
  // logic w_aw_fire;
  // assign w_aw_fire = M_AXI_AWVALID & M_AXI_AWREADY;
  // a new thing end

  // try a new thing!! (jimin-251115)
  // ------------------------------------------------------------------
  // C-buffer → FIFO → dma_write 경로
  // ------------------------------------------------------------------
  // FIFO (32bit, depth=64, FWFT)
  logic [DATA_W-1:0] fifo_din;
  logic [DATA_W-1:0] fifo_dout;
  logic              fifo_wr_en;
  logic              fifo_rd_en;
  logic              fifo_full;
  logic              fifo_empty;
  logic              fifo_valid;      // FWFT 모드일 때 dout 유효 표시

  // C-buffer read enable 1cycle delay (BRAM 1cycle latency 정렬용)
  logic              cbuf_rd_en_d1;

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn)
      cbuf_rd_en_d1 <= 1'b0;
    else
      cbuf_rd_en_d1 <= cbuf_rd_en;
  end

  // C-buffer data → FIFO write
  assign fifo_din   = cbuf_rd_data;
  assign fifo_wr_en = cbuf_rd_en_d1 & ~fifo_full;

  // dma_write가 요청할 때마다 FIFO read
  assign fifo_rd_en = indata_req_o & ~fifo_empty;
  // a new thing end


  // Address generator (WRITE): per-row bursts
  logic                 ag_start, ag_done;
  logic                 ag_req_valid, ag_req_ready, ag_req_last;
  logic [AXI_ADDR_WIDTH-1:0] ag_req_addr;
  logic [7:0]            ag_req_len; // beats-1

  // try a new thing!! (jimin-251115)
  c_buf_fifo u_c_buf_fifo (
    .clk    (clk),
    .srst   (~rstn),
    .din    (fifo_din),
    .wr_en  (fifo_wr_en),
    .rd_en  (fifo_rd_en),
    .dout   (fifo_dout),
    .full   (fifo_full),
    .empty  (fifo_empty),
    .valid  (fifo_valid)
  );
  // a new thing end

  axi_addr_gen #(
    .ADDR_W          (AXI_ADDR_WIDTH),
    .P_BYTES_PER_BEAT(AXI_DATA_WIDTH/8),
    .P_MAX_BURST     (16)
  ) u_ag (
    .clk(clk), .rstn(rstn),
    .start   (ag_start),
    .base_addr(row_base_q),
    .bytes_total(row_words_q* (AXI_DATA_WIDTH/8)),
    .stride_bytes(32'd0),
    .req_valid(ag_req_valid), .req_ready(ag_req_ready),
    .req_addr(ag_req_addr), .req_len(ag_req_len), .req_last(ag_req_last),
    .done(ag_done)
  );

  dma_write #(
    .AXI_WIDTH_AD     (AXI_ADDR_WIDTH),
    .AXI_WIDTH_DA     (DATA_W),
    .AXI_WIDTH_AWUSER (AXI_USER_WIDTH),
    .AXI_WIDTH_WUSER  (AXI_USER_WIDTH),
    .AXI_WIDTH_BUSER  (AXI_USER_WIDTH)
  ) u_dma_w (
    // AXI
    .M_AXI_AWVALID(M_AXI_AWVALID), .M_AXI_AWREADY(M_AXI_AWREADY), .M_AXI_AWADDR(M_AXI_AWADDR),
    .M_AXI_AWID   (M_AXI_AWID),    .M_AXI_AWLEN  (M_AXI_AWLEN),   .M_AXI_AWSIZE(M_AXI_AWSIZE),
    .M_AXI_AWBURST(M_AXI_AWBURST), .M_AXI_AWLOCK (M_AXI_AWLOCK),  .M_AXI_AWCACHE(M_AXI_AWCACHE),
    .M_AXI_AWPROT (M_AXI_AWPROT),  .M_AXI_AWQOS  (M_AXI_AWQOS),   .M_AXI_AWUSER(M_AXI_AWUSER),
    .M_AXI_WVALID (M_AXI_WVALID),  .M_AXI_WREADY (M_AXI_WREADY),  .M_AXI_WDATA (M_AXI_WDATA),
    .M_AXI_WSTRB  (M_AXI_WSTRB),   .M_AXI_WLAST  (M_AXI_WLAST),   .M_AXI_WUSER (M_AXI_WUSER),
    .M_AXI_BVALID (M_AXI_BVALID),  .M_AXI_BREADY (M_AXI_BREADY),  .M_AXI_BRESP (M_AXI_BRESP),
    .M_AXI_BID    (M_AXI_BID),     .M_AXI_BUSER  (M_AXI_BUSER),
    // functional
    .start_dma(start_dma), .done_o(done_dma),
    .num_trans(dma_words_q), .start_addr(dma_start_addr_q),
    // .indata(cbuf_rd_data), .indata_req_o(indata_req_o),
    // try a new thing!! (jimin-251115)
    .indata(fifo_dout), .indata_req_o(indata_req_o),
    // a new thing end
    .fail_check(),
    .clk(clk), .rstn(rstn)
  );

  // Single always_ff: state, outputs, and registers
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      state            <= S_IDLE;
      row_idx          <= '0; col_idx <= '0;
      burst_last_q     <= 1'b0;
      row_words_q      <= '0;
      row_base_q       <= '0;
      dma_words_q      <= '0;
      dma_start_addr_q <= '0;
      // outputs
      cbuf_consume_req <= 1'b0;
      cbuf_cons_commit <= 1'b0;
      cbuf_rd_addr     <= '0;
      cbuf_rd_en       <= 1'b0;
      wr_done          <= 1'b0;
      wr_busy          <= 1'b0;
      ag_start         <= 1'b0;
      ag_req_ready     <= 1'b0;
      start_dma        <= 1'b0;
      // WVALID track
//       wvalid_q         <= 1'b0;
      rd_window_q      <= 1'b0;
    end else begin
      // defaults each cycle
      cbuf_consume_req <= 1'b0;
      cbuf_cons_commit <= 1'b0;
    //   cbuf_rd_addr     <= '0;
      cbuf_rd_en       <= 1'b0;
      wr_done          <= 1'b0;
      ag_start         <= 1'b0;
      ag_req_ready     <= 1'b0;
      start_dma        <= 1'b0;
      // track previous WVALID each cycle (for edge-handling if needed)
//       wvalid_q         <= M_AXI_WVALID;

      case (state)
        S_IDLE: begin
          wr_busy      <= 1'b0;
          row_idx      <= '0; col_idx <= '0;
          burst_last_q <= 1'b0;
          if (start_store) begin
            cbuf_consume_req <= 1'b1;

            row_base_q       <= base_c_addr + (i0 * stride_c_row_bytes) + (j0 * (DATA_W/8));
            state            <= S_PREP_ROW;
          end else begin
            state            <= S_IDLE;
          end
        end

        S_PREP_ROW: begin
          wr_busy      <= 1'b1;
          row_words_q  <= m_eff;
          // row_base_q   <= base_c_addr + ( (i0 + row_idx) * stride_c_row_bytes ) + ( j0 * (DATA_W/8) );
          ag_start     <= 1'b1;  // one-cycle pulse
          col_idx      <= 4'd0;
          burst_last_q <= 1'b0;
          state        <= S_REQ;
        end

        S_REQ: begin
          wr_busy      <= 1'b1;
          ag_req_ready <= 1'b1;
          if (ag_req_valid) begin
            dma_words_q      <= ag_req_len + 8'd1;
            dma_start_addr_q <= ag_req_addr;
            burst_last_q     <= ag_req_last;
            cbuf_rd_addr      <= (row_idx*TILE_SIZE);

            // try a new thing!! (jimin-251114)
            col_idx          <= 4'd0;

            // start_dma        <= 1'b1;
            rd_window_q      <= 1'b0;   // clear window for new burst
            state            <= S_DMA;
          end else begin
            state            <= S_REQ;
          end
        end

        // S_DMA: begin
        //   wr_busy      <= 1'b1;
        //   start_dma    <= 1'b0;
        //   // Latch window open on the first W handshake; keep it until end of burst
        //   // try a new thing!! (jimin-251114)
        //   // if (!rd_window_q && w_fire) begin
        //   //   rd_window_q <= 1'b1;
        //   // end

        //   // try a new thing!! (jimin-251114)
        //   if (!rd_window_q && w_aw_fire) begin
        //     rd_window_q <= 1'b1;
        //     if (m_eff != 0) begin
        //       cbuf_rd_en <= 1'b1;
        //       col_idx    <= 4'd1;
        //       // cbuf_rd_addr <= cbuf_rd_addr + 1'b1;
        //     end else begin
        //       col_idx    <= 4'd0;
        //     end
        //   end

        //   if (rd_window_q && indata_req_o && (col_idx < m_eff)) begin
        //     cbuf_rd_en  <= 1'b1;
        //     cbuf_rd_addr <= cbuf_rd_addr + 1'b1;
        //     col_idx     <= col_idx + 1'b1;
        //   end
          
        //   // Drive BRAM read only after the first WVALID handshake
        //   // try a new thing!! (jimin-251114)
        //   // if ( (rd_window_q || w_fire) && (col_idx < m_eff) ) begin
        //   //   cbuf_rd_en  <= 1'b1;
        //   //   col_idx     <= col_idx + 1'b1;
        //   //   cbuf_rd_addr<= cbuf_rd_addr + 1'b1;
        //   // end

        //   if (done_dma) begin
        //     state <= (burst_last_q) ? S_NEXT_ROW : S_REQ;
        //     cbuf_rd_en <= 1'b0;
        //     rd_window_q <= 1'b0; // close window at burst end
        //   end else begin
        //     state <= S_DMA;
        //   end
        // end

        // try a new thing!! (jimin-251115)
        S_DMA: begin
          wr_busy   <= 1'b1;
          start_dma <= 1'b0;

          // 1) 아직 이 row의 C 데이터를 다 못 읽었으면
          //    C 버퍼에서 한 word씩 읽어서 FIFO에 채운다.
          //    cbuf_rd_en 은 항상 여기서만 발생.
          if (col_idx < m_eff) begin
            if (!fifo_full) begin
              cbuf_rd_en <= 1'b1;
              // 첫 번째 read는 S_REQ에서 설정한 주소를 그대로 쓰고,
              // 이후부터는 +1씩 증가
              if (col_idx != 0) begin
                cbuf_rd_addr <= cbuf_rd_addr + 1'b1;
              end
              col_idx <= col_idx + 1'b1;
            end
          end
          // 2) C에서 m_eff개를 다 읽어서 FIFO에 넣었으면
          //    이제 dma_write 를 시작한다.
          else begin
            // 아직 DMA를 안 시작했으면 한 번만 start_dma 펄스 발생
            if (!rd_window_q) begin
              start_dma   <= 1'b1;
              rd_window_q <= 1'b1;   // "DMA 시작했음" 플래그로 재사용
            end
          end

          // 3) dma_write 가 전체 burst를 끝내면 다음 row로 이동
          if (done_dma) begin
            rd_window_q  <= 1'b0;
            cbuf_rd_en   <= 1'b0;
            state        <= (burst_last_q) ? S_NEXT_ROW : S_REQ;
          end else begin
            state        <= S_DMA;
          end
        end
        // a new thing end

        S_NEXT_ROW: begin
          wr_busy      <= 1'b1;
          if (row_idx + 1 < n_eff) begin
            row_idx <= row_idx + 1'b1;
            col_idx <= 4'd0;
            row_base_q <= row_base_q + stride_c_row_bytes;
            // cbuf_consume_req <= 1'b1;
            state   <= S_PREP_ROW;
          end else begin
            state   <= S_DONE;
          end
        end

        S_DONE: begin
          wr_busy          <= 1'b0;
          cbuf_cons_commit <= 1'b1; // ignored by USE_CONS_COMMIT=0 but harmless
          wr_done          <= 1'b1;  // completion pulse
          state            <= S_IDLE;
        end

      endcase
    end
  end

endmodule
