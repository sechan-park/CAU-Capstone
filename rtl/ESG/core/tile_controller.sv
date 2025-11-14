//----------------------------------------------------------------+
// Project: Capstone Design
// Module : tile_controller (SystemVerilog module)
// Description:
//   3-stage pipelined orchestrator for READ/COMPUTE/WRITE overlap
//   - Loader FSM: prefetch B blocks into ping-pong banks
//   - Compute FSM: process tiles using loaded banks
//   - Store FSM: write-back C tiles
//
//   Pipeline:
//     T0: LOAD_A_BULK
//     T1: LOAD_B[block0→bank0]
//     T2: COMPUTE[tile0] + LOAD_B[block1→bank1]
//     T3: STORE[tile0] + COMPUTE[tile1] + LOAD_B continues
//     ...
//
// Last Updated: 2025-11-12 (by Jimin Hwang)
//----------------------------------------------------------------+

`timescale 1ns/1ps

import sa_params_pkg::*;
`include "sa_defs.svh"

module tile_controller #(
  parameter int TILE_SIZE = 8
)(
  input  logic                         clk,
  input  logic                         rstn,

  // ===== Control from AXI-Lite =====
  input  logic                         start,
  input  logic                         update_A,        // 1: reload A bulk
  input  logic [15:0]                  N, K, M,         // global matrix dims
  input  logic [15:0]                  BLOCK_M,         // B block width
  input  logic [AXI_ADDR_WIDTH-1:0]    base_A, base_B, base_C,
  input  logic [31:0]                  stride_A_row_bytes,
  input  logic [31:0]                  stride_B_row_bytes,
  input  logic [31:0]                  stride_C_row_bytes,

  // ===== To tile_loader =====
  output logic                         loader_req,      // renamed from load_req
  output logic                         loader_update_a,
  output logic [15:0]                  loader_N,
  output logic [15:0]                  loader_K,
  output logic [15:0]                  loader_M,
  output logic [15:0]                  loader_BLOCK_M,
  output logic [15:0]                  loader_j_block,
  output logic [AXI_ADDR_WIDTH-1:0]    loader_base_a,
  output logic [AXI_ADDR_WIDTH-1:0]    loader_base_b,
  output logic [31:0]                  loader_stride_a_row,
  output logic [31:0]                  loader_stride_b_row,
  input  logic                         ld_done,

  // ===== To tile_compute =====
  output logic                         start_tile,
  output logic [15:0]                  compute_k_total,
  output logic [3:0]                   compute_n_eff,
  output logic [3:0]                   compute_m_eff,
  input  logic                         tile_done,
  // Loader interface (tile_compute → orchestrator → A/B buffers)
  input  logic                         tile_load_req,   // renamed: from tile_compute
  input  logic [3:0]                   k_eff,           // from tile_compute
  output logic                         a_ld_start,
  output logic                         a_ld_valid,
  output logic [31:0]                  a_ld_data,
  output logic                         b_ld_start,
  output logic                         b_ld_valid,
  output logic [31:0]                  b_ld_data,
  
  // ===== C buffer fill completion (from C bram_pingpong) =====
  input  logic                         c_fill_done,     // C tile write complete

  // ===== A buffer read interface (dpram_wrapper) =====
  output logic [13:0]                  a_rd_addr,      // 14-bit for 12,288 words
  output logic                         a_rd_en,
  input  logic [31:0]                  a_rd_data,

  // ===== B buffer consume interface (bram_pingpong) =====
  output logic                         b_consume_req,
  input  logic                         b_consume_busy,
  output logic [15:0]                  b_rd_addr,      // 16-bit for 49,152 words
  output logic                         b_rd_en,
  input  logic [31:0]                  b_rd_data,
  output logic                         b_cons_commit,  // commit when block done

  // ===== To tile_store =====
  output logic                         start_store,
  output logic [15:0]                  store_i0,
  output logic [15:0]                  store_j0,
  output logic [3:0]                   store_n_eff,
  output logic [3:0]                   store_m_eff,
  output logic [AXI_ADDR_WIDTH-1:0]    store_base_c,
  output logic [31:0]                  store_stride_c_row,
  input  logic                         wr_done,

  // ===== Status =====
  output logic                         busy,
  output logic                         done
);

  // ========== Loader FSM ==========
  typedef enum logic [2:0] {
    LD_IDLE,
    LD_A_BULK,
    LD_A_WAIT,
    LD_B_BLOCK,
    LD_B_WAIT,
    LD_DONE
  } loader_state_e;

  loader_state_e ld_state;
  logic [15:0] ld_j_block;           // current B block being loaded
  logic        ld_a_loaded;          // A bulk loaded flag
  logic        ld_all_blocks_loaded; // all B blocks loaded

  // ========== Compute FSM ==========
  typedef enum logic [2:0] {
    CP_IDLE,
    CP_PREP,
    CP_RUN,
    CP_WAIT,
    CP_DONE
  } compute_state_e;

  compute_state_e cp_state;
  logic [15:0] cp_i0, cp_j0;         // current tile origin
  logic [15:0] cp_j_block;           // current B block being used
  logic [3:0]  cp_n_eff, cp_m_eff;   // effective tile size

  // ========== Streaming Sub-FSM (for load_req handling) ==========
  typedef enum logic [2:0] {
    STREAM_IDLE,
    STREAM_CONSUME,      // issue b_consume_req for new segment
    STREAM_START,        // issue a_ld_start, b_ld_start
    STREAM_PIPE,         // 1-cycle pipeline bubble for BRAM latency
    STREAM_DATA,         // latch rd_data, output ld_valid/ld_data
    STREAM_DONE
  } stream_state_e;

  stream_state_e stream_state;
  logic [15:0] stream_cnt_a;         // A stream word counter (max 8*8/4=16)
  logic [15:0] stream_cnt_b;         // B stream word counter (max 8*8/4=16)
  logic [3:0]  stream_k_eff;         // latched k_eff
  logic [15:0] k0_seg;               // current segment start in K dimension
  logic        first_segment;        // flag: first segment of tile (for b_consume_req)

  logic        new_tile_start;       // CP -> ST handshake signal

  // ========== Store FSM ==========
  typedef enum logic [2:0] {
    ST_IDLE,
    ST_STORE,
    ST_WAIT,
    ST_DONE
  } store_state_e;

  store_state_e st_state;

  // ========== Inter-stage Handshakes ==========
  // Loader → Compute: B block ready (with queue to handle timing)
  // logic        b_block_ready;        // pulse: new B block available (from loader)
  logic [15:0] b_block_id;           // j_block of ready block (from loader)
  
  // B block pending queue (simple 4-deep FIFO)
  localparam int B_QUEUE_DEPTH = 4;
  logic [15:0] b_queue [0:B_QUEUE_DEPTH-1];  // queue of pending j_blocks
  logic [2:0]  b_queue_wptr;                 // write pointer
  logic [2:0]  b_queue_rptr;                 // read pointer
  logic [2:0]  b_queue_count;                // number of entries
  logic        b_queue_push;                 // push signal (from loader)
  logic        b_queue_pop;                  // pop signal (from compute)
  logic        b_queue_empty;
  logic        b_queue_full;
  logic [15:0] b_queue_front;                // front entry

  // Compute → Store: C tile ready (with queue to handle timing)
  logic        c_tile_ready;         // pulse: tile computed, needs store
  logic [15:0] c_tile_i0, c_tile_j0; // tile origin
  logic [3:0]  c_tile_n_eff, c_tile_m_eff;
  
  // C tile pending queue (simple 4-deep FIFO)
  typedef struct packed {
    logic [15:0] i0;
    logic [15:0] j0;
    logic [3:0]  n_eff;
    logic [3:0]  m_eff;
  } c_tile_info_t;
  
  localparam int C_QUEUE_DEPTH = 4;
  c_tile_info_t c_queue [0:C_QUEUE_DEPTH-1];  // queue of pending C tiles
  logic [2:0]  c_queue_wptr;                  // write pointer
  logic [2:0]  c_queue_rptr;                  // read pointer
  logic [2:0]  c_queue_count;                 // number of entries
  logic        c_queue_push;                  // push signal (from compute)
  logic        c_queue_pop;                   // pop signal (from store)
  logic        c_queue_empty;
  logic        c_queue_full;
  c_tile_info_t c_queue_front;                // front entry

  // Store → Compute: backpressure
  logic        store_can_accept;     // 1: store can accept new tile

  // ========== Global Control ==========
  logic        global_start;
  logic        global_done;
  logic [15:0] reg_N, reg_K, reg_M, reg_BLOCK_M;
  logic [AXI_ADDR_WIDTH-1:0] reg_base_A, reg_base_B, reg_base_C;
  logic [31:0] reg_stride_A, reg_stride_B, reg_stride_C;
  logic        reg_update_A;

  // ========== Helper Functions ==========
  function automatic logic [3:0] calc_n_eff(input logic [15:0] i_origin);
    logic [15:0] rem;
    rem = reg_N - i_origin;
    if (rem >= TILE_SIZE)
      return TILE_SIZE[3:0];
    else
      return rem[3:0];
  endfunction

  function automatic logic [3:0] calc_m_eff(input logic [15:0] j_origin, input logic [15:0] j_blk);
    logic [15:0] block_end, rem;
    block_end = (j_blk + reg_BLOCK_M < reg_M) ? (j_blk + reg_BLOCK_M) : reg_M;
    rem = block_end - j_origin;
    if (rem >= TILE_SIZE)
      return TILE_SIZE[3:0];
    else
      return rem[3:0];
  endfunction

  // ========== Global Control Logic ==========
  // Detect global done (all 3 FSMs idle + all queues empty) - combinational
  assign global_done = (ld_state == LD_DONE) && 
                       (cp_state == CP_DONE) && 
                       (st_state == ST_DONE) &&  // ST_DONE: Store가 모든 작업 완료
                       b_queue_empty &&          // Compute가 처리할 B 블록이 없고
                       c_queue_empty;            // Store가 처리할 C 타일이 없어야 완료

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      global_start <= 1'b0;
      reg_N        <= '0;
      reg_K        <= '0;
      reg_M        <= '0;
      reg_BLOCK_M  <= '0;
      reg_base_A   <= '0;
      reg_base_B   <= '0;
      reg_base_C   <= '0;
      reg_stride_A <= '0;
      reg_stride_B <= '0;
      reg_stride_C <= '0;
      reg_update_A <= 1'b0;
      busy         <= 1'b0;
      done         <= 1'b0;
    end else begin
      done <= 1'b0;  // default pulse low

      if (start && !busy) begin
        // Latch params
        global_start <= 1'b1;
        reg_N        <= N;
        reg_K        <= K;
        reg_M        <= M;
        reg_BLOCK_M  <= BLOCK_M;
        reg_base_A   <= base_A;
        reg_base_B   <= base_B;
        reg_base_C   <= base_C;
        reg_stride_A <= stride_A_row_bytes;
        reg_stride_B <= stride_B_row_bytes;
        reg_stride_C <= stride_C_row_bytes;
        reg_update_A <= update_A;
        busy         <= 1'b1;
      end else if (global_done) begin
        global_start <= 1'b0;
        busy         <= 1'b0;
        done         <= 1'b1;  // pulse
      end
    end
  end

  // ========== B Block Queue Management ==========
  assign b_queue_empty = (b_queue_count == 0);
  assign b_queue_full  = (b_queue_count >= B_QUEUE_DEPTH);
  assign b_queue_front = b_queue[b_queue_rptr];  // front entry for compute to read

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      b_queue_wptr  <= '0;
      b_queue_rptr  <= '0;
      b_queue_count <= '0;
      for (int i=0; i<B_QUEUE_DEPTH; i++) b_queue[i] <= '0;
    end else begin
      // Push and Pop logic
      if (b_queue_push && !b_queue_full) begin
        b_queue[b_queue_wptr] <= b_block_id;
        b_queue_wptr          <= (b_queue_wptr + 1) % B_QUEUE_DEPTH;
        if (!b_queue_pop) begin
          b_queue_count <= b_queue_count + 1;
        end
      end
      
      if (b_queue_pop && !b_queue_empty) begin
        b_queue_rptr <= (b_queue_rptr + 1) % B_QUEUE_DEPTH;
        if (!b_queue_push) begin
          b_queue_count <= b_queue_count - 1;
        end
      end
      
      // Simultaneous push & pop: count stays same
    end
  end

  // ========== C Tile Queue Management ==========
  assign c_queue_empty = (c_queue_count == 0);
  assign c_queue_full  = (c_queue_count >= C_QUEUE_DEPTH);
  assign c_queue_front = c_queue[c_queue_rptr];  // front entry for store to read

  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      c_queue_wptr  <= '0;
      c_queue_rptr  <= '0;
      c_queue_count <= '0;
      for (int i=0; i<C_QUEUE_DEPTH; i++) c_queue[i] <= '0;
    end else begin
      // Push and Pop logic
      if (c_queue_push && !c_queue_full) begin
        c_queue[c_queue_wptr].i0    <= c_tile_i0;
        c_queue[c_queue_wptr].j0    <= c_tile_j0;
        c_queue[c_queue_wptr].n_eff <= c_tile_n_eff;
        c_queue[c_queue_wptr].m_eff <= c_tile_m_eff;
        c_queue_wptr                <= (c_queue_wptr + 1) % C_QUEUE_DEPTH;
        if (!c_queue_pop) begin
          c_queue_count <= c_queue_count + 1;
        end
      end
      
      if (c_queue_pop && !c_queue_empty) begin
        c_queue_rptr <= (c_queue_rptr + 1) % C_QUEUE_DEPTH;
        if (!c_queue_push) begin
          c_queue_count <= c_queue_count - 1;
        end
      end
      
      // Simultaneous push & pop: count stays same
    end
  end

  // ========== Loader FSM ==========
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      ld_state              <= LD_IDLE;
      ld_j_block            <= '0;
      ld_a_loaded           <= 1'b0;
      ld_all_blocks_loaded  <= 1'b0;
      loader_req            <= 1'b0;
      loader_update_a       <= 1'b0;
      loader_N              <= '0;
      loader_K              <= '0;
      loader_M              <= '0;
      loader_BLOCK_M        <= '0;
      loader_j_block        <= '0;
      loader_base_a         <= '0;
      loader_base_b         <= '0;
      loader_stride_a_row   <= '0;
      loader_stride_b_row   <= '0;
      // b_block_ready         <= 1'b0;
      b_block_id            <= '0;
      b_queue_push          <= 1'b0;
    end else begin
      // defaults
      loader_req    <= 1'b0;
      // b_block_ready <= 1'b0;
      b_queue_push  <= 1'b0;

      case (ld_state)
        LD_IDLE: begin
          ld_a_loaded          <= 1'b0;
          ld_all_blocks_loaded <= 1'b0;
          if (global_start) begin
            // Pass params
            loader_N            <= reg_N;
            loader_K            <= reg_K;
            loader_M            <= reg_M;
            loader_BLOCK_M      <= reg_BLOCK_M;
            loader_base_a       <= reg_base_A;
            loader_base_b       <= reg_base_B;
            loader_stride_a_row <= reg_stride_A;
            loader_stride_b_row <= reg_stride_B;
            ld_j_block          <= 16'd0;

            if (reg_update_A) begin
              ld_state <= LD_A_BULK;
            end else begin
              ld_a_loaded <= 1'b1;
              ld_state    <= LD_B_BLOCK;
            end
          end
        end

        LD_A_BULK: begin
          loader_req      <= 1'b1;  // pulse
          loader_update_a <= 1'b1;
          ld_state        <= LD_A_WAIT;
        end

        LD_A_WAIT: begin
          loader_update_a <= 1'b0;
          if (ld_done) begin
            ld_a_loaded <= 1'b1;
            ld_state    <= LD_B_BLOCK;
          end
        end

        LD_B_BLOCK: begin
          if (ld_j_block < reg_M) begin
            loader_req      <= 1'b1;  // pulse
            loader_update_a <= 1'b0;
            loader_j_block  <= ld_j_block;
            ld_state        <= LD_B_WAIT;
          end else begin
            // All blocks loaded
            ld_all_blocks_loaded <= 1'b1;
            ld_state             <= LD_DONE;
          end
        end

        LD_B_WAIT: begin
          if (ld_done) begin
            // Push B block to queue (instead of pulse)
            // b_block_ready <= 1'b1;
            b_block_id    <= ld_j_block;
            b_queue_push  <= 1'b1;  // push to queue
            // Advance to next block
            ld_j_block    <= ld_j_block + reg_BLOCK_M;
            ld_state      <= LD_B_BLOCK;
          end
        end

        LD_DONE: begin
          // Wait for global reset
          if (!busy) ld_state <= LD_IDLE;
        end

      endcase
    end
  end

  // ========== Compute FSM ==========
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      cp_state       <= CP_IDLE;
      cp_i0          <= '0;
      cp_j0          <= '0;
      cp_j_block     <= '0;
      cp_n_eff       <= '0;
      cp_m_eff       <= '0;
      start_tile     <= 1'b0;
      compute_k_total<= '0;
      compute_n_eff  <= '0;
      compute_m_eff  <= '0;
      c_tile_ready   <= 1'b0;
      c_tile_i0      <= '0;
      c_tile_j0      <= '0;
      c_tile_n_eff   <= '0;
      c_tile_m_eff   <= '0;
      b_queue_pop    <= 1'b0;
      c_queue_push   <= 1'b0;
      b_cons_commit  <= 1'b0;
      new_tile_start <= 1'b0;
    end else begin
      // defaults
      start_tile    <= 1'b0;
      c_tile_ready  <= 1'b0;
      b_queue_pop   <= 1'b0;
      c_queue_push  <= 1'b0;
      b_cons_commit <= 1'b0;
      new_tile_start <= 1'b0;
      case (cp_state)
        CP_IDLE: begin
          if (!b_queue_empty) begin
            // B block available in queue, pop it
            cp_j_block   <= b_queue_front;
            cp_i0        <= 16'd0;
            cp_j0        <= b_queue_front;
            b_queue_pop  <= 1'b1;  // consume from queue
            cp_state     <= CP_PREP;
          end
        end

        CP_PREP: begin
          // Prepare tile metadata
          cp_n_eff         <= calc_n_eff(cp_i0);
          cp_m_eff         <= calc_m_eff(cp_j0, cp_j_block);
          compute_k_total  <= reg_K;
          compute_n_eff    <= calc_n_eff(cp_i0);
          compute_m_eff    <= calc_m_eff(cp_j0, cp_j_block);
          new_tile_start   <= 1'b1;
          cp_state         <= CP_RUN;
        end

        CP_RUN: begin
          // Check if store can accept (backpressure)
          if (store_can_accept) begin
            start_tile <= 1'b1;  // pulse
            cp_state   <= CP_WAIT;
          end
        end

        CP_WAIT: begin
          // Wait for C buffer fill completion (not just tile_done!)
          // tile_done indicates PE computation done, but c_fill_done ensures
          // all 64 results are written to C bram_pingpong
          if (c_fill_done) begin
            // Push C tile to queue (instead of pulse)
            c_tile_ready  <= 1'b1;
            c_tile_i0     <= cp_i0;
            c_tile_j0     <= cp_j0;
            c_tile_n_eff  <= cp_n_eff;
            c_tile_m_eff  <= cp_m_eff;
            c_queue_push  <= 1'b1;  // push to queue

            // Advance to next tile
            if (cp_j0 + TILE_SIZE < ((cp_j_block + reg_BLOCK_M < reg_M) ? (cp_j_block + reg_BLOCK_M) : reg_M)) begin
              // Next column in same block
              cp_j0    <= cp_j0 + TILE_SIZE;
              cp_state <= CP_PREP;
            end else begin
              // Next row
              if (cp_i0 + TILE_SIZE < reg_N) begin
                cp_i0    <= cp_i0 + TILE_SIZE;
                cp_j0    <= cp_j_block;
                cp_state <= CP_PREP;
              end else begin
                // Current block done, commit B buffer
                b_cons_commit <= 1'b1;
                
                // Check queue for next B block
                if (!b_queue_empty) begin
                  // Next block available in queue
                  cp_j_block   <= b_queue_front;
                  cp_i0        <= 16'd0;
                  cp_j0        <= b_queue_front;
                  b_queue_pop  <= 1'b1;  // consume from queue
                  cp_state     <= CP_PREP;
                end else if (ld_all_blocks_loaded) begin
                  // No more blocks and queue empty
                  cp_state <= CP_DONE;
                end else begin
                  // Wait for next block in queue
                  cp_state <= CP_IDLE;
                end
              end
            end
          end
        end

        CP_DONE: begin
          // Wait for global reset
          if (!busy) cp_state <= CP_IDLE;
        end

      endcase
    end
  end

  // ========== Store FSM ==========
  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      st_state        <= ST_IDLE;
      start_store     <= 1'b0;
      store_i0        <= '0;
      store_j0        <= '0;
      store_n_eff     <= '0;
      store_m_eff     <= '0;
      store_base_c    <= '0;
      store_stride_c_row <= '0;
      c_queue_pop     <= 1'b0;
    end else begin
      // defaults
      start_store <= 1'b0;
      c_queue_pop <= 1'b0;

      case (st_state)
        ST_IDLE: begin
          if (!c_queue_empty) begin
            // C tile available in queue, pop it
            start_store        <= 1'b1;  // pulse
            store_i0           <= c_queue_front.i0;
            store_j0           <= c_queue_front.j0;
            store_n_eff        <= c_queue_front.n_eff;
            store_m_eff        <= c_queue_front.m_eff;
            store_base_c       <= reg_base_C;
            store_stride_c_row <= reg_stride_C;
            c_queue_pop        <= 1'b1;  // consume from queue
            st_state           <= ST_WAIT;
          end
        end

        ST_WAIT: begin
          if (wr_done) begin
            st_state <= ST_DONE;
          end
        end

        ST_DONE: begin
          // One-cycle completion state before returning to IDLE
          st_state <= ST_IDLE;
        end

      endcase
    end
  end

  // Store backpressure: can accept when queue not full
  assign store_can_accept = !c_queue_full;

  // ========== Streaming FSM (A/B buffer → tile_compute) ==========
  // Handles tile_load_req from tile_compute and streams A/B segment data
  logic [15:0] stream_word_cnt;      // word counter (0~15 for 8×8/4)
  logic [15:0] stream_total_words;   // TILE_SIZE * TILE_SIZE / 4 = 16 words
  logic [15:0] stream_valid_cnt;     // valid count counter
  
  // A/B tile word decomposition
  logic [3:0]  a_tile_row;           // A tile row index (0~7)
  logic [3:0]  a_word_in_row;        // word offset within A row
  logic [3:0]  b_tile_row;           // B tile row index (0~k_eff-1)
  logic [3:0]  b_word_in_row;        // word offset within B row (0~1 for 8 cols)
  
  // Words per row for current k_eff
  logic [3:0]  words_per_a_row;      // k_eff / 4 (e.g., 2 when k_eff=8)
  
  // DRAM word addresses
  logic [15:0] a_word_addr, b_word_addr;
  
  // Address calculation (combinational)
  // always_comb begin
  //   // A tile: 8 rows × k_eff cols
  //   // Each word contains 4 INT8 elements
  //   // Words per row = k_eff / 4 (e.g., k_eff=8 → 2 words/row)
  //   words_per_a_row = stream_k_eff >> 2;
    
  //   // Decompose stream_word_cnt into (row, word_in_row)
  //   a_tile_row     = stream_word_cnt / words_per_a_row;  // which row (0~7)
  //   a_word_in_row  = stream_word_cnt % words_per_a_row;  // word offset in row
    
  //   // A buffer word address (matches tile_loader fill pattern)
  //   // A[i0+row][k0_seg : k0_seg+k_eff-1]
  //   // Base: (i0+row) * (K/4) + (k0_seg/4)
  //   // Offset: word_in_row
  //   a_word_addr = (cp_i0 + a_tile_row) * (reg_K >> 2) + (k0_seg >> 2) + a_word_in_row;
    
  //   // B tile: k_eff rows × 8 cols
  //   // Words per row = 8 / 4 = 2 (fixed for TILE_SIZE=8)
  //   b_tile_row    = stream_word_cnt >> 1;        // which row (0~k_eff-1)
  //   b_word_in_row = stream_word_cnt[0];          // word offset: 0 or 1
    
  //   // B buffer word address (block-relative)
  //   // B[k0_seg+row][(j0-j_block) : (j0-j_block)+7]
  //   // Base: (k0_seg+row) * (BLOCK_M/4) + ((j0-j_block)/4)
  //   // Offset: word_in_row
  //   b_word_addr = (k0_seg + b_tile_row) * (reg_BLOCK_M >> 2) + 
  //                 ((cp_j0 - cp_j_block) >> 2) + b_word_in_row;
  // end

    // Address calculation (combinational) - no /,% anywhere
  always_comb begin
    // default value (safety) - to avoid latch in other branches
    a_tile_row    = '0;
    a_word_in_row = '0;
    b_tile_row    = '0;
    b_word_in_row = 1'b0;

    // words_per_a_row = stream_k_eff / 4
    // assume stream_k_eff is 4 or 8 → words_per_a_row = 1 or 2
    words_per_a_row = stream_k_eff >> 2;

    // ---------- A tile: 8 rows × k_eff cols ----------
    //   case 1: words_per_a_row = 1  → row = cnt, col = 0
    //   case 2: words_per_a_row = 2  → row = cnt>>1, col = cnt[0]
    //
    // integer division definition is the same as the original
    //   row = stream_word_cnt / words_per_a_row;
    //   col = stream_word_cnt % words_per_a_row;
    // and mathematically identical.

    unique case (words_per_a_row)
      4'd1: begin
        a_tile_row    = stream_word_cnt[3:0];          // /1
        a_word_in_row = 4'd0;                          // %1
      end

      4'd2: begin
        a_tile_row    = stream_word_cnt[15:1];         // /2
        a_word_in_row = {3'd0, stream_word_cnt[0]};    // %2
      end

      // other values are not expected, so deliberately no default
      // (initialized to '0 in the previous line, so no latch)
    endcase

    // A buffer word address (mathematically identical to the original formula)
    a_word_addr = (cp_i0 + a_tile_row) * (reg_K >> 2)
                  + (k0_seg >> 2)
                  + a_word_in_row;

    // ---------- B tile: k_eff rows × 8 cols ----------
    // B has fixed words_per_row=2 → row = cnt/2, col = cnt%2
    b_tile_row    = stream_word_cnt[15:1];   // /2
    b_word_in_row = stream_word_cnt[0];      // %2 (0 or 1)

    b_word_addr = (k0_seg + b_tile_row) * (reg_BLOCK_M >> 2)
                  + ((cp_j0 - cp_j_block) >> 2)
                  + {{3{1'b0}}, b_word_in_row};
  end


  always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
      stream_state       <= STREAM_IDLE;
      stream_cnt_a       <= '0;
      stream_cnt_b       <= '0;
      stream_k_eff       <= '0;
      stream_word_cnt    <= '0;
      stream_total_words <= '0;
      stream_valid_cnt   <= '0;
      a_ld_start         <= 1'b0;
      a_ld_valid         <= 1'b0;
      a_ld_data          <= '0;
      b_ld_start         <= 1'b0;
      b_ld_valid         <= 1'b0;
      b_ld_data          <= '0;
      b_consume_req      <= 1'b0;
      a_rd_en            <= 1'b0;
      a_rd_addr          <= '0;
      b_rd_en            <= 1'b0;
      b_rd_addr          <= '0;
      first_segment      <= 1'b0;
      k0_seg             <= '0;
    end else begin
      // defaults
      a_ld_start    <= 1'b0;
      b_ld_start    <= 1'b0;
      b_consume_req <= 1'b0;
      a_rd_en       <= 1'b0;
      b_rd_en       <= 1'b0;

      case (stream_state)
        STREAM_IDLE: begin
          a_ld_valid <= 1'b0;
          b_ld_valid <= 1'b0;

          if (new_tile_start) begin
            first_segment <= 1'b1;
            k0_seg        <= 16'd0;
          end
          
          if (tile_load_req && (cp_state == CP_WAIT)) begin
            // Latch k_eff and prepare streaming
            stream_k_eff       <= k_eff;
            stream_total_words <= 16;  // 8×8/4 words
            stream_word_cnt    <= '0;
            stream_valid_cnt   <= '0;
            if (first_segment) begin
              stream_state <= STREAM_CONSUME;
            end else begin
              stream_state <= STREAM_START;
            end
          end
        end

        STREAM_CONSUME: begin
          // Issue b_consume_req for first segment only
          b_consume_req <= 1'b1;
          stream_state  <= STREAM_START;
        end

        STREAM_START: begin
          // Issue start pulses and first address request
          a_ld_start      <= 1'b1;
          b_ld_start      <= 1'b1;
          a_rd_addr       <= a_word_addr[13:0];  // cnt=0 기준 → addr=0
          a_rd_en         <= 1'b1;
          b_rd_addr       <= b_word_addr[15:0];  // cnt=0 기준 → addr=0
          b_rd_en         <= 1'b1;
          stream_word_cnt <= 1;  // next address is 1
          stream_state    <= STREAM_PIPE;
        end

        STREAM_PIPE: begin
          // Pipeline bubble: data from STREAM_START (addr=0) arrives next cycle
          a_rd_addr       <= a_word_addr[13:0];  // cnt=1 기준 → addr=1
          a_rd_en         <= 1'b1;
          b_rd_addr       <= b_word_addr[15:0];  // cnt=1 기준 → addr=1
          b_rd_en         <= 1'b1;
          stream_word_cnt <= 2;  // next address is 2
          stream_state    <= STREAM_DATA;
        end

        STREAM_DATA: begin
          // Data from previous request is now valid
          a_ld_data       <= a_rd_data;
          b_ld_data       <= b_rd_data;
          a_ld_valid      <= 1'b1;
          b_ld_valid      <= 1'b1;
          stream_valid_cnt <= stream_valid_cnt + 1;  // valid count
          
          // Check if need more address requests
          if (stream_word_cnt < stream_total_words) begin
            // Issue next read request
            a_rd_addr       <= a_word_addr[13:0];
            a_rd_en         <= 1'b1;
            b_rd_addr       <= b_word_addr[15:0];
            b_rd_en         <= 1'b1;
            stream_word_cnt <= stream_word_cnt + 1;
          end
          
          // Check if all data sent
          if (stream_valid_cnt >= stream_total_words) begin
            a_ld_valid   <= 1'b0;
            b_ld_valid   <= 1'b0;
            stream_state <= STREAM_DONE;
          end else begin
            stream_state <= STREAM_DATA;
          end
        end

        STREAM_DONE: begin
          // Update segment tracking
          a_ld_valid    <= 1'b0;
          b_ld_valid    <= 1'b0;
          k0_seg        <= k0_seg + stream_k_eff;
          first_segment <= 1'b0;
          stream_state  <= STREAM_IDLE;
        end

      endcase
    end
  end

endmodule
