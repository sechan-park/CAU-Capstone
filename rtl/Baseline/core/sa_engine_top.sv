`timescale 1ns/1ps

// sa_engine_top wrapper
// - Provides AXI3-style top-level interface expected by the legacy testbench
// - Internally instantiates sa_core_pipeline (AXI4) and adapts signal differences

module sa_engine_top #(
    // AXI widths (match testbench parameters)
    parameter integer AXI_WIDTH_AD = 32,
    parameter integer AXI_WIDTH_ID = 4,
    parameter integer AXI_WIDTH_DA = 32,
    parameter integer AXI_WIDTH_DS = (AXI_WIDTH_DA/8),
    // Kept for compatibility with the original top (not used in wrapper)
    parameter integer MEM_BASE_ADDR       = 32'd0,
    parameter integer MEM_DATA_BASE_ADDR  = 32'd0
)(
    // Global
    input  wire                        clk,
    input  wire                        rstn,

    // Control registers (legacy)
    input  wire [31:0]                 i_ctrl_reg0, // bit[0]: start
    input  wire [31:0]                 i_ctrl_reg1, // read base addr
    input  wire [31:0]                 i_ctrl_reg2, // write base addr
    input  wire [31:0]                 i_ctrl_reg3, // optional: num_trans or reserved

    // AXI Read Address Channel (Master → Slave)
    output wire                        M_ARVALID,
    input  wire                        M_ARREADY,
    output wire [AXI_WIDTH_AD-1:0]     M_ARADDR,
    output wire [AXI_WIDTH_ID-1:0]     M_ARID,
    output wire [7:0]                  M_ARLEN,
    output wire [2:0]                  M_ARSIZE,
    output wire [1:0]                  M_ARBURST,
    output wire [1:0]                  M_ARLOCK,   // AXI3: 2-bit; map from AXI4 1-bit
    output wire [3:0]                  M_ARCACHE,
    output wire [2:0]                  M_ARPROT,
    output wire [3:0]                  M_ARQOS,
    output wire [3:0]                  M_ARREGION,
    output wire [0:0]                  M_ARUSER,

    // AXI Read Data Channel (Slave → Master)
    input  wire                        M_RVALID,
    output wire                        M_RREADY,
    input  wire [AXI_WIDTH_DA-1:0]     M_RDATA,
    input  wire                        M_RLAST,
    input  wire [AXI_WIDTH_ID-1:0]     M_RID,
    input  wire [0:0]                  M_RUSER,
    input  wire [1:0]                  M_RRESP,

    // AXI Write Address Channel (Master → Slave)
    output wire                        M_AWVALID,
    input  wire                        M_AWREADY,
    output wire [AXI_WIDTH_AD-1:0]     M_AWADDR,
    output wire [AXI_WIDTH_ID-1:0]     M_AWID,
    output wire [7:0]                  M_AWLEN,
    output wire [2:0]                  M_AWSIZE,
    output wire [1:0]                  M_AWBURST,
    output wire [1:0]                  M_AWLOCK,   // AXI3: 2-bit; map from AXI4 1-bit
    output wire [3:0]                  M_AWCACHE,
    output wire [2:0]                  M_AWPROT,
    output wire [3:0]                  M_AWQOS,
    output wire [3:0]                  M_AWREGION,
    output wire [0:0]                  M_AWUSER,

    // AXI Write Data Channel (Master → Slave)
    output wire                        M_WVALID,
    input  wire                        M_WREADY,
    output wire [AXI_WIDTH_DA-1:0]     M_WDATA,
    output wire [AXI_WIDTH_DS-1:0]     M_WSTRB,
    output wire                        M_WLAST,
    output wire [AXI_WIDTH_ID-1:0]     M_WID,      // AXI3 only; derived from AWID (or constant)
    output wire [0:0]                  M_WUSER,

    // AXI Write Response Channel (Slave → Master)
    input  wire                        M_BVALID,
    output wire                        M_BREADY,
    input  wire [1:0]                  M_BRESP,
    input  wire [AXI_WIDTH_ID-1:0]     M_BID,
    input  wire [0:0]                  M_BUSER,

    // Status
    output wire                        network_done,
    output wire                        network_done_led
);

    // -----------------------------
    // Internal wires to sa_core_pipeline (AXI4)
    // -----------------------------
    wire [AXI_WIDTH_ID-1:0]  m_awid;
    wire [AXI_WIDTH_AD-1:0]  m_awaddr;
    wire [7:0]               m_awlen;
    wire [2:0]               m_awsize;
    wire [1:0]               m_awburst;
    wire                     m_awlock_1b;  // AXI4 1-bit
    wire [3:0]               m_awcache;
    wire [2:0]               m_awprot;
    wire [3:0]               m_awqos;
    wire [0:0]               m_awuser;
    wire                     m_awvalid;

    wire [AXI_WIDTH_DA-1:0]  m_wdata;
    wire [AXI_WIDTH_DS-1:0]  m_wstrb;
    wire                     m_wlast;
    wire [0:0]               m_wuser;
    wire                     m_wvalid;

    wire [AXI_WIDTH_ID-1:0]  m_bid;
    wire [1:0]               m_bresp;
    wire [0:0]               m_buser;
    wire                     m_bvalid;
    wire                     m_bready;

    wire [AXI_WIDTH_ID-1:0]  m_arid;
    wire [AXI_WIDTH_AD-1:0]  m_araddr;
    wire [7:0]               m_arlen;
    wire [2:0]               m_arsize;
    wire [1:0]               m_arburst;
    wire                     m_arlock_1b;  // AXI4 1-bit
    wire [3:0]               m_arcache;
    wire [2:0]               m_arprot;
    wire [3:0]               m_arqos;
    wire [0:0]               m_aruser;
    wire                     m_arvalid;

    wire [AXI_WIDTH_ID-1:0]  m_rid;
    wire [AXI_WIDTH_DA-1:0]  m_rdata;
    wire [1:0]               m_rresp;
    wire                     m_rlast;
    wire [0:0]               m_ruser;
    wire                     m_rvalid;
    wire                     m_rready;

    // Status from pipeline
    wire busy_w;
    wire done_w;
    wire error_w;

    // -----------------------------
    // sa_core_pipeline instantiation
    // -----------------------------
    sa_core_pipeline #(
        .C_M_AXI_ID_WIDTH     (AXI_WIDTH_ID),
        .C_M_AXI_ADDR_WIDTH   (AXI_WIDTH_AD),
        .C_M_AXI_DATA_WIDTH   (AXI_WIDTH_DA),
        .C_M_AXI_AWUSER_WIDTH (1),
        .C_M_AXI_ARUSER_WIDTH (1),
        .C_M_AXI_WUSER_WIDTH  (1),
        .C_M_AXI_RUSER_WIDTH  (1),
        .C_M_AXI_BUSER_WIDTH  (1),
        .AXI_WIDTH_AD         (AXI_WIDTH_AD)
    ) u_core (
        .S_AXI_ACLK       (clk),
        .S_AXI_ARESETN    (rstn),
        .M_AXI_ACLK       (clk),
        .M_AXI_ARESETN    (rstn),

        // Control
        .i_start          (i_ctrl_reg0[0]),
        .i_read_base_addr (i_ctrl_reg1),
        .i_write_base_addr(i_ctrl_reg2),
        .i_num_trans_param(i_ctrl_reg3),
        .i_max_blk_param  (32'd32),

        // Status
        .o_busy           (busy_w),
        .o_done           (done_w),
        .o_error          (error_w),

        // AXI4-Full Master
        .M_AXI_AWID       (m_awid),
        .M_AXI_AWADDR     (m_awaddr),
        .M_AXI_AWLEN      (m_awlen),
        .M_AXI_AWSIZE     (m_awsize),
        .M_AXI_AWBURST    (m_awburst),
        .M_AXI_AWLOCK     (m_awlock_1b),
        .M_AXI_AWCACHE    (m_awcache),
        .M_AXI_AWPROT     (m_awprot),
        .M_AXI_AWQOS      (m_awqos),
        .M_AXI_AWUSER     (m_awuser),
        .M_AXI_AWVALID    (m_awvalid),
        .M_AXI_AWREADY    (M_AWREADY),

        .M_AXI_WDATA      (m_wdata),
        .M_AXI_WSTRB      (m_wstrb),
        .M_AXI_WLAST      (m_wlast),
        .M_AXI_WUSER      (m_wuser),
        .M_AXI_WVALID     (m_wvalid),
        .M_AXI_WREADY     (M_WREADY),

        .M_AXI_BID        (M_BID),
        .M_AXI_BRESP      (M_BRESP),
        .M_AXI_BUSER      (M_BUSER),
        .M_AXI_BVALID     (M_BVALID),
        .M_AXI_BREADY     (m_bready),

        .M_AXI_ARID       (m_arid),
        .M_AXI_ARADDR     (m_araddr),
        .M_AXI_ARLEN      (m_arlen),
        .M_AXI_ARSIZE     (m_arsize),
        .M_AXI_ARBURST    (m_arburst),
        .M_AXI_ARLOCK     (m_arlock_1b),
        .M_AXI_ARCACHE    (m_arcache),
        .M_AXI_ARPROT     (m_arprot),
        .M_AXI_ARQOS      (m_arqos),
        .M_AXI_ARUSER     (m_aruser),
        .M_AXI_ARVALID    (m_arvalid),
        .M_AXI_ARREADY    (M_ARREADY),

        .M_AXI_RID        (M_RID),
        .M_AXI_RDATA      (M_RDATA),
        .M_AXI_RRESP      (M_RRESP),
        .M_AXI_RLAST      (M_RLAST),
        .M_AXI_RUSER      (M_RUSER),
        .M_AXI_RVALID     (M_RVALID),
        .M_AXI_RREADY     (m_rready)
    );

    // -----------------------------
    // AXI3-style outward mapping
    // -----------------------------
    assign M_AWID     = m_awid;
    assign M_AWADDR   = m_awaddr;
    assign M_AWLEN    = m_awlen;
    assign M_AWSIZE   = m_awsize;
    assign M_AWBURST  = m_awburst;
    assign M_AWLOCK   = {1'b0, m_awlock_1b};
    assign M_AWCACHE  = m_awcache;
    assign M_AWPROT   = m_awprot;
    assign M_AWQOS    = m_awqos;
    assign M_AWREGION = 4'd0;
    assign M_AWUSER   = 1'b0;
    assign M_AWVALID  = m_awvalid;

    assign M_WDATA    = m_wdata;
    assign M_WSTRB    = m_wstrb;
    assign M_WLAST    = m_wlast;
    assign M_WUSER    = 1'b0;
    assign M_WVALID   = m_wvalid;
    assign M_WID      = m_awid; // Map write data ID to AWID (AXI3 compatibility)

    assign M_BREADY   = m_bready;

    assign M_ARID     = m_arid;
    assign M_ARADDR   = m_araddr;
    assign M_ARLEN    = m_arlen;
    assign M_ARSIZE   = m_arsize;
    assign M_ARBURST  = m_arburst;
    assign M_ARLOCK   = {1'b0, m_arlock_1b};
    assign M_ARCACHE  = m_arcache;
    assign M_ARPROT   = m_arprot;
    assign M_ARQOS    = m_arqos;
    assign M_ARREGION = 4'd0;
    assign M_ARUSER   = 1'b0;
    assign M_ARVALID  = m_arvalid;

    assign M_RREADY   = m_rready;

    // Status
    assign network_done     = done_w;
    assign network_done_led = done_w;

endmodule


