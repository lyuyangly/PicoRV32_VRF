//******************************************************************************
//  File    : ZYNQ_PRV32.v
//  Author  : Lyu Yang
//  Date    : 2019-10-15
//  Details :
//******************************************************************************
`timescale 1ns / 1ps
module ZYNQ_PRV32 (
    input                   clk         ,
    input                   rst_n       ,
    input                   uart_rxd    ,
    output                  uart_txd    ,
    output  [3:0]           led
);

// Clock and Reset
wire            rst_sync;

// System Bus
wire            mhbusreq;
wire            mhgrant;
wire    [31:0]  mhaddr;
wire    [1:0]   mhtrans;
wire    [2:0]   mhsize;
wire    [2:0]   mhburst;
wire    [3:0]   mhprot;
wire            mhwrite;
wire    [31:0]  mhwdata;
wire            mhmasterlock;
wire    [31:0]  mhrdata;
wire            mhready;
wire    [1:0]   mhresp;

wire    [31:0]  shaddr;
wire    [1:0]   shtrans;
wire    [2:0]   shsize;
wire    [2:0]   shburst;
wire    [3:0]   shprot;
wire            shwrite;
wire    [31:0]  shwdata;
wire            shready;

wire            ram_hsel;
wire    [31:0]  ram_hrdata;
wire            ram_hready;
wire    [1:0]   ram_hresp;

wire            pio_hsel;
wire    [31:0]  pio_hrdata;
wire            pio_hready;
wire            pio_hresp;

// APB Subsystem AHB Interface
wire            apbsys_hsel;
wire    [31:0]  apbsys_hrdata;
wire            apbsys_hready;
wire            apbsys_hresp;

// RST SYNC
rst_sync U_RST_SYNC (
    .clk                    (clk                ),
    .arst_i                 (rst_n              ),
    .srst_o                 (rst_sync           )
);

// PicoRV32 CPU
picorv32_ahb U_CPU (
    .hclk                   (clk                ),
    .hreset_n               (rst_sync           ),
    .hbusreq                (mhbusreq           ),
    .hgrant                 (mhgrant            ),
    .haddr                  (mhaddr             ),
    .htrans                 (mhtrans            ),
    .hsize                  (mhsize             ),
    .hburst                 (mhburst            ),
    .hprot                  (mhprot             ),
    .hwrite                 (mhwrite            ),
    .hwdata                 (mhwdata            ),
    .hmasterlock            (mhmasterlock       ),
    .hready                 (mhready            ),
    .hrdata                 (mhrdata            ),
    .hresp                  (mhresp             ),
    .pcpi_insn              (),
    .pcpi_rs1               (),
    .pcpi_rs2               (),
    .pcpi_wr                (1'b0               ),
    .pcpi_rd                (32'h0              ),
    .pcpi_wait              (1'b0               ),
    .pcpi_ready             (1'b0               ),
    .pcpi_valid             (),
    .irq                    (32'h0              ),
    .eoi                    (),
    .trap                   (),
    .trace_valid            (),
    .trace_data             ()
);

// AHB Interconnect
amba_ahb_m2s4 U_AHB_MATRIX (
    .HCLK                   (clk                ),
    .HRESETn                (rst_sync           ),
    .REMAP                  (1'b0               ),
    .M0_HBUSREQ             (mhbusreq           ),
    .M0_HGRANT              (mhgrant            ),
    .M0_HTRANS              (mhtrans            ),
    .M0_HBURST              (mhburst            ),
    .M0_HSIZE               (mhsize             ),
    .M0_HWRITE              (mhwrite            ),
    .M0_HPROT               (mhprot             ),
    .M0_HLOCK               (mhmasterlock       ),
    .M0_HADDR               (mhaddr             ),
    .M0_HWDATA              (mhwdata            ),
    .M1_HBUSREQ             (1'b0               ),
    .M1_HGRANT              (),
    .M1_HTRANS              (2'h0               ),
    .M1_HBURST              (),
    .M1_HSIZE               (),
    .M1_HWRITE              (),
    .M1_HPROT               (),
    .M1_HLOCK               (),
    .M1_HADDR               (),
    .M1_HWDATA              (),
    .M_HRDATA               (mhrdata            ),
    .M_HREADY               (mhready            ),
    .M_HRESP                (mhresp             ),
    .S_HTRANS               (shtrans            ),
    .S_HBURST               (shburst            ),
    .S_HSIZE                (shsize             ),
    .S_HWRITE               (shwrite            ),
    .S_HPROT                (shprot             ),
    .S_HMASTER              (                   ),
    .S_HMASTLOCK            (                   ),
    .S_HADDR                (shaddr             ),
    .S_HWDATA               (shwdata            ),
    .S_HREADY               (shready            ),
    .S0_HSEL                (ram_hsel           ),
    .S0_HREADY              (ram_hready         ),
    .S0_HRESP               (ram_hresp          ),
    .S0_HSPLIT              (16'h0              ),
    .S0_HRDATA              (ram_hrdata         ),
    .S1_HSEL                (pio_hsel           ),
    .S1_HREADY              (pio_hready         ),
    .S1_HRESP               ({1'b0,pio_hresp}   ),
    .S1_HSPLIT              (16'h0              ),
    .S1_HRDATA              (pio_hrdata         ),
    .S2_HSEL                (                   ),
    .S2_HREADY              (1'b1               ),
    .S2_HRESP               (2'b00              ),
    .S2_HSPLIT              (16'h0              ),
    .S2_HRDATA              (32'h0              ),
    .S3_HSEL                (apbsys_hsel        ),
    .S3_HREADY              (apbsys_hready      ),
    .S3_HRESP               ({1'b0,apbsys_hresp}),
    .S3_HSPLIT              (16'h0              ),
    .S3_HRDATA              (apbsys_hrdata      )
);

// RAM For CPU
ahb_ram U_RAM (
    .hclk                   (clk                ),
    .hreset_n               (rst_sync           ),
    .hsel                   (ram_hsel           ),
    .htrans                 (shtrans            ),
    .hwrite                 (shwrite            ),
    .haddr                  (shaddr             ),
    .hsize                  (shsize             ),
    .hready_in              (shready            ),
    .hwdata                 (shwdata            ),
    .hready_out             (ram_hready         ),
    .hrdata                 (ram_hrdata         ),
    .hresp                  (ram_hresp          )
);

// Simple PIO
apb_pio U_PIO (
    .HCLK                   (clk                ),
    .HRESETn                (rst_sync           ),
    .HSEL                   (pio_hsel           ),
    .HADDR                  (shaddr             ),
    .HTRANS                 (shtrans            ),
    .HSIZE                  (shsize             ),
    .HPROT                  (shprot             ),
    .HWRITE                 (shwrite            ),
    .HREADY                 (shready            ),
    .HWDATA                 (shwdata            ),
    .HREADYOUT              (pio_hready         ),
    .HRDATA                 (pio_hrdata         ),
    .HRESP                  (pio_hresp          ),
    .GPIO                   (led                )
);

// APB subsystem for UART
apb_subsystem #(
    .INCLUDE_IRQ_SYNCHRONIZER(1),
    .INCLUDE_APB_TEST_SLAVE  (1),
    .INCLUDE_APB_UART0       (1),
    .BE                      (0) ) U_APB_SUBSYS (
    .HCLK                   (clk                ),
    .HRESETn                (rst_sync           ),
    .HSEL                   (apbsys_hsel        ),
    .HADDR                  (shaddr[15:0]       ),
    .HTRANS                 (shtrans            ),
    .HWRITE                 (shwrite            ),
    .HSIZE                  (shsize             ),
    .HPROT                  (shprot             ),
    .HREADY                 (shready            ),
    .HWDATA                 (shwdata            ),
    .HREADYOUT              (apbsys_hready      ),
    .HRDATA                 (apbsys_hrdata      ),
    .HRESP                  (apbsys_hresp       ),
    // APB clock and reset
    .PCLK                   (clk                ),
    .PCLKG                  (clk                ),
    .PCLKEN                 (1'b1               ),
    .PRESETn                (~rst_sync          ),
    // APB extension ports
    .PADDR                  (                   ),
    .PWRITE                 (                   ),
    .PWDATA                 (                   ),
    .PENABLE                (                   ),
    // Status Output for clock gating
    .APBACTIVE              (                   ),
    // UART
    .uart0_rxd              (uart_rxd           ),
    .uart0_txd              (uart_txd           ),
    .uart0_txen             (                   )
);

// PS SUBSYS
`ifdef SYNTHESIS
PS7SYS U_PS7();
`endif

endmodule
