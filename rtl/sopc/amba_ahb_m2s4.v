//---------------------------------------------------------------------------
module amba_ahb_m2s4
     #(parameter P_NUMM=2 // num of masters
               , P_NUMS=4 // num of slaves
               , P_HSEL0_START=32'h00000000, P_HSEL0_SIZE=32'h20000000
               , P_HSEL1_START=32'h20000000, P_HSEL1_SIZE=32'h20000000
               , P_HSEL2_START=32'h40000000, P_HSEL2_SIZE=32'h40000000
               , P_HSEL3_START=32'h80000000, P_HSEL3_SIZE=32'h40000000
               )
(
        input   wire         HRESETn
      , input   wire         HCLK
      , input   wire         M0_HBUSREQ
      , output  wire         M0_HGRANT
      , input   wire  [31:0] M0_HADDR
      , input   wire  [ 1:0] M0_HTRANS
      , input   wire  [ 2:0] M0_HSIZE
      , input   wire  [ 2:0] M0_HBURST
      , input   wire  [ 3:0] M0_HPROT
      , input   wire         M0_HLOCK
      , input   wire         M0_HWRITE
      , input   wire  [31:0] M0_HWDATA
      , input   wire         M1_HBUSREQ
      , output  wire         M1_HGRANT
      , input   wire  [31:0] M1_HADDR
      , input   wire  [ 1:0] M1_HTRANS
      , input   wire  [ 2:0] M1_HSIZE
      , input   wire  [ 2:0] M1_HBURST
      , input   wire  [ 3:0] M1_HPROT
      , input   wire         M1_HLOCK
      , input   wire         M1_HWRITE
      , input   wire  [31:0] M1_HWDATA
      , output  wire  [31:0] M_HRDATA
      , output  wire  [ 1:0] M_HRESP
      , output  wire         M_HREADY
      , output  wire  [31:0] S_HADDR
      , output  wire         S_HWRITE
      , output  wire  [ 1:0] S_HTRANS
      , output  wire  [ 2:0] S_HSIZE
      , output  wire  [ 2:0] S_HBURST
      , output  wire  [31:0] S_HWDATA
      , output  wire  [ 3:0] S_HPROT
      , output  wire         S_HREADY
      , output  wire  [ 3:0] S_HMASTER
      , output  wire         S_HMASTLOCK
      , output  wire         S0_HSEL
      , input   wire         S0_HREADY
      , input   wire  [ 1:0] S0_HRESP
      , input   wire  [31:0] S0_HRDATA
      , input   wire  [15:0] S0_HSPLIT
      , output  wire         S1_HSEL
      , input   wire         S1_HREADY
      , input   wire  [ 1:0] S1_HRESP
      , input   wire  [31:0] S1_HRDATA
      , input   wire  [15:0] S1_HSPLIT
      , output  wire         S2_HSEL
      , input   wire         S2_HREADY
      , input   wire  [ 1:0] S2_HRESP
      , input   wire  [31:0] S2_HRDATA
      , input   wire  [15:0] S2_HSPLIT
      , output  wire         S3_HSEL
      , input   wire         S3_HREADY
      , input   wire  [ 1:0] S3_HRESP
      , input   wire  [31:0] S3_HRDATA
      , input   wire  [15:0] S3_HSPLIT
      , input   wire         REMAP
);
  ahb_arbiter #(.NUMM(2),.NUMS(4))
  u_ahb_arbiter (
         .HRESETn   (HRESETn    )
       , .HCLK      (HCLK       )
       , .HBUSREQ   ({M1_HBUSREQ,M0_HBUSREQ})
       , .HGRANT    ({M1_HGRANT,M0_HGRANT})
       , .HMASTER   (S_HMASTER  )
       , .HLOCK     ({M1_HLOCK,M0_HLOCK})
       , .HREADY    (M_HREADY   )
       , .HMASTLOCK (S_HMASTLOCK)
       , .HSPLIT    ({S3_HSPLIT,S2_HSPLIT,S1_HSPLIT,S0_HSPLIT})
  );
  wire  [31:0] M_HADDR;
  wire  [ 1:0] M_HTRANS;
  wire  [ 2:0] M_HSIZE;
  wire  [ 2:0] M_HBURST;
  wire  [ 3:0] M_HPROT;
  wire         M_HWRITE;
  wire  [31:0] M_HWDATA;
  ahb_m2s_m2 u_ahb_m2s (
       .HRESETn  (HRESETn    )
     , .HCLK     (HCLK       )
     , .HREADY   (M_HREADY   )
     , .HMASTER  (S_HMASTER  )
     , .HADDR    (M_HADDR    )
     , .HPROT    (M_HPROT    )
     , .HTRANS   (M_HTRANS   )
     , .HWRITE   (M_HWRITE   )
     , .HSIZE    (M_HSIZE    )
     , .HBURST   (M_HBURST   )
     , .HWDATA   (M_HWDATA   )
     , .HADDR0   (M0_HADDR  )
     , .HPROT0   (M0_HPROT  )
     , .HTRANS0  (M0_HTRANS )
     , .HWRITE0  (M0_HWRITE )
     , .HSIZE0   (M0_HSIZE  )
     , .HBURST0  (M0_HBURST )
     , .HWDATA0  (M0_HWDATA )
     , .HADDR1   (M1_HADDR  )
     , .HPROT1   (M1_HPROT  )
     , .HTRANS1  (M1_HTRANS )
     , .HWRITE1  (M1_HWRITE )
     , .HSIZE1   (M1_HSIZE  )
     , .HBURST1  (M1_HBURST )
     , .HWDATA1  (M1_HWDATA )
  );
  ahb_lite_s4 #(.P_NUM(4)
               ,.P_HSEL0_START(P_HSEL0_START), .P_HSEL0_SIZE(P_HSEL0_SIZE)
               ,.P_HSEL1_START(P_HSEL1_START), .P_HSEL1_SIZE(P_HSEL1_SIZE)
               ,.P_HSEL2_START(P_HSEL2_START), .P_HSEL2_SIZE(P_HSEL2_SIZE)
               ,.P_HSEL3_START(P_HSEL3_START), .P_HSEL3_SIZE(P_HSEL3_SIZE)
               )
  u_ahb_lite (
       .HRESETn   (HRESETn  )
     , .HCLK      (HCLK     )
     , .M_HADDR   (M_HADDR  )
     , .M_HTRANS  (M_HTRANS )
     , .M_HWRITE  (M_HWRITE )
     , .M_HSIZE   (M_HSIZE  )
     , .M_HBURST  (M_HBURST )
     , .M_HPROT   (M_HPROT  )
     , .M_HWDATA  (M_HWDATA )
     , .M_HRDATA  (M_HRDATA )
     , .M_HRESP   (M_HRESP  )
     , .M_HREADY  (M_HREADY )
     , .S_HADDR   (S_HADDR  )
     , .S_HTRANS  (S_HTRANS )
     , .S_HSIZE   (S_HSIZE  )
     , .S_HBURST  (S_HBURST )
     , .S_HWRITE  (S_HWRITE )
     , .S_HPROT   (S_HPROT  )
     , .S_HWDATA  (S_HWDATA )
     , .S_HREADY  (S_HREADY )
     , .S0_HSEL   (S0_HSEL  )
     , .S0_HRDATA (S0_HRDATA)
     , .S0_HRESP  (S0_HRESP )
     , .S0_HREADY (S0_HREADY)
     , .S1_HSEL   (S1_HSEL  )
     , .S1_HRDATA (S1_HRDATA)
     , .S1_HRESP  (S1_HRESP )
     , .S1_HREADY (S1_HREADY)
     , .S2_HSEL   (S2_HSEL  )
     , .S2_HRDATA (S2_HRDATA)
     , .S2_HRESP  (S2_HRESP )
     , .S2_HREADY (S2_HREADY)
     , .S3_HSEL   (S3_HSEL  )
     , .S3_HRDATA (S3_HRDATA)
     , .S3_HRESP  (S3_HRESP )
     , .S3_HREADY (S3_HREADY)
     , .REMAP     (REMAP    )
  );
endmodule
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
module ahb_arbiter
     #(parameter NUMM=2 // num of masters
               , NUMS=4)// num of slaves
(
       input   wire               HRESETn
     , input   wire               HCLK
     , input   wire [NUMM-1:0]    HBUSREQ // 0: highest priority
     , output  reg  [NUMM-1:0]    HGRANT={NUMM{1'b0}}
     , output  reg  [     3:0]    HMASTER=4'h0
     , input   wire [NUMM-1:0]    HLOCK
     , input   wire               HREADY
     , output  reg                HMASTLOCK=1'b0
     , input   wire [16*NUMS-1:0] HSPLIT
);
   reg  [NUMM-1:0] hmask={NUMM{1'b0}}; // 1=mask-out
   wire [     3:0] id=encoder(HGRANT);
   localparam ST_READY='h0
            , ST_STAY ='h1;
   reg state=ST_READY;
   always @ (posedge HCLK or negedge HRESETn) begin
   if (HRESETn==1'b0) begin
       HGRANT    <=  'h0;
       HMASTER   <= 4'h0;
       HMASTLOCK <= 1'b0;
       hmask     <=  'h0;
       state     <= ST_READY;
   end else if (HREADY==1'b1) begin
       HMASTER   <= id;
       HMASTLOCK <= HLOCK[id];
       case (state)
       ST_READY: begin
          if (HBUSREQ!=0) begin
              HGRANT  <= priority(HBUSREQ);
              hmask   <= 'h0;
              state   <= ST_STAY;
          end
          end // ST_READY
       ST_STAY: begin
          if (HBUSREQ=='b0) begin
              HGRANT <= 'h0;
              hmask  <= 'h0;
              state  <= ST_READY;
          end else if (HBUSREQ[id]==1'b0) begin
              if ((HBUSREQ&~hmask)=='b0) begin
                  HGRANT <= priority(HBUSREQ);
                  hmask  <= 'h0;
              end else begin
                  HGRANT    <= priority(HBUSREQ&~hmask);
                  hmask[id] <= 1'b1;
              end
          end
          end // ST_STAY
       default: begin
                HGRANT <= 'h0;
                state  <= ST_READY;
                end
       endcase
   end // if
   end // always
   function [NUMM-1:0] priority;
     input  [NUMM-1:0] req;
     reg    [15:0] val;
   begin
     casex ({{16-NUMM{1'b0}},req})
     16'bxxxx_xxxx_xxxx_xxx1: val = 'h0001;
     16'bxxxx_xxxx_xxxx_xx10: val = 'h0002;
     16'bxxxx_xxxx_xxxx_x100: val = 'h0004;
     16'bxxxx_xxxx_xxxx_1000: val = 'h0008;
     16'bxxxx_xxxx_xxx1_0000: val = 'h0010;
     16'bxxxx_xxxx_xx10_0000: val = 'h0020;
     16'bxxxx_xxxx_x100_0000: val = 'h0040;
     16'bxxxx_xxxx_1000_0000: val = 'h0080;
     16'bxxxx_xxx1_0000_0000: val = 'h0100;
     16'bxxxx_xx10_0000_0000: val = 'h0200;
     16'bxxxx_x100_0000_0000: val = 'h0400;
     16'bxxxx_1000_0000_0000: val = 'h0800;
     16'bxxx1_0000_0000_0000: val = 'h1000;
     16'bxx10_0000_0000_0000: val = 'h2000;
     16'bx100_0000_0000_0000: val = 'h4000;
     16'b1000_0000_0000_0000: val = 'h8000;
     default: val = 'h0000;
     endcase
     priority = val[NUMM-1:0];
   end
   endfunction // priority
   function [3:0] encoder;
     input  [NUMM-1:0] req;
   begin
     casex ({{16-NUMM{1'b0}},req})
     16'bxxxx_xxxx_xxxx_xxx1: encoder = 'h0;
     16'bxxxx_xxxx_xxxx_xx10: encoder = 'h1;
     16'bxxxx_xxxx_xxxx_x100: encoder = 'h2;
     16'bxxxx_xxxx_xxxx_1000: encoder = 'h3;
     16'bxxxx_xxxx_xxx1_0000: encoder = 'h4;
     16'bxxxx_xxxx_xx10_0000: encoder = 'h5;
     16'bxxxx_xxxx_x100_0000: encoder = 'h6;
     16'bxxxx_xxxx_1000_0000: encoder = 'h7;
     16'bxxxx_xxx1_0000_0000: encoder = 'h8;
     16'bxxxx_xx10_0000_0000: encoder = 'h9;
     16'bxxxx_x100_0000_0000: encoder = 'hA;
     16'bxxxx_1000_0000_0000: encoder = 'hB;
     16'bxxx1_0000_0000_0000: encoder = 'hC;
     16'bxx10_0000_0000_0000: encoder = 'hD;
     16'bx100_0000_0000_0000: encoder = 'hE;
     16'b1000_0000_0000_0000: encoder = 'hF;
     default: encoder = 'h0;
     endcase
   end
   endfunction // encoder
   `ifdef RIGOR
   // synthesis translate_off
   integer idx, idy;
   always @ ( posedge HCLK or negedge HRESETn) begin
   if (HRESETn==1'b0) begin
   end else begin
       if (|HGRANT) begin
           idy = 0;
           for (idx=0; idx<NUMM; idx=idx+1) if (HGRANT[idx]) idy = idy + 1;
           if (idy>1) $display("%04d %m ERROR AHB arbitration more than one granted", $time);
       end
   end // if
   end // always
   // synthesis translate_on
   `endif
endmodule
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
module ahb_m2s_m2
(
       input   wire          HRESETn
     , input   wire          HCLK
     , input   wire          HREADY
     , input   wire  [ 3:0]  HMASTER
     , output  reg   [31:0]  HADDR
     , output  reg   [ 3:0]  HPROT
     , output  reg   [ 1:0]  HTRANS
     , output  reg           HWRITE
     , output  reg   [ 2:0]  HSIZE
     , output  reg   [ 2:0]  HBURST
     , output  reg   [31:0]  HWDATA
     , input   wire  [31:0]  HADDR0
     , input   wire  [ 3:0]  HPROT0
     , input   wire  [ 1:0]  HTRANS0
     , input   wire          HWRITE0
     , input   wire  [ 2:0]  HSIZE0
     , input   wire  [ 2:0]  HBURST0
     , input   wire  [31:0]  HWDATA0
     , input   wire  [31:0]  HADDR1
     , input   wire  [ 3:0]  HPROT1
     , input   wire  [ 1:0]  HTRANS1
     , input   wire          HWRITE1
     , input   wire  [ 2:0]  HSIZE1
     , input   wire  [ 2:0]  HBURST1
     , input   wire  [31:0]  HWDATA1
);
       reg [3:0] hmaster_delay=4'h0;
       always @ (posedge HCLK or negedge HRESETn) begin
           if (HRESETn==1'b0) begin
                hmaster_delay <= 4'b0;
           end else begin
                if (HREADY) begin
                   hmaster_delay <= HMASTER;
                end
           end
       end
       always @ (HMASTER or HADDR0 or HADDR1) begin
           case (HMASTER)
           4'h0: HADDR = HADDR0;
           4'h1: HADDR = HADDR1;
           default: HADDR = ~32'b0;
           endcase
        end
       always @ (HMASTER or HPROT0 or HPROT1) begin
           case (HMASTER)
           4'h0: HPROT = HPROT0;
           4'h1: HPROT = HPROT1;
           default: HPROT = 4'b0;
           endcase
        end
       always @ (HMASTER or HTRANS0 or HTRANS1) begin
           case (HMASTER)
           4'h0: HTRANS = HTRANS0;
           4'h1: HTRANS = HTRANS1;
           default: HTRANS = 2'b0;
           endcase
        end
       always @ (HMASTER or HWRITE0 or HWRITE1) begin
           case (HMASTER)
           4'h0: HWRITE = HWRITE0;
           4'h1: HWRITE = HWRITE1;
           default: HWRITE = 1'b0;
           endcase
        end
       always @ (HMASTER or HSIZE0 or HSIZE1) begin
           case (HMASTER)
           4'h0: HSIZE = HSIZE0;
           4'h1: HSIZE = HSIZE1;
           default: HSIZE = 3'b0;
           endcase
        end
       always @ (HMASTER or HBURST0 or HBURST1) begin
           case (HMASTER)
           4'h0: HBURST = HBURST0;
           4'h1: HBURST = HBURST1;
           default: HBURST = 3'b0;
           endcase
        end
       always @ (hmaster_delay or HWDATA0 or HWDATA1) begin
           case (hmaster_delay)
           4'h0: HWDATA = HWDATA0;
           4'h1: HWDATA = HWDATA1;
           default: HWDATA = 3'b0;
           endcase
        end
endmodule
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
module ahb_lite_s4
     #(parameter P_NUM=4 // num of slaves
               , P_HSEL0_START=32'h00000000, P_HSEL0_SIZE=32'h00010000
               , P_HSEL1_START=32'h10000000, P_HSEL1_SIZE=32'h00010000
               , P_HSEL2_START=32'h20000000, P_HSEL2_SIZE=32'h00010000
               , P_HSEL3_START=32'h30000000, P_HSEL3_SIZE=32'h00010000
               )
(
        input   wire         HRESETn
      , input   wire         HCLK
      , input   wire  [31:0] M_HADDR
      , input   wire  [ 1:0] M_HTRANS
      , input   wire         M_HWRITE
      , input   wire  [ 2:0] M_HSIZE
      , input   wire  [ 2:0] M_HBURST
      , input   wire  [ 3:0] M_HPROT
      , input   wire  [31:0] M_HWDATA
      , output  wire  [31:0] M_HRDATA
      , output  wire  [ 1:0] M_HRESP
      , output  wire         M_HREADY
      , output  wire  [31:0] S_HADDR
      , output  wire  [ 1:0] S_HTRANS
      , output  wire  [ 2:0] S_HSIZE
      , output  wire  [ 2:0] S_HBURST
      , output  wire  [ 3:0] S_HPROT
      , output  wire         S_HWRITE
      , output  wire  [31:0] S_HWDATA
      , output  wire         S_HREADY
      , output  wire         S0_HSEL
      , input   wire         S0_HREADY
      , input   wire  [ 1:0] S0_HRESP
      , input   wire  [31:0] S0_HRDATA
      , output  wire         S1_HSEL
      , input   wire         S1_HREADY
      , input   wire  [ 1:0] S1_HRESP
      , input   wire  [31:0] S1_HRDATA
      , output  wire         S2_HSEL
      , input   wire         S2_HREADY
      , input   wire  [ 1:0] S2_HRESP
      , input   wire  [31:0] S2_HRDATA
      , output  wire         S3_HSEL
      , input   wire         S3_HREADY
      , input   wire  [ 1:0] S3_HRESP
      , input   wire  [31:0] S3_HRDATA
      , input   wire         REMAP
);
   wire        HSELd; // default slave
   wire [31:0] HRDATAd;
   wire [ 1:0] HRESPd;
   wire        HREADYd;
   assign S_HADDR  = M_HADDR;
   assign S_HTRANS = M_HTRANS;
   assign S_HSIZE  = M_HSIZE;
   assign S_HBURST = M_HBURST;
   assign S_HWRITE = M_HWRITE;
   assign S_HPROT  = M_HPROT;
   assign S_HWDATA = M_HWDATA;
   assign S_HREADY = M_HREADY;
   ahb_decoder_s4
      #(.P_NUM(4)
       ,.P_HSEL0_START(P_HSEL0_START),.P_HSEL0_SIZE(P_HSEL0_SIZE)
       ,.P_HSEL1_START(P_HSEL1_START),.P_HSEL1_SIZE(P_HSEL1_SIZE)
       ,.P_HSEL2_START(P_HSEL2_START),.P_HSEL2_SIZE(P_HSEL2_SIZE)
       ,.P_HSEL3_START(P_HSEL3_START),.P_HSEL3_SIZE(P_HSEL3_SIZE)
       )
   u_ahb_decoder (
                 .HADDR(M_HADDR)
                ,.HSELd(HSELd)
                ,.HSEL0(S0_HSEL)
                ,.HSEL1(S1_HSEL)
                ,.HSEL2(S2_HSEL)
                ,.HSEL3(S3_HSEL)
                ,.REMAP(REMAP));
   ahb_s2m_s4 u_ahb_s2m (
                 .HRESETn(HRESETn)
                ,.HCLK   (HCLK   )
                ,.HRDATA (M_HRDATA)
                ,.HRESP  (M_HRESP )
                ,.HREADY (M_HREADY)
                ,.HSEL0  (S0_HSEL)
                ,.HRDATA0(S0_HRDATA)
                ,.HRESP0 (S0_HRESP)
                ,.HREADY0(S0_HREADY)
                ,.HSEL1  (S1_HSEL)
                ,.HRDATA1(S1_HRDATA)
                ,.HRESP1 (S1_HRESP)
                ,.HREADY1(S1_HREADY)
                ,.HSEL2  (S2_HSEL)
                ,.HRDATA2(S2_HRDATA)
                ,.HRESP2 (S2_HRESP)
                ,.HREADY2(S2_HREADY)
                ,.HSEL3  (S3_HSEL)
                ,.HRDATA3(S3_HRDATA)
                ,.HRESP3 (S3_HRESP)
                ,.HREADY3(S3_HREADY)
                ,.HSELd  (HSELd  )
                ,.HRDATAd(HRDATAd)
                ,.HRESPd (HRESPd )
                ,.HREADYd(HREADYd));
   ahb_default_slave u_ahb_default_slave (
                 .HRESETn  (HRESETn  )
                ,.HCLK     (HCLK     )
                ,.HSEL     (HSELd    )
                ,.HADDR    (S_HADDR  )
                ,.HTRANS   (S_HTRANS )
                ,.HWRITE   (S_HWRITE )
                ,.HSIZE    (S_HSIZE  )
                ,.HBURST   (S_HBURST )
                ,.HWDATA   (S_HWDATA )
                ,.HRDATA   (HRDATAd  )
                ,.HRESP    (HRESPd   )
                ,.HREADYin (S_HREADY )
                ,.HREADYout(HREADYd  ));
endmodule
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
module ahb_decoder_s4
     #(parameter P_NUM        =4
               , P_HSEL0_START=32'h00000000, P_HSEL0_SIZE=32'h00010000
               , P_HSEL0_END  =P_HSEL0_START+P_HSEL0_SIZE
               , P_HSEL1_START=32'h10000000, P_HSEL1_SIZE=32'h00010000
               , P_HSEL1_END  =P_HSEL1_START+P_HSEL1_SIZE
               , P_HSEL2_START=32'h20000000, P_HSEL2_SIZE=32'h00010000
               , P_HSEL2_END  =P_HSEL2_START+P_HSEL2_SIZE
               , P_HSEL3_START=32'h30000000, P_HSEL3_SIZE=32'h00010000
               , P_HSEL3_END  =P_HSEL3_START+P_HSEL3_SIZE
               )
(
       input   wire [31:0] HADDR
     , output  wire        HSELd // default slave
     , output  wire        HSEL0
     , output  wire        HSEL1
     , output  wire        HSEL2
     , output  wire        HSEL3
     , input   wire        REMAP
);
   wire [3:0] ihsel;
   wire       ihseld = ~|ihsel;
   assign HSELd = ihseld;
   assign HSEL0 = (REMAP) ? ihsel[1] : ihsel[0];
   assign HSEL1 = (REMAP) ? ihsel[0] : ihsel[1];
   assign HSEL2 = ihsel[2];
   assign HSEL3 = ihsel[3];
   assign ihsel[0] = ((P_NUM>0)&&(HADDR>=P_HSEL0_START)&&(HADDR<P_HSEL0_END)) ? 1'b1 : 1'b0;
   assign ihsel[1] = ((P_NUM>1)&&(HADDR>=P_HSEL1_START)&&(HADDR<P_HSEL1_END)) ? 1'b1 : 1'b0;
   assign ihsel[2] = ((P_NUM>2)&&(HADDR>=P_HSEL2_START)&&(HADDR<P_HSEL2_END)) ? 1'b1 : 1'b0;
   assign ihsel[3] = ((P_NUM>3)&&(HADDR>=P_HSEL3_START)&&(HADDR<P_HSEL3_END)) ? 1'b1 : 1'b0;
   `ifdef RIGOR
   // synthesis translate_off
   initial begin
       if (P_HSEL0_SIZE==0) $display("%m ERROR P_HSEL0_SIZE should be positive 32-bit");
       if (P_HSEL1_SIZE==0) $display("%m ERROR P_HSEL1_SIZE should be positive 32-bit");
       if (P_HSEL2_SIZE==0) $display("%m ERROR P_HSEL2_SIZE should be positive 32-bit");
       if (P_HSEL3_SIZE==0) $display("%m ERROR P_HSEL3_SIZE should be positive 32-bit");
       if ((P_HSEL0_END>P_HSEL1_START)&&
           (P_HSEL0_END<=P_HSEL1_END)) $display("%m ERROR address range overlapped 0:1");
       if ((P_HSEL0_END>P_HSEL2_START)&&
           (P_HSEL0_END<=P_HSEL2_END)) $display("%m ERROR address range overlapped 0:2");
       if ((P_HSEL0_END>P_HSEL3_START)&&
           (P_HSEL0_END<=P_HSEL3_END)) $display("%m ERROR address range overlapped 0:3");
       if ((P_HSEL1_END>P_HSEL0_START)&&
           (P_HSEL1_END<=P_HSEL0_END)) $display("%m ERROR address range overlapped 1:0");
       if ((P_HSEL1_END>P_HSEL2_START)&&
           (P_HSEL1_END<=P_HSEL2_END)) $display("%m ERROR address range overlapped 1:2");
       if ((P_HSEL1_END>P_HSEL3_START)&&
           (P_HSEL1_END<=P_HSEL3_END)) $display("%m ERROR address range overlapped 1:3");
       if ((P_HSEL2_END>P_HSEL0_START)&&
           (P_HSEL2_END<=P_HSEL0_END)) $display("%m ERROR address range overlapped 2:0");
       if ((P_HSEL2_END>P_HSEL1_START)&&
           (P_HSEL2_END<=P_HSEL1_END)) $display("%m ERROR address range overlapped 2:1");
       if ((P_HSEL2_END>P_HSEL3_START)&&
           (P_HSEL2_END<=P_HSEL3_END)) $display("%m ERROR address range overlapped 2:3");
       if ((P_HSEL3_END>P_HSEL0_START)&&
           (P_HSEL3_END<=P_HSEL0_END)) $display("%m ERROR address range overlapped 3:0");
       if ((P_HSEL3_END>P_HSEL1_START)&&
           (P_HSEL3_END<=P_HSEL1_END)) $display("%m ERROR address range overlapped 3:1");
       if ((P_HSEL3_END>P_HSEL2_START)&&
           (P_HSEL3_END<=P_HSEL2_END)) $display("%m ERROR address range overlapped 3:2");
   end
   // synthesis translate_on
   `endif
endmodule
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
module ahb_s2m_s4
(
       input   wire         HRESETn
     , input   wire         HCLK
     , input   wire         HSELd
     , input   wire         HSEL0
     , input   wire         HSEL1
     , input   wire         HSEL2
     , input   wire         HSEL3
     , output  reg   [31:0] HRDATA
     , output  reg   [ 1:0] HRESP
     , output  reg          HREADY
     , input   wire  [31:0] HRDATA0
     , input   wire  [ 1:0] HRESP0
     , input   wire         HREADY0
     , input   wire  [31:0] HRDATA1
     , input   wire  [ 1:0] HRESP1
     , input   wire         HREADY1
     , input   wire  [31:0] HRDATA2
     , input   wire  [ 1:0] HRESP2
     , input   wire         HREADY2
     , input   wire  [31:0] HRDATA3
     , input   wire  [ 1:0] HRESP3
     , input   wire         HREADY3
     , input   wire  [31:0] HRDATAd
     , input   wire  [ 1:0] HRESPd
     , input   wire         HREADYd
);
  localparam D_HSEL0 = 5'h1;
  localparam D_HSEL1 = 5'h2;
  localparam D_HSEL2 = 5'h4;
  localparam D_HSEL3 = 5'h8;
  localparam D_HSELd = 5'h10;
  wire [4:0] _hsel = {HSELd,HSEL3,HSEL2,HSEL1,HSEL0};
  reg  [4:0] _hsel_reg;
  always @ (posedge HCLK or negedge HRESETn) begin
    if (HRESETn==1'b0)   _hsel_reg <= 5'h0;
    else if(HREADY) _hsel_reg <= _hsel; // default HREADY must be 1'b1
  end
  always @ (_hsel_reg or HREADYd or HREADY0 or HREADY1 or HREADY2 or HREADY3) begin
    case(_hsel_reg)
      D_HSEL0: HREADY = HREADY0;
      D_HSEL1: HREADY = HREADY1;
      D_HSEL2: HREADY = HREADY2;
      D_HSEL3: HREADY = HREADY3;
      D_HSELd: HREADY = HREADYd;
      default: HREADY = 1'b1;
    endcase
  end
  always @ (_hsel_reg or HRDATAd or HRDATA0 or HRDATA1 or HRDATA2 or HRDATA3) begin
    case(_hsel_reg)
      D_HSEL0: HRDATA = HRDATA0;
      D_HSEL1: HRDATA = HRDATA1;
      D_HSEL2: HRDATA = HRDATA2;
      D_HSEL3: HRDATA = HRDATA3;
      D_HSELd: HRDATA = HRDATAd;
      default: HRDATA = 32'b0;
    endcase
  end
  always @ (_hsel_reg or HRESPd or HRESP0 or HRESP1 or HRESP2 or HRESP3) begin
    case(_hsel_reg)
      D_HSEL0: HRESP = HRESP0;
      D_HSEL1: HRESP = HRESP1;
      D_HSEL2: HRESP = HRESP2;
      D_HSEL3: HRESP = HRESP3;
      D_HSELd: HRESP = HRESPd;
      default: HRESP = 2'b01; //`HRESP_ERROR;
    endcase
  end
endmodule
//---------------------------------------------------------------------------
//---------------------------------------------------------------------------
`ifndef AHB_DEFAULT_SLAVE_V
`define AHB_DEFAULT_SLAVE_V
module ahb_default_slave
(
       input   wire         HRESETn
     , input   wire         HCLK
     , input   wire         HSEL
     , input   wire  [31:0] HADDR
     , input   wire  [ 1:0] HTRANS
     , input   wire         HWRITE
     , input   wire  [ 2:0] HSIZE
     , input   wire  [ 2:0] HBURST
     , input   wire  [31:0] HWDATA
     , output  wire  [31:0] HRDATA
     , output  reg   [ 1:0] HRESP=2'b01
     , input   wire         HREADYin
     , output  reg          HREADYout=1'b1
);
   assign HRDATA = 32'h0;
   localparam STH_IDLE   = 2'h0
            , STH_WRITE  = 2'h1
            , STH_READ0  = 2'h2;
   reg [1:0] state=STH_IDLE;
   always @ (posedge HCLK or negedge HRESETn) begin
       if (HRESETn==0) begin
           HRESP     <= 2'b00; //`HRESP_OKAY;
           HREADYout <= 1'b1;
           state     <= STH_IDLE;
       end else begin // if (HRESETn==0) begin
           case (state)
           STH_IDLE: begin
                if (HSEL && HREADYin) begin
                   case (HTRANS)
                   //`HTRANS_IDLE, `HTRANS_BUSY: begin
                   2'b00, 2'b01: begin
                          HREADYout <= 1'b1;
                          HRESP     <= 2'b00; //`HRESP_OKAY;
                          state     <= STH_IDLE;
                    end // HTRANS_IDLE or HTRANS_BUSY
                   //`HTRANS_NONSEQ, `HTRANS_SEQ: begin
                   2'b10, 2'b11: begin
                          HREADYout <= 1'b0;
                          HRESP     <= 2'b01; //`HRESP_ERROR;
                          if (HWRITE) begin
                              state <= STH_WRITE;
                          end else begin
                              state <= STH_READ0;
                          end
                    end // HTRANS_NONSEQ or HTRANS_SEQ
                   endcase // HTRANS
                end else begin// if (HSEL && HREADYin)
                    HREADYout <= 1'b1;
                    HRESP     <= 2'b00; //`HRESP_OKAY;
                end
                end // STH_IDLE
           STH_WRITE: begin
                     HREADYout <= 1'b1;
                     HRESP     <= 2'b01; //`HRESP_ERROR;
                     state     <= STH_IDLE;
                end // STH_WRITE
           STH_READ0: begin
                    HREADYout <= 1'b1;
                    HRESP     <= 2'b01; //`HRESP_ERROR;
                    state     <= STH_IDLE;
                end // STH_READ0
           endcase // state
       end // if (HRESETn==0)
   end // always
endmodule
`endif
//---------------------------------------------------------------------------
