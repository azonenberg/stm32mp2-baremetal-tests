`timescale 1ns / 1ps
`default_nettype none

module top(

	//Onboard clock source
	input wire          clk_200mhz_p,
	input wire          clk_200mhz_n,

	//GPIO LEDs on the board
	output logic[3:0]   led   = 0,

	//PCIe high speed signals
	input wire			pcie_rx_p,
	input wire			pcie_rx_n,

	output wire			pcie_tx_p,
	output wire			pcie_tx_n,

	//PCIe reference clock (100 MHz)
	input wire			pcie_refclk_p,
	input wire			pcie_refclk_n,

	//PCIe low speed signals
	output logic	    pcie_clkreq_n	= 0,		//just always request refclk
	input wire			pcie_rst_n
);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Clock buffer and PLL

	wire    clk_200mhz_unbuf;
	wire    clk_200mhz;

	IBUFDS ibuf(.I(clk_200mhz_p), .IB(clk_200mhz_n), .O(clk_200mhz_unbuf));
	BUFG bufg(.I(clk_200mhz_unbuf), .O(clk_200mhz));

	wire	clk_100mhz;
	clk_wiz_0 clk_wiz(
		.clk_in1(clk_200mhz),
		.clk_out1(clk_100mhz));

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// The GTP

	wire	txusrclk;
	wire	txusrclk2;

	wire	rxusrclk;
	wire	rxusrclk2;

	wire[2:0]	subrate	= 2;	//1 = divide by 1 = 5 GT/s
								//2 = divide by 2 = 2.5 GT/s

	wire[15:0]	rx_data;
	wire[1:0]	rx_charisk;
	wire[1:0]	rx_disperr;
	wire[1:0]	rx_symbolerr;

	wire[4:0]	tx_postcursor = 3;
	wire[4:0]	tx_precursor = 0;
	wire[3:0]	tx_diffctrl = 7;

	wire[15:0]	tx_data;
	wire[1:0]	tx_charisk;

	wire		qpll_lock;

	//Blank the link when in reset
	//TODO: is this necessary?
	logic		electrical_idle = 1;
	always_ff @(posedge txusrclk2 or negedge pcie_rst_n) begin
		if(!pcie_rst_n) begin
			electrical_idle	<= 1;
		end
		else begin
			electrical_idle	<= 0;
		end
	end

	gtwizard_0  gtwizard_0_i
	(
		//SERDES side
		.gt0_gtprxp_in(pcie_rx_p),
		.gt0_gtprxn_in(pcie_rx_n),

		.gt0_gtptxp_out(pcie_tx_p),
		.gt0_gtptxn_out(pcie_tx_n),

		//Common ports
		.soft_reset_tx_in(1'b0),
		.soft_reset_rx_in(1'b0),
		.dont_reset_on_data_error_in(1'b0),
		.q0_clk0_gtrefclk_pad_n_in(pcie_refclk_n),
		.q0_clk0_gtrefclk_pad_p_in(pcie_refclk_p),
		.gt0_tx_fsm_reset_done_out(),
		.gt0_rx_fsm_reset_done_out(),
		.gt0_data_valid_in(1'b1),

		.gt0_pll0reset_out(),
		.gt0_pll0outclk_out(),
		.gt0_pll0outrefclk_out(),
		.gt0_pll0lock_out(qpll_lock),
		.gt0_pll0refclklost_out(),
		.gt0_pll1outclk_out(),
		.gt0_pll1outrefclk_out(),
		.sysclk_in(clk_100mhz),

		//Resets
		.gt0_gtrxreset_in(1'b0),
		.gt0_rxlpmreset_in(1'b0),
		.gt0_rxresetdone_out(),
		.gt0_gttxreset_in(1'b0),
		.gt0_txresetdone_out(),

		//Electrical idle until we're reset
		.gt0_txelecidle_in(electrical_idle),

		//Clocking
		.gt0_txuserrdy_in(qpll_lock),
		.gt0_rxuserrdy_in(qpll_lock),
		.gt0_txusrclk_out(txusrclk),
		.gt0_txusrclk2_out(txusrclk2),
		.gt0_rxusrclk_out(rxusrclk),
		.gt0_rxusrclk2_out(rxusrclk2),
		.gt0_rxoutclkfabric_out(),
		.gt0_txoutclkfabric_out(),
		.gt0_txoutclkpcs_out(),

		//TODO: APB here?
		.gt0_drpaddr_in(8'h0),
		.gt0_drpdi_in(16'h0),
		.gt0_drpdo_out(),
		.gt0_drpen_in(1'h0),
		.gt0_drprdy_out(),
		.gt0_drpwe_in(1'h0),

		//Sub-rate control
		.gt0_rxrate_in(subrate),
		.gt0_rxratedone_out(),
		.gt0_txrate_in(subrate),
		.gt0_txratedone_out(),

		//Eye scan stuff we're not using
		.gt0_eyescanreset_in(1'h0),
		.gt0_eyescandataerror_out(),
		.gt0_eyescantrigger_in(1'h0),
		.gt0_dmonitorout_out(1'h0),

		//RX equalizer and polarity
		.gt0_rxpolarity_in(1'b0),		//Invert RX input since LiteFury has pins swapped? or did the soc do that??
		.gt0_rxlpmhfhold_in(1'h0),
		.gt0_rxlpmlfhold_in(1'h0),

		//Fabric RX interface
		.gt0_rxdata_out(rx_data),
		.gt0_rxcharisk_out(rx_charisk),
		.gt0_rxdisperr_out(rx_disperr),
		.gt0_rxnotintable_out(rx_symbolerr),

		//TX driver
		.gt0_txpolarity_in(1'b1),		//Invert TX driver since LiteFury has pins swapped
		.gt0_txpostcursor_in(tx_postcursor),
		.gt0_txprecursor_in(tx_precursor),
		.gt0_txdiffctrl_in(tx_diffctrl),

		//Fabric TX interface
		.gt0_txdata_in(tx_data),
		.gt0_txcharisk_in(tx_charisk)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Soft PCIe endpoint block

	MinimalPCIeEndpoint ep(
		.rst_n(pcie_rst_n),

		.tx_clk(txusrclk2),
		.tx_data(tx_data),
		.tx_charisk(tx_charisk),

		.rx_clk(rxusrclk2),
		.rx_data(rx_data),
		.rx_charisk(rx_charisk),
		.rx_disperr(rx_disperr),
		.rx_symbolerr(rx_symbolerr)
	);

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug LED blinky

	logic[27:0] count = 0;
	always_ff @(posedge clk_200mhz) begin
		count       <= count + 1;

		led <= count[27:24];
	end

	/*
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Debug VIO

	vio_0 vio(
		.clk(clk_200mhz),

		.probe_out0(tx_postcursor),
		.probe_out1(tx_precursor),
		.probe_out2(tx_diffctrl)
	);
	*/

endmodule
