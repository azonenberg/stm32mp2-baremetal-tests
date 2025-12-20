set_property IOSTANDARD LVCMOS33 [get_ports {led[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {led[0]}]
set_property IOSTANDARD LVDS_25 [get_ports clk_200mhz_p]
set_property IOSTANDARD LVDS_25 [get_ports clk_200mhz_n]
set_property PACKAGE_PIN H4 [get_ports {led[3]}]
set_property PACKAGE_PIN G4 [get_ports {led[2]}]
set_property PACKAGE_PIN H3 [get_ports {led[1]}]
set_property PACKAGE_PIN G3 [get_ports {led[0]}]
set_property PACKAGE_PIN J19 [get_ports clk_200mhz_p]
create_clock -period 5.000 -name clk_200mhz_p -waveform {0.000 2.500} [get_ports clk_200mhz_p]

create_clock -period 10.000 -name pcie_refclk_p -waveform {0.000 5.000} [get_ports pcie_refclk_p]
set_property PACKAGE_PIN G1 [get_ports pcie_clkreq_n]
set_property IOSTANDARD LVCMOS33 [get_ports pcie_clkreq_n]
set_property PACKAGE_PIN J1 [get_ports pcie_rst_n]
set_property IOSTANDARD LVCMOS33 [get_ports pcie_rst_n]
set_property PACKAGE_PIN F6 [get_ports pcie_refclk_p]

set_property PACKAGE_PIN B10 [get_ports pcie_rx_p]

create_pblock pblock_rx_checksums
add_cells_to_pblock [get_pblocks pblock_rx_checksums] [get_cells -quiet [list ep/linklayer/rx_dllp_checksum ep/linklayer/rx_tlp_checksum]]
resize_pblock [get_pblocks pblock_rx_checksums] -add {SLICE_X52Y185:SLICE_X63Y199}
resize_pblock [get_pblocks pblock_rx_checksums] -add {DSP48_X1Y74:DSP48_X1Y79}
resize_pblock [get_pblocks pblock_rx_checksums] -add {RAMB18_X1Y74:RAMB18_X1Y79}
resize_pblock [get_pblocks pblock_rx_checksums] -add {RAMB36_X1Y37:RAMB36_X1Y39}
create_pblock pblock_byserdes
add_cells_to_pblock [get_pblocks pblock_byserdes] [get_cells -quiet [list ep/outmux ep/rx_cdc]]
resize_pblock [get_pblocks pblock_byserdes] -add {SLICE_X78Y150:SLICE_X81Y199}
set_property C_CLK_INPUT_FREQ_HZ 300000000 [get_debug_cores dbg_hub]
set_property C_ENABLE_CLK_DIVIDER false [get_debug_cores dbg_hub]
set_property C_USER_SCAN_CHAIN 1 [get_debug_cores dbg_hub]
connect_debug_port dbg_hub/clk [get_nets clk_200mhz]
